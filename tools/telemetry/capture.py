#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, TextIO

import serial
from serial.tools import list_ports

try:
    from .src.common import DEFAULT_OUTPUT_ROOT, ensure_session_dir, extract_tel_payload, parse_tel_payload
except ImportError:
    from src.common import DEFAULT_OUTPUT_ROOT, ensure_session_dir, extract_tel_payload, parse_tel_payload

LOG_LEVEL_RE = re.compile(r"^(?P<level>[DIWEV]) \(\d+\) ")
ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*m")
LOG_COLORS = {
    "D": "\x1b[90m",       # gray
    "V": "\x1b[90m",       # gray
    "I": "\x1b[32m",       # green
    "W": "\x1b[38;5;214m", # orange
    "E": "\x1b[31m",       # red
}
STATUS_BG = "\x1b[48;5;238m"
STATUS_FG = "\x1b[97m"
ANSI_RESET = "\x1b[0m"
PLOT_BLOCKS = " ▁▂▃▄▅▆▇█"
STATUS_KEY = "\x1b[38;5;153m"
STATUS_VALUE = "\x1b[38;5;231m"
STATUS_DIM = "\x1b[38;5;250m"
STATUS_ACCENT = "\x1b[38;5;120m"
STATUS_WARN = "\x1b[38;5;222m"
CHART_AXIS = "\x1b[38;5;145m"
CHART_BAR = "\x1b[38;5;117m"
SUMMARY_TITLE = "\x1b[38;5;230m"
SUMMARY_LABEL = "\x1b[38;5;153m"
SUMMARY_VALUE = "\x1b[38;5;231m"
SUMMARY_DIM = "\x1b[38;5;250m"
HELP_TITLE = "\x1b[38;5;230m"
HELP_LABEL = "\x1b[38;5;153m"
HELP_VALUE = "\x1b[38;5;120m"
HELP_DIM = "\x1b[38;5;250m"
LIVE_STATUS_HEADER_LINES = 3
WEIGHT_PLOT_HEIGHT = 8
# Always leave at least this many rows for the scrolling log area above the
# sticky footer. Without this the fixed-height footer fills a short terminal and
# the log region has no room to scroll (looks static on smaller laptop windows).
MIN_LOG_ROWS = 6


def compute_status_geometry(term_lines: int) -> tuple[int, int]:
    """Return (plot_height, status_line_count) sized to the current terminal.

    The weight plot shrinks (down to 0) so the footer never consumes more than
    ``term_lines - MIN_LOG_ROWS`` rows, guaranteeing a usable log scroll area
    regardless of window height."""
    footer_budget = max(1, term_lines - MIN_LOG_ROWS)
    plot_height = max(0, min(WEIGHT_PLOT_HEIGHT, footer_budget - LIVE_STATUS_HEADER_LINES))
    status_line_count = min(LIVE_STATUS_HEADER_LINES + plot_height, footer_budget)
    return plot_height, max(1, status_line_count)


@dataclass
class BufferedTelemetry:
    ts_us: int | None
    payload: str


@dataclass
class RunCapture:
    run_id: int
    path: Path
    handle: TextIO
    close_after_ts_us: int | None = None

    def write(self, payload: str) -> None:
        self.handle.write(payload)
        if not payload.endswith("\n"):
            self.handle.write("\n")

    def close(self) -> None:
        self.handle.close()


@dataclass
class LiveStatus:
    preset_name: str = "?"
    state_name: str = "?"
    state_num: int = 0
    last_fault: str = "-"
    last_end_reason: str = "-"
    run_id: int = 0
    slot_idx: int = 0
    target_g: float = 0.0
    weight_g: float = 0.0
    relative_fill_g: float = 0.0
    gate_pct: float = 0.0
    last_sample_ts_us: int | None = None


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Capture ESP32 console output, split TEL telemetry from normal logs, "
            "and create per-run NDJSON files."
        )
    )
    parser.add_argument("--port", help="Serial port, for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help="Root folder for captured sessions",
    )
    parser.add_argument(
        "--session-name",
        help="Optional readable suffix added after the timestamped session folder name",
    )
    parser.add_argument(
        "--pre-run-ms",
        type=int,
        default=1000,
        help="Telemetry context copied into each run file before run_start",
    )
    parser.add_argument(
        "--post-run-ms",
        type=int,
        default=1000,
        help="Telemetry context appended after run_end before closing a run file",
    )
    parser.add_argument(
        "--show-tel",
        action="store_true",
        help="Also print TEL lines to the terminal instead of only writing them to files",
    )
    parser.add_argument(
        "--encoding",
        default="utf-8",
        help="Serial text decoding to use for console output",
    )
    parser.add_argument(
        "--color",
        choices=("auto", "always", "never"),
        default="auto",
        help="Colorize ESP log levels in terminal output",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List available serial ports and exit",
    )
    parser.add_argument(
        "--no-clear-start",
        action="store_true",
        help="Do not clear/reset the terminal at startup before live capture UI starts",
    )
    return parser


def choose_serial_port(requested: str | None) -> str:
    if requested:
        return requested

    ports = [port.device for port in list_ports.comports()]
    if not ports:
        raise SystemExit("No serial ports found. Use --port explicitly.")
    if len(ports) == 1:
        return ports[0]
    if "/dev/ttyUSB0" in ports:
        return "/dev/ttyUSB0"

    lines = ["Multiple serial ports found. Use --port explicitly:"]
    lines.extend(f"  {port}" for port in ports)
    raise SystemExit("\n".join(lines))


def print_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        desc = f" - {port.description}" if port.description else ""
        print(f"{port.device}{desc}")


def reset_terminal_state() -> None:
    sys.stdout.write(f"{ANSI_RESET}\x1b[?25h\r")
    sys.stdout.flush()


def prepare_terminal_for_capture(*, clear_screen: bool) -> None:
    reset_terminal_state()
    if clear_screen and sys.stdout.isatty():
        # Match the effect of a normal terminal clear closely enough to start
        # the footer redraw logic from a known clean screen/cursor state.
        sys.stdout.write("\x1b[2J\x1b[H")
    sys.stdout.flush()


def clip_ansi_text(text: str, max_visible: int) -> str:
    if max_visible <= 0:
        return ""

    visible = 0
    out: list[str] = []
    i = 0
    while i < len(text) and visible < max_visible:
        if text[i] == "\x1b":
            end = i + 1
            while end < len(text) and text[end] != "m":
                end += 1
            if end < len(text):
                end += 1
            out.append(text[i:end])
            i = end
            continue
        out.append(text[i])
        visible += 1
        i += 1
    out.append(ANSI_RESET)
    return "".join(out)


def format_port_selection_note(requested: str | None) -> str | None:
    if requested:
        return None
    ports = list(list_ports.comports())
    if not ports:
        return None
    devices = [port.device for port in ports]
    if len(devices) <= 1 and "/dev/ttyUSB0" not in devices:
        return None

    selected = "/dev/ttyUSB0" if "/dev/ttyUSB0" in devices else devices[0]
    shown = ", ".join(devices)
    return (
        f"Available ports: {shown}. "
        f"Selecting default {selected}. Override with --port."
    )


def write_session_meta(meta_path: Path, *, port: str, baud: int, pre_run_ms: int, post_run_ms: int) -> None:
    meta = {
        "captured_at": datetime.now().astimezone().isoformat(),
        "port": port,
        "baud": baud,
        "pre_run_ms": pre_run_ms,
        "post_run_ms": post_run_ms,
    }
    meta_path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")


def print_usage_summary(*, status_line_enabled: bool, scroll_bottom: int | None = None) -> None:
    sep = f"{HELP_DIM}{'=' * 72}{ANSI_RESET}"
    lines = [
        sep,
        f"{HELP_TITLE}Telemetry Capture Quick Help{ANSI_RESET}",
        f"  {HELP_LABEL}Basic:{ANSI_RESET} {HELP_VALUE}capture.py [--port /dev/ttyUSB0] [--baud 115200] [--session-name thick-honey-test]{ANSI_RESET}",
        f"  {HELP_LABEL}Common flags:{ANSI_RESET}",
        f"    {HELP_VALUE}--output-root <dir>{ANSI_RESET}   {HELP_DIM}change capture root folder{ANSI_RESET}",
        f"    {HELP_VALUE}--pre-run-ms <ms>{ANSI_RESET}     {HELP_DIM}telemetry kept before run_start{ANSI_RESET}",
        f"    {HELP_VALUE}--post-run-ms <ms>{ANSI_RESET}    {HELP_DIM}telemetry kept after run_end{ANSI_RESET}",
        f"    {HELP_VALUE}--show-tel{ANSI_RESET}            {HELP_DIM}also print TEL JSON lines{ANSI_RESET}",
        f"    {HELP_VALUE}--color auto|always|never{ANSI_RESET}",
        f"    {HELP_VALUE}--list-ports{ANSI_RESET}          {HELP_DIM}list serial ports and exit{ANSI_RESET}",
        f"    {HELP_VALUE}--no-clear-start{ANSI_RESET}      {HELP_DIM}keep existing terminal content on startup{ANSI_RESET}",
        f"  {HELP_LABEL}More:{ANSI_RESET} {HELP_DIM}use --help for full argparse help{ANSI_RESET}",
        sep,
        f"{HELP_DIM}Starting live capture output below...{ANSI_RESET}",
        sep,
    ]
    for line in lines:
        print_status_note(status_line_enabled, line, scroll_bottom=scroll_bottom)


def open_run_capture(runs_dir: Path, run_id: int) -> RunCapture:
    path = runs_dir / f"run_{run_id:04d}.ndjson"
    handle = path.open("w", encoding="utf-8", buffering=1)
    return RunCapture(run_id=run_id, path=path, handle=handle)


def close_run_capture(run: RunCapture | None) -> None:
    if run is None:
        return
    run.close()


def status_enabled() -> bool:
    return sys.stdout.isatty() and os.environ.get("TERM", "dumb") != "dumb"


def set_scroll_region(top: int, bottom: int) -> None:
    """Restrict scrolling (via LF/IND at the bottom margin) to rows top..bottom.

    Rows below `bottom` (the sticky footer) are left untouched by log output,
    instead of relying on relative cursor motion to fake independent scrolling."""
    sys.stdout.write(f"\x1b[{top};{bottom}r")


def reset_scroll_region() -> None:
    sys.stdout.write("\x1b[r")


def move_cursor(row: int, col: int = 1) -> None:
    sys.stdout.write(f"\x1b[{row};{col}H")


def clear_rows(row_start: int, row_count: int) -> None:
    """Clear `row_count` absolute terminal rows starting at `row_start`."""
    if row_count <= 0:
        return
    for offset in range(row_count):
        sys.stdout.write(f"\x1b[{row_start + offset};1H\x1b[2K")


def print_status_block(lines: list[str], *, footer_top: int, term_cols: int) -> None:
    if not lines:
        return
    for offset, line in enumerate(lines):
        text = clip_ansi_text(line, max(0, term_cols - 1))
        sys.stdout.write(
            f"\x1b[{footer_top + offset};1H\x1b[2K"
            f"{STATUS_BG}{STATUS_FG}{text}\x1b[K{ANSI_RESET}"
        )
    sys.stdout.flush()


def detach_status_area(enabled: bool, term_lines: int) -> None:
    if not enabled:
        return
    reset_scroll_region()
    move_cursor(term_lines, 1)
    sys.stdout.write("\n")
    sys.stdout.write(ANSI_RESET)
    sys.stdout.flush()


def status_fmt_key(text: str) -> str:
    return f"{STATUS_KEY}{text}{STATUS_FG}"


def status_fmt_value(text: str, *, accent: bool = False, warn: bool = False, dim: bool = False) -> str:
    color = STATUS_VALUE
    if accent:
        color = STATUS_ACCENT
    elif warn:
        color = STATUS_WARN
    elif dim:
        color = STATUS_DIM
    return f"{color}{text}{STATUS_FG}"


def print_shutdown_summary(
    *,
    interrupted: bool,
    elapsed_s: float,
    console_line_count: int,
    tel_count: int,
    run_count: int,
    session_dir: Path,
    session_log_path: Path,
    telemetry_path: Path,
) -> None:
    title = "Capture stopped by user" if interrupted else "Capture finished"
    sep = f"{SUMMARY_DIM}{'-' * 72}{ANSI_RESET}"
    print(sep)
    print(f"{SUMMARY_TITLE}{title}{ANSI_RESET}")
    print(sep)
    print(
        f"  {SUMMARY_LABEL}Duration:{ANSI_RESET} "
        f"{SUMMARY_VALUE}{elapsed_s:.1f}s{ANSI_RESET}"
    )
    print(
        f"  {SUMMARY_LABEL}Console lines:{ANSI_RESET} "
        f"{SUMMARY_VALUE}{console_line_count}{ANSI_RESET}"
    )
    print(
        f"  {SUMMARY_LABEL}Telemetry rows:{ANSI_RESET} "
        f"{SUMMARY_VALUE}{tel_count}{ANSI_RESET}"
    )
    print(
        f"  {SUMMARY_LABEL}Runs:{ANSI_RESET} "
        f"{SUMMARY_VALUE}{run_count}{ANSI_RESET}"
    )
    print()
    print(f"  {SUMMARY_LABEL}Session dir:{ANSI_RESET}")
    print(f"    {SUMMARY_VALUE}{session_dir}{ANSI_RESET}")
    print(f"  {SUMMARY_LABEL}Session log:{ANSI_RESET}")
    print(
        f"    {SUMMARY_VALUE}{session_log_path}{ANSI_RESET} "
        f"{SUMMARY_DIM}({session_log_path.stat().st_size} B){ANSI_RESET}"
    )
    print(f"  {SUMMARY_LABEL}Telemetry:{ANSI_RESET}")
    print(
        f"    {SUMMARY_VALUE}{telemetry_path}{ANSI_RESET} "
        f"{SUMMARY_DIM}({telemetry_path.stat().st_size} B){ANSI_RESET}"
    )
    print(sep)


def print_status_note(enabled: bool, text: str, *, scroll_bottom: int | None = None) -> None:
    if not enabled or scroll_bottom is None:
        print(text)
        return
    move_cursor(scroll_bottom, 1)
    sys.stdout.write("\x1b[2K")
    print(text)
    sys.stdout.flush()


def should_colorize(mode: str) -> bool:
    if mode == "always":
        return True
    if mode == "never":
        return False
    if os.environ.get("NO_COLOR"):
        return False
    return sys.stdout.isatty() and os.environ.get("TERM", "dumb") != "dumb"


def colorize_console_line(line: str, enabled: bool) -> str:
    if not enabled:
        return line
    if ANSI_ESCAPE_RE.search(line):
        return line

    match = LOG_LEVEL_RE.match(line)
    if not match:
        return line

    color = LOG_COLORS.get(match.group("level"))
    if not color:
        return line

    if line.endswith("\n"):
        return f"{color}{line[:-1]}{ANSI_RESET}\n"
    return f"{color}{line}{ANSI_RESET}"


def write_console_line(line: str, *, status_line_enabled: bool, colorize_logs: bool, scroll_bottom: int) -> None:
    if status_line_enabled:
        move_cursor(scroll_bottom, 1)
        sys.stdout.write("\x1b[2K")
    sys.stdout.write(colorize_console_line(line, colorize_logs))
    sys.stdout.flush()


def format_float(value: float) -> str:
    return f"{value:.1f}"


def normalize_state_name(state_name: str, state_num: int) -> str:
    if state_name and state_name != "?":
        return state_name
    return str(state_num)


def build_weight_plot(samples: deque[float], width: int, height: int, min_g: float, max_g: float) -> list[str]:
    if width <= 0 or height <= 0:
        return []

    rows = [[" " for _ in range(width)] for _ in range(height)]
    recent = list(samples)[-width:]
    if not recent:
        recent = []
    pad = width - len(recent)

    span = max(max_g - min_g, 1.0)
    total_levels = height * 8
    for x, value in enumerate(recent, start=pad):
        clipped = min(max(value, min_g), max_g)
        filled_levels = int(round(((clipped - min_g) / span) * total_levels))
        filled_levels = max(0, min(total_levels, filled_levels))
        for row_idx in range(height):
            row_base = (height - 1 - row_idx) * 8
            cell_fill = max(0, min(8, filled_levels - row_base))
            rows[row_idx][x] = PLOT_BLOCKS[cell_fill]

    labels = []
    for row_idx, row in enumerate(rows):
        y = max_g - ((max_g - min_g) * row_idx / max(height - 1, 1))
        labels.append(
            f"{CHART_AXIS}{int(round(y)):>3}|{STATUS_FG}{CHART_BAR}{''.join(row)}{STATUS_FG}"
        )
    return labels


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.list_ports:
        print_serial_ports()
        return 0

    prepare_terminal_for_capture(clear_screen=not args.no_clear_start)
    port = choose_serial_port(args.port)
    session_dir = ensure_session_dir(args.output_root, args.session_name)
    runs_dir = session_dir / "runs"
    runs_dir.mkdir(parents=True, exist_ok=True)

    write_session_meta(
        session_dir / "session_meta.json",
        port=port,
        baud=args.baud,
        pre_run_ms=args.pre_run_ms,
        post_run_ms=args.post_run_ms,
    )

    session_log = (session_dir / "session.log").open("w", encoding="utf-8", buffering=1)
    telemetry_log = (session_dir / "telemetry.ndjson").open("w", encoding="utf-8", buffering=1)

    status_line_enabled = status_enabled()
    term_size = shutil.get_terminal_size((120, 24))
    plot_height, status_line_count = compute_status_geometry(term_size.lines)
    scroll_bottom = max(1, term_size.lines - status_line_count)
    if status_line_enabled:
        # Confine LF-triggered scrolling to rows 1..scroll_bottom so the sticky
        # footer below it never freezes the log area, regardless of terminal
        # height (see compute_status_geometry / MIN_LOG_ROWS).
        set_scroll_region(1, scroll_bottom)
        move_cursor(scroll_bottom, 1)

    print_usage_summary(status_line_enabled=status_line_enabled, scroll_bottom=scroll_bottom)
    print_status_note(status_line_enabled, f"Telemetry session: {session_dir}", scroll_bottom=scroll_bottom)
    port_note = format_port_selection_note(args.port)
    if port_note:
        print_status_note(status_line_enabled, port_note, scroll_bottom=scroll_bottom)
    print_status_note(status_line_enabled, f"Opening serial port {port} @ {args.baud} baud", scroll_bottom=scroll_bottom)

    serial_dev = serial.Serial(port=port, baudrate=args.baud, timeout=0.25)
    colorize_logs = should_colorize(args.color)

    pre_run_buffer: deque[BufferedTelemetry] = deque()
    pre_run_us = args.pre_run_ms * 1000
    post_run_us = args.post_run_ms * 1000
    current_run: RunCapture | None = None
    tel_count = 0
    run_count = 0
    console_line_count = 0
    last_status_refresh = 0.0
    pending_bytes = bytearray()
    live = LiveStatus()
    weight_history: deque[float] = deque(maxlen=180)
    started_at = time.monotonic()
    interrupted = False

    def refresh_status(force: bool = False) -> None:
        nonlocal last_status_refresh, status_line_count, scroll_bottom
        if not status_line_enabled:
            return
        now = time.monotonic()
        if not force and (now - last_status_refresh) < 0.1:
            return
        active = f"run={current_run.run_id}" if current_run is not None else "run=idle"
        term_size = shutil.get_terminal_size((120, 24))
        term_width = term_size.columns
        plot_width = max(24, min(108, term_width - 5))
        plot_height, new_status_count = compute_status_geometry(term_size.lines)
        new_scroll_bottom = max(1, term_size.lines - new_status_count)
        plot_lines = build_weight_plot(weight_history, plot_width, plot_height, 0.0, 500.0)
        if (new_scroll_bottom, new_status_count) != (scroll_bottom, status_line_count):
            # Geometry changed (window resize or chart-height change): clear
            # both the old and new footer rows, then move the scroll margin.
            clear_rows(scroll_bottom + 1, status_line_count)
            clear_rows(new_scroll_bottom + 1, new_status_count)
            set_scroll_region(1, new_scroll_bottom)
            scroll_bottom = new_scroll_bottom
            status_line_count = new_status_count
        visible_headers = max(0, status_line_count - len(plot_lines))
        print_status_block(
            [
                (
                    f"{status_fmt_key('[capture]')} "
                    f"{status_fmt_key('session=')}{status_fmt_value(session_dir.name, dim=True)} "
                    f"{status_fmt_key('port=')}{status_fmt_value(port)} "
                    f"{status_fmt_key('tel=')}{status_fmt_value(str(tel_count), accent=True)} "
                    f"{status_fmt_key('runs=')}{status_fmt_value(str(run_count), accent=True)} "
                    f"{status_fmt_key('run=')}{status_fmt_value(active.split('=', 1)[1])}"
                ),
                (
                    f"{status_fmt_key('[live]')} "
                    f"{status_fmt_key('preset=')}{status_fmt_value(live.preset_name[:18])} "
                    f"{status_fmt_key('state=')}{status_fmt_value(normalize_state_name(live.state_name, live.state_num)[:18], accent=True)} "
                    f"{status_fmt_key('fault=')}{status_fmt_value(live.last_fault[:18], warn=(live.last_fault != '-'), dim=(live.last_fault == '-'))}"
                ),
                (
                    f"{status_fmt_key('[live]')} "
                    f"{status_fmt_key('weight=')}{status_fmt_value(f'{format_float(live.weight_g)}g', accent=True)} "
                    f"{status_fmt_key('rel=')}{status_fmt_value(f'{format_float(live.relative_fill_g)}g')} "
                    f"{status_fmt_key('target=')}{status_fmt_value(f'{int(live.target_g)}g')} "
                    f"{status_fmt_key('gate=')}{status_fmt_value(f'{format_float(live.gate_pct)}%')} "
                    f"{status_fmt_key('slot=')}{status_fmt_value(str(live.slot_idx))} "
                    f"{status_fmt_key('end=')}{status_fmt_value(live.last_end_reason[:12], dim=(live.last_end_reason == '-'))}"
                ),
            ][:visible_headers]
            + plot_lines,
            footer_top=scroll_bottom + 1,
            term_cols=term_width,
        )
        last_status_refresh = now

    def handle_console_line(line: str) -> None:
        nonlocal current_run
        nonlocal run_count
        nonlocal tel_count
        nonlocal console_line_count

        session_log.write(line)
        console_line_count += 1

        payload = extract_tel_payload(line)
        if payload is None:
            write_console_line(line,
                               status_line_enabled=status_line_enabled,
                               colorize_logs=colorize_logs,
                               scroll_bottom=scroll_bottom)
            refresh_status(force=True)
            return

        if args.show_tel:
            write_console_line(line,
                               status_line_enabled=status_line_enabled,
                               colorize_logs=False,
                               scroll_bottom=scroll_bottom)
            refresh_status(force=True)

        telemetry_log.write(payload)
        if not payload.endswith("\n"):
            telemetry_log.write("\n")
        tel_count += 1

        record = parse_tel_payload(payload)
        if record is None:
            refresh_status()
            return

        update_live_status(live, weight_history, record)

        ts_us_raw = record.get("ts_us")
        ts_us = int(ts_us_raw) if isinstance(ts_us_raw, int) else None
        kind = record.get("kind")

        if ts_us is not None:
            min_ts_us = ts_us - pre_run_us
            while pre_run_buffer and pre_run_buffer[0].ts_us is not None and pre_run_buffer[0].ts_us < min_ts_us:
                pre_run_buffer.popleft()

        if kind == "run_start":
            run_id_raw = record.get("run_id")
            run_id = int(run_id_raw) if isinstance(run_id_raw, int) else None
            if run_id is not None:
                if current_run is not None:
                    print_status_note(
                        status_line_enabled,
                        f"Warning: closing unfinished run {current_run.run_id} before new run_start.",
                        scroll_bottom=scroll_bottom,
                    )
                    close_run_capture(current_run)
                current_run = open_run_capture(runs_dir, run_id)
                run_count += 1
                for buffered in pre_run_buffer:
                    current_run.write(buffered.payload)

        if current_run is not None:
            current_run.write(payload)
            if kind == "run_end" and record.get("run_id") == current_run.run_id and ts_us is not None:
                current_run.close_after_ts_us = ts_us + post_run_us

            if current_run.close_after_ts_us is not None and ts_us is not None and ts_us >= current_run.close_after_ts_us:
                close_run_capture(current_run)
                current_run = None

        pre_run_buffer.append(BufferedTelemetry(ts_us=ts_us, payload=payload))
        refresh_status()

    def update_live_status(status: LiveStatus, history: deque[float], record: dict[str, Any]) -> None:
        kind = record.get("kind")
        if kind == "preset":
            preset_name = record.get("text")
            if isinstance(preset_name, str) and preset_name:
                status.preset_name = preset_name
            return
        if kind == "state":
            state_name = record.get("text")
            if isinstance(state_name, str) and state_name:
                status.state_name = state_name
            state = record.get("state")
            if isinstance(state, int):
                status.state_num = state
            run_id = record.get("run_id")
            slot_idx = record.get("slot_idx")
            if isinstance(run_id, int):
                status.run_id = run_id
            if isinstance(slot_idx, int):
                status.slot_idx = slot_idx
            return
        if kind == "fault":
            fault_name = record.get("text")
            if isinstance(fault_name, str) and fault_name:
                status.last_fault = fault_name
            return
        if kind == "run_end":
            reason = record.get("text")
            if isinstance(reason, str) and reason:
                status.last_end_reason = reason
            return
        if kind != "sample":
            return

        run_id = record.get("run_id")
        slot_idx = record.get("slot_idx")
        state = record.get("state")
        target_g = record.get("target_g")
        weight_g = record.get("weight_g")
        rel_g = record.get("relative_fill_g")
        gate_pct = record.get("gate_pct")
        ts_us = record.get("ts_us")

        if isinstance(run_id, int):
            status.run_id = run_id
        if isinstance(slot_idx, int):
            status.slot_idx = slot_idx
        if isinstance(state, int):
            status.state_num = state
            if status.state_name == "?":
                status.state_name = str(state)
        if isinstance(target_g, (int, float)):
            status.target_g = float(target_g)
        if isinstance(weight_g, (int, float)):
            status.weight_g = float(weight_g)
        if isinstance(rel_g, (int, float)):
            status.relative_fill_g = float(rel_g)
            history.append(status.relative_fill_g)
        if isinstance(gate_pct, (int, float)):
            status.gate_pct = float(gate_pct)
        if isinstance(ts_us, int):
            status.last_sample_ts_us = ts_us

    try:
        while True:
            raw = serial_dev.read(serial_dev.in_waiting or 1)
            if not raw:
                refresh_status()
                continue

            pending_bytes.extend(raw)
            while True:
                newline_idx = pending_bytes.find(b"\n")
                if newline_idx < 0:
                    break
                line_bytes = pending_bytes[:newline_idx + 1]
                del pending_bytes[:newline_idx + 1]
                line = line_bytes.decode(args.encoding, errors="replace")
                handle_console_line(line)

    except KeyboardInterrupt:
        interrupted = True
    finally:
        if status_line_enabled:
            detach_status_area(status_line_enabled, shutil.get_terminal_size((120, 24)).lines)
        reset_terminal_state()
        session_log.flush()
        telemetry_log.flush()
        close_run_capture(current_run)
        telemetry_log.close()
        session_log.close()
        serial_dev.close()

    elapsed_s = time.monotonic() - started_at
    session_log_path = session_dir / "session.log"
    telemetry_path = session_dir / "telemetry.ndjson"
    print_shutdown_summary(
        interrupted=interrupted,
        elapsed_s=elapsed_s,
        console_line_count=console_line_count,
        tel_count=tel_count,
        run_count=run_count,
        session_dir=session_dir,
        session_log_path=session_log_path,
        telemetry_path=telemetry_path,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
