#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import TextIO

import serial
from serial.tools import list_ports

try:
    from .common import DEFAULT_OUTPUT_ROOT, ensure_session_dir, extract_tel_payload, parse_tel_payload
except ImportError:
    from common import DEFAULT_OUTPUT_ROOT, ensure_session_dir, extract_tel_payload, parse_tel_payload

LOG_LEVEL_RE = re.compile(r"^(?P<level>[DIWEV]) \(\d+\) ")
ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*m")
LOG_COLORS = {
    "D": "\x1b[90m",       # gray
    "V": "\x1b[90m",       # gray
    "I": "\x1b[32m",       # green
    "W": "\x1b[38;5;214m", # orange
    "E": "\x1b[31m",       # red
}
ANSI_RESET = "\x1b[0m"


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
    return parser


def choose_serial_port(requested: str | None) -> str:
    if requested:
        return requested

    ports = [port.device for port in list_ports.comports()]
    if not ports:
        raise SystemExit("No serial ports found. Use --port explicitly.")
    if len(ports) == 1:
        return ports[0]

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


def write_session_meta(meta_path: Path, *, port: str, baud: int, pre_run_ms: int, post_run_ms: int) -> None:
    meta = {
        "captured_at": datetime.now().astimezone().isoformat(),
        "port": port,
        "baud": baud,
        "pre_run_ms": pre_run_ms,
        "post_run_ms": post_run_ms,
    }
    meta_path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")


def open_run_capture(runs_dir: Path, run_id: int) -> RunCapture:
    path = runs_dir / f"run_{run_id:04d}.ndjson"
    handle = path.open("w", encoding="utf-8", buffering=1)
    return RunCapture(run_id=run_id, path=path, handle=handle)


def close_run_capture(run: RunCapture | None) -> None:
    if run is None:
        return
    run.close()


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


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.list_ports:
        print_serial_ports()
        return 0

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

    print(f"Telemetry session: {session_dir}", file=sys.stderr)
    print(f"Opening serial port {port} @ {args.baud} baud", file=sys.stderr)

    serial_dev = serial.Serial(port=port, baudrate=args.baud, timeout=0.25)
    colorize_logs = should_colorize(args.color)

    pre_run_buffer: deque[BufferedTelemetry] = deque()
    pre_run_us = args.pre_run_ms * 1000
    post_run_us = args.post_run_ms * 1000
    current_run: RunCapture | None = None

    try:
        while True:
            raw = serial_dev.read_until(b"\n")
            if not raw:
                continue

            line = raw.decode(args.encoding, errors="replace")
            session_log.write(line)

            payload = extract_tel_payload(line)
            if payload is None:
                sys.stdout.write(colorize_console_line(line, colorize_logs))
                sys.stdout.flush()
                continue

            if args.show_tel:
                sys.stdout.write(line)
                sys.stdout.flush()

            telemetry_log.write(payload)
            if not payload.endswith("\n"):
                telemetry_log.write("\n")

            record = parse_tel_payload(payload)
            if record is None:
                continue

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
                        print(
                            f"Warning: closing unfinished run {current_run.run_id} before new run_start.",
                            file=sys.stderr,
                        )
                        close_run_capture(current_run)
                    current_run = open_run_capture(runs_dir, run_id)
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

    except KeyboardInterrupt:
        print("\nCapture stopped by user.", file=sys.stderr)
        return 0
    finally:
        close_run_capture(current_run)
        telemetry_log.close()
        session_log.close()
        serial_dev.close()


if __name__ == "__main__":
    raise SystemExit(main())
