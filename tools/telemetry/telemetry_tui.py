#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from serial.tools import list_ports

try:
    from .src.common import DEFAULT_OUTPUT_ROOT, REPO_ROOT
except ImportError:
    from src.common import DEFAULT_OUTPUT_ROOT, REPO_ROOT

ANSI_RESET = "\x1b[0m"
ANSI_BOLD = "\x1b[1m"
ANSI_DIM = "\x1b[2m"
FG_TITLE = "\x1b[38;5;230m"
FG_LABEL = "\x1b[38;5;153m"
FG_VALUE = "\x1b[38;5;231m"
FG_ACCENT = "\x1b[38;5;120m"
FG_WARN = "\x1b[38;5;222m"
FG_ERROR = "\x1b[38;5;203m"
FG_HINT = "\x1b[38;5;250m"
BG_PANEL = "\x1b[48;5;238m"

SCRIPT_DIR = Path(__file__).resolve().parent
CAPTURE_SCRIPT = SCRIPT_DIR / "capture.py"
SPLIT_SCRIPT = SCRIPT_DIR / "split_runs.py"
PLOT_SCRIPT = SCRIPT_DIR / "plot_runs.py"


@dataclass(slots=True)
class SessionInfo:
    path: Path
    captured_at: str | None
    port: str | None
    pre_ms: int | None
    post_ms: int | None
    fill_count: int | None
    figure_count: int

    @property
    def name(self) -> str:
        return self.path.name


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Interactive TUI launcher for the telemetry toolchain. "
            "Recommended entrypoint for capture, split, and plot workflows."
        )
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help="Telemetry session root folder",
    )
    return parser


def supports_color() -> bool:
    return sys.stdout.isatty() and os.environ.get("TERM", "dumb") != "dumb"


def style(text: str, *codes: str) -> str:
    if not supports_color():
        return text
    return "".join(codes) + text + ANSI_RESET


def clear_screen() -> None:
    if supports_color():
        sys.stdout.write("\x1b[2J\x1b[H")
        sys.stdout.flush()


def print_header(title: str, subtitle: str) -> None:
    clear_screen()
    sep = style("=" * 78, FG_HINT)
    print(sep)
    print(style(title, ANSI_BOLD, FG_TITLE))
    print(style(subtitle, FG_HINT))
    print(sep)


def print_panel(title: str, lines: Sequence[str]) -> None:
    print(style(f" {title} ", ANSI_BOLD, FG_TITLE))
    for line in lines:
        if supports_color():
            print(f"{BG_PANEL}{FG_VALUE} {line}{ANSI_RESET}")
        else:
            print(f"  {line}")


def discover_sessions(output_root: Path) -> list[SessionInfo]:
    if not output_root.exists():
        return []

    sessions: list[SessionInfo] = []
    for entry in sorted(output_root.iterdir(), reverse=True):
        if not entry.is_dir():
            continue
        if not (entry / "telemetry.ndjson").exists():
            continue

        meta = load_json_file(entry / "session_meta.json")
        fills_manifest = load_json_file(entry / "fills" / "index.json")
        figures_dir = entry / "figures"
        figure_count = len(list(figures_dir.glob("*.pdf"))) if figures_dir.exists() else 0
        fill_count = None
        if isinstance(fills_manifest, dict) and isinstance(fills_manifest.get("fills"), list):
            fill_count = len(fills_manifest["fills"])

        sessions.append(
            SessionInfo(
                path=entry,
                captured_at=str(meta.get("captured_at")) if isinstance(meta, dict) and meta.get("captured_at") else None,
                port=str(meta.get("port")) if isinstance(meta, dict) and meta.get("port") else None,
                pre_ms=int(meta["pre_run_ms"]) if isinstance(meta, dict) and meta.get("pre_run_ms") is not None else None,
                post_ms=int(meta["post_run_ms"]) if isinstance(meta, dict) and meta.get("post_run_ms") is not None else None,
                fill_count=fill_count,
                figure_count=figure_count,
            )
        )
    return sessions


def load_json_file(path: Path) -> dict | list | None:
    if not path.exists():
        return None
    try:
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle)
    except (json.JSONDecodeError, OSError):
        return None


def prompt(text: str, default: str | None = None) -> str:
    suffix = f" [{default}]" if default not in (None, "") else ""
    while True:
        raw = input(style(f"{text}{suffix}: ", FG_LABEL)).strip()
        if raw:
            return raw
        if default is not None:
            return default


def prompt_int(text: str, default: int, *, minimum: int | None = None) -> int:
    while True:
        raw = prompt(text, str(default))
        try:
            value = int(raw)
        except ValueError:
            print(style("Enter a whole number.", FG_ERROR))
            continue
        if minimum is not None and value < minimum:
            print(style(f"Enter a value >= {minimum}.", FG_ERROR))
            continue
        return value


def prompt_yes_no(text: str, default: bool) -> bool:
    default_label = "Y/n" if default else "y/N"
    while True:
        raw = input(style(f"{text} [{default_label}]: ", FG_LABEL)).strip().lower()
        if not raw:
            return default
        if raw in {"y", "yes"}:
            return True
        if raw in {"n", "no"}:
            return False
        print(style("Enter y or n.", FG_ERROR))


def prompt_choice(text: str, options: Sequence[tuple[str, str]], default_key: str) -> str:
    labels = ", ".join(
        f"{style(key, FG_ACCENT)}={label}" if key == default_key else f"{key}={label}"
        for key, label in options
    )
    print(style(f"{text}: {labels}", FG_HINT))
    valid = {key for key, _ in options}
    while True:
        raw = prompt("Select", default_key).lower()
        if raw in valid:
            return raw
        print(style(f"Choose one of: {', '.join(sorted(valid))}", FG_ERROR))


def wait_for_enter() -> None:
    input(style("Press Enter to continue...", FG_HINT))


def print_command_preview(cmd: Sequence[str]) -> None:
    print()
    print(style("Command to run", ANSI_BOLD, FG_TITLE))
    print(style(shlex.join(cmd), FG_ACCENT))
    print()


def print_batch_preview(jobs: Sequence[tuple[str, Sequence[str]]]) -> None:
    print()
    print(style("Batch commands to run", ANSI_BOLD, FG_TITLE))
    for index, (label, cmd) in enumerate(jobs, start=1):
        print(style(f"[{index}/{len(jobs)}] {label}", FG_LABEL))
        print(style(shlex.join(cmd), FG_ACCENT))
    print()


def print_command_result(cmd: Sequence[str], returncode: int) -> None:
    print()
    print(style("-" * 78, FG_HINT))
    print(style("Command finished", ANSI_BOLD, FG_TITLE))
    print(f"{style('Exit code', FG_LABEL)}: {style(str(returncode), FG_VALUE if returncode == 0 else FG_ERROR)}")
    print(f"{style('Command', FG_LABEL)}: {style(shlex.join(cmd), FG_ACCENT)}")
    if returncode == 0:
        print(style("Review the output above. Press Enter to return to the workflow menu.", FG_HINT))
    else:
        print(style("The tool reported an error. Review the output above before continuing.", FG_WARN))
    print(style("-" * 78, FG_HINT))


def run_tool(cmd: Sequence[str]) -> int:
    print_command_preview(cmd)
    if not prompt_yes_no("Run this command now?", True):
        print(style("Cancelled.", FG_WARN))
        wait_for_enter()
        return 1
    print()
    returncode = subprocess.run(cmd, cwd=REPO_ROOT, check=False).returncode
    print_command_result(cmd, returncode)
    wait_for_enter()
    return returncode


def run_tool_batch(jobs: Sequence[tuple[str, Sequence[str]]]) -> int:
    if not jobs:
        print(style("Nothing to run.", FG_WARN))
        wait_for_enter()
        return 1

    print_batch_preview(jobs)
    if not prompt_yes_no("Run this batch now?", True):
        print(style("Cancelled.", FG_WARN))
        wait_for_enter()
        return 1

    results: list[tuple[str, int]] = []
    print()
    for index, (label, cmd) in enumerate(jobs, start=1):
        print(style("=" * 78, FG_HINT))
        print(style(f"Running {index}/{len(jobs)}: {label}", ANSI_BOLD, FG_TITLE))
        print(style(shlex.join(cmd), FG_ACCENT))
        print(style("-" * 78, FG_HINT))
        returncode = subprocess.run(cmd, cwd=REPO_ROOT, check=False).returncode
        results.append((label, returncode))
        print(style(f"Exit code: {returncode}", FG_VALUE if returncode == 0 else FG_ERROR))
        print()

    failures = sum(1 for _, code in results if code != 0)
    print(style("=" * 78, FG_HINT))
    print(style("Batch finished", ANSI_BOLD, FG_TITLE))
    print(f"{style('Jobs', FG_LABEL)}: {style(str(len(results)), FG_VALUE)}")
    print(f"{style('Failures', FG_LABEL)}: {style(str(failures), FG_VALUE if failures == 0 else FG_ERROR)}")
    for label, returncode in results:
        color = FG_VALUE if returncode == 0 else FG_ERROR
        print(f"  {style(label, FG_LABEL)}: {style(str(returncode), color)}")
    print(style("=" * 78, FG_HINT))
    wait_for_enter()
    return 0 if failures == 0 else 1


def _parse_selection_list(raw: str, *, max_index: int) -> list[int]:
    selected: list[int] = []
    seen: set[int] = set()
    for part in raw.split(","):
        item = part.strip()
        if not item:
            continue
        if "-" in item:
            start_text, end_text = item.split("-", 1)
            start = int(start_text)
            end = int(end_text)
            if start > end:
                start, end = end, start
            for value in range(start, end + 1):
                if value < 1 or value > max_index:
                    raise ValueError("Choice out of range")
                if value not in seen:
                    selected.append(value)
                    seen.add(value)
            continue
        value = int(item)
        if value < 1 or value > max_index:
            raise ValueError("Choice out of range")
        if value not in seen:
            selected.append(value)
            seen.add(value)
    return selected


def choose_sessions(output_root: Path, *, purpose: str) -> list[SessionInfo]:
    sessions = discover_sessions(output_root)
    if not sessions:
        print(style(f"No telemetry sessions found in {output_root}", FG_WARN))
        return []

    visible_sessions = sessions[:20]
    print(style(f"Select session(s) for {purpose}", ANSI_BOLD, FG_TITLE))
    print(style("Use comma lists or ranges like 1,3,5-7. Use 'a' for all shown.", FG_HINT))
    for index, session in enumerate(visible_sessions, start=1):
        parts = [session.name]
        if session.port:
            parts.append(f"port={session.port}")
        if session.fill_count is not None:
            parts.append(f"fills={session.fill_count}")
        if session.figure_count:
            parts.append(f"figures={session.figure_count}")
        print(f"  {style(str(index), FG_ACCENT)}. {' | '.join(parts)}")
    print(f"  {style('a', FG_ACCENT)}. All listed sessions")
    print(f"  {style('m', FG_ACCENT)}. Enter one or more paths manually")
    default_key = "1"
    while True:
        raw = prompt("Choice", default_key).lower()
        if raw == "a":
            return visible_sessions
        if raw == "m":
            manual_raw = prompt("Absolute or relative session paths (comma-separated)")
            selected_manual: list[SessionInfo] = []
            valid = True
            for part in manual_raw.split(","):
                manual = Path(part.strip()).expanduser()
                candidate = (manual if manual.is_absolute() else REPO_ROOT / manual).resolve()
                if not (candidate / "telemetry.ndjson").exists():
                    print(style(f"Not a telemetry session folder: {candidate}", FG_ERROR))
                    valid = False
                    break
                selected_manual.append(SessionInfo(candidate, None, None, None, None, None, 0))
            if valid and selected_manual:
                return selected_manual
            continue
        try:
            selected_indexes = _parse_selection_list(raw, max_index=len(visible_sessions))
        except ValueError:
            print(style("Enter listed numbers, ranges like 2-4, 'a', or 'm'.", FG_ERROR))
            continue
        if selected_indexes:
            return [visible_sessions[idx - 1] for idx in selected_indexes]
        print(style("No valid selections given.", FG_ERROR))


def choose_serial_port() -> str:
    ports = [port.device for port in list_ports.comports()]
    if not ports:
        return prompt("Serial port", "/dev/ttyUSB0")

    print(style("Available serial ports", ANSI_BOLD, FG_TITLE))
    for index, port in enumerate(ports, start=1):
        print(f"  {style(str(index), FG_ACCENT)}. {port}")
    default = "/dev/ttyUSB0" if "/dev/ttyUSB0" in ports else ports[0]
    while True:
        raw = prompt("Port number or path", default)
        if raw in ports:
            return raw
        try:
            idx = int(raw)
        except ValueError:
            return raw
        if 1 <= idx <= len(ports):
            return ports[idx - 1]
        print(style("Choice out of range.", FG_ERROR))


def parse_fill_selection(fill_count: int | None) -> str | None:
    if not fill_count:
        return None
    selection = prompt("Fill ids to plot (comma list or 'all')", "all").strip().lower()
    if selection in {"", "all"}:
        return None
    return selection


def parse_fill_selection_batch(sessions: Sequence[SessionInfo]) -> str | None:
    known_counts = sorted({session.fill_count for session in sessions if session.fill_count is not None})
    if len(sessions) == 1:
        return parse_fill_selection(sessions[0].fill_count)

    if known_counts:
        counts_text = ", ".join(str(value) for value in known_counts)
        print(style(f"Known fill counts across selected sessions: {counts_text}", FG_HINT))
    selection = prompt("Fill ids to plot for all selected sessions (comma list or 'all')", "all").strip().lower()
    if selection in {"", "all"}:
        return None
    return selection


def parse_rate_selection() -> str:
    print(style("Available flow-rate traces", ANSI_BOLD, FG_TITLE))
    print(style("  raw, 2sample, 4sample, filtered, medium, slow", FG_HINT))
    print(style("Use a comma list like raw,filtered,medium, or 'all' / 'none'.", FG_HINT))
    while True:
        selection = prompt("Rate traces to plot", "filtered").strip().lower()
        if selection in {"", "none", "off"}:
            return "none"
        if selection == "all":
            return "all"
        tokens = [part.strip() for part in selection.split(",") if part.strip()]
        valid = {"raw", "2sample", "4sample", "filtered", "medium", "slow"}
        invalid = [token for token in tokens if token not in valid]
        if invalid:
            print(style(f"Unknown rate traces: {', '.join(invalid)}", FG_ERROR))
            continue
        ordered = [name for name in ["raw", "2sample", "4sample", "filtered", "medium", "slow"] if name in tokens]
        if ordered:
            return ",".join(ordered)
        print(style("Enter at least one valid rate trace, 'all', or 'none'.", FG_ERROR))


def action_capture(output_root: Path) -> None:
    print_header(
        "Telemetry TUI: Capture",
        "Capture UART logs, split TEL telemetry, and create a new session folder.",
    )
    print_panel(
        "What This Does",
        [
            "Opens the ESP serial port and records the full console to session.log.",
            "Stores machine-readable telemetry in telemetry.ndjson.",
            "Creates per-run NDJSON files live while capture is running.",
        ],
    )
    print()

    port = choose_serial_port()
    baud = prompt_int("Baud rate", 115200, minimum=1)
    session_name = prompt("Session name suffix (optional)", "")
    pre_ms = prompt_int("Pre-run context [ms]", 1000, minimum=0)
    post_ms = prompt_int("Post-run context [ms]", 1000, minimum=0)
    show_tel = prompt_yes_no("Also print raw TEL lines?", False)

    cmd = [
        sys.executable,
        str(CAPTURE_SCRIPT),
        "--port",
        port,
        "--baud",
        str(baud),
        "--output-root",
        str(output_root),
        "--pre-run-ms",
        str(pre_ms),
        "--post-run-ms",
        str(post_ms),
    ]
    if session_name:
        cmd.extend(["--session-name", session_name])
    if show_tel:
        cmd.append("--show-tel")

    run_tool(cmd)


def action_split(output_root: Path) -> None:
    print_header(
        "Telemetry TUI: Split",
        "Convert one or more captured sessions into per-fill NDJSON files for plotting and comparison.",
    )
    print_panel(
        "What This Does",
        [
            "Loads telemetry.ndjson from one or more sessions.",
            "Groups each full jar fill including refill cycles into one run.",
            "Always clears old fills/ output first so no stale split files remain.",
            "Writes fills/index.json and fill_0001.ndjson, fill_0002.ndjson, ...",
        ],
    )
    print()

    sessions = choose_sessions(output_root, purpose="split")
    if not sessions:
        wait_for_enter()
        return

    pre_default = sessions[0].pre_ms if sessions[0].pre_ms is not None else 1000
    post_default = sessions[0].post_ms if sessions[0].post_ms is not None else 1000
    pre_ms = prompt_int("Pre-fill context [ms]", pre_default, minimum=0)
    post_ms = prompt_int("Post-fill context [ms]", post_default, minimum=0)

    jobs: list[tuple[str, Sequence[str]]] = []
    for session in sessions:
        cmd = [
            sys.executable,
            str(SPLIT_SCRIPT),
            str(session.path),
            "--pre-ms",
            str(pre_ms),
            "--post-ms",
            str(post_ms),
        ]
        jobs.append((session.name, cmd))
    run_tool_batch(jobs)


def action_plot(output_root: Path) -> None:
    print_header(
        "Telemetry TUI: Plot",
        "Generate thesis charts from one or more sessions or their already split fill files.",
    )
    print_panel(
        "What This Does",
        [
            "Plots directly from selected sessions so the latest split logic is always used.",
            "The standalone plotting script can still be called manually on fills/index.json.",
            "By default, the previous figure output folder is cleared first.",
            "Exports both plain and debug chart variants.",
            "Supports optional Zustandsband and Füllraten-Darstellung.",
            "Keeps the existing plotting script as the source of truth.",
        ],
    )
    print()

    sessions = choose_sessions(output_root, purpose="plot")
    if not sessions:
        wait_for_enter()
        return

    fill_ids = parse_fill_selection_batch(sessions)
    legend = prompt_choice(
        "Legend placement",
        [("o", "outside below plot"), ("i", "inside plot")],
        "o",
    )
    state_style = prompt_choice(
        "State visualization",
        [("n", "none"), ("b", "compact top band"), ("g", "full background")],
        "b",
    )
    show_rate = parse_rate_selection()
    rate_layout = "s"
    if show_rate != "none":
        print(style("Overlay keeps one compact chart but uses an extra right axis for g/s.", FG_HINT))
        print(style("Subplot keeps units visually cleaner and is usually the better thesis default.", FG_HINT))
        rate_layout = prompt_choice(
            "Rate layout",
            [("s", "separate subplot"), ("o", "overlay in same chart")],
            "s",
        )
    figure_profile = prompt_choice(
        "Figure width",
        [("d", "default 16:10 full-width"), ("n", "narrow taller variant for side-by-side")],
        "d",
    )
    plain_both_profiles = prompt_yes_no("Also export plain chart in both wide+narrow variants?", False)
    format_choice = prompt_choice(
        "Export formats",
        [("1", "pdf"), ("2", "pdf + svg"), ("3", "pdf + svg + png"), ("4", "png only")],
        "4",
    )
    output_dir_raw = prompt("Output dir override (optional)", "")
    output_dir_override: Path | None = None
    if output_dir_raw:
        output_dir_override = Path(output_dir_raw).expanduser()
        if not output_dir_override.is_absolute():
            output_dir_override = (REPO_ROOT / output_dir_override).resolve()
    clear_output = prompt_yes_no("Clear previous generated figures first?", True)

    format_args: list[str] = []
    if format_choice == "1":
        format_args.extend(["--format", "pdf"])
    elif format_choice == "2":
        format_args.extend(["--format", "pdf", "--format", "svg"])
    elif format_choice == "3":
        format_args.extend(["--format", "pdf", "--format", "svg", "--format", "png"])
    elif format_choice == "4":
        format_args.extend(["--format", "png"])

    jobs: list[tuple[str, Sequence[str]]] = []
    for session in sessions:
        cmd = [
            sys.executable,
            str(PLOT_SCRIPT),
            str(session.path),
            "--legend-placement",
            "outside" if legend == "o" else "inside",
            "--state-style",
            {"n": "none", "b": "band", "g": "background"}[state_style],
            "--show-rate",
            show_rate,
            "--rate-layout",
            "overlay" if rate_layout == "o" else "subplot",
            "--figure-profile",
            "narrow" if figure_profile == "n" else "default",
        ]
        if plain_both_profiles:
            cmd.append("--plain-both-profiles")
        if fill_ids:
            cmd.extend(["--fills", fill_ids])
        if output_dir_override:
            session_output_dir = output_dir_override / session.name
            cmd.extend(["--output-dir", str(session_output_dir)])
        if not clear_output:
            cmd.append("--keep-output")
        cmd.extend(format_args)
        jobs.append((session.name, cmd))

    run_tool_batch(jobs)


def action_list_sessions(output_root: Path) -> None:
    print_header(
        "Telemetry TUI: Sessions",
        "Browse recent telemetry sessions and see what is already available.",
    )
    sessions = discover_sessions(output_root)
    if not sessions:
        print(style(f"No telemetry sessions found in {output_root}", FG_WARN))
        print()
        wait_for_enter()
        return

    print(style(f"Session root: {output_root}", FG_HINT))
    print()
    for session in sessions[:20]:
        parts = [style(session.name, FG_ACCENT)]
        if session.captured_at:
            parts.append(f"captured={session.captured_at}")
        if session.port:
            parts.append(f"port={session.port}")
        if session.fill_count is not None:
            parts.append(f"fills={session.fill_count}")
        if session.figure_count:
            parts.append(f"figures={session.figure_count}")
        print("  " + " | ".join(parts))
    print()
    wait_for_enter()


def main() -> int:
    args = build_arg_parser().parse_args()
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    while True:
        print_header(
            "Telemetry TUI",
            "Recommended launcher for capture, split, and thesis chart generation.",
        )
        print_panel(
            "Recommended Workflow",
            [
                "1. Capture a session from the ESP serial console.",
                "2. Split that session into individual fill runs.",
                "3. Plot selected fills into plain and debug chart variants.",
            ],
        )
        print()
        print(style("Main Menu", ANSI_BOLD, FG_TITLE))
        print(f"  {style('1', FG_ACCENT)}. Capture new session")
        print(f"  {style('2', FG_ACCENT)}. Split session into fill runs")
        print(f"  {style('3', FG_ACCENT)}. Plot fill runs")
        print(f"  {style('4', FG_ACCENT)}. List recent sessions")
        print(f"  {style('0', FG_ACCENT)}. Quit")
        print()

        choice = prompt("Select action", "1")
        if choice == "1":
            action_capture(output_root)
        elif choice == "2":
            action_split(output_root)
        elif choice == "3":
            action_plot(output_root)
        elif choice == "4":
            action_list_sessions(output_root)
        elif choice == "0":
            return 0
        else:
            print(style("Unknown menu choice.", FG_ERROR))
            wait_for_enter()


if __name__ == "__main__":
    raise SystemExit(main())
