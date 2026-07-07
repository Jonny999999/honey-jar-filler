#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shlex
import statistics
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "honey-jar-filler-matplotlib"),
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


EXPORT_PAD_INCHES = 0.06
COMMAND_LOG_FILENAME = "manual_sweep_last_command.txt"
POINTS_CSV_FILENAME = "manual_sweep_rate_vs_gate_points.csv"
FIGURE_PROFILES = {
    "default": (8.67, 5.0),
    "narrow": (6.8, 4.65),
}


@dataclass(slots=True)
class SweepWindow:
    sweep_id: int
    run_id: int
    slot_idx: int
    start_us: int
    end_us: int
    base_weight_g: float | None


@dataclass(slots=True)
class SweepPoint:
    sweep_id: int
    run_id: int
    slot_idx: int
    gate_pct: float
    rate_gps: float
    segment_start_us: int
    segment_end_us: int
    fit_start_us: int
    fit_end_us: int
    segment_duration_s: float
    fit_duration_s: float
    sample_count: int


def _load_ndjson(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            records.append(json.loads(line))
    return records


def _resolve_session_dir(path: Path) -> Path:
    candidate = path.expanduser().resolve()
    if candidate.is_file():
        return candidate.parent.parent if candidate.parent.name in {"runs", "fills"} else candidate.parent
    if (candidate / "telemetry.ndjson").exists():
        return candidate
    if candidate.name in {"runs", "fills", "figures"} and (candidate.parent / "telemetry.ndjson").exists():
        return candidate.parent
    raise SystemExit(f"Cannot resolve telemetry session from {path}")


def _save_figure(fig: Any, out_path: Path, fmt: str) -> None:
    fig.canvas.draw()
    fig.savefig(
        out_path,
        dpi=180 if fmt == "png" else None,
        bbox_inches="tight",
        pad_inches=EXPORT_PAD_INCHES,
        facecolor="white",
    )


def _command_text(argv: list[str]) -> str:
    return shlex.join(argv)


def _write_command_log(session_dir: Path, command_text: str) -> Path:
    out_path = session_dir / COMMAND_LOG_FILENAME
    out_path.write_text(command_text + "\n", encoding="utf-8")
    return out_path


def _manual_records(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    filtered: list[dict[str, Any]] = []
    manual_run_ids = {
        int(record["run_id"])
        for record in records
        if record.get("kind") == "fill_start" and str(record.get("strategy_name", "")).strip().lower() == "manual"
    }
    for record in records:
        run_id = record.get("run_id")
        if run_id is None or int(run_id) not in manual_run_ids:
            continue
        filtered.append(record)
    return filtered


def _build_sweeps(records: list[dict[str, Any]]) -> list[SweepWindow]:
    fill_starts = [
        record for record in records
        if record.get("kind") == "fill_start" and str(record.get("strategy_name", "")).strip().lower() == "manual"
    ]
    if not fill_starts:
        return []
    all_ts = [int(record["ts_us"]) for record in records if record.get("ts_us") is not None]
    session_end_us = max(all_ts)
    sweeps: list[SweepWindow] = []
    for index, record in enumerate(fill_starts, start=1):
        next_start_us = int(fill_starts[index]["ts_us"]) if index < len(fill_starts) else session_end_us + 1
        sweeps.append(
            SweepWindow(
                sweep_id=index,
                run_id=int(record["run_id"]),
                slot_idx=int(record.get("slot_idx", index - 1)),
                start_us=int(record["ts_us"]),
                end_us=next_start_us,
                base_weight_g=float(record["base_weight_g"]) if record.get("base_weight_g") is not None else None,
            )
        )
    return sweeps


def _estimate_dead_time_s(
    sweeps: list[SweepWindow],
    records: list[dict[str, Any]],
    *,
    response_threshold_g: float,
    fallback_dead_time_s: float,
) -> float:
    samples = [record for record in records if record.get("kind") == "sample"]
    delays: list[float] = []
    for sweep in sweeps:
        sweep_samples = [sample for sample in samples if sweep.start_us <= int(sample["ts_us"]) < sweep.end_us]
        if not sweep_samples:
            continue
        base_weight_g = sweep.base_weight_g if sweep.base_weight_g is not None else float(sweep_samples[0]["weight_g"])
        first_open = next((sample for sample in sweep_samples if float(sample.get("gate_pct") or 0.0) > 0.0), None)
        if first_open is None:
            continue
        first_open_us = int(first_open["ts_us"])
        response = next(
            (
                sample for sample in sweep_samples
                if int(sample["ts_us"]) >= first_open_us
                and float(sample["weight_g"]) - base_weight_g >= response_threshold_g
            ),
            None,
        )
        if response is None:
            continue
        delay_s = (int(response["ts_us"]) - first_open_us) / 1_000_000.0
        if 0.2 <= delay_s <= 12.0:
            delays.append(delay_s)
    if not delays:
        return fallback_dead_time_s
    return statistics.median(delays)


def _fit_slope(samples: list[dict[str, Any]]) -> float | None:
    if len(samples) < 2:
        return None
    x = np.array([(int(sample["ts_us"]) - int(samples[0]["ts_us"])) / 1_000_000.0 for sample in samples], dtype=float)
    y = np.array([float(sample["weight_g"]) for sample in samples], dtype=float)
    if np.allclose(x, x[0]):
        return None
    slope, _intercept = np.polyfit(x, y, deg=1)
    return float(slope)


def _extract_points(
    sweeps: list[SweepWindow],
    records: list[dict[str, Any]],
    *,
    dead_time_s: float,
    tail_window_s: float,
    min_segment_s: float,
    min_samples: int,
    min_rate_gps: float,
) -> list[SweepPoint]:
    samples = [record for record in records if record.get("kind") == "sample"]
    points: list[SweepPoint] = []
    for sweep in sweeps:
        sweep_samples = [sample for sample in samples if sweep.start_us <= int(sample["ts_us"]) < sweep.end_us]
        if not sweep_samples:
            continue
        first_open_sample = next((sample for sample in sweep_samples if float(sample.get("gate_pct") or 0.0) > 0.0), None)
        if first_open_sample is None:
            continue
        first_open_us = int(first_open_sample["ts_us"])
        first_valid_rate_us = first_open_us + int(dead_time_s * 1_000_000.0)

        segment_start = 0
        while segment_start < len(sweep_samples):
            gate_pct = float(sweep_samples[segment_start].get("gate_pct") or 0.0)
            segment_end = segment_start + 1
            while (
                segment_end < len(sweep_samples)
                and float(sweep_samples[segment_end].get("gate_pct") or 0.0) == gate_pct
            ):
                segment_end += 1

            if gate_pct > 0.0:
                segment_records = sweep_samples[segment_start:segment_end]
                segment_start_us = int(segment_records[0]["ts_us"])
                segment_end_us = int(segment_records[-1]["ts_us"])
                segment_duration_s = (segment_end_us - segment_start_us) / 1_000_000.0
                fit_start_us = max(
                    segment_start_us,
                    segment_end_us - int(tail_window_s * 1_000_000.0),
                    first_valid_rate_us,
                )
                fit_samples = [
                    sample for sample in segment_records
                    if fit_start_us <= int(sample["ts_us"]) <= segment_end_us
                ]
                slope = _fit_slope(fit_samples)
                if (
                    segment_duration_s >= min_segment_s
                    and segment_end_us >= first_valid_rate_us
                    and len(fit_samples) >= min_samples
                    and slope is not None
                    and slope >= min_rate_gps
                ):
                    points.append(
                        SweepPoint(
                            sweep_id=sweep.sweep_id,
                            run_id=sweep.run_id,
                            slot_idx=sweep.slot_idx,
                            gate_pct=gate_pct,
                            rate_gps=slope,
                            segment_start_us=segment_start_us,
                            segment_end_us=segment_end_us,
                            fit_start_us=int(fit_samples[0]["ts_us"]),
                            fit_end_us=segment_end_us,
                            segment_duration_s=segment_duration_s,
                            fit_duration_s=(segment_end_us - int(fit_samples[0]["ts_us"])) / 1_000_000.0,
                            sample_count=len(fit_samples),
                        )
                    )
            segment_start = segment_end
    return points


def _write_points_csv(session_dir: Path, points: list[SweepPoint]) -> Path:
    out_path = session_dir / POINTS_CSV_FILENAME
    with out_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "sweep_id",
                "run_id",
                "slot_idx",
                "gate_pct",
                "rate_gps",
                "segment_start_us",
                "segment_end_us",
                "fit_start_us",
                "fit_end_us",
                "segment_duration_s",
                "fit_duration_s",
                "sample_count",
            ]
        )
        for point in points:
            writer.writerow(
                [
                    point.sweep_id,
                    point.run_id,
                    point.slot_idx,
                    f"{point.gate_pct:.3f}",
                    f"{point.rate_gps:.6f}",
                    point.segment_start_us,
                    point.segment_end_us,
                    point.fit_start_us,
                    point.fit_end_us,
                    f"{point.segment_duration_s:.6f}",
                    f"{point.fit_duration_s:.6f}",
                    point.sample_count,
                ]
            )
    return out_path


def _session_token(session_name: str) -> str:
    token = re.sub(r"[^A-Za-z0-9._-]+", "-", session_name.strip())
    return token.replace(":", "-")


def _figure_size(profile: str) -> tuple[float, float]:
    return FIGURE_PROFILES[profile]


def _plot_curve(
    session_dir: Path,
    points: list[SweepPoint],
    *,
    output_dir: Path,
    formats: list[str],
    dead_time_s: float,
    tail_window_s: float,
    fit_enabled: bool,
    figure_profile: str,
) -> list[Path]:
    def render_variant(*, debug: bool, suffix: str) -> list[Path]:
        figure_width, figure_height = _figure_size(figure_profile)
        fig, ax = plt.subplots(1, 1, figsize=(figure_width, figure_height))
        cmap = plt.get_cmap("tab10")
        sweep_ids = sorted({point.sweep_id for point in points})

        for index, sweep_id in enumerate(sweep_ids):
            sweep_points = [point for point in points if point.sweep_id == sweep_id]
            sweep_points.sort(key=lambda point: point.fit_end_us)
            color = cmap(index % 10)
            x = [point.gate_pct for point in sweep_points]
            y = [point.rate_gps for point in sweep_points]
            ax.plot(x, y, color=color, linewidth=1.0, alpha=0.35, zorder=2)
            ax.scatter(x, y, color=color, s=36, alpha=0.92, label=f"Sweep {sweep_id}", zorder=3)

        unique_gate_count = len({round(point.gate_pct, 6) for point in points})
        if fit_enabled and unique_gate_count >= 2 and len(points) >= 2:
            fit_x = np.array([point.gate_pct for point in points], dtype=float)
            fit_y = np.array([point.rate_gps for point in points], dtype=float)
            coeffs = np.polyfit(fit_x, fit_y, deg=1)
            poly = np.poly1d(coeffs)
            fit_curve_x = np.linspace(float(np.min(fit_x)), float(np.max(fit_x)), 400)
            fit_curve_y = poly(fit_curve_x)
            slope = float(coeffs[0])
            intercept = float(coeffs[1])
            ax.plot(
                fit_curve_x,
                fit_curve_y,
                color="#111827",
                linewidth=1.7,
                linestyle=(0, (5, 2)),
                label="Linearer Fit",
                zorder=4,
            )
            equation = f"v = {slope:.3f} (g/s)/% · alpha {intercept:+.2f} g/s"
            ax.text(
                0.985,
                0.04,
                equation,
                transform=ax.transAxes,
                ha="right",
                va="bottom",
                fontsize=8.8,
                color="#0f172a",
                bbox={"boxstyle": "round,pad=0.22", "facecolor": "white", "edgecolor": "#cbd5e1", "alpha": 0.92},
                zorder=5,
            )

        if debug:
            fig.text(
                0.08,
                0.965,
                (
                    f"Manueller Sweep | Sitzung: {session_dir.name} | Punkte: {len(points)} | "
                    f"Totzeit: {dead_time_s:.2f} s | Auswertefenster: {tail_window_s:.2f} s"
                ),
                ha="left",
                va="top",
                fontsize=9.2,
                color="#475569",
            )
        ax.set_xlabel("Klappenöffnung [%]")
        ax.set_ylabel("Manuell berechnete Füllrate [g/s]")
        ax.grid(True, axis="both", color="#cbd5e1", linewidth=0.7, alpha=0.65)
        ax.set_axisbelow(True)
        ax.set_xlim(left=max(0.0, min(point.gate_pct for point in points) - 2.0), right=min(102.0, max(point.gate_pct for point in points) + 2.0))
        ax.set_ylim(bottom=0.0)
        ax.legend(frameon=False, loc="upper left", ncol=min(4, max(1, len(sweep_ids) + (1 if fit_enabled else 0))))
        fig.tight_layout(rect=(0.035, 0.035, 0.985, (0.92 if debug else 0.985)), pad=0.35, w_pad=0.55, h_pad=0.5)

        output_dir.mkdir(parents=True, exist_ok=True)
        stem = output_dir / f"{_session_token(session_dir.name)}__manual-sweep-rate-vs-gate__{suffix}"
        exported_variant: list[Path] = []
        for fmt in formats:
            out_path = stem.parent / f"{stem.name}.{fmt}"
            _save_figure(fig, out_path, fmt)
            exported_variant.append(out_path)
        plt.close(fig)
        return exported_variant

    exported: list[Path] = []
    exported.extend(render_variant(debug=False, suffix="plain"))
    exported.extend(render_variant(debug=True, suffix="debug"))
    return exported


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Special analysis for manual sweep telemetry.\n"
            "It groups manual fill sweeps, estimates a dead time from the first visible\n"
            "mass response, derives a late steady-state rate for each gate plateau from\n"
            "a linear weight-vs-time fit, and plots fill rate over gate opening."
        ),
        epilog=(
            "Example:\n"
            "  python3 tools/telemetry/plot_manual_sweep_curve.py "
            "data/telemetry/2026.07.05_21:10-x-dunkler-honig-manual-sweep "
            "--format pdf --format svg --format png"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("input_path", type=Path, help="Session folder, runs/ folder, or telemetry.ndjson path")
    parser.add_argument("--output-dir", type=Path, help="Figure output folder (default: <session>/figures)")
    parser.add_argument("--format", action="append", choices=["pdf", "svg", "png"], help="Export format, repeatable. Default: pdf")
    parser.add_argument("--dead-time-s", type=float, help="Override dead time in seconds. Default: auto-estimate from sweep start response")
    parser.add_argument("--fallback-dead-time-s", type=float, default=4.0, help="Fallback dead time if auto-estimation fails")
    parser.add_argument("--response-threshold-g", type=float, default=0.8, help="Mass increase above base weight used for dead-time auto-estimation")
    parser.add_argument("--tail-window-s", type=float, default=0.8, help="Regression window length at the end of each gate plateau")
    parser.add_argument("--min-segment-s", type=float, default=0.25, help="Minimum plateau duration to consider")
    parser.add_argument("--min-samples", type=int, default=3, help="Minimum samples inside the regression window")
    parser.add_argument("--min-rate-gps", type=float, default=0.5, help="Drop plateau points below this derived rate")
    parser.add_argument("--figure-profile", choices=sorted(FIGURE_PROFILES.keys()), default="default", help="Use the normal 16:10 thesis size or the narrower variant")
    parser.add_argument("--no-fit", action="store_true", help="Disable the default linear fit overlay")
    return parser


def main() -> int:
    parser = build_arg_parser()
    if len(sys.argv) == 1:
        parser.print_help()
        return 0
    args = parser.parse_args()
    if args.tail_window_s <= 0:
        raise SystemExit("--tail-window-s must be > 0")
    if args.min_segment_s <= 0:
        raise SystemExit("--min-segment-s must be > 0")
    if args.min_samples < 2:
        raise SystemExit("--min-samples must be >= 2")
    if args.fallback_dead_time_s <= 0:
        raise SystemExit("--fallback-dead-time-s must be > 0")
    if args.response_threshold_g <= 0:
        raise SystemExit("--response-threshold-g must be > 0")

    session_dir = _resolve_session_dir(args.input_path)
    telemetry_path = session_dir / "telemetry.ndjson"
    if not telemetry_path.exists():
        raise SystemExit(f"No telemetry.ndjson found in {session_dir}")
    records = _manual_records(_load_ndjson(telemetry_path))
    sweeps = _build_sweeps(records)
    if not sweeps:
        raise SystemExit("No manual fill_start sweeps found in this session.")

    dead_time_s = (
        args.dead_time_s
        if args.dead_time_s is not None
        else _estimate_dead_time_s(
            sweeps,
            records,
            response_threshold_g=args.response_threshold_g,
            fallback_dead_time_s=args.fallback_dead_time_s,
        )
    )
    points = _extract_points(
        sweeps,
        records,
        dead_time_s=dead_time_s,
        tail_window_s=args.tail_window_s,
        min_segment_s=args.min_segment_s,
        min_samples=args.min_samples,
        min_rate_gps=args.min_rate_gps,
    )
    if not points:
        raise SystemExit("No usable sweep plateau points found. Try a smaller dead time or shorter minimum segment.")

    formats = args.format or ["pdf"]
    output_dir = args.output_dir.resolve() if args.output_dir else session_dir / "figures"
    command_text = _command_text([sys.executable, *sys.argv]) if sys.argv else _command_text([sys.executable])
    command_log_path = _write_command_log(session_dir, command_text)
    points_csv_path = _write_points_csv(session_dir, points)
    exported = _plot_curve(
        session_dir,
        points,
        output_dir=output_dir,
        formats=formats,
        dead_time_s=dead_time_s,
        tail_window_s=args.tail_window_s,
        fit_enabled=not args.no_fit,
        figure_profile=args.figure_profile,
    )

    print(f"Session:      {session_dir}")
    print(f"Sweeps:       {len(sweeps)}")
    print(f"Usable pts:   {len(points)}")
    print(f"Dead time:    {dead_time_s:.3f} s")
    print(f"Tail window:  {args.tail_window_s:.3f} s")
    print(f"Profile:      {args.figure_profile}")
    print(f"Linear fit:   {'off' if args.no_fit else 'on'}")
    print(f"CSV:          {points_csv_path}")
    print(f"Command:      {command_log_path}")
    for path in exported:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
