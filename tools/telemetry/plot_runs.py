#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import re
import shlex
import shutil
import sys
import tempfile
import textwrap
from pathlib import Path
from typing import Any

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "honey-jar-filler-matplotlib"),
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from analysis import (
    FillRun,
    build_fill_runs,
    extract_fill_records,
    format_fill_brief,
    important_params,
    load_fill_manifest,
    load_ndjson,
    load_session_meta,
    resolve_session_dir,
    telemetry_path_for_session,
)

STATE_COLORS = {
    "FIND_SLOT": "#e2e8f0",
    "SLOT_SETTLE": "#fef3c7",
    "VERIFY_EMPTY": "#fde68a",
    "FILL": "#dbeafe",
    "DRIP_WAIT": "#ffedd5",
    "VERIFY_TARGET": "#dcfce7",
    "DONE": "#e5e7eb",
    "IDLE": "#f8fafc",
    "FAULT": "#fecaca",
}
STATE_LABELS_DE = {
    "IDLE": "Bereit",
    "FIND_SLOT": "Bereit",
    "SLOT_SETTLE": "Beruhigen",
    "VERIFY_EMPTY": "Leerprüfung",
    "FILL": "Füllen",
    "DRIP_WAIT": "Nachtropfen",
    "VERIFY_TARGET": "Zielprüfung",
    "DONE": "Fertig",
    "FAULT": "Störung",
}
GATE_LABELS_DE = {
    "max_gate": "Voll öffnen",
    "near_close": "Teilschließen",
    "close_early": "Früh schließen",
    "target": "Sollwert erreicht",
    "drip_wait": "Geschlossen",
    "open": "Öffnen",
    "close": "Schließen",
    "percent": "Öffnungsgrad",
}
FIGURE_PROFILES = {
    "default": (8.67, 5.0),
    "narrow": (6.8, 4.65),
}
SESSION_SUMMARY_GROUP_ORDER = ["overview", "adaptive", "compare", "cascade"]
PLAIN_PROFILE_SUFFIX = {
    "default": "wide",
    "narrow": "narrow",
}
DEBUG_META_WIDTH = 8.6
LINE_FILL_MASS = "#c81e1e"
LINE_GATE = "#7e22ce"
LINE_TARGET = "#1f5f3a"
LINE_FAULT = "#b91c1c"
RATE_SERIES_SPECS: dict[str, dict[str, str]] = {
    "raw": {"field": "rate_raw_gps", "label": "Rohrate [g/s]", "color": "#94a3b8"},
    "2sample": {"field": "rate_2sample_gps", "label": "2-Sample-Rate [g/s]", "color": "#0f766e"},
    "4sample": {"field": "rate_4sample_gps", "label": "4-Sample-Rate [g/s]", "color": "#0ea5e9"},
    "filtered": {"field": "rate_filtered_gps", "label": "Gefilterte Füllrate [g/s]", "color": "#2563eb"},
    "medium": {"field": "rate_filtered_medium_gps", "label": "Mittlere Filterrate [g/s]", "color": "#7c3aed"},
    "slow": {"field": "rate_filtered_slow_gps", "label": "Langsame Filterrate [g/s]", "color": "#ea580c"},
}
RATE_SELECTION_ORDER = ["raw", "2sample", "4sample", "filtered", "medium", "slow"]
LINE_RATE_FILTERED = RATE_SERIES_SPECS["filtered"]["color"]
LINE_TARGET_RATE = "#15803d"
# Cascade / Smith-predictor model signals (flow-cascade only).
LINE_MODEL_RATE = "#9333ea"
STATE_BAND_EDGE = "#94a3b8"
STATE_BAND_KEYS = {"FILL", "DRIP_WAIT", "VERIFY_TARGET", "FAULT"}
RATE_AXIS_LABEL = "Füllrate [g/s]"
STATE_BAND_FIGURE_EXTRA_H = 0.52
STATE_BAND_HEIGHT_RATIO = 0.66
MAIN_PLOT_HEIGHT_RATIO = 3.8
RATE_PLOT_HEIGHT_RATIO = 1.7
RATE_PLOT_HEIGHT_RATIO_DENSE = 2.8
RATE_PLOT_HEIGHT_RATIO_CASCADE = MAIN_PLOT_HEIGHT_RATIO
RATE_FIGURE_EXTRA_H_DENSE = 0.55
RATE_FIGURE_EXTRA_H_CASCADE = 1.2
CONTROL_PLOT_HEIGHT_RATIO = 1.5
CONTROL_FIGURE_EXTRA_H = 1.05
DEBUG_FIGURE_EXTRA_H = 1.1
EXPORT_PAD_INCHES = 0.06
COMMAND_LOG_FILENAME = "plot_runs_last_command.txt"
# Cascade control-loop panel signals.
LINE_RATE_ERROR = "#e11d48"
LINE_CONTROL_INTEG = "#0891b2"
CONTROL_AXIS_LABEL = "Regelabw. [g/s]"
STATE_BAND_LABEL_MIN_WIDTH_S = 0.35
STATE_BAND_LABEL_OUTSIDE_WIDTH_S = 1.45
SESSION_LINE_COLORS = {
    "fill_error_g": "#c81e1e",
    "fill_duration_s": "#2563eb",
    "refill_count": "#7c3aed",
    "used_near_close_g": "#7e22ce",
    "used_close_early_g": "#db2777",
    "drip_wait_used_ms": "#ea580c",
    "measured_post_close_gain_g": "#dc2626",
    "next_post_close_gain_g": "#16a34a",
    "measured_dead_time_s": "#0f766e",
    "next_dead_time_s": "#1d4ed8",
    "measured_fast_rate_gps": "#2563eb",
    "next_fast_rate_gps": "#1e40af",
    "measured_slow_rate_gps": "#14b8a6",
    "next_slow_rate_gps": "#0f766e",
    "used_gate_gain_gps_per_pct": "#b45309",
    "next_gate_gain_gps_per_pct": "#f59e0b",
    "used_onset_gate_pct": "#6d28d9",
    "next_onset_gate_pct": "#a78bfa",
    "used_model_tau_s": "#0e7490",
    "next_model_tau_s": "#22d3ee",
}

# Cascade plant-model parameters for the per-fill (cross-run) learning chart.
# "used_" is the value the fill actually ran with, "next_" the value learned by
# the end of that fill -- the gap between them is the update that fill applied,
# and the "next_" line across fills is the learning trend.
CASCADE_SUMMARY_METRICS = [
    ("used_gate_gain_gps_per_pct", "next_gate_gain_gps_per_pct",
     "K [g/s pro %]", "Streckenverstärkung K̂ (lokal, EWMA)"),
    ("used_onset_gate_pct", "next_onset_gate_pct",
     "Totwinkel [%]", "Fluss-Einsatzpunkt (onset)"),
    ("used_model_tau_s", "next_model_tau_s",
     "tau [s]", "Zeitkonstante τ (FOPDT)"),
    ("used_dead_time_s", "next_dead_time_s",
     "Totzeit L [s]", "Totzeit L"),
    ("used_post_close_gain_g", "next_post_close_gain_g",
     "Nachlauf [g]", "Nachlaufmasse (post-close)"),
]


# Optional seam for drawing an extra layer on a finished fill figure, called
# just before the figure is saved with (fig, ax, rate_ax, fill_run, records).
# annotate_ch07.py installs the thesis ch. 07 annotation layer here; nothing in
# the normal plotting path sets it, so plot_runs.py on its own is unaffected.
ANNOTATION_HOOK: Any = None


def _parse_fill_selection(value: str) -> list[int]:
    selected: list[int] = []
    for part in value.split(","):
        item = part.strip()
        if not item:
            continue
        selected.append(int(item))
    return selected


def _session_defaults(session_dir: Path) -> tuple[int, int]:
    meta = load_session_meta(session_dir)
    return int(meta.get("pre_run_ms", 1000)), int(meta.get("post_run_ms", 1000))


def _load_input(
    input_path: Path,
    pre_ms: int | None,
    post_ms: int | None,
) -> tuple[Path, list[FillRun], dict[int, list[dict[str, Any]]]]:
    candidate = input_path.resolve()

    if candidate.is_dir() and (candidate / "index.json").exists():
        fills_dir = candidate
        session_dir = fills_dir.parent
        _, fill_runs = load_fill_manifest(fills_dir)
        fill_records = {
            fill_run.fill_id: load_ndjson(fills_dir / f"fill_{fill_run.fill_id:04d}.ndjson")
            for fill_run in fill_runs
        }
        return session_dir, fill_runs, fill_records

    session_dir = resolve_session_dir(candidate)
    telemetry_path = telemetry_path_for_session(session_dir)
    records = load_ndjson(telemetry_path)
    default_pre_ms, default_post_ms = _session_defaults(session_dir)
    fill_runs = build_fill_runs(
        records,
        pre_ms=pre_ms if pre_ms is not None else default_pre_ms,
        post_ms=post_ms if post_ms is not None else default_post_ms,
    )
    fill_records = {
        fill_run.fill_id: extract_fill_records(records, fill_run) for fill_run in fill_runs
    }
    return session_dir, fill_runs, fill_records


def _state_events(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [record for record in records if str(record.get("kind", "")) == "state"]


def _sample_events(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [record for record in records if str(record.get("kind", "")) == "sample"]


def _gate_events(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [record for record in records if str(record.get("kind", "")) == "gate"]


def _fault_events(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [record for record in records if str(record.get("kind", "")) == "fault"]


def _ts_us(record: dict[str, Any]) -> int:
    return int(record["ts_us"])


def _seconds_from_focus(fill_run: FillRun, record: dict[str, Any]) -> float:
    return (_ts_us(record) - fill_run.focus_start_us) / 1_000_000.0


def _float_value(record: dict[str, Any], key: str) -> float | None:
    try:
        value = record.get(key)
        return None if value is None else float(value)
    except (TypeError, ValueError):
        return None


def _format_param_value(label: str, value: Any) -> str:
    if isinstance(value, float):
        if label in {"Max gate", "Near gate"}:
            return f"{value:.0f} %"
        if label == "Drip wait":
            return f"{value:.0f} ms"
        return f"{value:.1f} g"
    if isinstance(value, int):
        if label in {"Max gate", "Near gate"}:
            return f"{value} %"
        if label == "Drip wait":
            return f"{value} ms"
        return f"{value} g"
    return str(value)


def _format_meta_value(value: Any, unit: str | None = None, decimals: int = 2) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        text = f"{value:.{decimals}f}"
    else:
        text = str(value)
    return f"{text} {unit}" if unit else text


def _format_series_value(value: float | None, unit: str, decimals: int = 2) -> str:
    if value is None:
        return "-"
    return f"{value:.{decimals}f} {unit}"


def _figure_size(profile: str) -> tuple[float, float]:
    return FIGURE_PROFILES.get(profile, FIGURE_PROFILES["default"])


def _plain_profile_suffix(profile: str) -> str:
    return PLAIN_PROFILE_SUFFIX.get(profile, profile)


def _slug_token(value: str | None, *, fallback: str) -> str:
    if not value:
        return fallback
    collapsed = re.sub(r"[^A-Za-z0-9._-]+", "-", value.strip())
    collapsed = re.sub(r"-{2,}", "-", collapsed).strip("-._")
    return collapsed or fallback


def _session_output_token(session_name: str) -> str:
    token = _slug_token(session_name, fallback="session")
    token = re.sub(
        r"^(\d{4}\.\d{2}\.\d{2}_\d{2}-\d{2})-x-",
        r"\1-",
        token,
        count=1,
    )
    return token


def _output_stem_base(session_dir: Path, fill_run: FillRun) -> str:
    session_token = _session_output_token(session_dir.name)
    strategy_token = _slug_token(fill_run.strategy_name, fallback="strategy")
    duration_token = f"{round(fill_run.focus_duration_s):d}s"
    return (
        f"{session_token}"
        f"__{strategy_token}"
        f"__fill-{fill_run.fill_id:02d}"
        f"__total-{duration_token}"
    )


def _session_output_stem_base(session_dir: Path) -> str:
    return _session_output_token(session_dir.name)


def _figure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Serif",
            "font.size": 10,
            "axes.labelsize": 11,
            "legend.fontsize": 9,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
        }
    )


def _parse_show_rate(value: str) -> str:
    raw = value.strip().lower()
    if raw in {"none", "off", "n"}:
        return "none"
    if raw in {"all", "a"}:
        return "all"
    aliases = {
        "both": "raw,filtered",
        "f": "filtered",
        "r": "raw",
        "m": "medium",
        "s": "slow",
    }
    raw = aliases.get(raw, raw)
    tokens = [part.strip().lower() for part in raw.split(",") if part.strip()]
    if not tokens:
        return "none"
    invalid = [token for token in tokens if token not in RATE_SERIES_SPECS]
    if invalid:
        raise argparse.ArgumentTypeError(
            f"Unknown rate series: {', '.join(invalid)}. "
            f"Use one of: {', '.join(RATE_SELECTION_ORDER)}, all, none."
        )
    ordered = [name for name in RATE_SELECTION_ORDER if name in tokens]
    return ",".join(ordered) if ordered else "none"


def _parse_session_summary_groups(value: str) -> list[str]:
    raw = value.strip().lower()
    if raw in {"all", "*"}:
        return SESSION_SUMMARY_GROUP_ORDER.copy()
    tokens = [part.strip().lower() for part in raw.split(",") if part.strip()]
    if not tokens:
        raise argparse.ArgumentTypeError(
            f"Use one or more of: {', '.join(SESSION_SUMMARY_GROUP_ORDER)}, all."
        )
    invalid = [token for token in tokens if token not in SESSION_SUMMARY_GROUP_ORDER]
    if invalid:
        raise argparse.ArgumentTypeError(
            f"Unknown session summary groups: {', '.join(invalid)}. "
            f"Use one or more of: {', '.join(SESSION_SUMMARY_GROUP_ORDER)}, all."
        )
    return [name for name in SESSION_SUMMARY_GROUP_ORDER if name in tokens]


def _selected_rate_series(show_rate: str) -> list[str]:
    if show_rate == "none":
        return []
    if show_rate == "all":
        return RATE_SELECTION_ORDER.copy()
    selected = [part.strip() for part in show_rate.split(",") if part.strip()]
    return [name for name in RATE_SELECTION_ORDER if name in selected]


def _gate_label_de(record: dict[str, Any]) -> str:
    text = str(record.get("text", "")).strip()
    return GATE_LABELS_DE.get(text, text or "Ereignis")


def _state_label_de(record: dict[str, Any]) -> str:
    text = str(record.get("text", "")).strip()
    return STATE_LABELS_DE.get(text, text or "Zustand")


def _format_state_band_label(state_name: str, width_s: float) -> str | None:
    label = STATE_LABELS_DE.get(state_name)
    if not label:
        return None
    if width_s < STATE_BAND_LABEL_MIN_WIDTH_S:
        return None
    if width_s >= 1.15:
        duration = f"{width_s:.2f}".replace(".", ",")
        return f"{label}\n({duration} s)"
    return label


def _state_band_label_should_be_outside(label: str, width_s: float) -> bool:
    if width_s < STATE_BAND_LABEL_OUTSIDE_WIDTH_S:
        return True
    longest_line = max(len(part) for part in label.splitlines())
    estimated_needed_width_s = longest_line * 0.19
    return width_s < estimated_needed_width_s


def _place_state_band_label(
    band_ax: Any,
    *,
    state_name: str,
    x0: float,
    x1: float,
    band_y: float,
    band_h: float,
) -> bool:
    width = max(0.0, x1 - x0)
    label = _format_state_band_label(state_name, width)
    if label is None:
        return state_name in {"FILL", "IDLE"}

    text_y = band_y + band_h / 2.0
    va = "center"
    clip_on = True
    if state_name in {"FILL", "IDLE"} and _state_band_label_should_be_outside(label, width):
        return True

    band_ax.text(
        x0 + width / 2.0,
        text_y,
        label,
        transform=band_ax.get_xaxis_transform(),
        ha="center",
        va=va,
        fontsize=7.0,
        color="#1e293b",
        zorder=2,
        clip_on=clip_on,
        linespacing=0.9,
    )
    return False


def _series_from_samples(
    fill_run: FillRun,
    samples: list[dict[str, Any]],
) -> tuple[
    list[float],
    list[float | None],
    list[float | None],
    dict[str, list[float | None]],
    list[float | None],
    dict[str, list[float | None]],
    float | None,
]:
    x_samples = [_seconds_from_focus(fill_run, record) for record in samples]
    absolute_weights = [_float_value(record, "weight_g") for record in samples]
    base_weight_g = fill_run.base_weight_g
    if base_weight_g is None:
        for weight in absolute_weights:
            if weight is not None:
                base_weight_g = weight
                break
    fill_mass = [
        None
        if weight is None or base_weight_g is None
        else (0.0 if x_value < 0.0 else weight - base_weight_g)
        for x_value, weight in zip(x_samples, absolute_weights, strict=True)
    ]
    gate_pct = [_float_value(record, "gate_pct") for record in samples]
    rate_series = {
        name: [_float_value(record, spec["field"]) for record in samples]
        for name, spec in RATE_SERIES_SPECS.items()
    }
    strategy = (fill_run.strategy_name or "").strip().lower()
    target_rate_series = [_float_value(record, "target_rate_gps") for record in samples]
    if strategy in ("flow-control", "flow-cascade"):
        target_rate_series = [
            0.0 if gate_value is not None and gate_value <= 0.5 else target_rate
            for gate_value, target_rate in zip(gate_pct, target_rate_series, strict=True)
        ]
    # Cascade Smith-predictor / control-loop signals (flow-cascade telemetry).
    control_series = {
        "model_rate": [_float_value(record, "model_rate_gps") for record in samples],
        "model_delayed": [_float_value(record, "model_delayed_gps") for record in samples],
        "rate_error": [_float_value(record, "rate_error_gps") for record in samples],
        "control_integ": [_float_value(record, "control_integ_pct") for record in samples],
        "gate_gain": [_float_value(record, "gate_gain_gps_per_pct") for record in samples],
    }
    return x_samples, fill_mass, gate_pct, rate_series, target_rate_series, control_series, base_weight_g


def _plot_state_background(
    ax: Any,
    fill_run: FillRun,
    states: list[dict[str, Any]],
    *,
    show_labels: bool,
) -> None:
    span_end_us = fill_run.window_end_us
    for index, state_record in enumerate(states):
        state_name = str(state_record.get("text", "")).strip() or "STATE"
        start_us = _ts_us(state_record)
        end_us = span_end_us
        if index + 1 < len(states):
            end_us = _ts_us(states[index + 1])
        x0 = (start_us - fill_run.focus_start_us) / 1_000_000.0
        x1 = (end_us - fill_run.focus_start_us) / 1_000_000.0
        ax.axvspan(
            x0,
            x1,
            color=STATE_COLORS.get(state_name, "#e2e8f0"),
            alpha=0.22,
            linewidth=0,
            zorder=0,
        )
        if not show_labels or state_name not in STATE_LABELS_DE or (x1 - x0) < 0.30:
            continue
        ax.annotate(
            _state_label_de(state_record),
            xy=((x0 + x1) / 2.0, 1.0),
            xycoords=("data", "axes fraction"),
            xytext=(0, -8),
            textcoords="offset points",
            ha="center",
            va="top",
            fontsize=8,
            color="#334155",
        )


def _plot_state_band(
    band_ax: Any,
    fill_run: FillRun,
    states: list[dict[str, Any]],
) -> None:
    if band_ax is None:
        return

    band_ax.set_ylim(0.0, 1.0)
    band_ax.axis("off")

    if not states:
        return

    span_end_us = fill_run.window_end_us
    band_y = 0.08
    band_h = 0.84
    segments: list[tuple[str, float, float]] = []

    if fill_run.window_start_us < fill_run.focus_start_us:
        segments.append(
            (
                "IDLE",
                (fill_run.window_start_us - fill_run.focus_start_us) / 1_000_000.0,
                0.0,
            )
        )

    for index, state_record in enumerate(states):
        state_name = str(state_record.get("text", "")).strip() or "STATE"
        start_us = _ts_us(state_record)
        end_us = span_end_us
        if index + 1 < len(states):
            end_us = _ts_us(states[index + 1])

        if start_us < fill_run.focus_start_us:
            continue
        if state_name not in STATE_BAND_KEYS:
            continue

        x0 = (start_us - fill_run.focus_start_us) / 1_000_000.0
        x1 = (end_us - fill_run.focus_start_us) / 1_000_000.0
        segments.append((state_name, x0, x1))

    outside_labels: list[tuple[str, float, float]] = []
    for state_name, x0, x1 in segments:
        width = max(0.0, x1 - x0)
        if width <= 0.0:
            continue

        rect = Rectangle(
            (x0, band_y),
            width,
            band_h,
            transform=band_ax.get_xaxis_transform(),
            facecolor=STATE_COLORS.get(state_name, "#e2e8f0"),
            edgecolor=STATE_BAND_EDGE,
            linewidth=0.6,
            alpha=0.95,
            zorder=1,
            clip_on=False,
        )
        band_ax.add_patch(rect)

        needs_outside = _place_state_band_label(
            band_ax,
            state_name=state_name,
            x0=x0,
            x1=x1,
            band_y=band_y,
            band_h=band_h,
        )
        if needs_outside:
            outside_labels.append((state_name, x0, x1))

    for state_name, x0, x1 in outside_labels:
        text_y = band_y + band_h + 0.06
        band_ax.text(
            x0 + max(0.0, x1 - x0) / 2.0,
            text_y,
            STATE_LABELS_DE.get(state_name, state_name),
            transform=band_ax.get_xaxis_transform(),
            ha="center",
            va="bottom",
            fontsize=7.0,
            color="#1e293b",
            zorder=3,
            clip_on=False,
            bbox={
                "boxstyle": "round,pad=0.12",
                "fc": "#ffffff",
                "ec": "#cbd5e1",
                "alpha": 0.92,
            },
        )


def _annotate_state_badges(ax: Any, fill_run: FillRun, states: list[dict[str, Any]]) -> None:
    for state_record in states:
        state_name = str(state_record.get("text", "")).strip()
        if state_name not in {"FILL", "DRIP_WAIT", "VERIFY_TARGET"}:
            continue
        x_pos = _seconds_from_focus(fill_run, state_record)
        ax.annotate(
            _state_label_de(state_record),
            xy=(x_pos, 1.015),
            xycoords=("data", "axes fraction"),
            ha="center",
            va="bottom",
            fontsize=7.5,
            color="#0f172a",
            bbox={"boxstyle": "round,pad=0.2", "fc": "#ffffff", "ec": "#cbd5e1", "alpha": 0.9},
            annotation_clip=False,
        )


def _annotate_gate_events(
    ax2: Any,
    fill_run: FillRun,
    gates: list[dict[str, Any]],
    samples: list[dict[str, Any]],
) -> None:
    strategy_name = (fill_run.strategy_name or "").strip().lower()
    flow_control_mode = strategy_name == "flow-control"
    first_open_index: int | None = None
    last_zero_index: int | None = None
    if flow_control_mode:
        for index, gate_record in enumerate(gates):
            gate_pct = _float_value(gate_record, "gate_pct")
            if gate_pct is None:
                continue
            if first_open_index is None and gate_pct > 5.0:
                first_open_index = index
            if gate_pct <= 5.0:
                last_zero_index = index

    seen_zero_labels = 0
    for index, gate_record in enumerate(gates):
        gate_pct = _float_value(gate_record, "gate_pct")
        if gate_pct is None:
            continue
        x_gate = _aligned_gate_x(fill_run, gate_record, samples, gate_pct)
        ax2.scatter([x_gate], [gate_pct], color=LINE_GATE, s=20, zorder=6)
        if flow_control_mode:
            if index not in {first_open_index, last_zero_index}:
                continue

        label = f"{gate_pct:.0f} %"
        xytext = (-8, 0)
        va = "center"
        ha = "right"
        if gate_pct <= 5.0:
            if seen_zero_labels == 0:
                xytext = (-8, 2)
                va = "bottom"
            else:
                xytext = (-8, 6)
                va = "bottom"
            seen_zero_labels += 1
        elif gate_pct >= 95.0:
            xytext = (0, 3)
            ha = "center"
            va = "bottom"
        ax2.annotate(
            label,
            (x_gate, gate_pct),
            textcoords="offset points",
            xytext=xytext,
            ha=ha,
            va=va,
            fontsize=7.5,
            color=LINE_GATE,
        )


def _aligned_gate_x(
    fill_run: FillRun,
    gate_record: dict[str, Any],
    samples: list[dict[str, Any]],
    gate_pct: float,
) -> float:
    gate_ts = _ts_us(gate_record)
    for sample in samples:
        sample_ts = _ts_us(sample)
        sample_gate = _float_value(sample, "gate_pct")
        if sample_ts < gate_ts:
            continue
        if sample_gate is None:
            continue
        if abs(sample_gate - gate_pct) < 0.001:
            return _seconds_from_focus(fill_run, sample)
    return _seconds_from_focus(fill_run, gate_record)


def _annotate_faults(ax: Any, fill_run: FillRun, faults: list[dict[str, Any]]) -> None:
    for fault_record in faults:
        x_fault = _seconds_from_focus(fill_run, fault_record)
        label = str(fault_record.get("text", "")).strip() or "FAULT"
        ax.axvline(
            x_fault,
            color=LINE_FAULT,
            linewidth=1.2,
            linestyle=(0, (2, 2)),
            zorder=6,
        )
        ax.annotate(
            label,
            (x_fault, 0.04),
            xycoords=("data", "axes fraction"),
            xytext=(4, 0),
            textcoords="offset points",
            rotation=90,
            va="bottom",
            fontsize=8,
            color=LINE_FAULT,
        )


def _matching_fill_record(fill_run: FillRun, record: dict[str, Any]) -> bool:
    return (
        int(record.get("run_id", 0)) == fill_run.run_id
        and int(record.get("slot_idx", -1)) == fill_run.slot_idx
    )


def _fill_summary_record(
    records: list[dict[str, Any]],
    fill_run: FillRun,
) -> dict[str, Any] | None:
    matches = [
        record
        for record in records
        if str(record.get("kind", "")) == "fill_summary" and _matching_fill_record(fill_run, record)
    ]
    return matches[-1] if matches else None


def _session_summary_entries(
    fill_runs: list[FillRun],
    fill_records: dict[int, list[dict[str, Any]]],
) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for fill_run in sorted(fill_runs, key=lambda item: item.fill_id):
        records = fill_records.get(fill_run.fill_id, [])
        summary = _fill_summary_record(records, fill_run)
        if summary is None:
            continue
        entries.append(
            {
                "fill_run": fill_run,
                "summary": summary,
                "fill_id": fill_run.fill_id,
                "run_id": fill_run.run_id,
                "slot_idx": fill_run.slot_idx,
                "preset_name": fill_run.preset_name,
                "strategy_name": fill_run.strategy_name,
            }
        )
    return entries


def _summary_series(
    entries: list[dict[str, Any]],
    field: str,
) -> tuple[list[int], list[float]]:
    x_values: list[int] = []
    y_values: list[float] = []
    for entry in entries:
        value = _float_value(entry["summary"], field)
        if value is None:
            continue
        x_values.append(int(entry["fill_id"]))
        y_values.append(value)
    return x_values, y_values


def _unique_or_mixed(values: list[str | None], fallback: str) -> str:
    filtered = [value for value in values if value]
    if not filtered:
        return fallback
    unique = list(dict.fromkeys(filtered))
    if len(unique) == 1:
        return unique[0]
    if len(unique) == 2:
        return f"{unique[0]}, {unique[1]}"
    return f"{unique[0]}, {unique[1]}, ..."


def _session_summary_subtitle(session_dir: Path, entries: list[dict[str, Any]]) -> str:
    presets = [entry.get("preset_name") for entry in entries]
    strategies = [entry.get("strategy_name") for entry in entries]
    subtitle = (
        f"Sitzung: {session_dir.name} | "
        f"Preset: {_unique_or_mixed(presets, '-')} | "
        f"Strategie: {_unique_or_mixed(strategies, '-')}"
    )
    return textwrap.fill(subtitle, width=110)


def _setup_session_axes(
    axes: Any,
    x_values: list[int],
    *,
    xlabel: str = "Fülllauf / Füll-ID [-]",
) -> None:
    if isinstance(axes, list):
        axes_list = axes
    elif hasattr(axes, "__iter__") and not hasattr(axes, "plot"):
        axes_list = list(axes)
    else:
        axes_list = [axes]
    for index, ax in enumerate(axes_list):
        ax.grid(True, axis="both", color="#cbd5e1", linewidth=0.7, alpha=0.65)
        ax.set_axisbelow(True)
        ax.set_xticks(x_values)
        ax.margins(x=0.04)
        if index < len(axes_list) - 1:
            ax.tick_params(labelbottom=False)
        else:
            ax.set_xlabel(xlabel)


def _plot_single_series(
    ax: Any,
    entries: list[dict[str, Any]],
    *,
    field: str,
    ylabel: str,
    title: str,
    color: str,
) -> bool:
    x_values, y_values = _summary_series(entries, field)
    if not x_values:
        return False
    ax.plot(
        x_values,
        y_values,
        color=color,
        linewidth=1.9,
        marker="o",
        markersize=4.8,
    )
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontsize=10.5, color="#0f172a")
    return True


def _plot_compare_series(
    ax: Any,
    entries: list[dict[str, Any]],
    *,
    left_field: str,
    left_label: str,
    left_color: str,
    right_field: str,
    right_label: str,
    right_color: str,
    ylabel: str,
    title: str,
    right_linestyle: str = "--",
) -> bool:
    plotted = False
    left_x, left_y = _summary_series(entries, left_field)
    if left_x:
        ax.plot(
            left_x,
            left_y,
            color=left_color,
            linewidth=1.8,
            marker="o",
            markersize=4.6,
            label=left_label,
        )
        plotted = True
    right_x, right_y = _summary_series(entries, right_field)
    if right_x:
        ax.plot(
            right_x,
            right_y,
            color=right_color,
            linewidth=1.8,
            linestyle=right_linestyle,
            marker="s",
            markersize=4.2,
            label=right_label,
        )
        plotted = True
    if not plotted:
        return False
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontsize=10.5, color="#0f172a")
    ax.legend(frameon=False, loc="upper left", ncol=2)
    return True


def _first_adaptive_sample(
    records: list[dict[str, Any]],
    fill_run: FillRun,
) -> dict[str, Any] | None:
    for record in records:
        if str(record.get("kind", "")) != "sample":
            continue
        if not _matching_fill_record(fill_run, record):
            continue
        if record.get("strategy_name") or record.get("adapted_close_early_g") is not None:
            return record
    return None


def _draw_meta_section(
    meta_ax: Any,
    title: str,
    entries: list[tuple[str, str]],
    *,
    x_label: float,
    x_value: float,
    y_start: float,
) -> float:
    if not entries:
        return y_start
    meta_ax.text(
        x_label,
        y_start,
        title,
        ha="left",
        va="top",
        fontsize=8.8,
        color="#0f172a",
        fontweight="bold",
    )
    y = y_start - 0.043
    for label, value in entries:
        meta_ax.text(x_label, y, f"{label}:", ha="left", va="top", fontsize=7.2, color="#475569")
        meta_ax.text(x_value, y, value, ha="left", va="top", fontsize=7.2, color="#0f172a")
        y -= 0.036
    return y - 0.022


def _draw_debug_metadata(
    meta_ax: Any,
    session_dir: Path,
    fill_run: FillRun,
    base_weight_g: float | None,
    records: list[dict[str, Any]],
    command_text: str | None,
) -> None:
    summary = _fill_summary_record(records, fill_run)
    first_adaptive_sample = _first_adaptive_sample(records, fill_run)

    session_text = session_dir.name
    wrapped_session = "\n".join(textwrap.wrap(session_text, width=42)) or session_text

    general_lines = [
        ("Fill id", str(fill_run.fill_id)),
        ("Run", str(fill_run.run_id)),
        ("Slot", str(fill_run.slot_idx)),
        ("Preset", fill_run.preset_name or "?"),
        ("Strategie", fill_run.strategy_name or "?"),
        ("Focus duration", f"{fill_run.focus_duration_s:.2f} s"),
        ("Window", f"{fill_run.window_duration_s:.2f} s"),
        ("Ende", fill_run.end_reason),
    ]
    if base_weight_g is not None:
        general_lines.append(("Leermasse", f"{base_weight_g:.2f} g"))
    if fill_run.end_weight_g is not None and base_weight_g is not None:
        general_lines.append(("Endmasse Füllung", f"{fill_run.end_weight_g - base_weight_g:.2f} g"))
    if fill_run.scale_period_ms_cfg is not None:
        general_lines.append(("Waage Sollperiode", f"{fill_run.scale_period_ms_cfg:.0f} ms"))
    if fill_run.scale_period_ms_avg is not None:
        general_lines.append(("Waage Istperiode", f"{fill_run.scale_period_ms_avg:.1f} ms"))
    if fill_run.scale_rate_hz_avg is not None:
        general_lines.append(("Waagenrate", f"{fill_run.scale_rate_hz_avg:.2f} Hz"))
    if fill_run.fsm_period_ms_cfg is not None:
        general_lines.append(("FSM Sollperiode", f"{fill_run.fsm_period_ms_cfg:.0f} ms"))

    preset_lines = [
        (label, _format_param_value(label, value))
        for label, value in important_params(fill_run)
    ]

    result_lines: list[tuple[str, str]] = []
    runtime_lines: list[tuple[str, str]] = []
    adaptive_next_lines: list[tuple[str, str]] = []

    if first_adaptive_sample is not None:
        runtime_lines.extend(
            [
                ("Nahe Schließen", _format_meta_value(_float_value(first_adaptive_sample, "adapted_near_close_g"), "g", 1)),
                ("Früh schließen", _format_meta_value(_float_value(first_adaptive_sample, "adapted_close_early_g"), "g", 1)),
                ("Nachtropfen Start", _format_meta_value(_float_value(first_adaptive_sample, "adapted_drip_wait_ms"), "ms", 0)),
                ("Totzeit Schätzwert", _format_meta_value(_float_value(first_adaptive_sample, "learned_dead_time_s"), "s", 3)),
                ("Nachlauf Schätzwert", _format_meta_value(_float_value(first_adaptive_sample, "learned_post_close_gain_g"), "g", 1)),
                ("Bias Nahe Schließen", _format_meta_value(_float_value(first_adaptive_sample, "learned_near_close_bias_g"), "g", 1)),
            ]
        )
    if summary is not None:
        result_lines.extend(
            [
                ("Ergebnis", str(summary.get("text", "?"))),
                ("Zielmasse", _format_meta_value(_float_value(summary, "target_g"), "g", 1)),
                ("Endmasse", _format_meta_value(_float_value(summary, "final_mass_g"), "g", 1)),
                ("Fehler", _format_meta_value(_float_value(summary, "fill_error_g"), "g", 1)),
                ("Fülldauer", _format_meta_value(_float_value(summary, "fill_duration_s"), "s", 2)),
                ("Refills", _format_meta_value(summary.get("refill_count"))),
            ]
        )
        runtime_lines.extend(
            [
                ("Totzeit Messung", _format_meta_value(_float_value(summary, "measured_dead_time_s"), "s", 3)),
                ("Nachlauf Messung", _format_meta_value(_float_value(summary, "measured_post_close_gain_g"), "g", 1)),
                ("Near-Close Ist", _format_meta_value(_float_value(summary, "measured_near_close_gain_g"), "g", 1)),
                ("Rate schnell", _format_meta_value(_float_value(summary, "measured_fast_rate_gps"), "g/s", 1)),
                ("Rate reduziert", _format_meta_value(_float_value(summary, "measured_slow_rate_gps"), "g/s", 1)),
                ("Rate beim Schließen", _format_meta_value(_float_value(summary, "rate_at_close_gps"), "g/s", 1)),
                ("2-Sample beim Schließen", _format_meta_value(_float_value(summary, "rate_2sample_at_close_gps"), "g/s", 1)),
                ("4-Sample beim Schließen", _format_meta_value(_float_value(summary, "rate_4sample_at_close_gps"), "g/s", 1)),
                ("Mittel beim Schließen", _format_meta_value(_float_value(summary, "rate_filtered_medium_at_close_gps"), "g/s", 1)),
                ("Langsam beim Schließen", _format_meta_value(_float_value(summary, "rate_filtered_slow_at_close_gps"), "g/s", 1)),
                ("Totzeit Start", _format_meta_value(_float_value(summary, "used_dead_time_s"), "s", 3)),
                ("Nachlauf Start", _format_meta_value(_float_value(summary, "used_post_close_gain_g"), "g", 1)),
                ("Rate schnell Start", _format_meta_value(_float_value(summary, "used_fast_rate_gps"), "g/s", 1)),
                ("Rate reduziert Start", _format_meta_value(_float_value(summary, "used_slow_rate_gps"), "g/s", 1)),
                ("Bias Start", _format_meta_value(_float_value(summary, "used_near_close_bias_g"), "g", 1)),
                ("Near-Close Start", _format_meta_value(_float_value(summary, "used_near_close_g"), "g", 1)),
                ("Früh schließen Start", _format_meta_value(_float_value(summary, "used_close_early_g"), "g", 1)),
                ("Nachtropfen Ist", _format_meta_value(_float_value(summary, "drip_wait_used_ms"), "ms", 0)),
            ]
        )
        adaptive_next_lines.extend(
            [
                ("Nächste Totzeit", _format_meta_value(_float_value(summary, "next_dead_time_s"), "s", 3)),
                ("Nächster Nachlauf", _format_meta_value(_float_value(summary, "next_post_close_gain_g"), "g", 1)),
                ("Nächste Rate schnell", _format_meta_value(_float_value(summary, "next_fast_rate_gps"), "g/s", 1)),
                ("Nächste Rate reduziert", _format_meta_value(_float_value(summary, "next_slow_rate_gps"), "g/s", 1)),
                ("Nächster Bias", _format_meta_value(_float_value(summary, "next_near_close_bias_g"), "g", 1)),
                ("Nächstes Nachtropfen", _format_meta_value(_float_value(summary, "next_drip_wait_ms"), "ms", 0)),
                ("Nächstes Nahe Schließen", _format_meta_value(_float_value(summary, "next_near_close_g"), "g", 1)),
                ("Nächstes Früh schließen", _format_meta_value(_float_value(summary, "next_close_early_g"), "g", 1)),
            ]
        )

    meta_ax.text(
        0.0,
        1.0,
        "Debug metadata",
        ha="left",
        va="top",
        fontsize=11,
        color="#0f172a",
        fontweight="bold",
    )
    meta_ax.text(0.0, 0.955, "Session", ha="left", va="top", fontsize=7.6, color="#475569")
    meta_ax.text(0.0, 0.925, wrapped_session, ha="left", va="top", fontsize=7.4, color="#0f172a", linespacing=1.02)
    session_lines = max(1, wrapped_session.count("\n") + 1)
    y_after_session = 0.925 - (session_lines * 0.042) - 0.045

    left_y = y_after_session
    right_y = y_after_session
    left_y = _draw_meta_section(meta_ax, "Lauf", general_lines, x_label=0.0, x_value=0.26, y_start=left_y)
    left_y = _draw_meta_section(meta_ax, "Ergebnis", result_lines, x_label=0.0, x_value=0.26, y_start=left_y)
    left_y = _draw_meta_section(meta_ax, "Preset-Parameter", preset_lines, x_label=0.0, x_value=0.26, y_start=left_y)
    right_y = _draw_meta_section(meta_ax, "Laufzeitwerte", runtime_lines, x_label=0.50, x_value=0.79, y_start=right_y)
    right_y = _draw_meta_section(meta_ax, "Adaptionswerte", adaptive_next_lines, x_label=0.50, x_value=0.79, y_start=right_y)

    if command_text:
        command_title_y = min(left_y, right_y)
        wrapped_command = "\n".join(textwrap.wrap(command_text, width=92)) or command_text
        meta_ax.text(
            0.0,
            command_title_y,
            "Plot command",
            ha="left",
            va="top",
            fontsize=8.8,
            color="#0f172a",
            fontweight="bold",
        )
        meta_ax.text(
            0.0,
            command_title_y - 0.043,
            wrapped_command,
            ha="left",
            va="top",
            fontsize=6.0,
            color="#334155",
            family="monospace",
            linespacing=1.08,
        )


def _create_figure_axes(
    *,
    debug: bool,
    show_rate: str,
    rate_layout: str,
    state_style: str,
    figure_profile: str,
    want_control_panel: bool,
    rate_plot_height_ratio: float,
    rate_figure_extra_h: float,
) -> tuple[Any, Any | None, Any, Any | None, Any | None, Any | None]:
    figure_width, figure_height_base = _figure_size(figure_profile)
    want_rate = show_rate != "none"
    want_band = state_style == "band"
    subplot_rate = want_rate and rate_layout == "subplot"

    figure_height = figure_height_base + (STATE_BAND_FIGURE_EXTRA_H if want_band else 0.0)
    if subplot_rate:
        figure_height += rate_figure_extra_h
    if want_control_panel:
        figure_height += CONTROL_FIGURE_EXTRA_H
    if debug:
        figure_height += DEBUG_FIGURE_EXTRA_H
    left_rows = 1 + (1 if want_band else 0) + (1 if subplot_rate else 0) + (1 if want_control_panel else 0)
    left_height_ratios: list[float] = []
    if want_band:
        left_height_ratios.append(STATE_BAND_HEIGHT_RATIO)
    left_height_ratios.append(MAIN_PLOT_HEIGHT_RATIO)
    if subplot_rate:
        left_height_ratios.append(rate_plot_height_ratio)
    if want_control_panel:
        left_height_ratios.append(CONTROL_PLOT_HEIGHT_RATIO)

    if debug:
        fig = plt.figure(figsize=(figure_width + DEBUG_META_WIDTH, figure_height))
        gs = fig.add_gridspec(
            left_rows,
            2,
            width_ratios=[figure_width, DEBUG_META_WIDTH],
            height_ratios=left_height_ratios,
        )
        main_row = 1 if want_band else 0
        ax = fig.add_subplot(gs[main_row, 0])
        band_ax = fig.add_subplot(gs[0, 0], sharex=ax) if want_band else None
        rate_ax = fig.add_subplot(gs[main_row + 1, 0], sharex=ax) if subplot_rate else None
        control_row = main_row + 1 + (1 if subplot_rate else 0)
        control_ax = fig.add_subplot(gs[control_row, 0], sharex=ax) if want_control_panel else None
        meta_ax = fig.add_subplot(gs[:, 1])
        meta_ax.axis("off")
        if want_rate and rate_layout == "overlay":
            return fig, band_ax, ax, ax.twinx(), control_ax, meta_ax
        return fig, band_ax, ax, rate_ax, control_ax, meta_ax

    fig = plt.figure(figsize=(figure_width, figure_height))
    gs = fig.add_gridspec(left_rows, 1, height_ratios=left_height_ratios)
    main_row = 1 if want_band else 0
    ax = fig.add_subplot(gs[main_row, 0])
    band_ax = fig.add_subplot(gs[0, 0], sharex=ax) if want_band else None
    control_row = main_row + 1 + (1 if subplot_rate else 0)
    control_ax = fig.add_subplot(gs[control_row, 0], sharex=ax) if want_control_panel else None
    if subplot_rate:
        rate_ax = fig.add_subplot(gs[main_row + 1, 0], sharex=ax)
        return fig, band_ax, ax, rate_ax, control_ax, None
    if want_rate and rate_layout == "overlay":
        return fig, band_ax, ax, ax.twinx(), control_ax, None
    return fig, band_ax, ax, None, control_ax, None


def _outside_legend_columns(
    *,
    legend_count: int,
    figure_profile: str,
    subplot_rate: bool,
) -> int:
    if legend_count <= 0:
        return 1
    if figure_profile == "narrow":
        return min(2, legend_count)
    if subplot_rate or legend_count > 6:
        return min(3, legend_count)
    return min(4, legend_count)


def _outside_legend_anchor_x(*, debug: bool, figure_profile: str) -> float:
    if not debug:
        return 0.5
    figure_width, _ = _figure_size(figure_profile)
    total_width = figure_width + DEBUG_META_WIDTH
    return (figure_width / total_width) * 0.5


def _layout_bottom_margin(
    *,
    debug: bool,
    subplot_rate: bool,
    outside_legend_rows: int,
) -> float:
    bottom = 0.08
    if subplot_rate:
        bottom = 0.11
    if outside_legend_rows > 0:
        bottom += 0.07 + 0.058 * max(0, outside_legend_rows - 1)
    if debug:
        bottom += 0.14 if subplot_rate else 0.07
    return min(bottom, 0.42)


def _apply_figure_layout(
    fig: Any,
    *,
    debug: bool,
    subplot_rate: bool,
    want_band: bool,
    outside_legend_rows: int,
) -> None:
    top = 0.945 if want_band else 0.965
    bottom = _layout_bottom_margin(
        debug=debug,
        subplot_rate=subplot_rate,
        outside_legend_rows=outside_legend_rows,
    )
    if debug:
        fig.subplots_adjust(
            left=0.075,
            right=0.985,
            top=top,
            bottom=bottom,
            wspace=0.10,
            hspace=0.28 if subplot_rate else 0.05,
        )
        return

    fig.tight_layout(
        rect=(0.03, bottom, 0.985, top),
        pad=0.35,
        h_pad=0.95 if subplot_rate else 0.45,
        w_pad=0.6,
    )


def _plot_rate_panel(
    rate_ax: Any,
    fill_run: FillRun,
    x_samples: list[float],
    rate_series: dict[str, list[float | None]],
    target_rate_series: list[float | None],
    control_series: dict[str, list[float | None]],
    *,
    show_rate: str,
    overlay: bool,
) -> tuple[list[Any], list[str]]:
    plotted_any = False
    handles: list[Any] = []
    labels: list[str] = []
    selected_series = _selected_rate_series(show_rate)
    for z_index, name in enumerate(selected_series, start=2):
        values = rate_series.get(name, [])
        if not any(value is not None for value in values):
            continue
        spec = RATE_SERIES_SPECS[name]
        (line_rate,) = rate_ax.plot(
            x_samples,
            values,
            color=spec["color"],
            linewidth=1.1 if name == "raw" else (1.2 if overlay else 1.5),
            label=spec["label"],
            zorder=(6 + z_index) if overlay else z_index,
            alpha=0.85 if name == "raw" else (0.92 if overlay else 1.0),
        )
        handles.append(line_rate)
        labels.append(spec["label"])
        plotted_any = True
    strategy = (fill_run.strategy_name or "").strip().lower()
    if strategy in ("flow-control", "flow-cascade"):
        if any(value is not None for value in target_rate_series):
            (line_target_rate,) = rate_ax.plot(
                x_samples,
                target_rate_series,
                color=LINE_TARGET_RATE,
                linewidth=1.5 if overlay else 1.6,
                linestyle=(0, (5, 2)),
                label="Ziel-Füllrate ṁ* [g/s]",
                zorder=5 if overlay else 2,
                alpha=0.95,
            )
            handles.append(line_target_rate)
            labels.append("Ziel-Füllrate ṁ* [g/s]")
            plotted_any = True
    if strategy == "flow-cascade":
        # Smith-predictor model rates: delay-free model output ŷ and its
        # dead-time-delayed version ŷ_d. Comparing ŷ_d with the measured
        # (filtered) rate shows the prediction error the controller reacts to.
        model_rate = control_series.get("model_rate", [])
        model_delayed = control_series.get("model_delayed", [])
        if any(value is not None for value in model_rate):
            (line_model,) = rate_ax.plot(
                x_samples, model_rate,
                color=LINE_MODEL_RATE, linewidth=1.2, linestyle="-",
                label="Modellrate ŷ [g/s]", zorder=(7 if overlay else 3), alpha=0.9,
            )
            handles.append(line_model)
            labels.append("Modellrate ŷ [g/s]")
            plotted_any = True
        if any(value is not None for value in model_delayed):
            (line_model_d,) = rate_ax.plot(
                x_samples, model_delayed,
                color=LINE_MODEL_RATE, linewidth=1.1, linestyle=(0, (1, 1)),
                label="Modellrate verz. ŷ_d [g/s]", zorder=(7 if overlay else 3), alpha=0.75,
            )
            handles.append(line_model_d)
            labels.append("Modellrate verz. ŷ_d [g/s]")
            plotted_any = True
    if overlay:
        rate_ax.set_ylabel(RATE_AXIS_LABEL, color=LINE_RATE_FILTERED)
    else:
        rate_ax.set_ylabel(RATE_AXIS_LABEL)
    if not overlay:
        rate_ax.set_xlabel("Zeit relativ zum Füllbeginn [s]", labelpad=10)
        rate_ax.grid(True, axis="both", color="#cbd5e1", linewidth=0.6, alpha=0.55)
        rate_ax.set_axisbelow(True)
    else:
        rate_ax.spines["right"].set_position(("axes", 1.10))
        rate_ax.tick_params(axis="y", colors=LINE_RATE_FILTERED, labelsize=9)
        rate_ax.spines["right"].set_color(LINE_RATE_FILTERED)
    if not plotted_any:
        rate_ax.text(
            0.5,
            0.5,
            "Keine Raten-Telemetrie in diesem Lauf",
            transform=rate_ax.transAxes,
            ha="center",
            va="center",
            fontsize=8.5,
            color="#64748b",
        )
    return handles, labels


def _plot_control_panel(
    control_ax: Any,
    x_samples: list[float],
    rate_series: dict[str, list[float | None]],
    control_series: dict[str, list[float | None]],
) -> tuple[list[Any], list[str]]:
    """Cascade control-loop internals, linkable to the block diagram:
    - rate error e_r = ṁ* − feedback (into the inner PI controller F_R,i)
    - prediction error ṁ_m − ŷ_d (the Smith correction, output of Σ_corr)
    both in g/s on the primary axis; the PI integrator on a secondary % axis."""
    handles: list[Any] = []
    labels: list[str] = []
    plotted_any = False

    filtered = rate_series.get("filtered", [])
    model_delayed = control_series.get("model_delayed", [])
    rate_error = control_series.get("rate_error", [])
    integ = control_series.get("control_integ", [])

    prediction_error = [
        (m - d) if (m is not None and d is not None) else None
        for m, d in zip(filtered, model_delayed, strict=True)
    ]

    control_ax.axhline(0.0, color="#94a3b8", linewidth=0.8, alpha=0.7, zorder=1)
    if any(v is not None for v in rate_error):
        (l_e,) = control_ax.plot(
            x_samples, rate_error, color=LINE_RATE_ERROR, linewidth=1.5,
            label="Ratenfehler e_r [g/s]", zorder=3, alpha=0.95,
        )
        handles.append(l_e)
        labels.append("Ratenfehler e_r [g/s]")
        plotted_any = True
    if any(v is not None for v in prediction_error):
        (l_p,) = control_ax.plot(
            x_samples, prediction_error, color=LINE_MODEL_RATE, linewidth=1.4,
            linestyle=(0, (5, 2)), label="Prädiktionsfehler ṁ_m−ŷ_d [g/s]", zorder=3, alpha=0.9,
        )
        handles.append(l_p)
        labels.append("Prädiktionsfehler ṁ_m−ŷ_d [g/s]")
        plotted_any = True

    control_ax.set_ylabel(CONTROL_AXIS_LABEL)
    control_ax.set_xlabel("Zeit relativ zum Füllbeginn [s]", labelpad=10)
    control_ax.grid(True, axis="both", color="#cbd5e1", linewidth=0.6, alpha=0.55)
    control_ax.set_axisbelow(True)

    if any(v is not None for v in integ):
        integ_ax = control_ax.twinx()
        (l_i,) = integ_ax.plot(
            x_samples, integ, color=LINE_CONTROL_INTEG, linewidth=1.3,
            linestyle=(0, (1, 1)), label="PI-Integrator [%]", zorder=2, alpha=0.9,
        )
        integ_ax.set_ylabel("PI-Integrator [%]", color=LINE_CONTROL_INTEG)
        integ_ax.tick_params(axis="y", colors=LINE_CONTROL_INTEG, labelsize=9)
        integ_ax.spines["right"].set_color(LINE_CONTROL_INTEG)
        handles.append(l_i)
        labels.append("PI-Integrator [%]")
        plotted_any = True

    if not plotted_any:
        control_ax.text(
            0.5, 0.5, "Keine Regler-Telemetrie in diesem Lauf",
            transform=control_ax.transAxes, ha="center", va="center",
            fontsize=8.5, color="#64748b",
        )
    return handles, labels


def _has_cascade_control_series(strategy: str, control_series: dict[str, list[float | None]]) -> bool:
    return strategy == "flow-cascade" and any(
        value is not None
        for key in ("model_delayed", "rate_error", "control_integ", "gate_gain")
        for value in control_series.get(key, [])
    )


def _rate_panel_layout(strategy: str, show_rate: str, rate_layout: str) -> tuple[float, float]:
    if show_rate == "none" or rate_layout != "subplot":
        return RATE_PLOT_HEIGHT_RATIO, 0.0

    selected_count = len(_selected_rate_series(show_rate))
    if strategy in ("flow-control", "flow-cascade"):
        selected_count += 1
    if strategy == "flow-cascade":
        selected_count += 2
        return RATE_PLOT_HEIGHT_RATIO_CASCADE, RATE_FIGURE_EXTRA_H_CASCADE
    if selected_count > 2:
        return RATE_PLOT_HEIGHT_RATIO_DENSE, RATE_FIGURE_EXTRA_H_DENSE
    return RATE_PLOT_HEIGHT_RATIO, 0.0


def _scaled_rate_panel_layout(
    strategy: str,
    show_rate: str,
    rate_layout: str,
    rate_height_scale: float,
) -> tuple[float, float]:
    rate_ratio, extra_h = _rate_panel_layout(strategy, show_rate, rate_layout)
    if show_rate == "none" or rate_layout != "subplot":
        return rate_ratio, extra_h
    scale = max(0.2, rate_height_scale)
    return rate_ratio * scale, extra_h * scale


def _save_figure(fig: Any, out_path: Path, fmt: str) -> None:
    # Final tight crop is more reliable than trying to predict every title/legend
    # combination up front with fixed subplot margins.
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


def _render_fill_variant(
    session_dir: Path,
    fill_run: FillRun,
    records: list[dict[str, Any]],
    output_dir: Path,
    formats: list[str],
    *,
    debug: bool,
    legend_placement: str,
    state_style: str,
    show_rate: str,
    rate_layout: str,
    figure_profile: str,
    output_name: str,
    include_control_panel: bool,
    show_target: bool,
    mass_y_max: float | None,
    rate_height_scale: float,
    command_text: str | None,
) -> list[Path]:
    samples = _sample_events(records)
    if not samples:
        raise ValueError(f"fill {fill_run.fill_id} has no sample records")

    states = _state_events(records)
    gates = _gate_events(records)
    faults = _fault_events(records)
    x_samples, y_fill_mass, y_gate, rate_series, target_rate_series, control_series, base_weight_g = _series_from_samples(fill_run, samples)
    strategy = (fill_run.strategy_name or "").strip().lower()
    show_control_panel = include_control_panel and _has_cascade_control_series(strategy, control_series)
    rate_plot_height_ratio, rate_figure_extra_h = _scaled_rate_panel_layout(
        strategy, show_rate, rate_layout, rate_height_scale
    )

    _figure_style()
    fig, band_ax, ax, rate_ax, control_ax, meta_ax = _create_figure_axes(
        debug=debug,
        show_rate=show_rate,
        rate_layout=rate_layout,
        state_style=state_style,
        figure_profile=figure_profile,
        want_control_panel=show_control_panel,
        rate_plot_height_ratio=rate_plot_height_ratio,
        rate_figure_extra_h=rate_figure_extra_h,
    )
    ax2 = ax.twinx()
    overlay_rate = rate_ax is not None and show_rate != "none" and rate_layout == "overlay"

    if state_style == "background":
        _plot_state_background(ax, fill_run, states, show_labels=False)
    elif state_style == "band":
        _plot_state_band(band_ax, fill_run, states)
    if debug:
        _annotate_state_badges(ax, fill_run, states)

    if show_target and fill_run.target_g is not None:
        tol_low = fill_run.params.get("VAR(target_tol_low_g)")
        tol_high = fill_run.params.get("VAR(target_tol_high_g)")
        if isinstance(tol_low, (int, float)) and isinstance(tol_high, (int, float)):
            ax.axhspan(
                fill_run.target_g - float(tol_low),
                fill_run.target_g + float(tol_high),
                color="#bbf7d0",
                alpha=0.18,
                zorder=0,
            )
        ax.axhline(
            fill_run.target_g,
            color=LINE_TARGET,
            linewidth=1.4,
            linestyle=(0, (4, 2)),
            label="Zielmasse [g]",
            zorder=2,
        )

    ax.plot(
        x_samples,
        y_fill_mass,
        color=LINE_FILL_MASS,
        linewidth=2.2,
        label="Füllmasse [g]",
        zorder=4,
    )
    ax2.step(
        x_samples,
        y_gate,
        where="post",
        color=LINE_GATE,
        linewidth=1.8,
        label="Klappenstellung [%]",
        zorder=5,
    )

    _annotate_gate_events(ax2, fill_run, gates, samples)
    _annotate_faults(ax, fill_run, faults)

    has_lower_panel = (rate_ax is not None and not overlay_rate) or control_ax is not None
    ax.set_xlabel("" if has_lower_panel else "Zeit relativ zum Füllbeginn [s]")
    ax.tick_params(labelbottom=True, axis="x", pad=1)
    ax.set_ylabel("Füllmasse [g]")
    ax2.set_ylabel("Klappenstellung [%]")
    ax2.set_ylim(-2, 112)
    ax2.set_yticks([0, 20, 40, 60, 80, 100])
    if mass_y_max is not None:
        ax.set_ylim(0.0, mass_y_max)
    ax.grid(True, axis="both", color="#cbd5e1", linewidth=0.7, alpha=0.65)
    ax.set_axisbelow(True)

    handles3: list[Any] = []
    labels3: list[str] = []
    if rate_ax is not None:
        handles3, labels3 = _plot_rate_panel(
            rate_ax,
            fill_run,
            x_samples,
            rate_series,
            target_rate_series,
            control_series,
            show_rate=show_rate,
            overlay=overlay_rate,
        )
        if not overlay_rate:
            rate_ax.tick_params(labelbottom=True, axis="x", pad=4)
    handles4: list[Any] = []
    labels4: list[str] = []
    if control_ax is not None:
        handles4, labels4 = _plot_control_panel(
            control_ax,
            x_samples,
            rate_series,
            control_series,
        )
        control_ax.tick_params(labelbottom=True, axis="x", pad=4)

    handles1, labels1 = ax.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    legend_count = len(handles1) + len(handles2) + len(handles3) + len(handles4)
    outside_legend_cols = _outside_legend_columns(
        legend_count=legend_count,
        figure_profile=figure_profile,
        subplot_rate=((rate_ax is not None and not overlay_rate) or control_ax is not None),
    )
    outside_legend_rows = 0
    if legend_placement == "outside":
        outside_legend_rows = max(1, (legend_count + outside_legend_cols - 1) // outside_legend_cols)
    bottom_margin = _layout_bottom_margin(
        debug=debug,
        subplot_rate=((rate_ax is not None and not overlay_rate) or control_ax is not None),
        outside_legend_rows=outside_legend_rows,
    )

    _apply_figure_layout(
        fig,
        debug=debug,
        subplot_rate=((rate_ax is not None and not overlay_rate) or control_ax is not None),
        want_band=(band_ax is not None),
        outside_legend_rows=outside_legend_rows,
    )

    legend_kwargs = {
        "handles": handles1 + handles2 + handles3 + handles4,
        "labels": labels1 + labels2 + labels3 + labels4,
        "frameon": False,
        "ncol": outside_legend_cols if legend_placement == "outside" else 1,
        "borderaxespad": 0.0,
    }
    if legend_placement == "inside":
        legend_kwargs.update(
            {
                "loc": "upper left",
                "bbox_to_anchor": (0.015, 0.985),
            }
        )
        ax.legend(**legend_kwargs)
        if rate_ax is not None and not overlay_rate and handles3:
            rate_ax.legend(
                handles=handles3,
                labels=labels3,
                frameon=False,
                loc="upper right",
                ncol=1,
                borderaxespad=0.2,
            )
        if control_ax is not None and handles4:
            control_ax.legend(
                handles=handles4,
                labels=labels4,
                frameon=False,
                loc="upper right",
                ncol=1,
                borderaxespad=0.2,
            )
    else:
        if legend_count > 0:
            legend_y = max(0.02, bottom_margin - (0.10 if debug else 0.01))
            fig.legend(
                **legend_kwargs,
                loc="upper center",
                bbox_to_anchor=(
                    _outside_legend_anchor_x(debug=debug, figure_profile=figure_profile),
                    legend_y,
                ),
                bbox_transform=fig.transFigure,
            )

    if debug and meta_ax is not None:
        _draw_debug_metadata(meta_ax, session_dir, fill_run, base_weight_g, records, command_text)

    if ANNOTATION_HOOK is not None:
        ANNOTATION_HOOK(fig=fig, ax=ax, rate_ax=rate_ax, fill_run=fill_run, records=records)

    exported: list[Path] = []
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = output_dir / f"{_output_stem_base(session_dir, fill_run)}__{output_name}"
    for fmt in formats:
        out_path = stem.parent / f"{stem.name}.{fmt}"
        _save_figure(fig, out_path, fmt)
        exported.append(out_path)
    plt.close(fig)
    return exported


def _render_control_only_variant(
    session_dir: Path,
    fill_run: FillRun,
    records: list[dict[str, Any]],
    output_dir: Path,
    formats: list[str],
    *,
    figure_profile: str,
    output_name: str,
    rate_height_scale: float,
) -> list[Path]:
    samples = _sample_events(records)
    if not samples:
        raise ValueError(f"fill {fill_run.fill_id} has no sample records")

    x_samples, _, _, rate_series, _, control_series, _ = _series_from_samples(fill_run, samples)
    strategy = (fill_run.strategy_name or "").strip().lower()
    if not _has_cascade_control_series(strategy, control_series):
        return []

    _figure_style()
    figure_width, figure_height_base = _figure_size(figure_profile)
    fig, control_ax = plt.subplots(1, 1, figsize=(figure_width, max(2.45, figure_height_base * 0.64 * max(0.2, rate_height_scale))))
    handles, labels = _plot_control_panel(control_ax, x_samples, rate_series, control_series)
    if handles:
        control_ax.legend(handles=handles, labels=labels, frameon=False, loc="upper right", ncol=1, borderaxespad=0.2)
    fig.tight_layout(rect=(0.04, 0.05, 0.985, 0.985), pad=0.35, h_pad=0.45, w_pad=0.6)

    exported: list[Path] = []
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = output_dir / f"{_output_stem_base(session_dir, fill_run)}__{output_name}"
    for fmt in formats:
        out_path = stem.parent / f"{stem.name}.{fmt}"
        _save_figure(fig, out_path, fmt)
        exported.append(out_path)
    plt.close(fig)
    return exported


def _plot_fill(
    session_dir: Path,
    fill_run: FillRun,
    records: list[dict[str, Any]],
    output_dir: Path,
    formats: list[str],
    legend_placement: str,
    state_style: str,
    show_rate: str,
    rate_layout: str,
    figure_profile: str,
    plain_both_profiles: bool,
    cascade_control_export: str,
    show_target: bool,
    mass_y_max: float | None,
    rate_height_scale: float,
    command_text: str | None,
) -> list[Path]:
    strategy = (fill_run.strategy_name or "").strip().lower()
    split_cascade_control = strategy == "flow-cascade" and cascade_control_export == "separate"
    exported = _render_fill_variant(
        session_dir,
        fill_run,
        records,
        output_dir,
        formats,
        debug=True,
        legend_placement=legend_placement,
        state_style=state_style,
        show_rate=show_rate,
        rate_layout=rate_layout,
        figure_profile=figure_profile,
        output_name="debug",
        include_control_panel=True,
        show_target=show_target,
        mass_y_max=mass_y_max,
        rate_height_scale=rate_height_scale,
        command_text=command_text,
    )
    plain_profiles = ["default", "narrow"] if plain_both_profiles else [figure_profile]
    for plain_profile in plain_profiles:
        output_name = "plain"
        if plain_both_profiles:
            output_name = f"plain_{_plain_profile_suffix(plain_profile)}"
        exported.extend(
            _render_fill_variant(
                session_dir,
                fill_run,
                records,
                output_dir,
                formats,
                debug=False,
                legend_placement=legend_placement,
                state_style=state_style,
                show_rate=show_rate,
                rate_layout=rate_layout,
                figure_profile=plain_profile,
                output_name=output_name,
                include_control_panel=not split_cascade_control,
                show_target=show_target,
                mass_y_max=mass_y_max,
                rate_height_scale=rate_height_scale,
                command_text=command_text,
            )
        )
        if split_cascade_control:
            control_output_name = "control-internals"
            if plain_both_profiles:
                control_output_name = f"control-internals_{_plain_profile_suffix(plain_profile)}"
            exported.extend(
                _render_control_only_variant(
                    session_dir,
                    fill_run,
                    records,
                    output_dir,
                    formats,
                    figure_profile=plain_profile,
                    output_name=control_output_name,
                    rate_height_scale=rate_height_scale,
                )
            )
    return exported


def _export_session_figure(
    fig: Any,
    output_dir: Path,
    *,
    session_dir: Path,
    suffix: str,
    formats: list[str],
) -> list[Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = output_dir / f"{_session_output_stem_base(session_dir)}__{suffix}"
    exported: list[Path] = []
    for fmt in formats:
        out_path = stem.parent / f"{stem.name}.{fmt}"
        _save_figure(fig, out_path, fmt)
        exported.append(out_path)
    plt.close(fig)
    return exported


def _plot_session_overview(
    session_dir: Path,
    entries: list[dict[str, Any]],
    output_dir: Path,
    formats: list[str],
) -> list[Path]:
    x_ticks = [int(entry["fill_id"]) for entry in entries]
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8.67, 7.9))
    fig.suptitle("Sitzungsübersicht Füllläufe", x=0.08, y=0.985, ha="left", fontsize=13, color="#0f172a")
    fig.text(0.08, 0.955, _session_summary_subtitle(session_dir, entries), ha="left", va="top", fontsize=9.2, color="#475569")

    plotted = [
        _plot_single_series(
            axes[0],
            entries,
            field="fill_error_g",
            ylabel="Fehler zur Zielmasse [g]",
            title="Abweichung pro Fülllauf",
            color=SESSION_LINE_COLORS["fill_error_g"],
        ),
        _plot_single_series(
            axes[1],
            entries,
            field="fill_duration_s",
            ylabel="Fülldauer [s]",
            title="Fülldauer pro Fülllauf",
            color=SESSION_LINE_COLORS["fill_duration_s"],
        ),
        _plot_single_series(
            axes[2],
            entries,
            field="refill_count",
            ylabel="Nachfüllungen [-]",
            title="Nachfüllzyklen pro Fülllauf",
            color=SESSION_LINE_COLORS["refill_count"],
        ),
    ]
    if not any(plotted):
        plt.close(fig)
        return []
    _setup_session_axes(list(axes), x_ticks)
    fig.tight_layout(rect=(0.06, 0.06, 0.98, 0.93))
    return _export_session_figure(
        fig,
        output_dir,
        session_dir=session_dir,
        suffix="session-summary",
        formats=formats,
    )


def _plot_session_adaptive(
    session_dir: Path,
    entries: list[dict[str, Any]],
    output_dir: Path,
    formats: list[str],
) -> list[Path]:
    metrics = [
        ("used_near_close_g", "Nahe Schließen [g]", "Eingesetzter Schwellwert für Nahe Schließen", SESSION_LINE_COLORS["used_near_close_g"]),
        ("used_close_early_g", "Früh schließen [g]", "Eingesetzter Früh-Schließen-Wert", SESSION_LINE_COLORS["used_close_early_g"]),
        ("drip_wait_used_ms", "Nachtropfzeit [ms]", "Tatsächlich verwendete Nachtropfzeit", SESSION_LINE_COLORS["drip_wait_used_ms"]),
    ]
    available = [
        metric for metric in metrics if _summary_series(entries, metric[0])[0]
    ]
    if not available:
        return []

    x_ticks = [int(entry["fill_id"]) for entry in entries]
    fig, axes = plt.subplots(len(available), 1, sharex=True, figsize=(8.67, 2.15 * len(available) + 1.0))
    axes_list = [axes] if len(available) == 1 else list(axes)
    fig.suptitle("Sitzungsübersicht adaptive Parameter", x=0.08, y=0.985, ha="left", fontsize=13, color="#0f172a")
    fig.text(0.08, 0.955, _session_summary_subtitle(session_dir, entries), ha="left", va="top", fontsize=9.2, color="#475569")

    for ax, (field, ylabel, title, color) in zip(axes_list, available, strict=True):
        _plot_single_series(
            ax,
            entries,
            field=field,
            ylabel=ylabel,
            title=title,
            color=color,
        )

    _setup_session_axes(axes_list, x_ticks)
    fig.tight_layout(rect=(0.06, 0.06, 0.98, 0.93))
    return _export_session_figure(
        fig,
        output_dir,
        session_dir=session_dir,
        suffix="session-learning",
        formats=formats,
    )


def _plot_session_compare(
    session_dir: Path,
    entries: list[dict[str, Any]],
    output_dir: Path,
    formats: list[str],
) -> list[Path]:
    panel_builders = [
        (
            "post_close",
            lambda ax: _plot_compare_series(
                ax,
                entries,
                left_field="measured_post_close_gain_g",
                left_label="Gemessener Nachlauf",
                left_color=SESSION_LINE_COLORS["measured_post_close_gain_g"],
                right_field="next_post_close_gain_g",
                right_label="Nächster Nachlauf",
                right_color=SESSION_LINE_COLORS["next_post_close_gain_g"],
                ylabel="Nachlauf [g]",
                title="Nachlauf: gemessen vs. nächster Schätzwert",
            ),
        ),
        (
            "dead_time",
            lambda ax: _plot_compare_series(
                ax,
                entries,
                left_field="measured_dead_time_s",
                left_label="Gemessene Totzeit",
                left_color=SESSION_LINE_COLORS["measured_dead_time_s"],
                right_field="next_dead_time_s",
                right_label="Nächste Totzeit",
                right_color=SESSION_LINE_COLORS["next_dead_time_s"],
                ylabel="Totzeit [s]",
                title="Totzeit: Messung vs. nächster Schätzwert",
            ),
        ),
        (
            "rates",
            lambda ax: (
                _plot_compare_series(
                    ax,
                    entries,
                    left_field="measured_fast_rate_gps",
                    left_label="Gemessene schnelle Rate",
                    left_color=SESSION_LINE_COLORS["measured_fast_rate_gps"],
                    right_field="next_fast_rate_gps",
                    right_label="Nächste schnelle Rate",
                    right_color=SESSION_LINE_COLORS["next_fast_rate_gps"],
                    ylabel="Füllrate [g/s]",
                    title="Füllraten schnell/langsam: Messung vs. nächster Schätzwert",
                )
                or _plot_compare_series(
                    ax,
                    entries,
                    left_field="measured_slow_rate_gps",
                    left_label="Gemessene reduzierte Rate",
                    left_color=SESSION_LINE_COLORS["measured_slow_rate_gps"],
                    right_field="next_slow_rate_gps",
                    right_label="Nächste reduzierte Rate",
                    right_color=SESSION_LINE_COLORS["next_slow_rate_gps"],
                    ylabel="Füllrate [g/s]",
                    title="Füllraten schnell/langsam: Messung vs. nächster Schätzwert",
                )
            ),
        ),
    ]
    available_keys: list[str] = []
    for key, _ in panel_builders:
        if key == "post_close" and (_summary_series(entries, "measured_post_close_gain_g")[0] or _summary_series(entries, "next_post_close_gain_g")[0]):
            available_keys.append(key)
        elif key == "dead_time" and (_summary_series(entries, "measured_dead_time_s")[0] or _summary_series(entries, "next_dead_time_s")[0]):
            available_keys.append(key)
        elif key == "rates" and (
            _summary_series(entries, "measured_fast_rate_gps")[0]
            or _summary_series(entries, "next_fast_rate_gps")[0]
            or _summary_series(entries, "measured_slow_rate_gps")[0]
            or _summary_series(entries, "next_slow_rate_gps")[0]
        ):
            available_keys.append(key)
    if not available_keys:
        return []

    x_ticks = [int(entry["fill_id"]) for entry in entries]
    fig, axes = plt.subplots(len(available_keys), 1, sharex=True, figsize=(8.67, 2.3 * len(available_keys) + 1.0))
    axes_list = [axes] if len(available_keys) == 1 else list(axes)
    fig.suptitle("Sitzungsübersicht Messwerte und Folgeschätzungen", x=0.08, y=0.985, ha="left", fontsize=13, color="#0f172a")
    fig.text(0.08, 0.955, _session_summary_subtitle(session_dir, entries), ha="left", va="top", fontsize=9.2, color="#475569")

    for ax, key in zip(axes_list, available_keys, strict=True):
        if key == "post_close":
            _plot_compare_series(
                ax,
                entries,
                left_field="measured_post_close_gain_g",
                left_label="Gemessener Nachlauf",
                left_color=SESSION_LINE_COLORS["measured_post_close_gain_g"],
                right_field="next_post_close_gain_g",
                right_label="Nächster Nachlauf",
                right_color=SESSION_LINE_COLORS["next_post_close_gain_g"],
                ylabel="Nachlauf [g]",
                title="Nachlauf: gemessen vs. nächster Schätzwert",
            )
        elif key == "dead_time":
            _plot_compare_series(
                ax,
                entries,
                left_field="measured_dead_time_s",
                left_label="Gemessene Totzeit",
                left_color=SESSION_LINE_COLORS["measured_dead_time_s"],
                right_field="next_dead_time_s",
                right_label="Nächste Totzeit",
                right_color=SESSION_LINE_COLORS["next_dead_time_s"],
                ylabel="Totzeit [s]",
                title="Totzeit: Messung vs. nächster Schätzwert",
            )
        else:
            plotted_fast = _plot_compare_series(
                ax,
                entries,
                left_field="measured_fast_rate_gps",
                left_label="Gemessene schnelle Rate",
                left_color=SESSION_LINE_COLORS["measured_fast_rate_gps"],
                right_field="next_fast_rate_gps",
                right_label="Nächste schnelle Rate",
                right_color=SESSION_LINE_COLORS["next_fast_rate_gps"],
                ylabel="Füllrate [g/s]",
                title="Füllraten: Messung vs. nächster Schätzwert",
            )
            slow_left_x, slow_left_y = _summary_series(entries, "measured_slow_rate_gps")
            slow_right_x, slow_right_y = _summary_series(entries, "next_slow_rate_gps")
            if slow_left_x:
                ax.plot(
                    slow_left_x,
                    slow_left_y,
                    color=SESSION_LINE_COLORS["measured_slow_rate_gps"],
                    linewidth=1.6,
                    marker="^",
                    markersize=4.0,
                    label="Gemessene reduzierte Rate",
                )
            if slow_right_x:
                ax.plot(
                    slow_right_x,
                    slow_right_y,
                    color=SESSION_LINE_COLORS["next_slow_rate_gps"],
                    linewidth=1.6,
                    linestyle=":",
                    marker="v",
                    markersize=4.0,
                    label="Nächste reduzierte Rate",
                )
            if plotted_fast or slow_left_x or slow_right_x:
                ax.legend(frameon=False, loc="upper left", ncol=2)

    _setup_session_axes(axes_list, x_ticks)
    fig.tight_layout(rect=(0.06, 0.06, 0.98, 0.93))
    return _export_session_figure(
        fig,
        output_dir,
        session_dir=session_dir,
        suffix="session-adaptation",
        formats=formats,
    )


def _plot_session_cascade_trend(
    session_dir: Path,
    entries: list[dict[str, Any]],
    output_dir: Path,
    formats: list[str],
) -> list[Path]:
    """Cross-run learning trend of the cascade plant-model parameters: one panel
    per parameter, each showing the value the fill ran with ("used", solid) and
    the value learned by the end of that fill ("next", dashed). This is the
    "do the parameters actually converge across fills?" chart for the thesis."""
    available = [
        m for m in CASCADE_SUMMARY_METRICS
        if _summary_series(entries, m[0])[0] or _summary_series(entries, m[1])[0]
    ]
    if not available:
        return []

    x_ticks = [int(entry["fill_id"]) for entry in entries]
    fig, axes = plt.subplots(len(available), 1, sharex=True, figsize=(8.67, 2.05 * len(available) + 1.0))
    axes_list = [axes] if len(available) == 1 else list(axes)
    fig.suptitle("Sitzungsübersicht: gelernte Streckenparameter (Kaskade)", x=0.08, y=0.985,
                 ha="left", fontsize=13, color="#0f172a")
    fig.text(0.08, 0.955, _session_summary_subtitle(session_dir, entries), ha="left", va="top",
             fontsize=9.2, color="#475569")

    for ax, (used_field, next_field, ylabel, title) in zip(axes_list, available, strict=True):
        _plot_compare_series(
            ax, entries,
            left_field=used_field, left_label="verwendet (used)",
            left_color=SESSION_LINE_COLORS.get(used_field, "#b45309"),
            right_field=next_field, right_label="gelernt (next)",
            right_color=SESSION_LINE_COLORS.get(next_field, "#f59e0b"),
            ylabel=ylabel, title=title,
        )

    _setup_session_axes(axes_list, x_ticks)
    fig.tight_layout(rect=(0.06, 0.06, 0.98, 0.93))
    return _export_session_figure(fig, output_dir, session_dir=session_dir,
                                  suffix="session-cascade-params", formats=formats)


def _plot_session_cascade_timeline(
    session_dir: Path,
    fill_runs: list[FillRun],
    fill_records: dict[int, list[dict[str, Any]]],
    output_dir: Path,
    formats: list[str],
) -> list[Path]:
    """Continuous within-session timeline of the cascade parameters at per-sample
    (sub-fill) resolution: shows the model adapting WITHIN each fill and across
    fills, with fill boundaries marked and each steady observation flagged. This
    is what reveals whether the estimator is actually being fed (observation
    ticks) and how K/onset move when it is."""
    ordered = sorted(fill_runs, key=lambda fr: fr.fill_id)
    t: list[float] = []
    K: list[float | None] = []
    onset: list[float | None] = []
    tau: list[float | None] = []
    meas: list[float | None] = []
    model: list[float | None] = []
    target: list[float | None] = []
    boundaries: list[tuple[float, int]] = []
    obs_t: list[float] = []
    obs_gate: list[float] = []
    obs_rate: list[float] = []

    clock = 0.0
    t0_us: int | None = None
    prev_obs_count: float | None = None
    for fr in ordered:
        samples = _sample_events(fill_records.get(fr.fill_id, []))
        if not samples:
            continue
        if t0_us is None:
            t0_us = _ts_us(samples[0])
        start = (_ts_us(samples[0]) - t0_us) / 1e6
        boundaries.append((start, fr.fill_id))
        for rec in samples:
            ts = (_ts_us(rec) - t0_us) / 1e6
            t.append(ts)
            K.append(_float_value(rec, "gate_gain_gps_per_pct"))
            onset.append(_float_value(rec, "flow_onset_gate_pct"))
            tau.append(_float_value(rec, "model_tau_s"))
            meas.append(_float_value(rec, "control_rate_gps"))
            model.append(_float_value(rec, "model_rate_gps"))
            target.append(_float_value(rec, "target_rate_gps"))
            count = _float_value(rec, "gain_obs_count")
            g = _float_value(rec, "gain_obs_gate_pct")
            r = _float_value(rec, "gain_obs_rate_gps")
            if count is not None and prev_obs_count is not None and count > prev_obs_count and g and r:
                obs_t.append(ts)
                obs_gate.append(g)
                obs_rate.append(r)
            if count is not None:
                prev_obs_count = count
        clock = t[-1] if t else clock

    if not t or all(v is None for v in K):
        return []

    fig, (axK, axT, axR) = plt.subplots(3, 1, sharex=True, figsize=(10.5, 7.4))
    fig.suptitle("Sitzungsverlauf: Streckenmodell über die Zeit (Kaskade)", x=0.06, y=0.985,
                 ha="left", fontsize=13, color="#0f172a")
    fig.text(0.06, 0.955, f"Sitzung: {session_dir.name}", ha="left", va="top", fontsize=9.2, color="#475569")

    def _mark_fills(ax: Any) -> None:
        for bx, fid in boundaries:
            ax.axvline(bx, color="#cbd5e1", linewidth=0.8, zorder=0)
        top = ax.get_ylim()[1]
        for bx, fid in boundaries:
            ax.annotate(f"#{fid}", xy=(bx, top), xytext=(2, -2), textcoords="offset points",
                        fontsize=7, color="#94a3b8", va="top")

    # Panel 1: K on the left axis, onset on a twin axis, observation ticks.
    axK.plot(t, K, color="#b45309", linewidth=1.7, label="K̂ [g/s pro %]")
    axK.set_ylabel("K̂ [g/s pro %]", color="#b45309")
    axK.tick_params(axis="y", labelcolor="#b45309")
    axK_o = axK.twinx()
    axK_o.plot(t, onset, color="#6d28d9", linewidth=1.4, linestyle="--", label="onset [%]")
    axK_o.set_ylabel("Totwinkel onset [%]", color="#6d28d9")
    axK_o.tick_params(axis="y", labelcolor="#6d28d9")
    if obs_t:
        axK.plot(obs_t, [K[min(range(len(t)), key=lambda i: abs(t[i] - ot))] for ot in obs_t],
                 linestyle="none", marker="v", markersize=6, color="#dc2626",
                 label=f"Beobachtung ({len(obs_t)})", zorder=5)
    axK.set_title("Verstärkung K̂ und Totwinkel (Beobachtungen = rote Marker)", loc="left",
                  fontsize=10.5, color="#0f172a")
    _mark_fills(axK)
    h1, l1 = axK.get_legend_handles_labels()
    h2, l2 = axK_o.get_legend_handles_labels()
    axK.legend(h1 + h2, l1 + l2, frameon=False, loc="upper right", ncol=3, fontsize=8)

    # Panel 2: tau.
    axT.plot(t, tau, color="#0e7490", linewidth=1.7)
    axT.set_ylabel("τ [s]")
    axT.set_title("Zeitkonstante τ (FOPDT)", loc="left", fontsize=10.5, color="#0f172a")
    _mark_fills(axT)

    # Panel 3: measured control rate vs model prediction vs target.
    axR.plot(t, meas, color="#0f172a", linewidth=1.3, label="gemessen (control_rate)")
    axR.plot(t, model, color="#2563eb", linewidth=1.3, linestyle="--", label="Modell ŷ")
    axR.plot(t, target, color="#16a34a", linewidth=1.1, linestyle=":", label="Sollrate ṁ*")
    axR.set_ylabel("Rate [g/s]")
    axR.set_xlabel("Zeit in der Sitzung [s]")
    axR.set_title("Ratenverfolgung: Modell ŷ vs. gemessen vs. Sollwert", loc="left",
                  fontsize=10.5, color="#0f172a")
    axR.legend(frameon=False, loc="upper right", ncol=3, fontsize=8)
    _mark_fills(axR)

    for ax in (axK, axT, axR):
        ax.grid(True, axis="both", color="#e2e8f0", linewidth=0.6, alpha=0.6)
        ax.set_axisbelow(True)
    fig.tight_layout(rect=(0.04, 0.04, 0.98, 0.93))
    return _export_session_figure(fig, output_dir, session_dir=session_dir,
                                  suffix="session-cascade-timeline", formats=formats)


def _plot_session_summaries(
    session_dir: Path,
    fill_runs: list[FillRun],
    fill_records: dict[int, list[dict[str, Any]]],
    output_dir: Path,
    formats: list[str],
    *,
    groups: list[str],
) -> tuple[list[Path], list[dict[str, Any]]]:
    entries = _session_summary_entries(fill_runs, fill_records)
    if not entries:
        return [], []

    _figure_style()
    exported: list[Path] = []
    if "overview" in groups:
        exported.extend(_plot_session_overview(session_dir, entries, output_dir, formats))
    if "adaptive" in groups:
        exported.extend(_plot_session_adaptive(session_dir, entries, output_dir, formats))
    if "compare" in groups:
        exported.extend(_plot_session_compare(session_dir, entries, output_dir, formats))
    if "cascade" in groups:
        exported.extend(_plot_session_cascade_trend(session_dir, entries, output_dir, formats))
        exported.extend(_plot_session_cascade_timeline(session_dir, fill_runs, fill_records, output_dir, formats))
    return exported, entries


def _print_session_summary_console(session_dir: Path, entries: list[dict[str, Any]]) -> None:
    if not entries:
        print("Session summary: no matching fill_summary records found.")
        return

    error_values = [abs(value) for value in (_float_value(entry["summary"], "fill_error_g") for entry in entries) if value is not None]
    duration_values = [value for value in (_float_value(entry["summary"], "fill_duration_s") for entry in entries) if value is not None]
    refill_values = [int(value) for value in (_float_value(entry["summary"], "refill_count") for entry in entries) if value is not None]

    first_summary = entries[0]["summary"]
    last_summary = entries[-1]["summary"]
    first_error = _float_value(first_summary, "fill_error_g")
    last_error = _float_value(last_summary, "fill_error_g")
    first_duration = _float_value(first_summary, "fill_duration_s")
    last_duration = _float_value(last_summary, "fill_duration_s")

    mean_abs_error = sum(error_values) / len(error_values) if error_values else None
    mean_duration = sum(duration_values) / len(duration_values) if duration_values else None
    total_refills = sum(refill_values)

    print()
    print(f"Session summary: {session_dir.name}")
    print(f"  Included fills:       {len(entries)}")
    print(f"  Mean abs. error:      {_format_series_value(mean_abs_error, 'g', 2)}")
    print(f"  Mean fill duration:   {_format_series_value(mean_duration, 's', 2)}")
    print(f"  Total refills:        {total_refills}")
    print(
        "  First -> last error:  "
        f"{_format_series_value(first_error, 'g', 2)} -> {_format_series_value(last_error, 'g', 2)}"
    )
    print(
        "  First -> last Dauer:  "
        f"{_format_series_value(first_duration, 's', 2)} -> {_format_series_value(last_duration, 's', 2)}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Create thesis-ready charts for individual jar fill runs. "
            "Input can be a session directory, a generated fills/ directory, or a "
            "directory below a session."
        )
    )
    parser.add_argument("input_path", type=Path, help="Session folder or fills/ folder")
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Figure output folder (default: <session>/figures)",
    )
    parser.add_argument(
        "--clear-output",
        dest="clear_output",
        action="store_true",
        help="Clear the figure output folder before exporting charts (default)",
    )
    parser.add_argument(
        "--keep-output",
        dest="clear_output",
        action="store_false",
        help="Keep existing figure files in the output folder",
    )
    parser.add_argument(
        "--pre-ms",
        type=int,
        help="Only relevant when plotting directly from a session without fills/index.json",
    )
    parser.add_argument(
        "--post-ms",
        type=int,
        help="Only relevant when plotting directly from a session without fills/index.json",
    )
    parser.add_argument(
        "--fills",
        type=_parse_fill_selection,
        help="Comma-separated fill ids to export, default: all detected fills",
    )
    parser.add_argument(
        "--format",
        action="append",
        choices=["pdf", "svg", "png"],
        help="Export format, repeatable. Default: pdf",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="Only list detected fill runs and exit",
    )
    parser.add_argument(
        "--legend-placement",
        choices=["inside", "outside"],
        default="outside",
        help="Place the one-line legend inside the chart or outside below it",
    )
    parser.add_argument(
        "--state-style",
        choices=["background", "band", "none"],
        default="none",
        help="Visualize firmware states as the existing full-height background, a compact top band, or disable it",
    )
    parser.add_argument(
        "--show-rate",
        type=_parse_show_rate,
        default="none",
        help=(
            "Show rate telemetry as none, all, or a comma list such as "
            "raw,filtered,medium"
        ),
    )
    parser.add_argument(
        "--rate-layout",
        choices=["subplot", "overlay"],
        default="subplot",
        help="Draw rate telemetry in a dedicated subplot or overlaid with a separate right axis",
    )
    parser.add_argument(
        "--figure-profile",
        choices=sorted(FIGURE_PROFILES.keys()),
        default="default",
        help="Use the normal thesis chart size or a narrower 16:10 variant for side-by-side placement",
    )
    parser.add_argument(
        "--cascade-control-export",
        choices=["separate", "merged"],
        default="separate",
        help=(
            "For flow-cascade plain exports: create a separate control-internals figure "
            "or merge that panel into the main fill chart. Debug export is always merged."
        ),
    )
    parser.add_argument(
        "--hide-target",
        action="store_true",
        help="Disable target-mass line and tolerance band overlays",
    )
    parser.add_argument(
        "--mass-y-max",
        type=float,
        help="Force the fill-mass axis upper limit, for example 200",
    )
    parser.add_argument(
        "--rate-height-scale",
        type=float,
        default=1.0,
        help="Multiply the dedicated rate subplot height, for example 1.5",
    )
    parser.add_argument(
        "--plain-both-profiles",
        action="store_true",
        help="Export the plain chart in both wide and narrow variants with suffixed filenames; debug is exported once",
    )
    parser.add_argument(
        "--session-summary",
        dest="session_summary",
        action="store_true",
        help="Export additional session-level summary figures based on fill_summary records (default)",
    )
    parser.add_argument(
        "--no-session-summary",
        dest="session_summary",
        action="store_false",
        help="Disable session-level summary figure export",
    )
    parser.add_argument(
        "--session-summary-groups",
        type=_parse_session_summary_groups,
        default=SESSION_SUMMARY_GROUP_ORDER.copy(),
        help=(
            "Comma list of session summary groups to export: "
            "overview,adaptive,compare, or all"
        ),
    )
    parser.set_defaults(clear_output=True, session_summary=True)
    args = parser.parse_args()
    if args.rate_height_scale <= 0:
        raise SystemExit("--rate-height-scale must be > 0")
    if args.mass_y_max is not None and args.mass_y_max <= 0:
        raise SystemExit("--mass-y-max must be > 0")

    session_dir, fill_runs, fill_records = _load_input(
        args.input_path,
        pre_ms=args.pre_ms,
        post_ms=args.post_ms,
    )
    if not fill_runs:
        raise SystemExit("No fill runs detected.")

    if args.list:
        print(f"Session: {session_dir}")
        for fill_run in fill_runs:
            print(format_fill_brief(fill_run))
        return 0

    selected_ids = set(args.fills or [fill_run.fill_id for fill_run in fill_runs])
    selected_fills = [fill_run for fill_run in fill_runs if fill_run.fill_id in selected_ids]
    if not selected_fills:
        raise SystemExit("No matching fill ids selected.")

    output_dir = args.output_dir.resolve() if args.output_dir else session_dir / "figures"
    formats = args.format or ["pdf"]
    cleared_output = False
    if args.clear_output and output_dir.exists():
        shutil.rmtree(output_dir)
        cleared_output = True
    command_text = _command_text([sys.executable, *sys.argv]) if sys.argv else _command_text([sys.executable])
    command_log_path = _write_command_log(session_dir, command_text)
    exported: list[Path] = []
    for fill_run in selected_fills:
        exported.extend(
            _plot_fill(
                session_dir=session_dir,
                fill_run=fill_run,
                records=fill_records[fill_run.fill_id],
                output_dir=output_dir,
                formats=formats,
                legend_placement=args.legend_placement,
                state_style=args.state_style,
                show_rate=args.show_rate,
                rate_layout=args.rate_layout,
                figure_profile=args.figure_profile,
                plain_both_profiles=args.plain_both_profiles,
                cascade_control_export=args.cascade_control_export,
                show_target=not args.hide_target,
                mass_y_max=args.mass_y_max,
                rate_height_scale=args.rate_height_scale,
                command_text=command_text,
            )
        )
    session_summary_exported: list[Path] = []
    session_summary_entries: list[dict[str, Any]] = []
    if args.session_summary:
        session_summary_exported, session_summary_entries = _plot_session_summaries(
            session_dir=session_dir,
            fill_runs=selected_fills,
            fill_records=fill_records,
            output_dir=output_dir,
            formats=formats,
            groups=args.session_summary_groups,
        )
        exported.extend(session_summary_exported)

    print(f"Session:  {session_dir}")
    print(f"Output:   {output_dir}")
    print(f"Cleared:  {'yes' if cleared_output else 'no'}")
    print(f"Formats:  {', '.join(formats)}")
    print(f"Command:  {command_log_path}")
    print(f"Plotted:  {len(selected_fills)} fill runs")
    print("Variants: debug, plain")
    print(f"Legend:   {args.legend_placement}")
    print(f"Rate:     {args.show_rate}")
    print(f"Layout:   {args.rate_layout}")
    print(f"Cascade:  {args.cascade_control_export} plain export")
    print(f"Profile:  {args.figure_profile}")
    print(f"Plain:    {'wide+narrow' if args.plain_both_profiles else args.figure_profile}")
    print(f"Session summaries: {'on' if args.session_summary else 'off'}")
    if args.session_summary:
        print(f"Summary groups: {', '.join(args.session_summary_groups)}")
    print()
    for fill_run in selected_fills:
        print(format_fill_brief(fill_run))
    if args.session_summary:
        _print_session_summary_console(session_dir, session_summary_entries)
    print()
    for path in exported:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
