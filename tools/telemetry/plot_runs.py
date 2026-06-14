#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
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
PLAIN_FIGURE_SIZE = (8.0, 5.0)
DEBUG_META_WIDTH = 5.2
LINE_FILL_MASS = "#c81e1e"
LINE_GATE = "#7e22ce"
LINE_TARGET = "#1f5f3a"
LINE_FAULT = "#b91c1c"
LINE_RATE_RAW = "#94a3b8"
LINE_RATE_FILTERED = "#2563eb"
STATE_BAND_EDGE = "#94a3b8"
STATE_BAND_KEYS = {"FILL", "DRIP_WAIT", "VERIFY_TARGET", "FAULT"}
RATE_AXIS_LABEL = "Füllrate [g/s]"


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
    if width_s < 0.35:
        return None
    if width_s >= 1.15:
        duration = f"{width_s:.2f}".replace(".", ",")
        return f"{label}\n({duration} s)"
    return label


def _series_from_samples(
    fill_run: FillRun,
    samples: list[dict[str, Any]],
) -> tuple[
    list[float],
    list[float | None],
    list[float | None],
    list[float | None],
    list[float | None],
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
    raw_rate = [_float_value(record, "rate_raw_gps") for record in samples]
    filtered_rate = [_float_value(record, "rate_filtered_gps") for record in samples]
    return x_samples, fill_mass, gate_pct, raw_rate, filtered_rate, base_weight_g


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
    ax: Any,
    fill_run: FillRun,
    states: list[dict[str, Any]],
) -> None:
    if not states:
        return

    span_end_us = fill_run.window_end_us
    band_y = 0.91
    band_h = 0.065
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

    for state_name, x0, x1 in segments:
        width = max(0.0, x1 - x0)
        if width <= 0.0:
            continue

        rect = Rectangle(
            (x0, band_y),
            width,
            band_h,
            transform=ax.get_xaxis_transform(),
            facecolor=STATE_COLORS.get(state_name, "#e2e8f0"),
            edgecolor=STATE_BAND_EDGE,
            linewidth=0.6,
            alpha=0.95,
            zorder=1,
            clip_on=False,
        )
        ax.add_patch(rect)

        label = _format_state_band_label(state_name, width)
        if label is None:
            continue
        ax.text(
            x0 + width / 2.0,
            band_y + band_h / 2.0,
            label,
            transform=ax.get_xaxis_transform(),
            ha="center",
            va="center",
            fontsize=7.0,
            color="#1e293b",
            zorder=2,
            clip_on=True,
            linespacing=0.9,
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
    for gate_record in gates:
        gate_pct = _float_value(gate_record, "gate_pct")
        if gate_pct is None:
            continue
        x_gate = _aligned_gate_x(fill_run, gate_record, samples, gate_pct)
        ax2.scatter([x_gate], [gate_pct], color=LINE_GATE, s=20, zorder=6)
        label = f"{gate_pct:.0f} %"
        ax2.annotate(
            label,
            (x_gate, gate_pct),
            textcoords="offset points",
            xytext=(-8, 0),
            ha="right",
            va="center",
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
        fontsize=9.5,
        color="#0f172a",
        fontweight="bold",
    )
    y = y_start - 0.05
    for label, value in entries:
        meta_ax.text(x_label, y, f"{label}:", ha="left", va="top", fontsize=8.2, color="#475569")
        meta_ax.text(x_value, y, value, ha="left", va="top", fontsize=8.2, color="#0f172a")
        y -= 0.042
    return y - 0.03


def _draw_debug_metadata(
    meta_ax: Any,
    session_dir: Path,
    fill_run: FillRun,
    base_weight_g: float | None,
    records: list[dict[str, Any]],
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
                ("Rate schnell", _format_meta_value(_float_value(summary, "measured_fast_rate_gps"), "g/s", 1)),
                ("Rate reduziert", _format_meta_value(_float_value(summary, "measured_slow_rate_gps"), "g/s", 1)),
                ("Rate beim Schließen", _format_meta_value(_float_value(summary, "rate_at_close_gps"), "g/s", 1)),
                ("Nachtropfen Ist", _format_meta_value(_float_value(summary, "drip_wait_used_ms"), "ms", 0)),
            ]
        )
        adaptive_next_lines.extend(
            [
                ("Nächste Totzeit", _format_meta_value(_float_value(summary, "next_dead_time_s"), "s", 3)),
                ("Nächster Nachlauf", _format_meta_value(_float_value(summary, "next_post_close_gain_g"), "g", 1)),
                ("Nächste Rate schnell", _format_meta_value(_float_value(summary, "next_fast_rate_gps"), "g/s", 1)),
                ("Nächste Rate reduziert", _format_meta_value(_float_value(summary, "next_slow_rate_gps"), "g/s", 1)),
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
    meta_ax.text(0.0, 0.955, "Session", ha="left", va="top", fontsize=8.2, color="#475569")
    meta_ax.text(0.0, 0.925, wrapped_session, ha="left", va="top", fontsize=8.2, color="#0f172a", linespacing=1.05)
    session_lines = max(1, wrapped_session.count("\n") + 1)
    y_after_session = 0.925 - (session_lines * 0.042) - 0.045

    left_y = y_after_session
    right_y = y_after_session
    left_y = _draw_meta_section(meta_ax, "Lauf", general_lines, x_label=0.0, x_value=0.30, y_start=left_y)
    left_y = _draw_meta_section(meta_ax, "Ergebnis", result_lines, x_label=0.0, x_value=0.30, y_start=left_y)
    _draw_meta_section(meta_ax, "Preset-Parameter", preset_lines, x_label=0.0, x_value=0.30, y_start=left_y)
    right_y = _draw_meta_section(meta_ax, "Laufzeitwerte", runtime_lines, x_label=0.56, x_value=0.86, y_start=right_y)
    _draw_meta_section(meta_ax, "Adaptionswerte", adaptive_next_lines, x_label=0.56, x_value=0.86, y_start=right_y)


def _create_figure_axes(*, debug: bool, show_rate: str, rate_layout: str) -> tuple[Any, Any, Any | None, Any | None]:
    want_rate = show_rate != "none"
    if debug and want_rate and rate_layout == "subplot":
        fig = plt.figure(
            figsize=(PLAIN_FIGURE_SIZE[0] + DEBUG_META_WIDTH, PLAIN_FIGURE_SIZE[1]),
            constrained_layout=True,
        )
        gs = fig.add_gridspec(
            2,
            2,
            width_ratios=[PLAIN_FIGURE_SIZE[0], DEBUG_META_WIDTH],
            height_ratios=[3.8, 1.15],
        )
        ax = fig.add_subplot(gs[0, 0])
        rate_ax = fig.add_subplot(gs[1, 0], sharex=ax)
        meta_ax = fig.add_subplot(gs[:, 1])
        meta_ax.axis("off")
        return fig, ax, rate_ax, meta_ax
    if debug:
        fig, (ax, meta_ax) = plt.subplots(
            ncols=2,
            figsize=(PLAIN_FIGURE_SIZE[0] + DEBUG_META_WIDTH, PLAIN_FIGURE_SIZE[1]),
            gridspec_kw={"width_ratios": [PLAIN_FIGURE_SIZE[0], DEBUG_META_WIDTH]},
            constrained_layout=True,
        )
        meta_ax.axis("off")
        if want_rate and rate_layout == "overlay":
            rate_ax = ax.twinx()
            return fig, ax, rate_ax, meta_ax
        return fig, ax, None, meta_ax
    if want_rate and rate_layout == "subplot":
        fig = plt.figure(figsize=PLAIN_FIGURE_SIZE, constrained_layout=True)
        gs = fig.add_gridspec(2, 1, height_ratios=[3.8, 1.15])
        ax = fig.add_subplot(gs[0, 0])
        rate_ax = fig.add_subplot(gs[1, 0], sharex=ax)
        return fig, ax, rate_ax, None
    if want_rate and rate_layout == "overlay":
        fig, ax = plt.subplots(figsize=PLAIN_FIGURE_SIZE, constrained_layout=True)
        rate_ax = ax.twinx()
        return fig, ax, rate_ax, None
    fig, ax = plt.subplots(figsize=PLAIN_FIGURE_SIZE, constrained_layout=True)
    return fig, ax, None, None


def _plot_rate_panel(
    rate_ax: Any,
    x_samples: list[float],
    raw_rate: list[float | None],
    filtered_rate: list[float | None],
    *,
    show_rate: str,
    overlay: bool,
) -> None:
    plotted_any = False
    if show_rate in {"both"}:
        if any(value is not None for value in raw_rate):
            rate_ax.plot(
                x_samples,
                raw_rate,
                color=LINE_RATE_RAW,
                linewidth=1.0 if overlay else 1.0,
                label="Rohrate [g/s]",
                zorder=6 if overlay else 2,
                alpha=0.85 if overlay else 1.0,
            )
            plotted_any = True
    if show_rate in {"filtered", "both"}:
        if any(value is not None for value in filtered_rate):
            rate_ax.plot(
                x_samples,
                filtered_rate,
                color=LINE_RATE_FILTERED,
                linewidth=1.2 if overlay else 1.6,
                label="Gefilterte Rate [g/s]",
                zorder=7 if overlay else 3,
                alpha=0.9 if overlay else 1.0,
            )
            plotted_any = True
    if overlay:
        rate_ax.set_ylabel(RATE_AXIS_LABEL, color=LINE_RATE_FILTERED)
    else:
        rate_ax.set_ylabel(RATE_AXIS_LABEL)
    if not overlay:
        rate_ax.set_xlabel("Zeit relativ zum Füllbeginn [s]")
        rate_ax.grid(True, axis="both", color="#cbd5e1", linewidth=0.6, alpha=0.55)
        rate_ax.set_axisbelow(True)
    else:
        rate_ax.spines["right"].set_position(("axes", 1.10))
        rate_ax.tick_params(axis="y", colors=LINE_RATE_FILTERED, labelsize=9)
        rate_ax.spines["right"].set_color(LINE_RATE_FILTERED)
    if plotted_any and not overlay:
        rate_ax.legend(
            frameon=False,
            loc="upper right",
            ncol=2 if not overlay else 1,
            borderaxespad=0.2,
        )
    else:
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
) -> list[Path]:
    samples = _sample_events(records)
    if not samples:
        raise ValueError(f"fill {fill_run.fill_id} has no sample records")

    states = _state_events(records)
    gates = _gate_events(records)
    faults = _fault_events(records)
    x_samples, y_fill_mass, y_gate, raw_rate, filtered_rate, base_weight_g = _series_from_samples(fill_run, samples)

    _figure_style()
    fig, ax, rate_ax, meta_ax = _create_figure_axes(debug=debug, show_rate=show_rate, rate_layout=rate_layout)
    ax2 = ax.twinx()
    overlay_rate = rate_ax is not None and show_rate != "none" and rate_layout == "overlay"

    if state_style == "background":
        _plot_state_background(ax, fill_run, states, show_labels=False)
    elif state_style == "band":
        _plot_state_band(ax, fill_run, states)
    if debug:
        _annotate_state_badges(ax, fill_run, states)

    if fill_run.target_g is not None:
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

    if rate_ax is None or overlay_rate:
        ax.set_xlabel("Zeit relativ zum Füllbeginn [s]")
    else:
        ax.tick_params(labelbottom=False)
    ax.set_ylabel("Füllmasse [g]")
    ax2.set_ylabel("Klappenstellung [%]")
    ax2.set_ylim(-2, 102)
    ax.grid(True, axis="both", color="#cbd5e1", linewidth=0.7, alpha=0.65)
    ax.set_axisbelow(True)

    if rate_ax is not None:
        _plot_rate_panel(
            rate_ax,
            x_samples,
            raw_rate,
            filtered_rate,
            show_rate=show_rate,
            overlay=overlay_rate,
        )

    handles1, labels1 = ax.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    handles3: list[Any] = []
    labels3: list[str] = []
    if overlay_rate:
        handles3, labels3 = rate_ax.get_legend_handles_labels()
    legend_kwargs = {
        "handles": handles1 + handles2 + handles3,
        "labels": labels1 + labels2 + labels3,
        "frameon": False,
        "ncol": 3,
        "borderaxespad": 0.0,
    }
    if legend_placement == "inside":
        legend_kwargs.update(
            {
                "loc": "upper left",
                "bbox_to_anchor": (0.015, 0.905 if state_style == "band" else 0.985),
            }
        )
    else:
        legend_kwargs.update(
            {
                "loc": "upper center",
                "bbox_to_anchor": (0.5, -0.14 if not overlay_rate else -0.16),
            }
        )
    ax.legend(**legend_kwargs)

    if debug and meta_ax is not None:
        _draw_debug_metadata(meta_ax, session_dir, fill_run, base_weight_g, records)

    exported: list[Path] = []
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = output_dir / f"fill_{fill_run.fill_id:04d}_{'debug' if debug else 'plain'}"
    for fmt in formats:
        out_path = stem.with_suffix(f".{fmt}")
        fig.savefig(out_path, dpi=180 if fmt == "png" else None, bbox_inches="tight")
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
) -> list[Path]:
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
    )
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
        )
    )
    return exported


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
        choices=["none", "filtered", "both"],
        default="none",
        help="Show fill-rate telemetry in addition to mass and gate traces",
    )
    parser.add_argument(
        "--rate-layout",
        choices=["subplot", "overlay"],
        default="subplot",
        help="Draw rate telemetry in a dedicated subplot or overlaid with a separate right axis",
    )
    args = parser.parse_args()

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
            )
        )

    print(f"Session:  {session_dir}")
    print(f"Output:   {output_dir}")
    print(f"Formats:  {', '.join(formats)}")
    print(f"Plotted:  {len(selected_fills)} fill runs")
    print("Variants: debug, plain")
    print(f"Legend:   {args.legend_placement}")
    print(f"Rate:     {args.show_rate}")
    print(f"Layout:   {args.rate_layout}")
    print()
    for fill_run in selected_fills:
        print(format_fill_brief(fill_run))
    print()
    for path in exported:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
