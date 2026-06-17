#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import re
import shutil
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
PLAIN_PROFILE_SUFFIX = {
    "default": "wide",
    "narrow": "narrow",
}
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
STATE_BAND_FIGURE_EXTRA_H = 0.42
STATE_BAND_HEIGHT_RATIO = 0.52
MAIN_PLOT_HEIGHT_RATIO = 3.8
RATE_PLOT_HEIGHT_RATIO = 1.7
STATE_BAND_LABEL_MIN_WIDTH_S = 0.35
STATE_BAND_LABEL_OUTSIDE_WIDTH_S = 1.45


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
    seen_zero_labels = 0
    for gate_record in gates:
        gate_pct = _float_value(gate_record, "gate_pct")
        if gate_pct is None:
            continue
        x_gate = _aligned_gate_x(fill_run, gate_record, samples, gate_pct)
        ax2.scatter([x_gate], [gate_pct], color=LINE_GATE, s=20, zorder=6)
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


def _create_figure_axes(
    *,
    debug: bool,
    show_rate: str,
    rate_layout: str,
    state_style: str,
    figure_profile: str,
) -> tuple[Any, Any | None, Any, Any | None, Any | None]:
    figure_width, figure_height_base = _figure_size(figure_profile)
    want_rate = show_rate != "none"
    want_band = state_style == "band"
    subplot_rate = want_rate and rate_layout == "subplot"

    figure_height = figure_height_base + (STATE_BAND_FIGURE_EXTRA_H if want_band else 0.0)
    left_rows = 1 + (1 if want_band else 0) + (1 if subplot_rate else 0)
    left_height_ratios: list[float] = []
    if want_band:
        left_height_ratios.append(STATE_BAND_HEIGHT_RATIO)
    left_height_ratios.append(MAIN_PLOT_HEIGHT_RATIO)
    if subplot_rate:
        left_height_ratios.append(RATE_PLOT_HEIGHT_RATIO)

    if debug:
        fig = plt.figure(
            figsize=(figure_width + DEBUG_META_WIDTH, figure_height),
            constrained_layout=True,
        )
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
        meta_ax = fig.add_subplot(gs[:, 1])
        meta_ax.axis("off")
        if want_rate and rate_layout == "overlay":
            return fig, band_ax, ax, ax.twinx(), meta_ax
        return fig, band_ax, ax, rate_ax, meta_ax

    fig = plt.figure(figsize=(figure_width, figure_height), constrained_layout=True)
    gs = fig.add_gridspec(left_rows, 1, height_ratios=left_height_ratios)
    main_row = 1 if want_band else 0
    ax = fig.add_subplot(gs[main_row, 0])
    band_ax = fig.add_subplot(gs[0, 0], sharex=ax) if want_band else None
    if subplot_rate:
        rate_ax = fig.add_subplot(gs[main_row + 1, 0], sharex=ax)
        return fig, band_ax, ax, rate_ax, None
    if want_rate and rate_layout == "overlay":
        return fig, band_ax, ax, ax.twinx(), None
    return fig, band_ax, ax, None, None


def _plot_rate_panel(
    rate_ax: Any,
    x_samples: list[float],
    raw_rate: list[float | None],
    filtered_rate: list[float | None],
    *,
    show_rate: str,
    overlay: bool,
) -> tuple[list[Any], list[str]]:
    plotted_any = False
    handles: list[Any] = []
    labels: list[str] = []
    if show_rate in {"both"}:
        if any(value is not None for value in raw_rate):
            (line_raw,) = rate_ax.plot(
                x_samples,
                raw_rate,
                color=LINE_RATE_RAW,
                linewidth=1.0 if overlay else 1.0,
                label="Rohrate [g/s]",
                zorder=6 if overlay else 2,
                alpha=0.85 if overlay else 1.0,
            )
            handles.append(line_raw)
            labels.append("Rohrate [g/s]")
            plotted_any = True
    if show_rate in {"filtered", "both"}:
        if any(value is not None for value in filtered_rate):
            filtered_label = "Gefilterte Füllrate [g/s]" if show_rate == "both" else "Füllrate [g/s]"
            (line_filtered,) = rate_ax.plot(
                x_samples,
                filtered_rate,
                color=LINE_RATE_FILTERED,
                linewidth=1.2 if overlay else 1.6,
                label=filtered_label,
                zorder=7 if overlay else 3,
                alpha=0.9 if overlay else 1.0,
            )
            handles.append(line_filtered)
            labels.append(filtered_label)
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
) -> list[Path]:
    samples = _sample_events(records)
    if not samples:
        raise ValueError(f"fill {fill_run.fill_id} has no sample records")

    states = _state_events(records)
    gates = _gate_events(records)
    faults = _fault_events(records)
    x_samples, y_fill_mass, y_gate, raw_rate, filtered_rate, base_weight_g = _series_from_samples(fill_run, samples)

    _figure_style()
    fig, band_ax, ax, rate_ax, meta_ax = _create_figure_axes(
        debug=debug,
        show_rate=show_rate,
        rate_layout=rate_layout,
        state_style=state_style,
        figure_profile=figure_profile,
    )
    ax2 = ax.twinx()
    overlay_rate = rate_ax is not None and show_rate != "none" and rate_layout == "overlay"

    if state_style == "background":
        _plot_state_background(ax, fill_run, states, show_labels=False)
    elif state_style == "band":
        _plot_state_band(band_ax, fill_run, states)
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

    ax.set_xlabel("" if (rate_ax is not None and not overlay_rate) else "Zeit relativ zum Füllbeginn [s]")
    ax.tick_params(labelbottom=True)
    ax.set_ylabel("Füllmasse [g]")
    ax2.set_ylabel("Klappenstellung [%]")
    ax2.set_ylim(-2, 112)
    ax2.set_yticks([0, 20, 40, 60, 80, 100])
    ax.grid(True, axis="both", color="#cbd5e1", linewidth=0.7, alpha=0.65)
    ax.set_axisbelow(True)

    handles3: list[Any] = []
    labels3: list[str] = []
    if rate_ax is not None:
        handles3, labels3 = _plot_rate_panel(
            rate_ax,
            x_samples,
            raw_rate,
            filtered_rate,
            show_rate=show_rate,
            overlay=overlay_rate,
        )
        if not overlay_rate:
            rate_ax.tick_params(labelbottom=True)

    handles1, labels1 = ax.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    legend_count = len(handles1) + len(handles2) + len(handles3)
    outside_legend_cols = max(1, legend_count)
    if figure_profile == "narrow" and not overlay_rate:
        outside_legend_cols = min(3, outside_legend_cols)
    legend_kwargs = {
        "handles": handles1 + handles2 + handles3,
        "labels": labels1 + labels2 + labels3,
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
    else:
        legend_y = -0.16 if overlay_rate else -0.54
        if not overlay_rate and figure_profile == "narrow":
            legend_y = -0.62
        if not overlay_rate and debug:
            legend_y -= 0.10
        legend_kwargs.update(
            {
                "loc": "upper center",
                "bbox_to_anchor": (0.5, legend_y),
            }
        )
        legend_target = rate_ax if (rate_ax is not None and not overlay_rate) else ax
        legend_target.legend(**legend_kwargs)

    if debug and meta_ax is not None:
        _draw_debug_metadata(meta_ax, session_dir, fill_run, base_weight_g, records)

    exported: list[Path] = []
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = output_dir / f"{_output_stem_base(session_dir, fill_run)}__{output_name}"
    for fmt in formats:
        out_path = stem.parent / f"{stem.name}.{fmt}"
        fig.savefig(out_path, dpi=180 if fmt == "png" else None)
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
        figure_profile=figure_profile,
        output_name="debug",
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
    parser.add_argument(
        "--figure-profile",
        choices=sorted(FIGURE_PROFILES.keys()),
        default="default",
        help="Use the normal thesis chart size or a narrower 16:10 variant for side-by-side placement",
    )
    parser.add_argument(
        "--plain-both-profiles",
        action="store_true",
        help="Export the plain chart in both wide and narrow variants with suffixed filenames; debug is exported once",
    )
    parser.set_defaults(clear_output=True)
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
    cleared_output = False
    if args.clear_output and output_dir.exists():
        shutil.rmtree(output_dir)
        cleared_output = True
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
            )
        )

    print(f"Session:  {session_dir}")
    print(f"Output:   {output_dir}")
    print(f"Cleared:  {'yes' if cleared_output else 'no'}")
    print(f"Formats:  {', '.join(formats)}")
    print(f"Plotted:  {len(selected_fills)} fill runs")
    print("Variants: debug, plain")
    print(f"Legend:   {args.legend_placement}")
    print(f"Rate:     {args.show_rate}")
    print(f"Layout:   {args.rate_layout}")
    print(f"Profile:  {args.figure_profile}")
    print(f"Plain:    {'wide+narrow' if args.plain_both_profiles else args.figure_profile}")
    print()
    for fill_run in selected_fills:
        print(format_fill_brief(fill_run))
    print()
    for path in exported:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
