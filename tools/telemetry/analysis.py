#!/usr/bin/env python3
from __future__ import annotations

import json
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

FILL_STATE = "FILL"
STOP_STATES = {"FIND_SLOT", "IDLE", "DONE"}
IMPORTANT_PARAM_KEYS = [
    "VAR(target_grams)",
    "VAR(target_tol_low_g)",
    "VAR(target_tol_high_g)",
    "VAR(max_gate_pct)",
    "VAR(slow_remaining_g)",
    "VAR(slow_gate_pct)",
    "VAR(close_remaining_g)",
    "VAR(drip_delay_ms)",
]

PARAM_LABELS = {
    "VAR(target_grams)": "Target",
    "VAR(target_tol_low_g)": "Tol low",
    "VAR(target_tol_high_g)": "Tol high",
    "VAR(max_gate_pct)": "Max gate",
    "VAR(slow_remaining_g)": "Slow rem",
    "VAR(slow_gate_pct)": "Slow gate",
    "VAR(close_remaining_g)": "Close rem",
    "VAR(drip_delay_ms)": "Drip wait",
}

# Legacy parameter keys kept for backward compatibility with telemetry captured
# before the near-close/close-early parameters were renamed. Maps the current
# key to the historic key(s) that carried the same value in older log files.
PARAM_KEY_ALIASES = {
    "VAR(slow_remaining_g)": ["VAR(near_close_delta_g)"],
    "VAR(slow_gate_pct)": ["VAR(near_close_gate_pct)"],
    "VAR(close_remaining_g)": ["VAR(close_early_g)"],
}


def resolve_param(params: dict[str, Any], key: str) -> tuple[str, Any] | None:
    """Look up a parameter by its current key, falling back to legacy aliases.

    Returns the (present_key, value) that was found, or None if neither the
    current key nor any known legacy alias is present.
    """
    if key in params:
        return key, params[key]
    for legacy_key in PARAM_KEY_ALIASES.get(key, ()):
        if legacy_key in params:
            return legacy_key, params[legacy_key]
    return None


@dataclass(slots=True)
class FillRun:
    fill_id: int
    run_id: int
    slot_idx: int
    focus_start_us: int
    focus_end_us: int
    window_start_us: int
    window_end_us: int
    window_start_index: int
    window_end_index: int
    end_reason: str
    preset_name: str | None
    strategy_name: str | None
    target_g: float | None
    params: dict[str, Any]
    base_weight_g: float | None
    scale_period_ms_cfg: float | None
    fsm_period_ms_cfg: float | None
    scale_period_ms_avg: float | None
    scale_rate_hz_avg: float | None
    record_count: int
    sample_count: int
    start_weight_g: float | None
    end_weight_g: float | None
    fault_texts: list[str]

    @property
    def focus_duration_s(self) -> float:
        return max(0.0, (self.focus_end_us - self.focus_start_us) / 1_000_000.0)

    @property
    def window_duration_s(self) -> float:
        return max(0.0, (self.window_end_us - self.window_start_us) / 1_000_000.0)

    def to_manifest_entry(self) -> dict[str, Any]:
        return {
            "fill_id": self.fill_id,
            "file": f"fill_{self.fill_id:04d}.ndjson",
            "run_id": self.run_id,
            "slot_idx": self.slot_idx,
            "focus_start_us": self.focus_start_us,
            "focus_end_us": self.focus_end_us,
            "window_start_us": self.window_start_us,
            "window_end_us": self.window_end_us,
            "focus_duration_s": round(self.focus_duration_s, 3),
            "window_duration_s": round(self.window_duration_s, 3),
            "end_reason": self.end_reason,
            "preset_name": self.preset_name,
            "strategy_name": self.strategy_name,
            "target_g": self.target_g,
            "base_weight_g": self.base_weight_g,
            "scale_period_ms_cfg": self.scale_period_ms_cfg,
            "fsm_period_ms_cfg": self.fsm_period_ms_cfg,
            "scale_period_ms_avg": self.scale_period_ms_avg,
            "scale_rate_hz_avg": self.scale_rate_hz_avg,
            "record_count": self.record_count,
            "sample_count": self.sample_count,
            "start_weight_g": self.start_weight_g,
            "end_weight_g": self.end_weight_g,
            "fault_texts": self.fault_texts,
            "params": self.params,
        }

    @classmethod
    def from_manifest_entry(cls, entry: dict[str, Any]) -> "FillRun":
        return cls(
            fill_id=int(entry["fill_id"]),
            run_id=int(entry["run_id"]),
            slot_idx=int(entry["slot_idx"]),
            focus_start_us=int(entry["focus_start_us"]),
            focus_end_us=int(entry["focus_end_us"]),
            window_start_us=int(entry["window_start_us"]),
            window_end_us=int(entry["window_end_us"]),
            window_start_index=-1,
            window_end_index=-1,
            end_reason=str(entry.get("end_reason", "unknown")),
            preset_name=_opt_str(entry.get("preset_name")),
            strategy_name=_opt_str(entry.get("strategy_name")),
            target_g=_opt_float(entry.get("target_g")),
            params=_opt_dict(entry.get("params")),
            base_weight_g=_opt_float(entry.get("base_weight_g")),
            scale_period_ms_cfg=_opt_float(entry.get("scale_period_ms_cfg")),
            fsm_period_ms_cfg=_opt_float(entry.get("fsm_period_ms_cfg")),
            scale_period_ms_avg=_opt_float(entry.get("scale_period_ms_avg")),
            scale_rate_hz_avg=_opt_float(entry.get("scale_rate_hz_avg")),
            record_count=int(entry.get("record_count", 0)),
            sample_count=int(entry.get("sample_count", 0)),
            start_weight_g=_opt_float(entry.get("start_weight_g")),
            end_weight_g=_opt_float(entry.get("end_weight_g")),
            fault_texts=[str(value) for value in entry.get("fault_texts", [])],
        )


def _opt_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _opt_str(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _opt_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_ndjson(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                parsed = json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_no}: invalid JSON: {exc}") from exc
            if not isinstance(parsed, dict):
                raise ValueError(f"{path}:{line_no}: expected JSON object")
            records.append(parsed)
    return records


def load_session_meta(session_dir: Path) -> dict[str, Any]:
    meta_path = session_dir / "session_meta.json"
    if not meta_path.exists():
        return {}
    with meta_path.open("r", encoding="utf-8") as handle:
        parsed = json.load(handle)
    return parsed if isinstance(parsed, dict) else {}


def telemetry_path_for_session(session_dir: Path) -> Path:
    telemetry_path = session_dir / "telemetry.ndjson"
    if not telemetry_path.exists():
        raise FileNotFoundError(f"telemetry file not found: {telemetry_path}")
    return telemetry_path


def resolve_session_dir(path: Path) -> Path:
    candidate = path.resolve()
    if candidate.is_dir() and (candidate / "telemetry.ndjson").exists():
        return candidate
    if candidate.is_dir() and (candidate.parent / "telemetry.ndjson").exists():
        return candidate.parent
    raise FileNotFoundError(
        f"{path} is not a telemetry session directory and does not sit below one"
    )


def record_ts_us(record: dict[str, Any]) -> int:
    value = record.get("ts_us")
    if value is None:
        raise ValueError(f"record has no ts_us: {record}")
    return int(value)


def record_kind(record: dict[str, Any]) -> str:
    return str(record.get("kind", ""))


def record_text(record: dict[str, Any]) -> str:
    return str(record.get("text", "")).strip()


def latest_preset_name(
    preset_timeline: list[tuple[int, str]],
    timestamp_us: int,
) -> str | None:
    if not preset_timeline:
        return None
    times = [item[0] for item in preset_timeline]
    index = bisect_right(times, timestamp_us) - 1
    return preset_timeline[index][1] if index >= 0 else None


def build_fill_runs(
    records: list[dict[str, Any]],
    *,
    pre_ms: int,
    post_ms: int,
) -> list[FillRun]:
    if not records:
        return []

    timestamps = [record_ts_us(record) for record in records]
    first_ts = timestamps[0]
    last_ts = timestamps[-1]
    preset_timeline = [
        (record_ts_us(record), record_text(record))
        for record in records
        if record_kind(record) == "preset" and record_text(record)
    ]
    run_summaries = {
        int(record.get("run_id", 0)): record
        for record in records
        if record_kind(record) == "run_summary"
    }

    fill_runs: list[FillRun] = []
    next_start_index_allowed = 0
    for index, record in enumerate(records):
        if index < next_start_index_allowed:
            continue
        if record_kind(record) != "state" or record_text(record) != FILL_STATE:
            continue
        focus_start_us = timestamps[index]
        focus_end_us, end_reason, stop_index = _find_fill_end(records, timestamps, index, record)
        next_start_index_allowed = stop_index + 1
        next_fill_ts = _find_next_fill_start(records, timestamps, stop_index + 1)
        next_foreign_sample_ts = _find_next_foreign_slot_sample(
            records,
            timestamps,
            stop_index + 1,
            int(record.get("slot_idx", -1)),
        )

        window_start_us = max(first_ts, focus_start_us - pre_ms * 1_000)
        window_end_us = min(last_ts, focus_end_us + post_ms * 1_000)
        if next_fill_ts is not None:
            window_end_us = min(window_end_us, max(focus_end_us, next_fill_ts - 1))
        if next_foreign_sample_ts is not None:
            window_end_us = min(window_end_us, max(focus_end_us, next_foreign_sample_ts - 1))

        window_start_index = bisect_left(timestamps, window_start_us)
        window_end_index = bisect_right(timestamps, window_end_us) - 1
        if window_end_index < window_start_index:
            continue

        window_records = records[window_start_index : window_end_index + 1]
        sample_records = [item for item in window_records if record_kind(item) == "sample"]
        fill_start = _find_fill_start_metadata(records, index, stop_index, record)
        summary = run_summaries.get(int(record.get("run_id", 0)))
        preset_name = None
        strategy_name = None
        params: dict[str, Any] = {}
        if fill_start:
            preset_name = _opt_str(fill_start.get("preset_name"))
            strategy_name = _opt_str(fill_start.get("strategy_name"))
            params = _opt_dict(fill_start.get("params"))
        if summary and not params:
            preset_name = _opt_str(summary.get("preset_name"))
            strategy_name = _opt_str(summary.get("strategy_name"))
            params = _opt_dict(summary.get("params"))
        if summary:
            preset_name = preset_name or _opt_str(summary.get("preset_name"))
            strategy_name = strategy_name or _opt_str(summary.get("strategy_name"))
        if not preset_name:
            preset_name = latest_preset_name(preset_timeline, focus_start_us)
        target_g = _infer_target_g(record, sample_records, fill_start, summary)
        base_weight_g = _infer_base_weight_g(fill_start, sample_records, focus_start_us)
        scale_period_ms_cfg = _opt_float((fill_start or {}).get("scale_period_ms_cfg"))
        if scale_period_ms_cfg is None:
            scale_period_ms_cfg = _opt_float((summary or {}).get("scale_period_ms_cfg"))
        fsm_period_ms_cfg = _opt_float((fill_start or {}).get("fsm_period_ms_cfg"))
        if fsm_period_ms_cfg is None:
            fsm_period_ms_cfg = _opt_float((summary or {}).get("fsm_period_ms_cfg"))
        scale_period_ms_avg, scale_rate_hz_avg = _compute_scale_timing(sample_records, focus_start_us)
        fault_texts = [
            record_text(item)
            for item in window_records
            if record_kind(item) == "fault" and record_text(item)
        ]
        fill_runs.append(
            FillRun(
                fill_id=len(fill_runs) + 1,
                run_id=int(record.get("run_id", 0)),
                slot_idx=int(record.get("slot_idx", -1)),
                focus_start_us=focus_start_us,
                focus_end_us=focus_end_us,
                window_start_us=window_start_us,
                window_end_us=window_end_us,
                window_start_index=window_start_index,
                window_end_index=window_end_index,
                end_reason=end_reason,
                preset_name=preset_name,
                strategy_name=strategy_name,
                target_g=target_g,
                params=params,
                base_weight_g=base_weight_g,
                scale_period_ms_cfg=scale_period_ms_cfg,
                fsm_period_ms_cfg=fsm_period_ms_cfg,
                scale_period_ms_avg=scale_period_ms_avg,
                scale_rate_hz_avg=scale_rate_hz_avg,
                record_count=len(window_records),
                sample_count=len(sample_records),
                start_weight_g=_sample_weight_before_or_at(sample_records, focus_start_us),
                end_weight_g=_sample_weight_before_or_at(sample_records, focus_end_us),
                fault_texts=fault_texts,
            )
        )
    return fill_runs


def extract_fill_records(
    records: list[dict[str, Any]],
    fill_run: FillRun,
) -> list[dict[str, Any]]:
    if fill_run.window_start_index >= 0 and fill_run.window_end_index >= fill_run.window_start_index:
        return records[fill_run.window_start_index : fill_run.window_end_index + 1]
    return [
        record
        for record in records
        if fill_run.window_start_us <= record_ts_us(record) <= fill_run.window_end_us
    ]


def write_fill_manifest(
    session_dir: Path,
    output_dir: Path,
    fill_runs: list[FillRun],
    records: list[dict[str, Any]],
    *,
    pre_ms: int,
    post_ms: int,
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    for stale_file in output_dir.glob("fill_*.ndjson"):
        stale_file.unlink()

    for fill_run in fill_runs:
        fill_path = output_dir / f"fill_{fill_run.fill_id:04d}.ndjson"
        with fill_path.open("w", encoding="utf-8") as handle:
            for record in extract_fill_records(records, fill_run):
                handle.write(json.dumps(record, separators=(",", ":")))
                handle.write("\n")

    manifest = {
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "source_session": session_dir.name,
        "source_telemetry": "telemetry.ndjson",
        "pre_ms": pre_ms,
        "post_ms": post_ms,
        "fills": [fill_run.to_manifest_entry() for fill_run in fill_runs],
    }
    manifest_path = output_dir / "index.json"
    with manifest_path.open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)
        handle.write("\n")
    return manifest_path


def load_fill_manifest(fills_dir: Path) -> tuple[dict[str, Any], list[FillRun]]:
    manifest_path = fills_dir / "index.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"fill manifest not found: {manifest_path}")
    with manifest_path.open("r", encoding="utf-8") as handle:
        parsed = json.load(handle)
    if not isinstance(parsed, dict):
        raise ValueError(f"invalid fill manifest: {manifest_path}")
    fills_raw = parsed.get("fills")
    if not isinstance(fills_raw, list):
        raise ValueError(f"invalid fill list in manifest: {manifest_path}")
    fills = [FillRun.from_manifest_entry(entry) for entry in fills_raw if isinstance(entry, dict)]
    return parsed, fills


def format_fill_brief(fill_run: FillRun) -> str:
    preset = fill_run.preset_name or "?"
    target = "?" if fill_run.target_g is None else f"{fill_run.target_g:.0f} g"
    return (
        f"fill {fill_run.fill_id:02d} | run {fill_run.run_id} slot {fill_run.slot_idx} | "
        f"{fill_run.focus_duration_s:.2f} s | preset={preset} | target={target} | "
        f"end={fill_run.end_reason}"
    )


def important_params(fill_run: FillRun) -> list[tuple[str, Any]]:
    items: list[tuple[str, Any]] = []
    for key in IMPORTANT_PARAM_KEYS:
        resolved = resolve_param(fill_run.params, key)
        if resolved is not None:
            items.append((PARAM_LABELS.get(key, key), resolved[1]))
    return items


def _find_fill_end(
    records: list[dict[str, Any]],
    timestamps: list[int],
    start_index: int,
    start_record: dict[str, Any],
) -> tuple[int, str, int]:
    run_id = int(start_record.get("run_id", 0))
    slot_idx = int(start_record.get("slot_idx", -1))
    for index in range(start_index + 1, len(records)):
        record = records[index]
        kind = record_kind(record)
        if kind == "fill_summary":
            if int(record.get("run_id", 0)) != run_id:
                continue
            if int(record.get("slot_idx", -1)) != slot_idx:
                continue
            result = record_text(record) or "fill_summary"
            return timestamps[index], f"fill_summary:{result}", index
        if kind == "fault":
            if int(record.get("run_id", 0)) != run_id:
                continue
            if int(record.get("slot_idx", slot_idx)) != slot_idx:
                continue
            reason = record_text(record) or "fault"
            return timestamps[index], f"fault:{reason}", index
        if kind == "state":
            state_name = record_text(record)
            if int(record.get("run_id", run_id)) != run_id:
                continue
            if int(record.get("slot_idx", slot_idx)) != slot_idx:
                continue
            if state_name == FILL_STATE:
                continue
            if state_name == "FAULT":
                return timestamps[index], "state:FAULT", index
            if state_name in STOP_STATES:
                return timestamps[index], f"state:{state_name}", index
        if kind == "run_end":
            reason = record_text(record) or "run_end"
            return timestamps[index], f"run_end:{reason}", index
    return timestamps[-1], "session_end", len(records) - 1


def _find_next_fill_start(
    records: list[dict[str, Any]],
    timestamps: list[int],
    start_index: int,
) -> int | None:
    for index in range(start_index, len(records)):
        record = records[index]
        if record_kind(record) == "state" and record_text(record) == FILL_STATE:
            return timestamps[index]
    return None


def _find_next_foreign_slot_sample(
    records: list[dict[str, Any]],
    timestamps: list[int],
    start_index: int,
    slot_idx: int,
) -> int | None:
    for index in range(start_index, len(records)):
        record = records[index]
        if record_kind(record) != "sample":
            continue
        if int(record.get("slot_idx", slot_idx)) != slot_idx:
            return timestamps[index]
    return None


def _infer_target_g(
    start_record: dict[str, Any],
    sample_records: list[dict[str, Any]],
    fill_start: dict[str, Any] | None,
    summary: dict[str, Any] | None,
) -> float | None:
    if fill_start and fill_start.get("target_g") is not None:
        return _opt_float(fill_start.get("target_g"))
    if summary and summary.get("target_g") is not None:
        return _opt_float(summary.get("target_g"))
    for record in sample_records:
        if record.get("target_g") is not None:
            return _opt_float(record.get("target_g"))
    return _opt_float(start_record.get("target_g"))


def _infer_base_weight_g(
    fill_start: dict[str, Any] | None,
    sample_records: list[dict[str, Any]],
    focus_start_us: int,
) -> float | None:
    if fill_start and fill_start.get("base_weight_g") is not None:
        return _opt_float(fill_start.get("base_weight_g"))
    return _sample_weight_before_or_at(sample_records, focus_start_us)


def _sample_weight_before_or_at(
    sample_records: list[dict[str, Any]],
    timestamp_us: int,
) -> float | None:
    latest: float | None = None
    for record in sample_records:
        if record_ts_us(record) > timestamp_us:
            break
        latest = _opt_float(record.get("weight_g"))
    return latest


def _compute_scale_timing(
    sample_records: list[dict[str, Any]],
    focus_start_us: int,
) -> tuple[float | None, float | None]:
    relevant_ts = [
        record_ts_us(record)
        for record in sample_records
        if record_ts_us(record) >= focus_start_us
    ]
    if len(relevant_ts) < 2:
        return None, None
    deltas_us = [b - a for a, b in zip(relevant_ts, relevant_ts[1:]) if b > a]
    if not deltas_us:
        return None, None
    avg_us = sum(deltas_us) / len(deltas_us)
    avg_ms = avg_us / 1000.0
    hz = 1_000_000.0 / avg_us if avg_us > 0 else None
    return avg_ms, hz


def _find_fill_start_metadata(
    records: list[dict[str, Any]],
    start_index: int,
    stop_index: int,
    state_record: dict[str, Any],
) -> dict[str, Any] | None:
    run_id = int(state_record.get("run_id", 0))
    slot_idx = int(state_record.get("slot_idx", -1))
    for index in range(start_index, min(stop_index + 1, len(records))):
        record = records[index]
        if record_kind(record) != "fill_start":
            continue
        if int(record.get("run_id", 0)) != run_id:
            continue
        if int(record.get("slot_idx", -1)) != slot_idx:
            continue
        return record
    return None
