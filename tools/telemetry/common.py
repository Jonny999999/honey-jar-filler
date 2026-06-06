#!/usr/bin/env python3
from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any

TEL_PREFIX = "TEL "
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "data" / "telemetry"


def timestamp_slug() -> str:
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")


def ensure_session_dir(output_root: Path, session_name: str | None = None) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    base = session_name or timestamp_slug()
    session_dir = output_root / base
    suffix = 1
    while session_dir.exists():
        session_dir = output_root / f"{base}_{suffix:02d}"
        suffix += 1
    session_dir.mkdir(parents=True, exist_ok=False)
    return session_dir


def extract_tel_payload(line: str) -> str | None:
    if not line.startswith(TEL_PREFIX):
        return None
    payload = line[len(TEL_PREFIX):].strip()
    return payload or None


def parse_tel_payload(payload: str) -> dict[str, Any] | None:
    try:
        parsed = json.loads(payload)
    except json.JSONDecodeError:
        return None
    return parsed if isinstance(parsed, dict) else None

