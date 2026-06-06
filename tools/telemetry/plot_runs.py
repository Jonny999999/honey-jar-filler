#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Reserved for thesis plotting of per-run telemetry files into vector "
            "PDF/SVG charts with event overlays."
        )
    )
    parser.add_argument("input_dir", type=Path, help="Session runs/ folder or similar")
    parser.add_argument("--output-dir", type=Path, help="Optional explicit figure output folder")
    parser.parse_args()

    print(
        "plot_runs.py is not implemented yet.\n"
        "Planned role: load per-run NDJSON, generate one chart per fill run by default,\n"
        "and later support overlays for comparing strategies or parameter sets."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

