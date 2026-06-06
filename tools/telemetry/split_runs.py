#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Reserved for offline re-splitting of telemetry.ndjson into per-run "
            "files when capture settings or pre/post windows should be changed."
        )
    )
    parser.add_argument("session_dir", type=Path, help="Session folder created by capture.py")
    parser.parse_args()

    print(
        "split_runs.py is not implemented yet.\n"
        "Current workflow: use capture.py, which already writes per-run files live.\n"
        "Later this script will rebuild runs/ from telemetry.ndjson with adjustable windows."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

