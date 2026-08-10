#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from analysis import (
    build_fill_runs,
    format_fill_brief,
    load_ndjson,
    load_session_meta,
    resolve_session_dir,
    telemetry_path_for_session,
    write_fill_manifest,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Split one captured telemetry session into per-fill NDJSON files. "
            "A fill means one jar filling process starting at state=FILL, not the "
            "broader machine run tracked by capture.py."
        )
    )
    parser.add_argument("session_dir", type=Path, help="Session folder created by capture.py")
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output folder for fill_XXXX.ndjson and index.json (default: <session>/fills)",
    )
    parser.add_argument(
        "--pre-ms",
        type=int,
        help="Context kept before each FILL state (default: session_meta pre_run_ms or 1000)",
    )
    parser.add_argument(
        "--post-ms",
        type=int,
        help="Context kept after fill completion (default: session_meta post_run_ms or 1000)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Deprecated compatibility flag. Existing split output is replaced automatically.",
    )
    args = parser.parse_args()

    session_dir = resolve_session_dir(args.session_dir)
    session_meta = load_session_meta(session_dir)
    pre_ms = args.pre_ms if args.pre_ms is not None else int(session_meta.get("pre_run_ms", 1000))
    post_ms = args.post_ms if args.post_ms is not None else int(session_meta.get("post_run_ms", 1000))
    output_dir = args.output_dir.resolve() if args.output_dir else session_dir / "fills"

    cleared_existing = False
    if output_dir.exists():
        shutil.rmtree(output_dir)
        cleared_existing = True

    telemetry_path = telemetry_path_for_session(session_dir)
    records = load_ndjson(telemetry_path)
    fill_runs = build_fill_runs(records, pre_ms=pre_ms, post_ms=post_ms)
    manifest_path = write_fill_manifest(
        session_dir,
        output_dir,
        fill_runs,
        records,
        pre_ms=pre_ms,
        post_ms=post_ms,
    )

    print(f"Session:   {session_dir}")
    print(f"Telemetry: {telemetry_path}")
    print(f"Output:    {output_dir}")
    print(f"Cleared:   {'yes' if cleared_existing else 'no'}")
    print(f"Windows:   pre={pre_ms} ms, post={post_ms} ms")
    print(f"Detected:  {len(fill_runs)} fill runs")
    print()
    for fill_run in fill_runs:
        print(format_fill_brief(fill_run))
    print()
    print(f"Manifest:  {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
