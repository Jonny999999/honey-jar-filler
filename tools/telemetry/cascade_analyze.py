#!/usr/bin/env python3
"""Quick cascade-strategy diagnostics from a captured telemetry session.

Reusable across tuning iterations so a session (Claude or human) does not have
to re-derive the analysis each time. Reads a session's telemetry.ndjson, splits
it into fill segments (state==FILL), and prints:

  * a per-fill summary table (duration, start gate, final/max mass, the
    identified affine model K/b/onset, the learned FOPDT lag tau, and -- most
    diagnostic of all -- how many steady observations the RLS fit actually got
    and over what gate span, plus the resulting slope variance P_k), and
  * an optional per-sample trace of one fill (--fill N) showing gate, the
    smoothed control rate vs the fast filtered rate, target rate, model rate,
    onset and K -- the view used to diagnose the drip/oscillation behaviour.

Read the observation count/span first: K is only identifiable if the fit is fed
several observations spanning a decent gate range. Few observations, or all at
one gate, means no estimator can recover the plant -- that is an excitation
problem, not a tuning problem.

The telemetry field names are the cascade sample fields emitted by
components/telemetry/telemetry.c (rate_filtered_gps, control_rate_gps,
gate_gain_gps_per_pct, gain_offset_b_gps, flow_onset_gate_pct, gain_obs_*,
gain_rls_p_k, model_tau_s, ...). Sessions captured before the RLS rework lack
the obs/onset/tau fields; the onset is then derived from -b/K.

Usage:
  python3 tools/telemetry/cascade_analyze.py <session_dir_or_ndjson>
  python3 tools/telemetry/cascade_analyze.py <session> --fill 2
  python3 tools/telemetry/cascade_analyze.py <session> --fill 2 --every 1
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

FILL_STATE_NUM = 3  # filler_state_t FILLER_FILL


def load_samples(path: Path) -> list[dict[str, Any]]:
    if path.is_dir():
        path = path / "telemetry.ndjson"
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    return [r for r in rows if r.get("kind") == "sample"]


def split_fills(samples: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    """Contiguous runs of state==FILL are individual fill attempts."""
    segments: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    for r in samples:
        if r.get("state") == FILL_STATE_NUM:
            current.append(r)
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    return segments


def onset_pct(r: dict[str, Any]) -> float:
    """Flow-onset gate ("dead angle"). Logged directly since the RLS rework;
    older sessions only carry K and b, so derive it there (gate where rate
    crosses 0)."""
    direct = r.get("flow_onset_gate_pct")
    if direct is not None:
        return direct
    k = r.get("gate_gain_gps_per_pct", 0.0)
    b = r.get("gain_offset_b_gps", 0.0)
    return (-b / k) if k > 0.01 else 0.0


def observations(segment: list[dict[str, Any]]) -> list[tuple[float, float]]:
    """The (gate, rate) steady observations the RLS fit consumed in this fill.

    The firmware logs each one as a single-sample impulse and clears it, so a
    non-zero gain_obs_rate_gps is exactly one observation.
    """
    return [
        (r.get("gain_obs_gate_pct", 0.0), r.get("gain_obs_rate_gps", 0.0))
        for r in segment
        if r.get("gain_obs_rate_gps", 0.0) > 0.0
    ]


def print_summary(segments: list[list[dict[str, Any]]]) -> None:
    hdr = (
        f"{'fill':>4}{'dur_s':>7}{'startg':>7}{'final':>7}{'max':>7}"
        f"{'K_end':>7}{'b_end':>7}{'onset':>7}{'tau':>6}{'n_obs':>6}"
        f"{'obs_gate_span':>15}{'P_k':>8}"
    )
    print(hdr)
    print("-" * len(hdr))
    for i, s in enumerate(segments):
        e = s[-1]
        dur = (e["ts_us"] - s[0]["ts_us"]) / 1e6
        obs = observations(s)
        span = f"{min(g for g, _ in obs):.1f}-{max(g for g, _ in obs):.1f}" if obs else "-"
        print(
            f"{i:>4}{dur:>7.1f}{s[0].get('gate_pct', 0):>7.1f}"
            f"{e.get('relative_fill_g', 0):>7.1f}"
            f"{max(x.get('relative_fill_g', 0) for x in s):>7.1f}"
            f"{e.get('gate_gain_gps_per_pct', 0):>7.2f}"
            f"{e.get('gain_offset_b_gps', 0):>7.1f}{onset_pct(e):>7.1f}"
            f"{e.get('model_tau_s', 0):>6.2f}"
            f"{len(obs):>6}{span:>15}{e.get('gain_rls_p_k', 0):>8.4f}"
        )
    # The observations ARE the identification evidence: too few, or all at one
    # gate, means K is not identifiable no matter how good the estimator is.
    allobs = [o for s in segments for o in observations(s)]
    if allobs:
        gates = [g for g, _ in allobs]
        print(
            f"\n{len(allobs)} observations total, gate span "
            f"{min(gates):.1f}-{max(gates):.1f} % "
            f"({max(gates) - min(gates):.1f} % of excitation)"
        )


def print_trace(segment: list[dict[str, Any]], every: int) -> None:
    t0 = segment[0]["ts_us"]
    below = sum(1 for r in segment if r.get("gate_pct", 99) < onset_pct(r))
    noflow = sum(1 for r in segment if r.get("rate_filtered_gps", 0) < 0.5)
    print(
        f"\ntrace: n={len(segment)} samples, gate<onset in {below}, "
        f"fast_rate~0 in {noflow}"
    )
    print(
        f"{'t':>6}{'rel':>7}{'gate':>6}{'ctrlR':>7}{'filtR':>7}"
        f"{'tgt':>6}{'model':>7}{'onset':>7}{'K':>6}{'b':>7}{'phase':>13}"
    )
    for i, r in enumerate(segment):
        if i % every:
            continue
        print(
            f"{(r['ts_us'] - t0) / 1e6:6.1f}{r.get('relative_fill_g', 0):7.1f}"
            f"{r.get('gate_pct', 0):6.1f}{r.get('control_rate_gps', 0):7.1f}"
            f"{r.get('rate_filtered_gps', 0):7.1f}{r.get('target_rate_gps', 0):6.1f}"
            f"{r.get('model_rate_gps', 0):7.1f}{onset_pct(r):7.1f}"
            f"{r.get('gate_gain_gps_per_pct', 0):6.2f}{r.get('gain_offset_b_gps', 0):7.1f}"
            f"{r.get('gate_phase', ''):>13}"
        )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path, help="session dir or telemetry.ndjson")
    ap.add_argument("--fill", type=int, default=None, help="also print a per-sample trace of this fill index")
    ap.add_argument("--every", type=int, default=2, help="print every Nth sample in the trace (default 2)")
    args = ap.parse_args()

    samples = load_samples(args.session)
    segments = split_fills(samples)
    print(f"{len(samples)} samples, {len(segments)} fill segments")
    if not segments:
        return 0
    print()
    print_summary(segments)
    if args.fill is not None:
        if 0 <= args.fill < len(segments):
            print_trace(segments[args.fill], max(1, args.every))
        else:
            print(f"\n(no fill {args.fill}; valid range 0..{len(segments) - 1})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
