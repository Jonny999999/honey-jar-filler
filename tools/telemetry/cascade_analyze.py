#!/usr/bin/env python3
"""Quick cascade-strategy diagnostics from a captured telemetry session.

Reusable across tuning iterations so a session (Claude or human) does not have
to re-derive the analysis each time. Reads a session's telemetry.ndjson, splits
it into fill segments (state==FILL), and prints:

  * a per-fill summary table: the firmware's believed model K/onset/tau beside
    an offline ground-truth affine fit over that fill's reconstructed steady
    observations, so a diverged on-board estimate is obvious at a glance, and
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
gain_obs_count, model_tau_s, ...). Sessions captured before this rework lack
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


def reconstruct_observations(segment: list[dict[str, Any]]) -> list[tuple[float, float, float]]:
    """Reconstruct the steady (t, gate, rate) observations OFFLINE from raw
    weight + gate, independent of the firmware's own (currently unreliable)
    gain_obs logging.

    A dwell = the gate held within 1 % for at least dead_time + 0.8 s. The rate
    is delivered-mass / time measured only AFTER one dead time has elapsed, so it
    reflects the current gate rather than the previous one, and is drip-immune
    because it averages over the whole steady window (not an instantaneous slope).
    This is the same definition the firmware uses -- so it shows what the on-board
    estimator is being fed, without depending on the impulse logging surviving.
    """
    out: list[tuple[float, float, float]] = []
    i = 0
    n = len(segment)
    while i < n:
        g = segment[i].get("gate_pct", 0.0)
        j = i
        while j + 1 < n and abs(segment[j + 1].get("gate_pct", 0.0) - g) <= 1.0:
            j += 1
        t0 = segment[i]["ts_us"] / 1e6
        dead = segment[j].get("learned_dead_time_s", 0.4) or 0.4
        if g > 3.0 and (segment[j]["ts_us"] / 1e6 - t0) >= dead + 0.8:
            k = i
            while k < j and (segment[k]["ts_us"] / 1e6 - t0) < dead:
                k += 1
            span = (segment[j]["ts_us"] - segment[k]["ts_us"]) / 1e6
            if span >= 0.8:
                dm = _weight(segment[j]) - _weight(segment[k])
                rate = dm / span
                if rate > 0.5:
                    out.append((segment[k]["ts_us"] / 1e6, g, rate))
        i = j + 1
    return out


def _weight(r: dict[str, Any]) -> float:
    for key in ("weight_g", "relative_fill_g", "grams"):
        if key in r:
            return r[key]
    return 0.0


def affine_fit(points: list[tuple[float, float]]) -> tuple[float, float] | None:
    """Least-squares K, b for rate = K*gate + b over (gate, rate) points."""
    n = len(points)
    if n < 2:
        return None
    sx = sum(g for g, _ in points)
    sy = sum(r for _, r in points)
    sxx = sum(g * g for g, _ in points)
    sxy = sum(g * r for g, r in points)
    denom = n * sxx - sx * sx
    if abs(denom) < 1e-9:
        return None
    k = (n * sxy - sx * sy) / denom
    return k, (sy - k * sx) / n


def observations(segment: list[dict[str, Any]]) -> list[tuple[float, float]]:
    """(gate, rate) steady observations, reconstructed offline."""
    return [(g, r) for _t, g, r in reconstruct_observations(segment)]


def print_summary(segments: list[list[dict[str, Any]]]) -> None:
    # Firmware's believed model (K_end/onset/tau) side by side with an offline
    # ground-truth fit over the SAME fill's reconstructed observations. A large
    # gap between the two is the identifier failing, not the plant being weird.
    hdr = (
        f"{'fill':>4}{'dur_s':>7}{'final':>7}"
        f"{'fwK':>6}{'fwOns':>6}{'tau':>6}"
        f"{'|  gtK':>7}{'gtOns':>6}{'nobs':>5}{'gatespan':>11}  notes"
    )
    print(hdr)
    print("-" * len(hdr))
    for i, s in enumerate(segments):
        e = s[-1]
        dur = (e["ts_us"] - s[0]["ts_us"]) / 1e6
        final = e.get("relative_fill_g", _weight(e))
        obs = observations(s)
        span = f"{min(g for g, _ in obs):.0f}-{max(g for g, _ in obs):.0f}" if obs else "-"
        fit = affine_fit(obs)
        if fit:
            gtk, gtb = fit
            gt_onset = (-gtb / gtk) if gtk > 0.05 else float("nan")
            gtk_s, gto_s = f"{gtk:6.2f}", f"{gt_onset:6.1f}"
        else:
            gtk_s, gto_s = f"{'-':>6}", f"{'-':>6}"
        note = ""
        fwk = e.get("gate_gain_gps_per_pct", 0.0)
        if fwk <= 0.031 or fwk >= 2.99:
            note = "fw K on rail!"
        elif fit and fit[0] > 0.05 and (fwk / fit[0] > 2 or fwk / fit[0] < 0.5):
            note = f"fw {fwk / fit[0]:.1f}x truth"
        print(
            f"{i:>4}{dur:>7.1f}{final:>7.1f}"
            f"{fwk:>6.2f}{onset_pct(e):>6.1f}{e.get('model_tau_s', 0):>6.2f}"
            f"{gtk_s:>7}{gto_s}{len(obs):>5}{span:>11}  {note}"
        )
    print_diagnosis(segments)


def print_diagnosis(segments: list[list[dict[str, Any]]]) -> None:
    allobs = [(t, g, r, i) for i, s in enumerate(segments) for (t, g, r) in reconstruct_observations(s)]
    print("\n--- diagnosis ---")
    if not allobs:
        print("no steady observations reconstructed (gate never held long enough).")
        return
    gates = [g for _, g, _, _ in allobs]
    print(f"{len(allobs)} steady observations, gate span {min(gates):.0f}-{max(gates):.0f} %")
    # Firmware K rail-hits across the session.
    rails = sum(
        1
        for s in segments
        for r in s
        if r.get("gate_gain_gps_per_pct", 1) <= 0.031 or r.get("gate_gain_gps_per_pct", 0) >= 2.99
    )
    total = sum(len(s) for s in segments)
    if rails:
        print(f"firmware K pinned to a rail (<=0.03 or >=3.0) in {rails}/{total} samples "
              f"-> the identifier is diverging, feedforward gate is garbage there.")
    # Global vs local: does one affine line describe the whole session?
    fit = affine_fit([(g, r) for _, g, r, _ in allobs])
    if fit and fit[0] <= 0.05:
        print(f"a single global affine fit gives K={fit[0]:.2f} (<=0): the gate->rate "
              f"relation is NON-monotonic / non-stationary across the session "
              f"(nonlinear at high gate + falling bucket head), so a global K,b "
              f"cannot fit it -- the estimator must stay LOCAL.")
    # Show the observations so the trajectory is visible.
    print("\n  t(s)  fill  gate   rate   K=rate/(gate-onset)  [onset=17%]")
    for t, g, r, i in allobs:
        loc = r / (g - 17.0) if g - 17.0 > 3 else float("nan")
        print(f"  {t:6.1f}{i:>5}{g:7.1f}{r:7.2f}       {loc:6.2f}")


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
