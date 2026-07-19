#!/usr/bin/env python3
"""Synthesize a plant-response telemetry session for the thesis (ch. 07).

Feature transplant, in one command:

  * The *plant characteristics* are identified from a real measured fill
    (thick honey): dead time L, the affine gate->flow line (K, onset), the
    FOPDT lag tau, post-close Nachlauf, jet-impact peaks and scale noise.
  * The *excitation* is a scripted gate sequence (default 30 % -> 70 % -> close),
    i.e. the plant probe we want to show, not a dosing run.
  * The result is written as a normal capture session (telemetry.ndjson +
    session_meta.json) so split_runs.py / plot_runs.py / analysis.py all work
    on it unchanged.

The output is SIMULATED data. Every session it writes is marked synthetic in
session_meta.json and in the session name, and the report separates identified
from assumed values. Label figures made from it as simulated.

Model structure - each line is a statement the thesis already makes in ch. 07:

  gate -> steady flow    affine line rate = K*(gate - onset), dead angle at onset
  dead time              a delay on the *gate* signal, and asymmetric. Opening
                         costs the full dead time: the honey has to clear the
                         gaskets and build a strand. Closing costs almost
                         nothing: the flap pinches the strand off. The reference
                         shows exactly this - 4.1 s from open to first mass, but
                         the mass settles 0.6 s after the close command. The
                         delay cannot be transport (4 s of free fall is ~78 m),
                         so it is build-up latency, which closing destroys
                         rather than incurs.
  flow build-up          first-order lag tau. After a close this tau is what
                         delivers the fast part of the Nachlauf.
  strand break-up        continuous jet above breakup_rate, discrete drops below
  flap residue           honey wetting the flap, draining slowly -> the slow part
                         of the Nachlauf and the late drip spikes
  jet impact             momentum transfer is *apparent weight*, not mass: it is
                         added to the force reading and rings out, so the mass
                         balance stays exact. The blob at first contact is the
                         opposite: real mass that stays.
  scale                  moving average + gaussian noise, 10 Hz sampling

Example:

  python3 tools/telemetry/synth_plant_session.py \\
      --reference data/telemetry/done-old/2026.06.14_20:54-x-heuristic-settings-changed \\
      --reference-fill 3 --script 30:18,70:22,0:30 --plot
"""

from __future__ import annotations

import argparse
import json
import math
import random
import subprocess
import sys
from collections import deque
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path

from analysis import load_ndjson, resolve_session_dir

GENERATOR_VERSION = 1

TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent.parent

# Rate estimator constants, kept identical to filler_strategy_sequence.c so the
# synthesized rate channels go through the same shape of estimator the firmware
# would have used.
RATE_FILTER_ALPHA_RISE = 0.18
RATE_FILTER_ALPHA_FALL = 0.55
RATE_MIN_VALID_GPS = 2.0
RATE_ZERO_EPS_GPS = 0.5
RATE_NO_FLOW_RESET_SAMPLES = 2
DELTA_RATE_MIN_DT_S = 0.05
DELTA_RATE_MAX_DT_S = 0.80

SIM_DT_S = 0.005
SAMPLE_DT_S = 0.100

STATE_FIND_SLOT = 1
STATE_VERIFY_EMPTY = 2
STATE_FILL = 3
STATE_SLOT_SETTLE = 4

FLOW_EPS_GPS = 0.02
HOLD_S = 0.4          # a slope crossing must hold this long to count (flap shakes the scale)

# Names of every tunable plant parameter, in CLI order.
PARAM_NAMES = (
    "gate_gain_gps_per_pct", "flow_onset_gate_pct", "dead_time_s", "lag_tau_s",
    "scale_noise_g", "onset_pool_g", "close_impact_g", "residue_g", "residue_tau_s",
    "close_delay_s", "onset_pool_s", "onset_impact_g", "impact_tau_s",
    "breakup_rate_gps", "drop_mass_g", "flow_noise_frac", "turb_tau_s",
    "scale_avg_ms",
)


# --------------------------------------------------------------------------
# plant parameters
# --------------------------------------------------------------------------


@dataclass
class PlantParams:
    """Physical description of the Strecke."""

    # --- identified from the reference fill ---
    gate_gain_gps_per_pct: float = 0.109   # K in rate = K*(gate - onset)
    flow_onset_gate_pct: float = 7.4       # gate % the gaskets must clear before flow
    dead_time_s: float = 4.14              # gate OPENING -> first visible weight change
    lag_tau_s: float = 1.2                 # FOPDT lag of the flow build-up
    scale_noise_g: float = 0.08            # HX711 noise (sd) after firmware averaging
    onset_pool_g: float = 7.8              # real mass pooled at the lip during the dead time
    close_impact_g: float = 12.2           # apparent weight when the flap shuts and the strand snaps

    residue_g: float = 5.0                 # honey wetting the flap, drains after close
    residue_tau_s: float = 10.0            # how slowly that residue drains

    # --- assumed (not robustly identifiable from a single fill) ---
    close_delay_s: float = 0.5             # flap shut -> flow stops (NOT the dead time)
    onset_pool_s: float = 0.2              # how fast the first-contact blob lets go
    onset_impact_g: float = 3.5            # force transient riding on the first contact
    impact_tau_s: float = 0.25             # how fast an impact transient rings out
    breakup_rate_gps: float = 3.0          # below this the strand breaks into drops
    drop_mass_g: float = 0.45              # mass of one drop once broken up
    flow_noise_frac: float = 0.16          # turbulence, as a fraction of the flow
    turb_tau_s: float = 0.4                # turbulence correlation time
    scale_avg_ms: float = 200.0            # firmware averaging window

    provenance: dict = field(default_factory=dict)

    def steady_rate_gps(self, gate_pct: float) -> float:
        return max(0.0, self.gate_gain_gps_per_pct * (gate_pct - self.flow_onset_gate_pct))


IDENTIFIED_NAMES = ("gate_gain_gps_per_pct", "flow_onset_gate_pct", "dead_time_s",
                    "lag_tau_s", "scale_noise_g", "onset_pool_g", "close_impact_g",
                    "residue_g", "residue_tau_s")


# --------------------------------------------------------------------------
# identification from a measured reference fill
# --------------------------------------------------------------------------


def _samples_of(records: list[dict]) -> list[dict]:
    return [r for r in records if r.get("kind") == "sample"]


def _gate_events(records: list[dict], t0_us: int) -> list[tuple[float, float, str]]:
    return [((r["ts_us"] - t0_us) / 1e6, float(r.get("gate_pct", 0.0)), r.get("text", ""))
            for r in records if r.get("kind") == "gate"]


def _mean_true_rate(t: list[float], w: list[float], a: float, b: float) -> float:
    """Mean flow over [a,b) from delivered mass, not from the filtered rate.

    The firmware clips its rate channel below RATE_MIN_VALID_GPS to zero, which
    biases any slow phase low. Mass over time does not lie.
    """
    seg = [(ti, wi) for ti, wi in zip(t, w) if a <= ti < b]
    if len(seg) < 2 or seg[-1][0] <= seg[0][0]:
        return 0.0
    return (seg[-1][1] - seg[0][1]) / (seg[-1][0] - seg[0][0])


def _slope(t: list[float], w: list[float], at: float, win: float = 0.6) -> float:
    """Instantaneous rate around `at`, as a centred least-squares slope."""
    seg = [(ti, wi) for ti, wi in zip(t, w) if at - win / 2 <= ti <= at + win / 2]
    n = len(seg)
    if n < 3:
        return 0.0
    mt = sum(x for x, _ in seg) / n
    mw = sum(y for _, y in seg) / n
    num = sum((x - mt) * (y - mw) for x, y in seg)
    den = sum((x - mt) ** 2 for x, _ in seg)
    return num / den if den > 1e-9 else 0.0


def _identify_tau(t: list[float], w: list[float], gates: list[tuple[float, float, str]],
                  points: list[tuple[float, float]], dead_time_s: float) -> float | None:
    """FOPDT lag from a step between two flowing gates.

    The slope approaches the new steady rate as
    r(t) = r_new + (r_old - r_new)*exp(-t/tau).

    Both ends are measured off the transition itself rather than off the
    assumed dead time: the step's own delay need not equal the one identified
    at first open, and anchoring to it can collapse the estimate to zero. The
    10 % and 63.2 % crossings are 0.894*tau apart, which fixes the scale.
    """
    rate_of = dict(points)
    for i in range(len(gates) - 1):
        _gt_a, gate_a, _ = gates[i]
        gt_b, gate_b, _ = gates[i + 1]
        if gate_a <= 0.0 or gate_b <= 0.0:
            continue
        r_a, r_b = rate_of.get(gate_a), rate_of.get(gate_b)
        if r_a is None or r_b is None or abs(r_a - r_b) < 1.0:
            continue
        end = gates[i + 2][0] if i + 2 < len(gates) else t[-1]
        if end - gt_b < 2.0:
            continue
        falling = r_b < r_a
        begin_at = r_a + 0.10 * (r_b - r_a)
        cross_at = r_a + 0.632 * (r_b - r_a)

        def _first(level: float, frm: float) -> float | None:
            """First *sustained* crossing of `level`.

            Moving the flap shakes the scale, so right at the step the slope
            swings far to both sides within one sample. A single crossing is
            therefore meaningless; the level has to hold for HOLD_S.
            """
            hold_n = int(round(HOLD_S / 0.05))
            ti, run, run_start = frm, 0, frm
            while ti < end:
                s = _slope(t, w, ti, win=1.0)
                if (falling and s <= level) or (not falling and s >= level):
                    if run == 0:
                        run_start = ti
                    run += 1
                    if run >= hold_n:
                        return run_start
                else:
                    run = 0
                ti += 0.05
            return None

        t_begin = _first(begin_at, gt_b)
        if t_begin is None:
            continue
        t_cross = _first(cross_at, t_begin)
        if t_cross is None:
            continue
        tau = (t_cross - t_begin) / 0.894
        if 0.05 <= tau <= 8.0:
            return tau
    return None


def identify_plant(records: list[dict], settle_s: float = 1.5) -> tuple[PlantParams, list[str]]:
    """Identify the plant from one measured fill. Returns params + a report."""
    samples = _samples_of(records)
    if len(samples) < 20:
        raise SystemExit("reference fill has too few samples to identify a plant")

    t0 = samples[0]["ts_us"]
    t = [(r["ts_us"] - t0) / 1e6 for r in samples]
    w = [float(r.get("relative_fill_g", 0.0)) for r in samples]
    raw = [float(r.get("rate_raw_gps", 0.0) or 0.0) for r in samples]

    gates = _gate_events(records, t0)
    if len(gates) < 2:
        raise SystemExit("reference fill has no usable gate events")

    open_t, open_gate, _ = gates[0]
    close_t = next((g[0] for g in gates if g[1] <= 0.0), t[-1])

    report: list[str] = []
    p = PlantParams()

    # --- scale noise: the settled tail, after the drip has finished -------
    tail = [wi for ti, wi in zip(t, w) if ti > close_t + 20.0]
    if len(tail) > 20:
        tail = tail[-80:]
        mean = sum(tail) / len(tail)
        p.scale_noise_g = math.sqrt(sum((x - mean) ** 2 for x in tail) / len(tail))
        report.append(f"scale_noise_g       = {p.scale_noise_g:6.3f} g    sd of the settled tail (n={len(tail)})")
    else:
        report.append(f"scale_noise_g       = {p.scale_noise_g:6.3f} g    [default: no settled tail]")

    # --- dead time: gate open -> first weight rise clear of the noise -----
    thresh = max(5.0 * p.scale_noise_g, 0.5)
    w_open = next((wi for ti, wi in zip(t, w) if ti >= open_t), 0.0)
    first_rise = next((ti for ti, wi in zip(t, w) if ti > open_t and wi - w_open > thresh), None)
    if first_rise is not None:
        p.dead_time_s = first_rise - open_t
        report.append(f"dead_time_s         = {p.dead_time_s:6.2f} s    gate {open_gate:.0f} % -> weight rises > {thresh:.2f} g")
    else:
        report.append(f"dead_time_s         = {p.dead_time_s:6.2f} s    [default: no rise detected]")

    # --- affine gate->flow line from the steady dwells --------------------
    points: list[tuple[float, float]] = []
    for i, (gt, gate, _txt) in enumerate(gates):
        if gate <= 0.0:
            continue
        end = gates[i + 1][0] if i + 1 < len(gates) else t[-1]
        a = gt + p.dead_time_s + settle_s
        if end - a < 3.0:
            continue
        r = _mean_true_rate(t, w, a, end)
        points.append((gate, r))
        report.append(f"  steady dwell: gate {gate:5.1f} %  ->  {r:5.2f} g/s   (window {a:.1f}..{end:.1f} s)")

    r_ss_max = 0.0
    if len(points) >= 2:
        (g_hi, r_hi), (g_lo, r_lo) = max(points), min(points)
        r_ss_max = r_hi
        if abs(g_hi - g_lo) >= 5.0 and abs(r_hi - r_lo) > 1e-6:
            p.gate_gain_gps_per_pct = (r_hi - r_lo) / (g_hi - g_lo)
            p.flow_onset_gate_pct = g_hi - r_hi / p.gate_gain_gps_per_pct
            report.append(f"gate_gain (K)       = {p.gate_gain_gps_per_pct:6.4f}      g/s per %, two-point secant")
            report.append(f"flow_onset_gate_pct = {p.flow_onset_gate_pct:6.1f} %    dead angle, b = -K*onset = {-p.gate_gain_gps_per_pct * p.flow_onset_gate_pct:.2f} g/s")
        else:
            report.append("gate_gain (K)       = [default: steady dwells too close together]")
    else:
        report.append("gate_gain (K)       = [default: fewer than two steady dwells]")

    # --- lag tau: from a gate step between two flowing dwells -------------
    # Deliberately not measured at first contact: the jet impact throws a force
    # transient there that swamps the slope. A step from one flowing gate to
    # another has no such transient, so the approach to the new rate is clean.
    tau = _identify_tau(t, w, gates, points, p.dead_time_s)
    if tau is not None:
        p.lag_tau_s = tau
        report.append(f"lag_tau_s           = {p.lag_tau_s:6.2f} s    approach to 37 % of a gate-to-gate step")
    else:
        report.append(f"lag_tau_s           = {p.lag_tau_s:6.2f} s    [default: no flowing-to-flowing gate step]")

    # --- first contact: a pooled blob, and it is real mass ----------------
    # The reference shows the mass jumping ~18 g in 0.2 s at first contact and
    # *staying* there, so this is honey that pooled at the gate during the dead
    # time and let go at once - not a force transient. Identified as the excess
    # over what the steady rate alone would have delivered in that second.
    if first_rise is not None and r_ss_max > 0.0:
        m_a = next((wi for ti, wi in zip(t, w) if ti >= first_rise), 0.0)
        m_b = next((wi for ti, wi in zip(t, w) if ti >= first_rise + 1.0), None)
        if m_b is not None:
            p.onset_pool_g = max(0.0, (m_b - m_a) - r_ss_max * 1.0)
            report.append(f"onset_pool_g        = {p.onset_pool_g:6.2f} g    blob at first contact "
                          f"(real mass: {m_b - m_a:.1f} g in 1 s vs {r_ss_max:.1f} g/s steady)")

    # --- close: a force transient, and it rings back out ------------------
    # The reference peaks at +16 g within 0.4 s of the close command and settles
    # back ~12 g. That is momentum transfer / flap shock, so it is apparent
    # weight, and it appears immediately - it is not transport-delayed.
    seg = [(ti, wi) for ti, wi in zip(t, w) if close_t <= ti <= close_t + 2.0]
    settled = next((wi for ti, wi in zip(t, w) if ti >= close_t + 2.0), None)
    if seg and settled is not None:
        peak_t, peak_m = max(seg, key=lambda x: x[1])
        p.close_impact_g = max(0.0, peak_m - settled)
        report.append(f"close_impact_g      = {p.close_impact_g:6.2f} g    flap-shock overshoot "
                      f"(peak {peak_m:.1f} g at +{peak_t - close_t:.1f} s, settles to {settled:.1f} g)")

    # --- Nachlauf: split into the fast tail and the slow flap drain -------
    # There is no single "Nachlauf" parameter. The fast part is the flow decaying
    # through its own lag once the flap cuts it (~r_close*tau); whatever is still
    # missing arrived far too late for that and is the flap draining, which is
    # what `residue_g` describes.
    w_close = next((wi for ti, wi in zip(t, w) if ti >= close_t), None)
    if w_close is not None:
        measured = w[-1] - w_close
        r_close = _slope(t, w, close_t - 1.0)
        fast = r_close * p.lag_tau_s
        p.residue_g = max(0.0, measured - fast)
        report.append(f"residue_g           = {p.residue_g:6.2f} g    Nachlauf {measured:.2f} g minus the "
                      f"fast tail ({r_close:.2f} g/s x tau {p.lag_tau_s:.2f} s = {fast:.2f} g)")

        # how slowly it drains: 63 % of the residue, measured from where the
        # fast tail has died away (two lags after the close)
        t_slow = close_t + 2.0 * p.lag_tau_s
        w_slow = next((wi for ti, wi in zip(t, w) if ti >= t_slow), None)
        if w_slow is not None and w[-1] - w_slow > 0.5:
            target = w_slow + 0.632 * (w[-1] - w_slow)
            t63 = next((ti - t_slow for ti, wi in zip(t, w) if ti > t_slow and wi >= target), None)
            if t63 is not None and 1.0 <= t63 <= 60.0:
                p.residue_tau_s = t63
                report.append(f"residue_tau_s       = {p.residue_tau_s:6.2f} s    63 % of the slow drain")

    report.append("")
    report.append("assumed (not identifiable from one fill; override on the command line):")
    for name in PARAM_NAMES:
        if name in IDENTIFIED_NAMES:
            continue
        report.append(f"  --{name.replace('_', '-'):<22} {getattr(p, name)}")

    return p, report


# --------------------------------------------------------------------------
# simulation
# --------------------------------------------------------------------------


@dataclass
class GateStep:
    gate_pct: float
    duration_s: float


def parse_script(spec: str) -> list[GateStep]:
    steps = []
    for chunk in spec.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        try:
            gate_s, dur_s = chunk.split(":")
            steps.append(GateStep(float(gate_s), float(dur_s)))
        except ValueError:
            raise SystemExit(f"bad --script step {chunk!r}, expected GATE:SECONDS (e.g. 30:18)")
    if not steps:
        raise SystemExit("--script is empty")
    return steps


def simulate(p: PlantParams, steps: list[GateStep], seed: int) -> dict[str, list[float]]:
    """Simulate the plant under the gate script.

    Returns sampled channels at SAMPLE_DT_S: t, mass (measured, relative),
    gate (as commanded), and true_flow (the noiseless flow at the jar).

    The identified dead time already contains the measurement delay the
    firmware's averaging adds, so the delay used here is the identified value
    minus half the averaging window; otherwise the averaging is counted twice.
    """
    # Separate streams per noise source, on purpose. With one shared stream the
    # scale noise is drawn after every turbulence draw, so changing the script
    # length changes how many draws precede it and re-rolls the whole chart -
    # shortening the last dwell would silently alter the first second. Split,
    # each sample's noise depends only on its own index, so changing the tail
    # truncates the run and leaves everything before the cut identical.
    rng_turb = random.Random(seed)
    rng_meas = random.Random(seed + 1)

    total_s = sum(s.duration_s for s in steps)
    n_sim = int(round(total_s / SIM_DT_S))
    open_delay_s = max(0.0, p.dead_time_s - (p.scale_avg_ms / 1000.0) / 2.0)

    bounds: list[tuple[float, float, float]] = []   # (start, end, gate)
    acc = 0.0
    for s in steps:
        bounds.append((acc, acc + s.duration_s, s.gate_pct))
        acc += s.duration_s

    def gate_at(t: float) -> float:
        for a, b, g in bounds:
            if t < b:
                return g
        return bounds[-1][2]

    # The gate signal the plant actually acts on: each commanded step takes
    # effect after its own delay - the full dead time when opening, a short
    # one when closing (see the module docstring).
    eff: list[tuple[float, float]] = []
    prev_gate, acc, prev_eff_t = 0.0, 0.0, -1e9
    for s in steps:
        delay = open_delay_s if s.gate_pct > prev_gate else p.close_delay_s
        eff_t = max(acc + delay, prev_eff_t)   # a later command cannot land first
        eff.append((eff_t, s.gate_pct))
        prev_gate, prev_eff_t = s.gate_pct, eff_t
        acc += s.duration_s

    def gate_eff_at(t: float) -> float:
        g = 0.0
        for eff_t, gv in eff:
            if eff_t <= t:
                g = gv
            else:
                break
        return g

    q = 0.0                       # flow at the gate (g/s)
    turb = 0.0                    # correlated turbulence (Ornstein-Uhlenbeck)
    residue = 0.0                 # honey clinging to the flap
    pool_left = 0.0               # blob pooled at the lip, still letting go
    pending_g = 0.0               # arrived but not yet released as a drop
    landed_g = 0.0                # real mass resting on the scale
    impact_g = 0.0                # apparent weight from momentum transfer
    prev_gate_cmd = 0.0
    first_contact = False

    landed_trace: list[float] = []
    apparent_trace: list[float] = []
    flow_trace: list[float] = []

    turb_sigma = p.flow_noise_frac * math.sqrt(2.0 * SIM_DT_S / max(p.turb_tau_s, 1e-3))

    for i in range(n_sim):
        t = i * SIM_DT_S
        gate_cmd = gate_at(t)
        q_ss = p.steady_rate_gps(gate_eff_at(t))

        # Shutting the flap shocks the frame and snaps the strand, and the scale
        # sees that at once. Keyed to the *command*: the reference peaks 0.4 s
        # after the close command, i.e. it does not wait for the dead time.
        if prev_gate_cmd > 0.0 and gate_cmd <= 0.0:
            impact_g += p.close_impact_g
        prev_gate_cmd = gate_cmd

        # first-order flow build-up / decay at the gate
        q += (q_ss - q) * (SIM_DT_S / max(p.lag_tau_s, 1e-3))

        # turbulence: correlated, so it survives the scale's averaging the way
        # real flow noise does (white noise would just average away)
        turb += -turb * (SIM_DT_S / max(p.turb_tau_s, 1e-3)) + turb_sigma * rng_turb.gauss(0.0, 1.0)
        q_out = max(0.0, q * (1.0 + turb)) if q > FLOW_EPS_GPS else 0.0

        # the dead time already sits on the gate signal, so the strand flow is
        # simply the flow leaving the gate
        strand_flow = q_out

        # the flap wets up while honey runs over it and drains afterwards; this
        # is the slow part of the Nachlauf and the late drip spikes
        if q_ss > 0.0:
            residue = p.residue_g
            residue_flow = 0.0
        elif residue > 0.0:
            drained = residue * (SIM_DT_S / max(p.residue_tau_s, 1e-3))
            residue -= drained
            residue_flow = drained / SIM_DT_S
        else:
            residue_flow = 0.0

        jar_flow = strand_flow + residue_flow
        arriving = jar_flow * SIM_DT_S

        # Honey pooled at the lip during the dead time lets go at first contact.
        # This is real mass, not a force spike: the reference's mass steps up
        # and stays up. It is released over onset_pool_s, which is what makes
        # the first-jet rate peak.
        if arriving > 0.0 and not first_contact:
            first_contact = True
            pool_left = p.onset_pool_g
            impact_g += p.onset_impact_g
        if pool_left > 0.0:
            rel = min(pool_left, p.onset_pool_g * (SIM_DT_S / max(p.onset_pool_s, 1e-3)))
            pool_left -= rel
            arriving += rel

        pending_g += arriving

        # jet break-up: a fast strand delivers continuously, a slow one drips
        if jar_flow > p.breakup_rate_gps:
            landed_g += pending_g
            pending_g = 0.0
        else:
            while pending_g >= p.drop_mass_g:
                landed_g += p.drop_mass_g
                pending_g -= p.drop_mass_g

        # momentum transfer is force, not mass: it rings out and leaves no trace
        impact_g *= math.exp(-SIM_DT_S / max(p.impact_tau_s, 1e-3))

        landed_trace.append(landed_g)
        apparent_trace.append(landed_g + impact_g)
        flow_trace.append(jar_flow)

    # --- measurement chain: moving average, noise, 10 Hz sampling ---------
    avg_n = max(1, int(round((p.scale_avg_ms / 1000.0) / SIM_DT_S)))
    step = int(round(SAMPLE_DT_S / SIM_DT_S))

    out: dict[str, list[float]] = {"t": [], "mass": [], "gate": [], "true_flow": []}
    for i in range(0, n_sim, step):
        lo = max(0, i - avg_n + 1)
        window = apparent_trace[lo : i + 1]
        t = i * SIM_DT_S
        out["t"].append(t)
        out["mass"].append(sum(window) / len(window) + rng_meas.gauss(0.0, p.scale_noise_g))
        out["gate"].append(gate_at(t))
        out["true_flow"].append(flow_trace[i])

    return out


# --------------------------------------------------------------------------
# firmware rate estimator (port of seq_update_rate in filler_strategy_sequence.c)
# --------------------------------------------------------------------------


def firmware_rates(times: list[float], mass: list[float]) -> tuple[list[float], list[float]]:
    raw_out: list[float] = []
    filt_out: list[float] = []

    filtered = 0.0
    no_flow = 0
    last_t: float | None = None
    last_m = 0.0

    def ewma(prev: float, sample: float, alpha: float) -> float:
        return sample if prev <= 0.0 else prev + alpha * (sample - prev)

    for t, m in zip(times, mass):
        raw = 0.0
        if last_t is not None:
            dt = t - last_t
            if DELTA_RATE_MIN_DT_S <= dt <= DELTA_RATE_MAX_DT_S:
                delta = m - last_m
                if delta < -0.5:
                    delta = 0.0
                raw = (delta / dt) if delta > 0.0 else 0.0
                if raw < RATE_MIN_VALID_GPS:
                    raw = 0.0
                if raw <= 0.0:
                    no_flow = min(255, no_flow + 1)
                    if no_flow >= RATE_NO_FLOW_RESET_SAMPLES:
                        filtered = 0.0
                    else:
                        filtered = ewma(filtered, 0.0, RATE_FILTER_ALPHA_FALL)
                else:
                    no_flow = 0
                    alpha = RATE_FILTER_ALPHA_RISE if raw >= filtered else RATE_FILTER_ALPHA_FALL
                    filtered = ewma(filtered, raw, alpha)
                if filtered < RATE_ZERO_EPS_GPS:
                    filtered = 0.0
        last_t, last_m = t, m
        raw_out.append(raw)
        filt_out.append(filtered)

    return raw_out, filt_out


# --------------------------------------------------------------------------
# session emission
# --------------------------------------------------------------------------


DEFAULT_PARAMS = {
    "version": 15,
    "VAR(target_grams)": 495,
    "VAR(target_tol_low_g)": 10,
    "VAR(target_tol_high_g)": 20,
    "VAR(fill_timeout_ms)": 600000,
    "VAR(slow_remaining_g)": 110,
    "VAR(slow_gate_pct)": 27,
    "VAR(max_gate_pct)": 50,
    "VAR(close_remaining_g)": 5,
    "VAR(drip_delay_ms)": 3000,
    "VAR(empty_glass_min_g)": 150,
    "VAR(empty_glass_max_g)": 300,
    "VAR(advance_timeout_ms)": 4000,
    "VAR(find_ignore_ms)": 1000,
    "VAR(slot_settle_ms)": 1000,
    "VAR(slots_total)": 6,
    "VAR(gate_open_deg)": 4.0,
    "VAR(gate_close_deg)": 101.5,
    "VAR(scale_cal_ref_g)": 500,
}

# Plot style for the ch. 07 plant-probe figure. The target line is off on
# purpose: this figure shows the Strecke reacting to gate steps, not a dosing
# run, so a target mass would only invite the wrong reading.
DEFAULT_PLOT_ARGS = [
    "--hide-target",
    "--legend-placement", "outside",
    "--state-style", "none",
    "--show-rate", "filtered",
    "--rate-layout", "subplot",
    "--plain-both-profiles",
    "--no-session-summary",
    "--format", "pdf",
    "--format", "svg",
    "--format", "png",
]

SAMPLE_ZERO_FIELDS = (
    "rate_2sample_gps", "rate_4sample_gps", "rate_filtered_medium_gps",
    "rate_filtered_slow_gps", "target_rate_gps", "rate_error_gps",
    "predicted_remaining_g", "measured_dead_time_s", "measured_post_close_gain_g",
    "measured_near_close_gain_g", "learned_dead_time_s", "learned_post_close_gain_g",
    "learned_fast_rate_gps", "learned_slow_rate_gps", "learned_near_close_bias_g",
    "adapted_near_close_g", "adapted_close_early_g", "adapted_drip_wait_ms",
    "model_rate_gps", "model_delayed_gps", "control_integ_pct",
    "gate_gain_gps_per_pct",
)


def _sample(ts_us: int, run_id: int, state: int, target_g: int, weight_g: float,
            rel_g: float, gate_pct: float, phase: str, raw: float, filt: float) -> dict:
    rec = {
        "ts_us": ts_us, "kind": "sample", "run_id": run_id, "slot_idx": 0,
        "state": state, "target_g": target_g,
        "weight_g": round(weight_g, 3), "relative_fill_g": round(rel_g, 3),
        "gate_pct": round(gate_pct, 1),
        "strategy_name": "sequence", "gate_phase": phase,
        "rate_raw_gps": round(raw, 3),
        "rate_filtered_gps": round(filt, 3),
        "refill_count": 0,
    }
    for f in SAMPLE_ZERO_FIELDS:
        rec.setdefault(f, 0.0)
    return rec


def emit_session(out_dir: Path, sim: dict[str, list[float]], raw: list[float], filt: list[float],
                 steps: list[GateStep], p: PlantParams, base_weight_g: float, target_g: int,
                 preset_name: str, pre_s: float, post_s: float, seed: int, command: str) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)

    boot_offset_s = 18.5
    fill_t0_s = boot_offset_s + pre_s

    def us(t_rel: float) -> int:
        return int(round((fill_t0_s + t_rel) * 1e6))

    bounds: list[tuple[float, str]] = []
    acc = 0.0
    for idx, s in enumerate(steps):
        acc += s.duration_s
        bounds.append((acc, f"seq_s{idx}"))

    def phase_for(t_rel: float) -> str:
        for end, name in bounds:
            if t_rel < end:
                return name
        return bounds[-1][1]

    records: list[dict] = []
    run_id = 1

    records.append({"ts_us": us(-pre_s - 2.2), "kind": "run_start", "run_id": run_id, "slot_idx": 0})
    records.append({"ts_us": us(-pre_s - 2.2), "kind": "state", "run_id": run_id, "slot_idx": 0,
                    "state": STATE_FIND_SLOT, "text": "FIND_SLOT"})
    records.append({"ts_us": us(-pre_s - 1.1), "kind": "state", "run_id": run_id, "slot_idx": 0,
                    "state": STATE_SLOT_SETTLE, "text": "SLOT_SETTLE"})
    records.append({"ts_us": us(-pre_s - 0.02), "kind": "state", "run_id": run_id, "slot_idx": 0,
                    "state": STATE_VERIFY_EMPTY, "text": "VERIFY_EMPTY"})
    records.append({"ts_us": us(-pre_s), "kind": "state", "run_id": run_id, "slot_idx": 0,
                    "state": STATE_FILL, "text": "FILL"})
    records.append({
        "ts_us": us(-pre_s + 0.001), "kind": "fill_start", "run_id": run_id, "slot_idx": 0,
        "text": "start", "preset_name": preset_name, "strategy_name": "sequence",
        "target_g": target_g, "base_weight_g": round(base_weight_g, 3),
        "scale_period_ms_cfg": 100, "fsm_period_ms_cfg": 10, "params": DEFAULT_PARAMS,
    })

    acc = 0.0
    for idx, s in enumerate(steps):
        records.append({
            "ts_us": us(acc + 0.001), "kind": "gate", "run_id": run_id, "slot_idx": 0,
            "gate_pct": float(s.gate_pct), "text": f"seq_s{idx}_{int(round(s.gate_pct))}",
        })
        acc += s.duration_s

    rng = random.Random(seed ^ 0xC0FFEE)

    for i in range(int(round(pre_s / SAMPLE_DT_S))):
        t_rel = -pre_s + i * SAMPLE_DT_S
        n = rng.gauss(0.0, p.scale_noise_g)
        records.append(_sample(us(t_rel), run_id, STATE_FILL, target_g,
                               base_weight_g + n, n, 0.0, "seq_pre", 0.0, 0.0))

    for t_rel, m, g, r_raw, r_filt in zip(sim["t"], sim["mass"], sim["gate"], raw, filt):
        records.append(_sample(us(t_rel), run_id, STATE_FILL, target_g,
                               base_weight_g + m, m, g, phase_for(t_rel), r_raw, r_filt))

    total_s = sum(s.duration_s for s in steps)
    final_g = sim["mass"][-1]

    records.append({
        "ts_us": us(total_s + 0.01), "kind": "fill_summary", "run_id": run_id, "slot_idx": 0,
        "text": "sequence", "preset_name": preset_name, "strategy_name": "sequence",
        "target_g": target_g,
        "final_mass_g": round(final_g, 3), "final_relative_fill_g": round(final_g, 3),
        "fill_error_g": round(final_g - target_g, 3),
        "fill_duration_s": round(total_s, 2), "refill_count": 0, "params": DEFAULT_PARAMS,
    })
    records.append({"ts_us": us(total_s + 0.05), "kind": "state", "run_id": run_id, "slot_idx": 1,
                    "state": STATE_FIND_SLOT, "text": "FIND_SLOT"})

    for i in range(int(round(post_s / SAMPLE_DT_S))):
        t_rel = total_s + i * SAMPLE_DT_S
        n = rng.gauss(0.0, p.scale_noise_g)
        records.append(_sample(us(t_rel), run_id, STATE_FIND_SLOT, target_g,
                               base_weight_g + final_g + n, final_g + n, 0.0, "seq_done", 0.0, 0.0))

    records.append({
        "ts_us": us(total_s + post_s + 0.01), "kind": "run_summary", "run_id": run_id, "slot_idx": 1,
        "text": "ok", "preset_name": preset_name, "strategy_name": "sequence", "target_g": target_g,
        "final_weight_g": round(base_weight_g + final_g, 3),
        "final_relative_fill_g": round(final_g, 3),
        "scale_period_ms_cfg": 100, "fsm_period_ms_cfg": 10, "params": DEFAULT_PARAMS,
    })
    records.append({"ts_us": us(total_s + post_s + 0.02), "kind": "run_end", "run_id": run_id,
                    "slot_idx": 1, "text": "ok"})

    records.sort(key=lambda r: r["ts_us"])

    tel_path = out_dir / "telemetry.ndjson"
    with tel_path.open("w") as fh:
        for r in records:
            fh.write(json.dumps(r) + "\n")

    meta = {
        "captured_at": datetime.now(timezone.utc).astimezone().isoformat(),
        "port": "SYNTHETIC",
        "baud": 0,
        "pre_run_ms": int(pre_s * 1000),
        "post_run_ms": int(post_s * 1000),
        "synthetic": True,
        "synthetic_note": (
            "SIMULATED DATA - not a hardware capture. The plant parameters were identified "
            "from a measured fill (see generator.identified_from); the gate excitation is "
            "scripted. Any figure made from this session must be labelled as simulated."
        ),
        "generator": {
            "tool": "synth_plant_session.py",
            "version": GENERATOR_VERSION,
            "command": command,
            "seed": seed,
            "gate_script": [{"gate_pct": s.gate_pct, "duration_s": s.duration_s} for s in steps],
            "plant_params": {k: v for k, v in asdict(p).items() if k != "provenance"},
            "identified_from": p.provenance,
        },
    }
    (out_dir / "session_meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    return tel_path


# --------------------------------------------------------------------------
# validation: does the synthetic run behave like the reference?
# --------------------------------------------------------------------------


def validate(p: PlantParams, sim: dict[str, list[float]], raw: list[float], filt: list[float],
             steps: list[GateStep]) -> list[str]:
    t, mass = sim["t"], sim["mass"]
    out: list[str] = []
    acc = 0.0
    close_t = None
    for s in steps:
        a, b = acc, acc + s.duration_s
        acc = b
        if s.gate_pct <= 0.0:
            close_t = close_t if close_t is not None else a
            continue
        wa = a + p.dead_time_s + 1.5
        if b - wa < 3.0:
            continue
        r = _mean_true_rate(t, mass, wa, b)
        seg = [x for ti, x in zip(t, filt) if wa <= ti < b]
        sd = 0.0
        if len(seg) > 2:
            mu = sum(seg) / len(seg)
            sd = math.sqrt(sum((x - mu) ** 2 for x in seg) / len(seg))
        out.append(f"  gate {s.gate_pct:5.1f} %  delivered {r:5.2f} g/s (model {p.steady_rate_gps(s.gate_pct):5.2f}), "
                   f"filtered-rate sd {sd:4.2f} g/s")

    if close_t is not None:
        w_close = next((m for ti, m in zip(t, mass) if ti >= close_t), 0.0)
        r_close = _slope(t, mass, close_t - 1.0)
        fast = r_close * (p.lag_tau_s + p.close_delay_s)
        out.append(f"  Nachlauf after close: {mass[-1] - w_close:5.2f} g "
                   f"(fast tail {fast:.1f} g + flap residue {p.residue_g:.1f} g)")
        late = [ti - close_t for ti, r in zip(t, raw) if ti > close_t + p.dead_time_s + 2.0 and r > 2.0]
        out.append(f"  late drip spikes: {len(late)} at t_close +{', +'.join(f'{x:.0f}' for x in late[:8])} s"
                   if late else "  late drip spikes: none")

    peak_open = max(raw[: len(raw) // 2]) if raw else 0.0
    out.append(f"  first-jet raw peak: {peak_open:5.1f} g/s")
    out.append(f"  final mass: {mass[-1]:.1f} g")
    return out


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "Synthesize a plant-response telemetry session: plant characteristics "
            "identified from a measured fill, gate excitation scripted. Writes "
            "SIMULATED data, marked as such in session_meta.json."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="See the module docstring for the model structure.",
    )
    ap.add_argument("--reference", type=Path, required=True,
                    help="Session dir of the measured fill to take the plant characteristics from")
    ap.add_argument("--reference-fill", type=int, default=3, help="Fill number in that session (default: 3)")
    ap.add_argument("--script", default="30:18,70:22,0:30",
                    help="Gate script GATE:SECONDS,... (default: 30:18,70:22,0:30)")
    ap.add_argument("--out", type=Path, help="Output session dir (default: data/telemetry/<stamp>-SYNTHETIC-plant-probe)")
    ap.add_argument("--base-weight", type=float, default=213.3, help="Empty jar weight on the scale [g]")
    ap.add_argument("--target-g", type=int, default=495, help="Nominal preset target [g]; the sequence ignores it")
    ap.add_argument("--preset-name", default="Synthetic thick honey")
    ap.add_argument("--pre-s", type=float, default=2.0, help="Quiet seconds before the first gate step")
    ap.add_argument("--post-s", type=float, default=1.5, help="Quiet seconds after the script ends")
    ap.add_argument("--seed", type=int, default=20260716, help="RNG seed, so runs are reproducible")

    ov = ap.add_argument_group("plant overrides (default: identified from --reference)")
    for name in PARAM_NAMES:
        ov.add_argument(f"--{name.replace('_', '-')}", type=float, default=None)

    ap.add_argument("--plot", action="store_true",
                    help="Run split_runs.py + plot_runs.py on the result, in the ch. 07 plant-probe "
                         f"style ({' '.join(DEFAULT_PLOT_ARGS)})")
    ap.add_argument("--plot-arg", action="append", default=[], metavar="ARG",
                    help="Extra argument appended to plot_runs.py, repeatable. Use --plot-arg=--foo "
                         "for flags, so argparse does not eat them.")
    ap.add_argument("--dry-run", action="store_true", help="Identify and report only, write nothing")
    args = ap.parse_args()

    command = "python3 " + " ".join([Path(sys.argv[0]).name] + sys.argv[1:])

    ref_dir = resolve_session_dir(args.reference)
    fill_path = ref_dir / "fills" / f"fill_{args.reference_fill:04d}.ndjson"
    if not fill_path.exists():
        raise SystemExit(f"reference fill not found: {fill_path}\n"
                         f"run: python3 {TOOLS_DIR / 'split_runs.py'} {ref_dir}")

    print(f"Reference fill: {fill_path}")
    print("\nIdentified plant:")
    p, report = identify_plant(load_ndjson(fill_path))
    for line in report:
        print(f"  {line}")

    p.provenance = {"session": str(ref_dir), "fill": args.reference_fill, "fill_file": str(fill_path)}

    overridden = []
    for name in PARAM_NAMES:
        v = getattr(args, name)
        if v is not None:
            setattr(p, name, v)
            overridden.append(f"{name}={v}")
    if overridden:
        print("\nOverridden: " + ", ".join(overridden))
    p.provenance["overrides"] = overridden

    steps = parse_script(args.script)
    print("\nGate script: " + " -> ".join(f"{s.gate_pct:.0f} % for {s.duration_s:.0f} s" for s in steps))
    for s in steps:
        if s.gate_pct > 0:
            print(f"  gate {s.gate_pct:5.1f} % -> steady flow {p.steady_rate_gps(s.gate_pct):5.2f} g/s, "
                  f"visible after ~{p.dead_time_s:.1f} s")

    sim = simulate(p, steps, args.seed)
    raw, filt = firmware_rates(sim["t"], sim["mass"])

    print("\nSynthetic run:")
    for line in validate(p, sim, raw, filt, steps):
        print(line)

    if args.dry_run:
        print("\n--dry-run: nothing written")
        return 0

    out_dir = args.out
    if out_dir is None:
        stamp = datetime.now().strftime("%Y.%m.%d_%H:%M")
        out_dir = REPO_ROOT / "data" / "telemetry" / f"{stamp}-SYNTHETIC-plant-probe"
    out_dir = out_dir.resolve()

    tel = emit_session(out_dir, sim, raw, filt, steps, p, args.base_weight, args.target_g,
                       args.preset_name, args.pre_s, args.post_s, args.seed, command)
    print(f"\nWrote SIMULATED session: {out_dir}")
    print(f"  telemetry: {tel}")

    if args.plot:
        print()
        subprocess.run([sys.executable, str(TOOLS_DIR / "split_runs.py"), str(out_dir)],
                       check=True, cwd=TOOLS_DIR)
        print()
        subprocess.run([sys.executable, str(TOOLS_DIR / "plot_runs.py"), str(out_dir)]
                       + DEFAULT_PLOT_ARGS + args.plot_arg, check=True, cwd=TOOLS_DIR)
        print(f"\nFigures: {out_dir / 'figures'}")

    print("\nNOTE: this session is SIMULATED. Label figures made from it as simulated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
