"""Closed-loop honey plant model for the strategy simulator.

Same physics as tools/telemetry/synth_plant_session.py (which is open-loop and
script-driven), restructured as a stateful object that is stepped every 10 ms
with the gate the firmware currently commands, plus the effects a whole
*session* needs and a single fill does not:

  bucket head        flow scales with the honey left in the bucket,
                     factor = ((m + m_off)/(m_ref + m_off))**head_exp
                     (head_exp 1 ~ viscous/Hagen-Poiseuille, 0.5 ~ thin/Torricelli)
  head-dependent L   the opening dead time is the time to build a strand of
                     fixed mass:  L_open = l_min + m_build / q_ss.
                     Fitted from the thick-honey session (adaptive-fix4: L grows
                     4.8 -> 8.2 s while the 100 % rate falls 7.0 -> 3.6 g/s,
                     L*rate ~ const) and the thin dummy (test6: 0.65 -> 1.0 s).
  gate -> flow       q_ss = K * head * max(0, gate - onset)**curve
  gate moves while   increase: short adjust delay (strand already exists),
  flowing            decrease/close: close_delay
  jar swap           the scale reads the new empty jar; drips during the swap
                     are lost to the carousel; the bucket keeps draining only
                     through the gate.

Calibrations below are fitted by hand from the recorded sessions (see
tools/sim/README.md). They are deliberately "plausible ranges", not exact
identifications - scenarios sweep around them.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, replace, asdict


@dataclass
class Medium:
    name: str
    K: float                 # g/s per %**curve at the reference head
    onset: float             # gate % where flow starts (gasket overlap)
    curve: float = 1.0       # exponent of (gate - onset); <1 concave/saturating
    l_min_s: float = 0.5     # dead time floor (servo + fall)
    m_build_g: float = 5.0   # strand build-up mass -> L_open = l_min + m_build/q
    adjust_delay_s: float = 0.4   # delay of a gate INCREASE while already flowing
    close_delay_s: float = 0.3    # flap shut / reduced -> flow follows
    tau_s: float = 0.4       # first-order flow lag
    residue_g: float = 1.5   # flap residue that drains after close (slow drip)
    residue_tau_s: float = 4.0
    onset_pool_g: float = 2.0
    close_impact_g: float = 3.0
    onset_impact_g: float = 1.5
    impact_tau_s: float = 0.25
    breakup_rate_gps: float = 3.0
    drop_mass_g: float = 0.15
    flow_noise_frac: float = 0.16
    turb_tau_s: float = 0.4
    head_exp: float = 1.0
    m_ref_g: float = 6000.0  # bucket mass at which K is specified
    m_off_g: float = 300.0   # residual head when "empty" at the outlet
    scale_noise_g: float = 0.08
    scale_avg_ms: float = 200.0


MEDIA: dict[str, Medium] = {
    # Real thick honey, 2026-06-14 (adaptive-fix4 / heuristic sessions):
    # 100 % -> ~7-11 g/s, 35 % -> ~2.6-3.1 g/s at the start of a session,
    # L 4.8 s rising to 8.2 s, Nachlauf ~9 g, 500 g jars take 100-240 s.
    "thick": Medium(name="thick", K=0.095, onset=7.4, curve=1.0,
                    l_min_s=1.25, m_build_g=25.0, adjust_delay_s=1.5, close_delay_s=0.5,
                    tau_s=1.2, residue_g=5.0, residue_tau_s=10.0, onset_pool_g=7.8,
                    close_impact_g=12.2, onset_impact_g=3.5, breakup_rate_gps=3.0,
                    drop_mass_g=0.45, head_exp=1.0, m_ref_g=7400.0),
    # Same honey but with a saturating gate curve (the fix4 35/100 % ratio
    # suggests (gate-onset)**0.6 with onset 13 %): tests model mismatch.
    "thick_concave": Medium(name="thick_concave", K=0.48, onset=13.0, curve=0.6,
                            l_min_s=1.25, m_build_g=25.0, adjust_delay_s=1.5, close_delay_s=0.5,
                            tau_s=1.2, residue_g=5.0, residue_tau_s=10.0, onset_pool_g=7.8,
                            close_impact_g=12.2, onset_impact_g=3.5, breakup_rate_gps=3.0,
                            drop_mass_g=0.45, head_exp=1.0, m_ref_g=7400.0),
    # Water-thin dummy, 2026-07-15..19 (test6 etc.): offline truth K~1.4,
    # onset ~19 %; 50 % gives ~41 g/s early, collapsing to ~11 g/s as the
    # (small) bucket drains; L 0.65-1.0 s, Nachlauf ~5-6 g, drips.
    "thin_dummy": Medium(name="thin_dummy", K=1.4, onset=19.0, curve=1.0,
                         l_min_s=0.5, m_build_g=5.5, adjust_delay_s=0.3, close_delay_s=0.3,
                         tau_s=0.3, residue_g=1.5, residue_tau_s=3.0, onset_pool_g=2.0,
                         close_impact_g=2.5, onset_impact_g=1.5, breakup_rate_gps=4.0,
                         drop_mass_g=0.12, flow_noise_frac=0.25, turb_tau_s=0.5,
                         head_exp=0.5, m_ref_g=2300.0, m_off_g=200.0),
    # Low-viscosity dark honey, 2026-07-05: 50 % -> 66 g/s, 27 % -> 8.5 g/s
    # (K~2.5, onset ~23.6 %), L ~1.3 s, Nachlauf ~9 g, mild drain.
    "dark_thin_honey": Medium(name="dark_thin_honey", K=2.5, onset=23.6, curve=1.0,
                              l_min_s=0.6, m_build_g=12.0, adjust_delay_s=0.5, close_delay_s=0.4,
                              tau_s=0.5, residue_g=3.0, residue_tau_s=5.0, onset_pool_g=4.0,
                              close_impact_g=5.0, onset_impact_g=2.0, breakup_rate_gps=3.0,
                              drop_mass_g=0.3, head_exp=1.0, m_ref_g=8700.0),
    # Very thick / cold or creamed honey (extrapolated, not recorded): half the
    # thick flow, longer strand build -> ~2 g/s at 50 %, ~20 s opening dead time.
    "very_thick": Medium(name="very_thick", K=0.05, onset=10.0, curve=1.0,
                         l_min_s=1.5, m_build_g=40.0, adjust_delay_s=2.0, close_delay_s=0.6,
                         tau_s=1.5, residue_g=6.0, residue_tau_s=12.0, onset_pool_g=8.0,
                         close_impact_g=12.0, onset_impact_g=3.0, breakup_rate_gps=2.0,
                         drop_mass_g=0.5, head_exp=1.0, m_ref_g=7400.0),
    # Fake honey, 2026-06-17: 27 % stable ~9 g/s, 50 % ~39 g/s (K~1.3, onset ~20 %).
    "fake_honey": Medium(name="fake_honey", K=1.3, onset=20.0, curve=1.0,
                         l_min_s=0.5, m_build_g=6.0, adjust_delay_s=0.3, close_delay_s=0.3,
                         tau_s=0.35, residue_g=1.5, residue_tau_s=3.0, onset_pool_g=2.0,
                         close_impact_g=2.5, onset_impact_g=1.5, breakup_rate_gps=4.0,
                         drop_mass_g=0.12, flow_noise_frac=0.22, head_exp=0.5, m_ref_g=4500.0),
}


SIM_DT_S = 0.01
SAMPLE_DT_S = 0.1


class Plant:
    def __init__(self, medium: Medium, bucket_g: float, jar_tare_g: float = 230.0, seed: int = 1):
        self.m = medium
        self.bucket_g = bucket_g
        self.rng_turb = random.Random(seed)
        self.rng_meas = random.Random(seed + 1)
        self.rng_jar = random.Random(seed + 2)
        self.t = 0.0
        self.q = 0.0
        self.turb = 0.0
        self.residue = 0.0
        self.pool_left = 0.0
        self.pending = 0.0
        self.impact = 0.0
        self.jar_tare = jar_tare_g
        self.jar_mass = 0.0          # honey resting in the current jar
        self.gate_cmd = 0.0
        self.gate_eff = 0.0
        self.sched: list[tuple[float, float]] = []   # (effective time, gate)
        self.first_contact = False
        self.hist: list[float] = []  # apparent mass for the scale averaging window
        self.dispensed_g = 0.0       # total mass that left the bucket
        self.ext_g = 0.0             # external force on the scale (hand press), apparent grams
        self.jar_present = True      # False: jar taken away, honey lands on the carousel
        self.spilled_g = 0.0         # honey that did not land in a jar

    # ------------------------------------------------------------------
    def head_factor(self) -> float:
        m = self.m
        x = max(0.0, self.bucket_g + m.m_off_g) / (m.m_ref_g + m.m_off_g)
        return x ** m.head_exp

    def steady_rate(self, gate: float) -> float:
        d = gate - self.m.onset
        if d <= 0.0 or self.bucket_g <= 0.0:
            return 0.0
        return self.m.K * self.head_factor() * d ** self.m.curve

    def open_dead_time(self, gate: float) -> float:
        q = self.steady_rate(gate)
        if q <= 0.05:
            return 30.0
        return self.m.l_min_s + self.m.m_build_g / q

    def set_medium(self, medium: Medium):
        self.m = medium

    def add_to_bucket(self, grams: float):
        self.bucket_g += grams

    def remove_jar(self):
        self.jar_present = False

    def swap_jar(self, tare_g: float | None = None):
        self.jar_present = True
        self.jar_tare = tare_g if tare_g is not None else 225.0 + 10.0 * self.rng_jar.random()
        self.jar_mass = 0.0
        self.pending = 0.0
        self.impact = 0.0
        self.hist.clear()

    # ------------------------------------------------------------------
    def command(self, gate: float):
        """Apply the firmware's gate command at the current time."""
        if abs(gate - self.gate_cmd) < 1e-6:
            return
        prev = self.gate_cmd
        last_eff_t = self.sched[-1][0] if self.sched else -1e9
        if gate <= 0.0:
            delay = self.m.close_delay_s
        elif prev <= 0.0 and self.q < 0.05:
            delay = self.open_dead_time(gate)
            self.first_contact = False
        elif gate > prev:
            delay = self.m.adjust_delay_s
        else:
            delay = self.m.close_delay_s
        eff_t = max(self.t + delay, last_eff_t)
        # a command that lands before an older, not-yet-effective one replaces it
        self.sched = [(et, g) for et, g in self.sched if et < eff_t]
        self.sched.append((eff_t, gate))
        self.gate_cmd = gate

    def step(self):
        m = self.m
        dt = SIM_DT_S
        while self.sched and self.sched[0][0] <= self.t:
            self.gate_eff = self.sched.pop(0)[1]
        q_ss = self.steady_rate(self.gate_eff)
        if q_ss > 0.0:
            self.q += (q_ss - self.q) * (dt / max(m.tau_s, 1e-3))
        else:
            self.q += (0.0 - self.q) * (dt / max(m.tau_s, 1e-3))
            if self.q < 0.02:
                self.q = 0.0

        sigma = m.flow_noise_frac * math.sqrt(2.0 * dt / max(m.turb_tau_s, 1e-3))
        self.turb += -self.turb * (dt / max(m.turb_tau_s, 1e-3)) + sigma * self.rng_turb.gauss(0.0, 1.0)
        q_out = max(0.0, self.q * (1.0 + self.turb)) if self.q > 0.02 else 0.0
        if q_out * dt > self.bucket_g:
            q_out = max(0.0, self.bucket_g / dt)

        if self.gate_cmd > 0.0 and q_ss > 0.0:
            self.residue = m.residue_g
            residue_flow = 0.0
        elif self.residue > 0.0:
            drained = self.residue * (dt / max(m.residue_tau_s, 1e-3))
            self.residue -= drained
            residue_flow = drained / dt
        else:
            residue_flow = 0.0

        self.bucket_g -= q_out * dt
        self.dispensed_g += q_out * dt
        jar_flow = q_out + residue_flow
        arriving = jar_flow * dt
        if q_out > 0.0 and not self.first_contact:
            self.first_contact = True
            self.pool_left = m.onset_pool_g
            self.impact += m.onset_impact_g
        if self.pool_left > 0.0:
            rel = min(self.pool_left, m.onset_pool_g * (dt / 0.2))
            self.pool_left -= rel
            arriving += rel
        if not self.jar_present:
            self.spilled_g += arriving
            arriving = 0.0
        self.pending += arriving
        if jar_flow > m.breakup_rate_gps:
            self.jar_mass += self.pending
            self.pending = 0.0
        else:
            while self.pending >= m.drop_mass_g:
                self.jar_mass += m.drop_mass_g
                self.pending -= m.drop_mass_g
        self.impact *= math.exp(-dt / max(m.impact_tau_s, 1e-3))
        on_scale = (self.jar_tare + self.jar_mass + self.impact) if self.jar_present else 0.0
        self.hist.append(on_scale + self.ext_g)
        n = max(1, int(round((m.scale_avg_ms / 1000.0) / dt)))
        if len(self.hist) > n:
            del self.hist[: len(self.hist) - n]
        self.t += dt

    def close_impact(self):
        """Called when the gate command goes to 0 (flap shuts, strand snaps)."""
        self.impact += self.m.close_impact_g

    def read_scale(self) -> float:
        avg = sum(self.hist) / len(self.hist) if self.hist else self.jar_tare
        return avg + self.rng_meas.gauss(0.0, self.m.scale_noise_g)


def medium(name: str, **overrides) -> Medium:
    return replace(MEDIA[name], **overrides)


def medium_dict(m: Medium) -> dict:
    return asdict(m)
