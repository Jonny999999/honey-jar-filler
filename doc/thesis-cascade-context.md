# Thesis context brief — regelungsbasierter Ansatz (flow-cascade)

**Purpose.** Self-contained context for a separate Claude session that writes the
LaTeX thesis section on the cascade / control-based dosing strategy. It captures
the *design rationale, rejected alternatives, and limitations* that are NOT
recoverable from reading the code alone, plus pointers to the code and the
telemetry data to interpret. Read this together with the code and the charts;
this file is the "why", the code is the "what", the telemetry is the "results".

> **Do not paste code verbatim into the thesis.** Use this to reason about and
> justify the design in prose/equations in Jonny's established chapter style.

---

## 0. Target file / chapter — CONFIRM FIRST

Jonny pointed at `09_erweiterte_dosierstrategie.tex`, but the chapter naming says
otherwise. Current chapters in `/home/jonny/HAW-LA/s8/latex/Chapters/`:
- `08_heuristische_dosierstrategie.tex` — heuristic (baseline).
- `09_erweiterte_dosierstrategie.tex` — **adaptive-heuristic** (the "erweiterte"
  learning strategy). The cascade is NOT this one.
- `10_regelungsbasierter_ansatz.tex` — **the cascade / control-based approach.
  This is the section this brief is for.**
- `11_strategievergleich.tex` — comparison.

→ The cascade material almost certainly belongs in **`10_regelungsbasierter_ansatz.tex`**.
Confirm with Jonny before writing into `09`. (09 and 10 share the same learning
substrate, so some cross-referencing between them is expected.)

---

## 1. Absolute paths

**Firmware — the cascade strategy and its shared substrate**
- Cascade implementation (primary):
  `/home/jonny/projects/honey-jar-filler/firmware_honey-jar-filler/components/filler/filler_strategy_flow_cascade.c`
- Runtime/learned-entry structs, telemetry field defs:
  `/home/jonny/projects/honey-jar-filler/firmware_honey-jar-filler/components/filler/filler_strategy.h`
- Adaptive strategy (shared learning model, ch.09, useful for contrast):
  `/home/jonny/projects/honey-jar-filler/firmware_honey-jar-filler/components/filler/filler_strategy_adaptive_heuristic.c`
- Flow-control strategy (single P-loop, the conceptual step *before* the cascade):
  `/home/jonny/projects/honey-jar-filler/firmware_honey-jar-filler/components/filler/filler_strategy_flow_control.c`
- Sequence strategy (scripted plant probe, produces the clean Strecke reference):
  `/home/jonny/projects/honey-jar-filler/firmware_honey-jar-filler/components/filler/filler_strategy_sequence.c`
- Preset params (targets/seeds/bounds): 
  `/home/jonny/projects/honey-jar-filler/firmware_honey-jar-filler/components/app/app_params_def.h`

**Supporting docs**
- Handoff/orientation (project-wide context): `/home/jonny/projects/honey-jar-filler/CLAUDE.md`
- Hardware retest checklist for the cascade fixes:
  `/home/jonny/projects/honey-jar-filler/doc/cascade-affine-gain-testnotes.md`

**Telemetry data — 2026-07-05 hardware session** (dir:
`/home/jonny/projects/honey-jar-filler/data/telemetry/`). Each run dir has
`telemetry.ndjson` (per-sample) + a per-fill summary; plot with
`tools/telemetry/plot_runs.py`.
- `2026.07.05_21:00-x-dunkler-honig_cascade_ratereduced-learnfromoverfill/` —
  **THE cascade run to discuss.** Real dark honey. Shows the three problems the
  fixes address (overfill +15..23 g every fill, gate stays ~full open until
  close, model K drifts 0.59→0.29). This is "cascade, pre-fix" data.
- `2026.07.05_19:55…_cascade-default01/`, `…20:07…cascade-02-fix-more-flow/`,
  `…20:21…cascade-03-allow100percent/`, `…20:24…cascade-04-retry/` — earlier
  same-day cascade runs on **dummy honey** (tuning iterations; lower value,
  context only).
- `2026.07.05_21:09-x-dunkler-honig_sequence_30-70longer/` — **scripted Strecke
  probe** (sequence strategy, gate 30 %→70 %→close). Best data for showing the
  raw plant response in isolation: dead time, rate build-up per gate step,
  post-close drip. Use for the Strecke/Prozessanalyse figure this section leans on.
- `2026.07.05_21:10-x-dunkler-honig-manual-sweep/` — **manual gate sweep**.
  Directly evidences the gate→flow relationship (nonlinearity + non-zero flow
  onset). Strong empirical support for the affine-model justification below.
- `2026.07.05_20:41…adaptive_default-quick-learn/`,
  `…20:45…adaptive_slow-learning/` — adaptive strategy (ch.09 learning trend).
- `2026.07.05_21:13…flowcontrol_rest/` — flow-control strategy.

> Caveat: all cascade fills in these runs are **pre-fix**. The affine-model +
> overfill + profile fixes (commit `63dc0d6`) are **untested on hardware**. Write
> the results section around the pre-fix data as "erste Inbetriebnahme / erkannte
> Probleme", and frame the fixes as the corrective design step — do NOT claim
> post-fix performance numbers until a retest exists.

---

## 2. What the cascade strategy is (architecture)

The "regelungsbasierter Ansatz": a **cascade control** with feedforward and
dead-time compensation, closing the loop on the **mass flow rate** ṁ rather than
on weight thresholds. Actuator is the gate/flap position α ∈ [0,100] %; the
controlled variable inside the loop is ṁ [g/s]; the outer objective is hitting
the target mass.

Layers, outer → inner:

1. **Outer trajectory / deceleration profile** (`profile_target_rate_gps`,
   flow_cascade.c ~L547). A *function of the remaining mass*, NOT a feedback
   loop. Produces the rate setpoint ṁ*(remaining):
   - `terminal = clamp(fast · 0.25, 3 g/s, fast)` — a genuine slow approach rate.
   - `in_flight = L̂·ṁ_meas + post_close_gain` — mass that will still arrive after
     "now" (dead-time + drip), subtracted so the taper is referenced to the
     mass that is *still controllable*: `eff_remaining = remaining − in_flight`.
   - `band = max(20 g, post_close·1.6 + L̂·fast)` — the deceleration window width,
     scaled by the plant's own lag/drip.
   - `frac = clamp(eff_remaining / band, 0, 1)`;  **ṁ\* = terminal + (fast − terminal)·frac**.
   → a linear taper from the fast cruise rate down to the terminal approach rate
   as the jar fills.

2. **Inner PI rate loop with feedforward** (`cascade_control_step`, ~L590):
   - Feedforward (Steuerung baseline): **gate_ff = (ṁ* − b) / K̂** — invert the
     learned affine plant model to get the gate that should produce ṁ*.
   - PI trims the residual: `gate = gate_ff + Kp·e + I`, with `Kp = 1.2 %/(g/s)`,
     integral gain `Ki = 0.40 %/(g/s·s)`, anti-windup clamp `I ∈ ±35 %`,
     per-update slew clamp `±6 %`, deadband `0.6 g/s`.
   - **Dead-time-paced holdoff**: the integrator is advanced by a holdoff time
     tied to the dead time (300–2500 ms), so the loop does not react faster than
     the plant can physically respond — a pragmatic dead-time defense.

3. **Smith predictor** (`cascade_update_model`, ~L389) forms the feedback the PI
   error uses: **e = ṁ* − fb**, where **fb = ŷ + (ṁ_meas − ŷ_d)**.
   - ŷ = model rate: the affine steady-state rate `K̂·gate + b` passed through a
     first-order lag `τ = 0.60 s` (`CASC_MODEL_TAU_S`).
   - ŷ_d = ŷ delayed by the learned dead time L̂ (a 256-deep ring buffer).
   - Cold-start: while history < L̂, ŷ_d = 0, which prevents predictor windup
     during the initial dead time when measured rate is also ~0.
   → Standard Smith structure: the controller "sees" the fast model prediction ŷ
   for responsiveness, corrected by the slow real measurement mismatch
   (ṁ_meas − ŷ_d) for accuracy.

4. **Online plant identification** — the affine model `K̂, b` (section 3).

**Shared FSM & learning substrate** (common with adaptive/flow-control):
`FILLER_FILL → DRIP_WAIT → VERIFY_TARGET → (refill loop)`. Per-preset learned
entries via count-warmup EWMA (`learn_ewma`, `α = max(base, 1/(n+1))`) so the
first successful fill dominates the conservative seed → converges in ~1–2 jars.
Learned per preset: dead_time, post_close_gain, fast/slow rate, drip_wait,
finish_trim, and `K̂,b`. **Learned state is RAM-only** (no NVS) — a reboot resets
to seeds; this is intentional so cold-start convergence is demonstrable.

---

## 3. The central design decision: the plant model  ṁ = K̂·gate + b

This is the most defensible/interesting design story for the thesis, because two
alternatives were tried and rejected first.

**The plant:** gate→flow is nonlinear, has a **non-zero flow onset** (~15 % gate
before any honey moves — see the manual-sweep run), and drifts with viscosity /
temperature / falling bucket level. So no fixed gate↔rate map holds.

**Attempt 1 — single scalar gain K̂ (`ṁ = K̂·gate`, line through origin).**
Failed because forcing the line through the origin makes `K̂ = ṁ/gate`
operating-point-dependent: near onset it's small, wide open it's larger. Sampling
K̂ at 60 % gate then using it at 30 % mis-predicts. In the 07-05 cascade run this
showed as an *apparent* K drift 0.59→0.29 — not the plant changing, but the
scalar being read off a curve at different points.

**Attempt 2 — 6-point gate→rate breakpoint schedule (10/20/…/90 %), interpolated.**
Rejected on a structural argument (this is the key insight): **the cascade
controller only ever visits two gate regimes** — a high gate during the fast
cruise phase and a low gate during the terminal approach. It never parks at the
intermediate breakpoints long enough (past the dead time) to gather steady-state
data there, so those points stay at their seed values forever and the
interpolation is fiction. A model the controller cannot excite is untrainable.

**Chosen — affine model `ṁ = K̂·gate + b`.** One slope + one offset:
- `b` (offset, clamped [−60, +15] g/s, **normally negative**) captures the flow
  onset the origin-line missed. Interpretation: gate at which flow starts ≈ −b/K̂.
- Fit **live from exactly the two operating points the controller does visit**:
  the fast phase feeds a high-gate EWMA point (gate_hi, rate_hi), the slow phase
  a low-gate point (gate_lo, rate_lo). Refit each update: `K̂ = Δrate/Δgate`,
  `b = rate_hi − K̂·gate_hi`, but only when the two gates are ≥ 8 %
  (`CASC_GAIN_MIN_GATE_SEP_PCT`) apart (else hold K̂, re-solve only b).
- Samples are taken **only in steady state** — gate held longer than
  `L̂ + 0.5 s` — so a measured rate reflects the *current* gate, not the one
  commanded a dead time ago. In-fill EWMA `α = 0.15`; the converged K̂,b persist
  across fills via signed-safe `learn_ewma`.
- **First-fill caveat:** `b` seeds to 0, so the very first fill's feedforward runs
  slightly rich at low gates until `b` is learned. Converges within a fill or two.

Framing for the thesis: this is a **linearized (affine) secant model** valid
*between* the two operating points, identified online — deliberately the simplest
model that (a) the controller can actually excite and (b) captures the onset
offset. Honest about being a local approximation of a nonlinear Strecke.

---

## 4. Steuerung vs. Regelung — the terminology framing

Useful for precise German control-engineering language:
- The **outer deceleration profile is a Steuerung / Trajektoriengenerator**: ṁ*
  is a pure function of remaining mass, no feedback. It is *not* a control loop.
- The **inner PI + feedforward is the Regelung**: feedback closes on the rate
  error via the Smith-predicted rate.
- The **feedforward term (gate_ff = (ṁ*−b)/K̂) is a modellbasierte Vorsteuerung**
  inside the Regelung — it does the bulk of the work; the PI only trims.
- The overall structure = **Kaskadenregelung mit Smith-Prädiktor und Vorsteuerung**.
Calling the whole strategy "regelungsbasierter Ansatz" is correct; be precise
that the outer layer itself is a Steuerung.

---

## 5. Why a smooth taper instead of a hard step (design justification)

For a plant with **dead time + drip (Nachlauf)**, arriving at the close point
*slowly* is what makes the close precise. Reasons to develop in prose:
- **Dead time:** a command arrives ~L̂ later. A hard step down still has the old
  high flow in flight during L̂ → overshoot. A ramp bleeds the rate down *before*
  the close, so the in-flight mass in the final L̂ is already small.
- **Drip ∝ rate-at-close:** post-close gain scales with the flow when the gate
  shuts. Arriving at ~terminal (≈0.25·fast) instead of full rate makes the drip
  small *and predictable*, which is what the post-close-gain learning relies on.
- **Loop health:** a hard step slams the PI with a huge transient and excites
  windup; the ramp keeps the error small and the loop in its linear regime.
- The optimum is "arrive as slowly as precision needs, but no slower" (slower =
  wasted cycle time). Linear taper = simplest monotonic profile with an
  independently tunable terminal rate. A `sqrt`/constant-deceleration profile
  would be marginally more time-optimal — note as *Ausblick/future work*, not done.

---

## 6. Deliberately NOT implemented (limitations / future work — be honest)

Good thesis material; frame as documented limitations, several absorbed to first
order by other mechanisms:
- **Within-fill dead-time variation not compensated.** L̂ is measured once at fill
  start and held constant, but L actually *shrinks* as the jar fills (fall height
  drops). The `finish_trim` cross-run integrator + the Smith correction absorb the
  resulting bias to first order, but it is not explicitly modeled.
- **K̂ is a single linearized/affine secant**, sampled only in steady state.
  Gate→flow is genuinely nonlinear; extrapolating K̂,b outside the two operating
  points is unreliable.
- **No NVS persistence** of learned state — reset on reboot (intentional for the
  cold-start demonstration, but a product would persist it).
- **Terminal rate is a fixed fraction (0.25) of fast**, not itself optimized.
- **Two-point ID needs a distinct slow phase** (≥ 8 % gate separation). If a fill
  never really slows, only `b` updates and K̂ is held — a structural dependency
  worth stating.
- **Untested post-fix on hardware** — the affine model and the overfill/profile
  fixes are validated only in reasoning/replay, not yet on the machine.

---

## 7. The empirical story (interpreting the 07-05 cascade run)

The `…21:00…cascade_ratereduced-learnfromoverfill` run is the honest "erste
Inbetriebnahme" evidence. Three coupled failure modes, all with a root cause:
1. **Overfill +15..23 g every fill.** Root cause: the close-threshold clamp
   collapsed to 10 g while the predicted remaining-at-close was 70+ g, so the
   gate closed far too late. (Also a double-count: post_close_gain already
   includes the in-flight mass, and the old code added dead_time·rate on top.)
2. **Gate stayed ~fully open until the very end** (no visible deceleration).
   Root cause: fast_rate and slow_rate had both collapsed to ~30 g/s, so the
   taper referenced to the learned slow rate was flat.
3. **Model prediction drifted worse across fills** — the scalar-K artifact of
   section 3 (K read off a nonlinear curve at a shifting operating point).

The corrective design step (commit `63dc0d6`): affine model (fixes 3), decouple
terminal rate from the learned slow rate as `0.25·fast` (fixes 2), and fix the
close-threshold clamp + remove the double-count (fixes 1). Present the fixes as
*design iterations driven by the measured Strecke behaviour* — a strong narrative
for a "regelungsbasierter Ansatz" chapter.

Supporting figures worth generating: the **manual-sweep** run for the gate→flow
nonlinearity + onset (justifies the affine model empirically); the **sequence**
run for the isolated Strecke response (dead time, per-step rate build-up, drip);
the cascade run for the failure modes above.

---

## 8. Key constants (values, for the equations/tables)

From `filler_strategy_flow_cascade.c`. Cite as tuned #defines, not learned.

| Constant | Value | Meaning |
|---|---|---|
| `CASC_KP_PCT_PER_GPS` | 1.2 | PI proportional gain [%/(g/s)] |
| `CASC_KI_PCT_PER_GPS_S` | 0.40 | PI integral gain [%/(g/s·s)] |
| `CASC_STEP_MAX_PCT` | 6.0 | max gate change per update [%] |
| `CASC_INTEG_LIMIT_PCT` | ±35 | anti-windup integrator clamp [%] |
| `CASC_DEADBAND_GPS` | 0.6 | rate-error deadband [g/s] |
| `CASC_HOLDOFF_MIN/MAX_MS` | 300 / 2500 | dead-time-paced integrator holdoff [ms] |
| `CASC_TERMINAL_RATE_FRAC` | 0.25 | terminal rate = frac·fast |
| `CASC_RATE_MIN_FLOOR_GPS` | 3.0 | terminal-rate floor [g/s] |
| `CASC_DECEL_BAND_MIN_G` | 20 | min deceleration window [g] |
| `CASC_DECEL_BAND_MULT` | 1.6 | post_close multiplier in band width |
| `CASC_GATE_GAIN_MIN/MAX` | 0.02 / 3.00 | affine slope K̂ bounds [g/s per %] |
| `CASC_GAIN_B_MIN/MAX` | −60 / +15 | affine offset b bounds [g/s] |
| `CASC_GAIN_MIN_GATE_SEP_PCT` | 8 | min gate separation to refit slope [%] |
| `CASC_GAIN_SETTLE_MARGIN_S` | 0.5 | settle margin over L̂ before sampling [s] |
| `CASC_GAIN_LIVE_ALPHA` | 0.15 | in-fill operating-point EWMA α |
| `CASC_MODEL_TAU_S` | 0.60 | Smith model first-order lag τ [s] |
| `SMITH_DELAY_MAX` | 256 | Smith delay ring-buffer depth |

Telemetry fields to reference in result plots (per-sample NDJSON): `weight`,
`gate`, `filtered_rate`, `target_rate_gps` (ṁ*), `rate_error_gps`,
`model_rate_gps` (ŷ), `model_delayed_gps` (ŷ_d), `control_integ_pct` (PI state),
`gate_gain_gps_per_pct` (K̂). Per-fill summary has `used_*`/`next_*` for
learning-trend plots.

---

## 9. Suggested section structure (10_regelungsbasierter_ansatz.tex)

1. Motivation: why thresholds/heuristics can't stay optimal on a drifting,
   dead-time+drip plant → close the loop on rate (link back to ch.07 Strecke).
2. Regelungskonzept: the cascade (Steuerung profile → PI+Vorsteuerung → Smith),
   with the block diagram (drawio `diagrams_flow-cascade_DRAFT.drawio`).
3. Streckenmodell: the affine ṁ = K̂·gate + b, the online two-point ID, and the
   rejected-alternatives argument (scalar / breakpoints) — the design highlight.
4. Trajektorie/Steuerung: the deceleration profile + why smooth vs. hard step.
5. Totzeitkompensation: the Smith predictor.
6. Inbetriebnahme & erkannte Probleme: the 07-05 results + corrective iteration.
7. Grenzen & Ausblick: section 6 limitations.

Keep consistent with ch.08/09 style (pastel figures, `#555555` borders,
Helvetica 13 in diagrams). Cross-reference the shared learning model in ch.09
rather than repeating it.
