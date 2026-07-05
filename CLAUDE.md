# Honey-Jar-Filler — project & session context

This file is auto-loaded by Claude Code. It carries the context needed to
continue firmware/thesis work without re-explaining (e.g. from a synced laptop).
It is a **handoff/orientation doc**, not exhaustive API docs — read the code for
specifics.

## What this is
Firmware + KiCad PCB + CAD for a machine that fills honey jars, and the subject
of Jonny's **bachelor thesis**. The thesis evaluates **dosing strategies** and
the difficulty of accurately dosing a viscous medium.

- Thesis LaTeX: `/home/jonny/HAW-LA/s8/latex` (may not be present on every
  machine). Well-developed chapters: `07_prozessanalyse` (the plant problem),
  `08_heuristische`, `09_erweiterte` (adaptive); `10_regelungsbasierter` and
  `11_strategievergleich` are still skeletons.
- Strategy control-diagram drafts (drawio, in the thesis repo):
  `Figures_src/diagrams_flow-cascade_DRAFT.drawio` — portrait + horizontal
  cascade/Smith-predictor block diagrams and a flowchart, in Jonny's style
  (pastel fills, `#555555` borders, Helvetica 13). Master diagrams:
  `Figures_src/diagrams_honey-jar-filler.drawio`.

## Build / run
- ESP-IDF v5.5.1, ESP32. Source the env then build:
  `source /home/jonny/esp/v5.5.1/esp-idf/export.sh` then in
  `firmware_honey-jar-filler/`: `idf.py build` (or `idf.py flash`).
- Telemetry capture (host): `tools/telemetry/capture.py` — captures ESP32
  console, splits `TEL {...}` JSON telemetry into
  `data/telemetry/<session>/telemetry.ndjson`, shows a live weight chart +
  scrolling log. Plot: `tools/telemetry/plot_runs.py`, `split_runs.py`,
  `analysis.py`.

## The core plant problem (thesis ch. 07)
Actuator = gate/flap position `α` (%). Measured/target = weight (HX711). Hard parts:
- **Dead time** `L` (~2–6 s, thick honey more) between a gate change and a
  visible weight change; worsened by discrete, averaged sampling. **`L` is not
  constant during a fill** — as the jar fills, fall height shrinks and `L` drops.
- **Drip / Nachlauf**: weight keeps rising after the gate closes (post-close gain).
- **Nonlinear** gate→flow; **drifting disturbances** (viscosity, temperature,
  falling bucket level) change *within* and *between* fills.
So fixed thresholds can't stay optimal → the strategies compensate differently.

## Strategies (`firmware_honey-jar-filler/components/filler/`)
Registered in `app.h` (`app_fill_strategy_t`), `app.c` (`k_strategy_names`),
`filler_strategy.c` (dispatch). Selectable in the OLED menu.
- **heuristic** (`filler_strategy_heuristic.c`) — fixed thresholds, baseline (ch.08).
- **adaptive-heuristic** (`filler_strategy_adaptive_heuristic.c`) — same FSM,
  learns dead time / post-close gain / rates / near-close / drip between fills
  via EWMA (ch.09). **Primary learning-trend chart.**
- **flow-control** (`filler_strategy_flow_control.c`) — adaptive + a P-controller
  that regulates the gate to a rate setpoint (two fixed setpoints).
- **flow-cascade** (`filler_strategy_flow_cascade.c`) — the "regelungsbasierter
  Ansatz" (thesis 3rd strategy). Cascade: inner **PI** rate loop with feedforward
  gate `= ṁ*/K̂`, outer **continuous deceleration rate profile**, **Smith
  predictor** dead-time compensation `fb = ŷ + (ṁ_m − ŷ_d)`, online plant-gain
  `K̂` ID. **UNTESTED on hardware.** Conservative by design.
- **manual** (`filler_strategy_manual.c`) — encoder→gate, button→advance.
- **sequence** (`filler_strategy_sequence.c`) — scripted timed gate script
  (edit `k_seq[]`), no target/learning; for a clean Strecke (plant) reference
  chart for the thesis.

Shared FSM: `FILLER_FILL → DRIP_WAIT → VERIFY_TARGET → (refill loop)`; runtime
state in `filler_strategy_runtime_t` (`filler_strategy.h`).

## Learning model (adaptive / flow-control / flow-cascade)
Per-preset learned entry, EWMA across fills with a **count warmup**
(`learn_ewma`: `α = max(base, 1/(n+1))`) so the **first successful fill
dominates** the conservative seed → converges in ~1–2 jars, then smooth.
Learned: dead_time, post_close_gain, fast/slow rate, drip_wait, finish_trim,
and (cascade) `K̂`. Seeds derived from preset params. `finish_trim` is a signed
cross-run integrator on terminal error — use a plain signed EWMA (the
`ewma()` `prev<=0` shortcut corrupts signed values).

**Thesis illustration toggle:** `ADAPT_LEARN_SLOW` (compile-time, default 0) in
the adaptive strategy. Set 1 to disable warmup + scale alphas ×0.35 so the
convergence trend is clearly visible over many fills. Temporary; reflash to
switch. (Commit `475a68e`.)

## Parameter map
- **Auto-learned** (per preset, across fills): dead_time, post_close_gain,
  fast/slow rate, `K̂` (cascade), finish_trim, drip_wait.
- **Preset params** you set (seeds/bounds/target; `app_params_def.h`):
  target_grams+tol, `max_gate_pct` (cap + seeds `K̂`), `close_remaining_g`,
  `drip_delay_ms`, `fill_timeout_ms`. Note `slow_remaining_g`/`slow_gate_pct`
  are **unused by flow-cascade** (its profile replaces them).
- **Static `#define`** (tune in code): cascade `CASC_KP/KI`, `CASC_STEP_MAX`,
  `CASC_DECEL_BAND_*`, `CASC_MODEL_TAU_S`, `CASC_GATE_GAIN_MIN/MAX`, learn
  alphas, safety limits.

Param renames done earlier: `near_close_delta_g→slow_remaining_g`,
`near_close_gate_pct→slow_gate_pct`, `close_early_g→close_remaining_g`. Telemetry
logs preset params as `VAR(<field>)`; `analysis.py` keeps legacy aliases.

## Safety failsafes (in adaptive & flow-* strategies)
hard-overfill fault, no-response (empty bucket), fill-timeout (stall), safe-rate
cutback with an absolute jar-size ceiling + spike debounce + runaway→fault
(protects the first jar / thin honey), proportional refill relax. Every fault
publishes a summary.

## Telemetry fields (per-sample, NDJSON)
weight, gate, raw/filtered rate, `target_rate_gps` (ṁ*), `rate_error_gps`,
learned/measured dead_time/post_close/rates, adapted close/near thresholds, and
(cascade) `model_rate_gps` (ŷ), `model_delayed_gps` (ŷ_d), `control_integ_pct`
(PI state), `gate_gain_gps_per_pct` (K̂). Per-fill summary has `used_*`/`next_*`
for the learning-trend plots.

## Known limitations / to validate on hardware
- flow-cascade is untested; watch: `K̂` converging after the gate steadies,
  `ŷ_d` tracking measured rate once flow starts, PI integrator not railing,
  gate smooth (not hunting).
- Dead time is measured once at fill start and held constant — the within-fill
  decrease (falling head height) is NOT compensated (documented limitation;
  `finish_trim` + Smith correction absorb the bias to first order).
- `K̂` is a single linearized gain; sampled only in steady state (gate held >
  dead time). Gate→flow is nonlinear.

## Conventions
- Commits: `FW:` (firmware) / `TOOL:` (host tools) prefix, comma-separated
  changes; "Add new fill strategy: <name> (...)" for new strategies. End with
  `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`.
- The clangd "Unknown argument '-mlongcalls'…" IDE errors are xtensa-vs-clang
  noise — ignore; trust `idf.py build`.

## Current state / next
All strategies build clean. Recent commits: honey-flow param rename → adaptive
learning+failsafe optimizations → flow-control port → flow-cascade strategy →
sequence strategy + slow-learn toggle → capture.py scroll fix. **Next:** test
all strategies on hardware and collect representative thesis charts (especially
the adaptive learning trend); likely small live tuning of the cascade
gains/profile and possibly the LaTeX control equations to match the diagrams.
