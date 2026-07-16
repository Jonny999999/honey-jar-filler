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
  gate `= (ṁ* − b)/K̂` (**affine** plant model, see below), outer **continuous
  deceleration rate profile**, **Smith predictor** dead-time compensation
  `fb = ŷ + (ṁ_m − ŷ_d)`, online plant-model ID. **UNTESTED on hardware** (last
  hardware run 2026-07-05 exposed 3 bugs, now fixed but unretested — see
  `doc/cascade-affine-gain-testnotes.md` for the test checklist). Conservative by
  design. Framing: outer profile is a *Steuerung*/trajectory generator (function
  of remaining mass, no feedback); inner PI+feedforward is the *Regelung*.
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
and (cascade) the affine plant model `K̂,b`. Seeds derived from preset params.
`finish_trim` is a signed cross-run integrator on terminal error — use a plain
signed EWMA (the `ewma()` `prev<=0` shortcut corrupts signed values; `learn_ewma`
is signed-safe, use it for anything that can go negative, incl. `b`).

**Cascade affine plant model** (`rate = K̂·gate + b`, commit `63dc0d6`): replaced
an earlier single scalar gain and a briefly-tried 6-point gate→rate breakpoint
schedule. Both failed for the same reason — **the controller only ever visits
two gate regimes** (fast phase = high gate, slow/terminal phase = low gate), so
mid-range breakpoints never get data, and a scalar `rate/gate` slides around
because flow onset is ~15 % gate (**the line does not pass through the origin**;
that's what `b` captures, usually negative). Feedforward inverts it:
`gate = (ṁ*−b)/K̂`. Don't reintroduce a gate-sweep / per-percent schedule — it
can't be trained by this controller.

**K̂,b are identified by RLS** (commit `6215bef`), not a two-point fit. The
two-point version was structurally broken and its failure is worth not repeating:
the "high" point was seeded at 60 % gate, the controller only visits ~12–37 %, and
the bin boundary was the midpoint of the two points (45 %) → every observation
fell in the low bin and the high point never left its seed. At the fixed point the
algebra collapses to `K = rate_lo/(gate_lo − onset)` with `onset = gate − rate/K`
from that *same* point: **two unknowns, one equation**, so the pair just drifted
along the under-determined manifold. Now: RLS over every steady observation,
regressed centered (`rate = K·(gate−REF) + c`) for conditioning, parameters **and
covariance** persisted per preset (identification continues across fills), clamped
with state projection to keep `b ≤ 0` / onset bounded, and `P` bounded so a narrow
gate band can't wind up the unexcited direction. **Don't EWMA the RLS output** —
RLS already *is* the recursive average.

**Thin/sticky-medium tuning (branch `testing`).** Hardware runs with water-thin
fake honey exposed several issues the medium-honey tests never hit. Fixes, all in
`filler_strategy_flow_cascade.c`:
- **Setpoint must be decoupled from the learned rate.** The outer profile's fast
  target is now `target_g / CASC_FAST_FILL_S` (a chosen ~16 s trajectory), NOT
  `learned_fast_rate`. Using the measured rate as the setpoint was a positive-
  feedback loop (thin medium flows fast → learned as fast_rate → targeted → gate
  opens wider → …), diverging start gate 35→100 % and overfilling to 180 g. Only
  the *plant model* (K,b,L,post-close) is learned; the setpoint is a fixed ref.
- **K identification: drip-averaged steady dwell, not instantaneous rate.** A
  sticky medium drips in ~1 s bursts, so any short-window rate is noise. Only
  measure a gate once HELD constant past dead-time, as delivered-mass ÷ dwell-time.
  **One observation is harvested per dwell, at dwell END** (`6680bc0`) — the old
  "fire every sample once the window ≥1.5 s" needed `dead_time + 1.5 s` of a still
  gate, which the `23:10` run met **0–1× per fill**, and only ever on low-gate/
  low-flow dwells (the gate only holds still when little is flowing) → K fitted
  from the flattest part of the curve. ⚠️ The earlier claim "K settles ~0.5 ⇒
  confirmed working" was **wrong**: an offline fit over that session gives
  **K=1.42, b=−27.6, onset=19.5** — the firmware's K was **2.7× too low**.
- **Flow-onset gate ("dead angle") is now a first-class learned near-constant.**
  It's the % the gaskets must clear before any flow (mechanical, ~viscosity-
  independent). Learned as `onset = gate − rate/K̂` (slow EWMA, per-preset), and
  `b = −K̂·onset` is anchored to it so b stops swinging. Controller enforces a
  **minimum-flow floor** = `onset + margin` while filling (never above feedforward)
  so a transient over-reaction can't dip into no-flow and stall.
- **Control feedback = LS slope over ~1.5 s (drip period)** not 0.5 s, + deadband
  0.6→1.5 g/s, so the loop sees the mean flow and holds the gate steady instead of
  chasing each drip burst (the flow↔no-flow limit cycle).
- **`tau` (FOPDT lag) is learned, and is NOT cosmetic** (`ef28112`). A steady-dwell
  window opens exactly when the plant *starts* responding, so the average is a
  blend: `avg = r_new·(1−B) + r_old·B`, `B = (τ/T)(1−e^{−T/τ})`. At τ=0.6 s a 1.0 s
  window reads only **51 %** of the true rate — and the fill's opening dwell (highest
  gate, `r_old=0`) is the most biased, which flattens the fit. So τ is learned from
  the opening dwell (`τ ≈ (1 − avg/r_tail)·T`) and used to invert the blend before
  feeding RLS; it also drives the Smith lag. Windows too short vs τ are rejected,
  not corrected (the de-bias divides by `1−B`).
- Telemetry logs `gain_offset_b_gps`, `flow_onset_gate_pct`, `model_tau_s`,
  `gain_obs_gate_pct`/`gain_obs_rate_gps` (each RLS input as a one-sample impulse →
  scatter them against the fitted line), `gain_rls_p_k` (slope variance =
  convergence signal), and `control_rate_gps`. The per-fill summary now carries
  `used_/next_` for K, b, onset, τ (it previously carried **no** cascade model
  state — cross-run trend charts had nothing to plot). Analyse with
  `tools/telemetry/cascade_analyze.py <session> [--fill N]` — **read `n_obs` and
  `obs_gate_span` first**: too few observations or all at one gate = an excitation
  problem, not a tuning problem.

**Thesis illustration toggle:** `ADAPT_LEARN_SLOW` (compile-time, default 0) in
the adaptive strategy. Set 1 to disable warmup + scale alphas ×0.35 so the
convergence trend is clearly visible over many fills. Temporary; reflash to
switch. (Commit `475a68e`.)

## Parameter map
- **Auto-learned** (per preset, across fills, **RAM-only — no NVS, lost on
  reboot, by design**): dead_time, post_close_gain, fast/slow rate, `K̂`+`b`
  (cascade affine model), finish_trim, drip_wait.
- **Preset params** you set (seeds/bounds/target; `app_params_def.h`):
  target_grams+tol, `max_gate_pct` (cap + seeds `K̂`), `close_remaining_g`,
  `drip_delay_ms`, `fill_timeout_ms`. Note `slow_remaining_g`/`slow_gate_pct`
  are **unused by flow-cascade** (its profile replaces them).
- **Static `#define`** (tune in code): cascade `CASC_KP/KI`, `CASC_STEP_MAX`,
  `CASC_DECEL_BAND_*`, `CASC_MODEL_TAU_S`, `CASC_GATE_GAIN_MIN/MAX`,
  `CASC_GAIN_B_MIN/MAX`, `CASC_GAIN_MIN_GATE_SEP_PCT` (8), `CASC_GAIN_LIVE_ALPHA`
  (0.15, in-fill refit), `CASC_TERMINAL_RATE_FRAC` (0.25, terminal rate =
  fast·frac, decoupled from learned slow_rate), learn alphas, safety limits.

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
(PI state), `gate_gain_gps_per_pct` (K̂), `gain_offset_b_gps` (b),
`gain_hi/lo_gate_pct`+`gain_hi/lo_rate_gps` (the two affine operating points),
`control_rate_gps` (drip-averaged control feedback). Per-fill summary has
`used_*`/`next_*` for the learning-trend plots.

## Known limitations / to validate on hardware
- **flow-cascade retest pending** after the identification rework (`6680bc0`,
  `6215bef`, `ef28112`) — builds clean, NOT yet on hardware. Watch with
  `cascade_analyze.py`, in this order:
  1. `n_obs` / `obs_gate_span` — the fit needs several observations over a decent
     gate span. Few, or all at one gate, = an **excitation** problem no estimator
     can fix (consider a deliberate gate step early in the fill).
  2. `K̂` should land near the offline truth for the thin dummy (**~1.4**, NOT the
     old ~0.5), `onset` ~19–20 %, `P_k` shrinking across fills.
  3. `tau` should settle to something plausible (thin dummy reacts fast, so
     expect well under the 0.6 s seed) and the plotted model curve should track
     the measured rate far more closely.
  4. Only then judge the gate oscillation: it was a *symptom* of K̂ being 2.7×
     low (feedforward opened to 37.7 % for a 9.4 g/s setpoint while the plant
     actually delivered ~31 g/s there).
- **Jar accuracy is currently NOT evidence the cascade works.** In the `23:10`
  run all 7 fills landed −2.4…+0.7 g of 150 g *while* the rate loop was running a
  2.7×-wrong model, overshooting its setpoint 3.3× and oscillating. The accuracy
  came from the close threshold + `post_close_gain`/drip learning (the shared
  adaptive substrate), not from the Regelung. **Don't claim cascade performance
  from final mass alone** — check the rate tracking. (Matters for the thesis.)
- **Medium-specific constants are where dummy-honey trimming hides.** Anything in
  **g/s or gain units** doesn't scale with the medium: `CASC_FAST_FILL_S`,
  `CASC_DEADBAND_GPS` (1.5 g/s is ~50 % of the 3 g/s terminal setpoint — the
  terminal approach is effectively unregulated), `CASC_FLOW_EPS_GPS`,
  `CASC_GATE_GAIN_MIN`. Constants in **% of gate** (onset, margins) are geometry
  and are naturally medium-independent. Smell-test new constants this way.
- **Thick-honey risk in the decoupled setpoint (untested):** `fast = target/16 s`
  may be **unachievable** for real honey (a 500 g jar demands 31 g/s). The gate
  then pins at the ceiling and the profile tapers from an unachievable rate, so no
  real deceleration happens until very late → abrupt close. Non-circular fix if
  seen: `fast = min(target/T, K̂·(ceiling − onset))` — clamps to the model's
  achievable max at a *fixed* gate, so it reintroduces no positive feedback.
- Dead time is measured once at fill start and held constant — the within-fill
  decrease (falling head height) is NOT compensated (documented limitation;
  `finish_trim` + Smith correction absorb the bias to first order).
- `K̂,b` is a linearized (affine) model sampled only in steady state; gate→flow is
  genuinely nonlinear, so K̂ is a local secant, not a true tangent — good between
  the visited operating points, extrapolate with care.
- The dead-time-paced holdoff partly **duplicates** the Smith predictor's job (a
  good predictor is what lets you run the loop fast). It's justified here by model
  uncertainty, a noisy differentiated rate, and the fact that the gain-ID needs
  steady dwells — but it is a trade-off, not the optimum. Holdoff saturates at
  `CASC_HOLDOFF_MAX_MS` (2.5 s), so thick honey (L≈10 s) already updates at ~L/4
  and thus *already* leans on the predictor.

## Pitfalls already hit (don't rediscover these)
- **Syncthing must NOT sync `.git`** — it copies the object store file-by-file,
  races, and corrupts it (symptoms: "packfile … index unavailable", or a commit
  capturing a stale tree). `.stignore` excludes `/.git`, build, `.history`. Sync
  code between machines with **git push/pull via the remote**, never file-based.
- **`ewma()` `prev<=0` shortcut corrupts signed values.** Anything that can be
  negative (`finish_trim`, cascade `b`) must use `learn_ewma` (signed-safe).
- **Learned state is RAM-only** — a reboot resets every preset to seeds. This is
  intentional (thesis wants to show cold-start convergence), not a bug to "fix".
- **Don't add gate-sweep / per-percent gain schedules** to the cascade — the
  controller only visits two gate regimes so the middle never trains (see the
  affine-model note above; this was tried and reverted).
- Learning warmup (`learn_ewma α=max(base,1/(n+1))`) makes the **first** fill
  dominate — a bad first fill sticks. Overweight-miss fills are allowed to learn
  (not just successes) so a systematic overshoot can self-correct.
- **A thin/sticky medium drips** — no instantaneous rate is meaningful. Anything
  that consumes flow rate (gain-ID, control feedback) must average over ≥~1.5 s
  (the drip period). Confirmed: short windows make both K̂ and the control loop
  garbage. See the thin-medium tuning block under "Learning model".
- **Don't use `learned_fast_rate` as the cascade setpoint** — it's a positive-
  feedback loop on a fast medium (diverges + overfills). Setpoint is the fixed
  `target_g/CASC_FAST_FILL_S` trajectory; only the plant model is learned.

## Analysis tooling
`tools/telemetry/cascade_analyze.py <session_dir> [--fill N] [--every K]` — per-fill
K/b/onset/operating-point summary table + optional per-sample gate/rate/model
trace. Built this session to stop re-deriving the cascade diagnostics each time;
extend it rather than writing throwaway scripts.

## Conventions
- Commits: `FW:` (firmware) / `TOOL:` (host tools) / `DOC:` (docs/notes) prefix,
  comma-separated changes; "Add new fill strategy: <name> (...)" for new
  strategies. End with `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`.
  Merges: `Merge branch 'fill-strategies' into dev - <summary>`.
- The clangd "Unknown argument '-mlongcalls'…" IDE errors are xtensa-vs-clang
  noise — ignore; trust `idf.py build`.

## Current state / next
All strategies build clean. **Active work is on branch `testing`** — a scratch
branch for iterative hardware tuning of the cascade, one small commit per test
iteration, to be squash/merged to `dev` manually later.

Latest (`ef28112`): the cascade **plant identification was reworked** after the
`23:10` run was analysed and found to be fitting a 2.7×-wrong K̂ —
`6680bc0` (harvest one observation per steady dwell, at dwell end),
`6215bef` (RLS replaces the seed-anchored two-point fit; K floor relaxed),
`ef28112` (learn τ, use it to de-bias observations, log the full learned model).
Builds clean, **flash + retest pending**.

**Next:**
- Flash `ef28112`, rerun the thin dummy cascade fill, analyse with
  `cascade_analyze.py` following the ordered checklist in "Known limitations"
  (`n_obs`/`obs_gate_span` first, then K̂≈1.4 / onset≈19–20 % / `P_k` shrinking,
  then τ, then the oscillation). Iterate on `testing`.
- If `n_obs` is still small or `obs_gate_span` narrow, the blocker is
  **excitation**, not the estimator: consider a short deliberate gate step early
  in the fill (textbook step-response ID, and a good thesis story alongside the
  RLS "rekursive Parameterschätzung" angle).
- Re-tune the symptom-fighting constants (deadband 1.5, asymmetric slew 15/6)
  once K̂ is right — they were tuned against a broken model.
- Then: collect representative thesis charts (esp. the adaptive learning trend and
  now the cascade parameter-evolution trends from the new `used_/next_` summary
  fields), and fill in thesis skeletons `10_regelungsbasierter` /
  `11_strategievergleich` + LaTeX control equations to match the drawio diagrams.

**Uncommitted (host side, was noted earlier):** `plot_runs.py` `_plot_control_panel`
(rate error / prediction error / PI integrator) written but NOT wired into the
gridspec/render path — finish + commit as `TOOL:`.
