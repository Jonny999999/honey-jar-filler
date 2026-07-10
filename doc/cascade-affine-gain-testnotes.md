# Flow-cascade: affine gain rework — what changed & what to test

Context for future-me: worked on this ~2026-07-08..10 after the first hardware
cascade run (`data/telemetry/2026.07.05_21:00-x-dunkler-honig_cascade_ratereduced-learnfromoverfill/`).
That run showed 3 problems: overfill every fill (+15..23 g), gate stayed ~full
open until the very end (no visible slowdown), and the model prediction drifted
*worse* over successive fills. This commit is the fix. UNTESTED on hardware.

## What changed on a logical level

1. **Plant model: 6-point breakpoint schedule → affine `rate = K·gate + b`.**
   - Old: tried to learn a rate value at each of 10/20/…/90 % gate and
     interpolate. Dead idea — the controller only ever visits 2 gate regimes
     (fast phase high, terminal phase low), so the middle breakpoints never got
     data and stayed at defaults. The "effective gain" sampled at 60 % then slid
     around on a curve that never moved → the apparent K-drift 0.59→0.29.
   - New: one slope `K` + one offset `b`. `b` matters because flow onset is
     ~15 % gate (line does NOT pass through origin); forcing b=0 is what made a
     single scalar gain unstable across the range. `b` clamped [-60, +15] g/s,
     usually negative.
   - Learned live from **two EWMA operating points**: fast phase feeds the
     high-gate point, slow/terminal phase feeds the low-gate point. Refit
     (`cascade_refit_gain`) every update if the two gates are ≥ 8 % apart; else
     hold K and only re-solve b. In-fill alpha 0.15 (aggressive). Updates ONLY
     when gate stable past dead_time + settle margin (true steady state).
   - Persisted per preset across fills via signed-safe `learn_ewma` (K and b).
   - Feedforward is now `gate = (ṁ* − b) / K`; start gate + on_enter seeding use
     the same inversion. Seed at fill start: hi point = (60 %, K·60+b),
     lo point = (30 %, K·30+b) from the persisted entry.
   - NOTE seed `entry->gain_b = 0.0f` in `cascade_defaults_from_params` — first
     fill starts as pure linear-through-origin, so early feedforward will run a
     bit rich at low gates until b is learned. Expected; converges fast.

2. **Overfill fix (close threshold).** Root cause: `close_max` clamp collapsed
   to 10 g (`max(close_remaining_g*1.8, 10)` with close_remaining_g=5) while the
   computed `predicted_remaining` was 71–82 g → gate closed way too late.
   - Removed a double-count: `predicted_remaining_g = learned_post_close_gain_g`
     only (post_close already includes the in-flight mass; the old code also
     added dead_time·close_rate on top).
   - New clamp: `close_max = min( max(close_remaining*1.8, 10, post_close*1.5+15),
     target*0.6 )`.
   - post_close seed raised to `max(close_remaining*0.55, target*0.025)`.

3. **Flat profile fix (no visible slowdown).** fast_rate and slow_rate had both
   collapsed to ~30 g/s, so the taper was flat. Terminal rate is now decoupled
   from the learned slow_rate: `terminal = clamp(fast * 0.25, floor, fast)`
   (CASC_TERMINAL_RATE_FRAC=0.25). Profile tapers fast→terminal over the decel
   band.

## What to watch / test on hardware next

Flash this, run the same dark-honey preset, capture telemetry, then in the chart:

- [ ] **Overfill gone?** fill_error should land within ~±5 g from fill 2 on
      (fill 1 may still miss while b/post_close seed). Old run was +15..23 g.
- [ ] **Visible deceleration?** Gate should ramp down through a terminal crawl,
      not stay pinned high until close. Look at the gate trace + `target_rate`
      (ṁ*) taper in the rate panel.
- [ ] **K stable, not drifting?** `gate_gain_gps_per_pct` should settle and stay
      put across fills instead of sliding down. Also check `b` (new) goes
      negative and plausible (~ -K·onset%, e.g. onset 15 % → b ≈ -0.3·15 ≈ -4).
- [ ] **Two operating points separating?** The refit needs ≥ 8 % gate separation
      between fast and slow phases. If the fill never really slows (gate hi==lo),
      K holds and only b moves — check the fill actually enters slow_control
      phase (`near_close_logged`).
- [ ] **Feedforward sane at start?** `control_gate_cmd_pct` at fill open should
      invert to a reasonable opening, not slam to ceiling or to ~0.
- [ ] **Smith predictor:** ŷ_d (`model_delayed_gps`) should track measured rate
      once flow starts; ŷ (`model_rate_gps`) leads it by dead_time. PI
      integrator (`control_integ_pct`) should not rail.

## Still uncommitted / not done here (host tool side)
- `plot_runs.py`: `model_rate` overlay in the rate panel is done & verified, but
  the separate `_plot_control_panel` (rate error, prediction error, PI
  integrator) is written yet NOT wired into the gridspec / render path. Finish
  that wiring before relying on the control panel. Commit as `TOOL:` separately.

Key constants (in `filler_strategy_flow_cascade.c`): CASC_GAIN_B_MIN/MAX,
CASC_GAIN_MIN_GATE_SEP_PCT (8), CASC_GAIN_LIVE_ALPHA (0.15),
CASC_TERMINAL_RATE_FRAC (0.25), CASC_GAIN_SETTLE_MARGIN_S.
