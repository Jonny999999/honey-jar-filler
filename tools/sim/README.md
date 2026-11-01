# Strategy simulator (host)

Runs the **real firmware strategy code** (`components/filler/filler_strategy_*.c`)
on the PC against a closed-loop plant model, so strategy changes can be tested
over whole sessions (draining bucket, refills, honey change, reboot) before
flashing.

```
tools/sim/build.sh                       # compile the strategies + harness -> build/libfillsim.so
python3 tools/sim/scenarios.py           # comparison suite (all scenarios x configs x 3 seeds)
python3 tools/sim/scenarios.py -s thin_drain -c gate -d    # one scenario, per-fill detail
```

Rebuild (`build.sh`) after every firmware change; the Python side loads the `.so`.

## Pieces

| file | role |
|---|---|
| `shim/` | stub ESP-IDF / FreeRTOS headers (`esp_log`, `nvs`, `esp_timer`, ...). NVS is file-backed so persistence across a simulated reboot works |
| `harness.c` | reproduces what `filler_fsm.c` does around a strategy: `on_enter()` on FILL / DRIP_WAIT / VERIFY_TARGET with the right `prev_state` (refill = FILL from VERIFY_TARGET), `step()` every 10 ms tick, the env callbacks, param access by name |
| `plant.py` | closed-loop plant: the physics of `tools/telemetry/synth_plant_session.py` (dead time on the gate signal, FOPDT lag, strand break-up/drops, flap residue, jet impact, scale averaging + noise) plus bucket head, head-dependent dead time and media profiles |
| `fillsim.py` | session runner (`run_session`) - jar swaps, events, per-fill results, optional NDJSON telemetry in the capture format |
| `scenarios.py` | comparison suite + aggregate metrics (ground truth = true jar mass) |

Simulated reboot = a fresh copy of the `.so` is loaded (all RAM/static state
gone), the NVS files stay.

## Plant calibration (from the recorded sessions in `~/projects/honey-jar-filler/data/telemetry`)

* **gate -> flow** `q = K * head * (gate - onset)^curve`. Thin dummy K~1.4 / onset 19 %
  (test6); dark thin honey K~2.5 / onset 23.6 % (07-05); thick honey K~0.095 / onset
  7.4 % (synth identification, 06-14); `thick_concave` = same honey with the
  saturating `(gate-13)^0.6` the fix4 35/100 % ratio suggests (model-mismatch test).
* **bucket head** `((m + m_off)/(m_ref + m_off))^head_exp`: thick 7.0 -> 2.5 g/s at
  100 % over ~5 kg (fix4, exp 1); thin dummy 41 -> 11 g/s at 50 % over ~1.8 kg (test6,
  exp 0.5, small bucket nearly emptied).
* **dead time grows as the bucket drains**: `L = l_min + m_build / q`. fix4: L 4.8 ->
  8.2 s while the 100 % rate fell 7.0 -> 3.6 g/s (`L*rate ~ const`) -> l_min 1.25 s,
  m_build 25 g; thin dummy 0.5 s / 5.5 g.

Validation (unmodified `adaptive-heuristic`, the recorded session's own parameters):

| | real | sim |
|---|---|---|
| test6 thin dummy: error | +7.6, then -2.6...+4.2 g | +11.3, then -3.8...+2.1 g |
| test6: dead time / Nachlauf | 0.65-1.0 s / 11.8 -> 4.3 g | 0.8-0.9 s / 14 -> 7 g |
| fix4 thick, fills 3-11: error | -5.6...-1.3 g | -2.5...-1.4 g |
| fix4: fill duration | 110 -> 243 s | 105 -> 225 s |
| fix4: dead time / Nachlauf | 5.6 -> 8.2 s / 10.3 -> 7.3 g | 4.6 -> 9.1 s / 8.7 -> 5.8 g |

It is a plausibility model, not an identification: use it to find logic bugs,
limit cycles and robustness problems, and to compare strategies relative to each
other - confirm absolute numbers on hardware.

The sim already found one real baseline quirk: `adaptive-heuristic` does not
latch the reduced phase, so the gate can return 35 % -> 100 % when the jet's
impact force drops after the reduction (also visible on hardware, e.g.
`adaptive-test1` fill 3: near -> max -> near).
