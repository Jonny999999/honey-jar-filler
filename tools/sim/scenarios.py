#!/usr/bin/env python3
"""Strategy comparison suite on the closed-loop plant.

Runs every scenario for every strategy configuration (several noise seeds) and
prints an aggregate table plus, with --detail, the per-fill tables.

    python3 tools/sim/scenarios.py                  # all scenarios
    python3 tools/sim/scenarios.py -s thin_drain -d # one scenario, per-fill detail
    python3 tools/sim/scenarios.py --seeds 5

Metrics are ground truth from the plant (true jar mass), not the firmware's
own estimate.
"""

from __future__ import annotations

import argparse
import statistics as st
import sys

from fillsim import Event, run_session, table, gate_table

THIN_TUNED = dict(target_grams=300, target_tol_low_g=10, target_tol_high_g=20, slow_remaining_g=100,
                  slow_gate_pct=30, max_gate_pct=50, close_remaining_g=5, drip_delay_ms=2000)
THICK_TUNED = dict(target_grams=500, target_tol_low_g=10, target_tol_high_g=10, slow_remaining_g=200,
                   slow_gate_pct=35, max_gate_pct=100, close_remaining_g=100, drip_delay_ms=25000,
                   fill_timeout_ms=600000)
# What an operator gets without tuning: the built-in default ("Medium viscosity")
DEFAULT_PRESET = dict(target_tol_low_g=10, target_tol_high_g=10)
AUTO = dict(target_tol_low_g=10, target_tol_high_g=20, close_remaining_g=5, drip_delay_ms=2000)

# name -> (medium, fills, bucket_g, events, {config_name: (strategy, params)})
def scenarios():
    thin_cfg = {
        "adaptive tuned": ("adaptive-heuristic", THIN_TUNED),
        "adaptive default-preset": ("adaptive-heuristic", dict(DEFAULT_PRESET, target_grams=300)),
        "gate default": ("adaptive-gate", dict(AUTO, target_grams=300)),
        "gate untouched-preset": ("adaptive-gate", dict(DEFAULT_PRESET, target_grams=300)),
        "gate fast(12s)": ("adaptive-gate", dict(AUTO, target_grams=300, auto_fill_time_s=12)),
    }
    thick_cfg = {
        "adaptive tuned": ("adaptive-heuristic", THICK_TUNED),
        "adaptive default-preset": ("adaptive-heuristic", dict(DEFAULT_PRESET, target_grams=500, fill_timeout_ms=600000)),
        "gate default": ("adaptive-gate", dict(AUTO, target_grams=500, target_tol_high_g=10, drip_delay_ms=25000,
                                               fill_timeout_ms=600000)),
        "gate untouched-preset": ("adaptive-gate", dict(DEFAULT_PRESET, target_grams=500, fill_timeout_ms=600000)),
    }
    return {
        "thin_drain": ("thin_dummy", 11, 3500, [], thin_cfg),
        "thick_drain": ("thick", 11, 7400, [], thick_cfg),
        "thick_concave": ("thick_concave", 8, 7400, [], thick_cfg),
        "dark_thin_honey": ("dark_thin_honey", 10, 6000, [], {
            "adaptive tuned(27/50)": ("adaptive-heuristic", dict(THIN_TUNED, slow_gate_pct=27)),
            "adaptive default-preset": ("adaptive-heuristic", dict(DEFAULT_PRESET, target_grams=300)),
            "gate default": ("adaptive-gate", dict(AUTO, target_grams=300)),
        }),
        "bucket_topup": ("thin_dummy", 10, 1500, [Event(5, "refill_bucket", 2500)], thin_cfg),
        "topup_midfill": ("thin_dummy", 8, 1300, [Event(4, "refill_midfill", 2500, at_s=6.0)], thin_cfg),
        "honey_switch": ("thick", 10, 7000,
                         [Event(5, "medium", "dark_thin_honey")], {
            "adaptive tuned-thick": ("adaptive-heuristic", dict(THICK_TUNED)),
            "gate default": ("adaptive-gate", dict(AUTO, target_grams=500, target_tol_high_g=10,
                                                   drip_delay_ms=25000, fill_timeout_ms=600000)),
        }),
        "reboot_refilled": ("thin_dummy", 10, 2000,
                            [Event(6, "reboot"), Event(6, "refill_bucket", 2500)], thin_cfg),
    }


def metrics(res: list[dict], tol_high: float) -> dict:
    errs = [r["true_err_g"] for r in res if r["state"] in ("FIND_SLOT",) and not r["fault"]]
    faults = [r["fault"] or r["summary"].get("text", "") for r in res if r["state"] == "FAULT"]
    refills = sum(int(r["summary"].get("refill_count", 0)) for r in res)
    moves = sum(max(0, r["gate_changes"] - 2) for r in res)
    corr = sum(int(r["summary"].get("gate_corrections", 0)) for r in res)
    opens = [r["open_s"] for r in res if not r["fault"]]
    return {
        "ok": len(errs), "n": len(res), "faults": faults, "refills": refills,
        "mean": st.mean(errs) if errs else float("nan"),
        "sd": st.pstdev(errs) if len(errs) > 1 else 0.0,
        "maxabs": max((abs(e) for e in errs), default=float("nan")),
        "over": sum(1 for e in errs if e > 5.0),
        "first": res[0]["true_err_g"] if res else float("nan"),
        "open_mean": st.mean(opens) if opens else float("nan"),
        "open_first_last": (opens[0], opens[-1]) if opens else (0, 0),
        "moves": moves, "corr": corr,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--scenario", action="append")
    ap.add_argument("-c", "--config", action="append", help="only configs containing this text")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("-d", "--detail", action="store_true")
    a = ap.parse_args()
    sc = scenarios()
    names = a.scenario or list(sc)
    print(f"{'scenario':16s} {'config':24s} {'ok/n':>7} {'refl':>4} {'1st':>6} {'mean':>6} {'sd':>5} {'max|e|':>6} "
          f"{'>+5g':>4} {'open_s':>6} {'1st->last':>11} {'xmoves':>6} {'corr':>4}  faults")
    for name in names:
        medium, n, bucket, events, cfgs = sc[name]
        for cname, (strategy, params) in cfgs.items():
            if a.config and not any(c in cname for c in a.config):
                continue
            agg = []
            for seed in range(1, a.seeds + 1):
                res = run_session(strategy, medium, n_fills=n, bucket_g=bucket, params=params,
                                  events=events, seed=seed)
                agg.append((res, metrics(res, params.get("target_tol_high_g", 10))))
                if a.detail and seed == 1:
                    print(f"\n--- {name} / {cname} (seed 1) ---")
                    print(table(res))
                    if strategy == "adaptive-gate":
                        print(gate_table(res))
                    print()
            m = [x[1] for x in agg]
            ok = sum(x["ok"] for x in m); tot = sum(x["n"] for x in m)
            allerr = [r["true_err_g"] for res, _ in agg for r in res if r["state"] == "FIND_SLOT"]
            faults = sorted({f for x in m for f in x["faults"]})
            fl = [x["open_first_last"] for x in m]
            print(f"{name:16s} {cname:24s} {ok:3d}/{tot:<3d} {sum(x['refills'] for x in m)/len(m):4.1f} "
                  f"{st.mean(x['first'] for x in m):6.1f} "
                  f"{(st.mean(allerr) if allerr else float('nan')):6.1f} {(st.pstdev(allerr) if len(allerr)>1 else 0):5.1f} "
                  f"{max((abs(e) for e in allerr), default=float('nan')):6.1f} "
                  f"{sum(x['over'] for x in m)/len(m):4.1f} {st.mean(x['open_mean'] for x in m):6.1f} "
                  f"{st.mean(f[0] for f in fl):5.0f}->{st.mean(f[1] for f in fl):<5.0f} "
                  f"{sum(x['moves'] for x in m)/len(m):6.1f} {sum(x['corr'] for x in m)/len(m):4.1f}  "
                  f"{','.join(faults)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
