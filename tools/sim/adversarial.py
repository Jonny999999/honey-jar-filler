#!/usr/bin/env python3
"""Adversarial cases for adaptive-gate: does it act like a human operator would?

    python3 tools/sim/adversarial.py [log_dir]

Each line prints how the disturbed fill ended next to the expected behaviour
(jar taken away, bucket running dry at different times, clog, honey thickening /
thinning mid-fill). Firmware logs per case go to log_dir (default /tmp).
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fillsim import run_session, Event
from plant import medium
LOG_DIR = sys.argv[1] if len(sys.argv) > 1 else '/tmp'
CLOG = medium('thick', K=0.0001)            # flow (almost) stops
def case(name, med, n, bucket, events, expect):
    r = run_session('adaptive-gate', med, n_fills=n, bucket_g=bucket, params={}, events=events, seed=1,
                    log_path=os.path.join(LOG_DIR, f'adv_{name}.log'),
                    max_fill_s=900)
    f = [x for x in r if x['fill'] == max(e.fill for e in events)][0] if events else r[-1]
    print(f"{name:26s} fill{f['fill']}: end={f['fault'] or f['summary'].get('text','?'):14s} jar_t={f['jar_time_s']:6.1f}s "
          f"err={f['true_err_g']:+7.1f} spilled={f['spilled_g']:6.1f}g max_gate={f['max_gate']:5.1f} gates={f['gates'][:10]}  | expect: {expect}")
case('jar_removed_fast', 'thick', 3, 7000, [Event(3, 'jar_removed_midfill', at_s=30)], 'close + NO_JAR fault quickly, ~0 spill')
case('jar_removed_thin', 'dark_thin_honey', 3, 7000, [Event(3, 'jar_removed_midfill', at_s=6)], 'close + NO_JAR fault quickly')
case('bucket_empty_fast', 'thick', 3, 7000, [Event(3, 'empty_midfill', at_s=30)], 'EMPTY fault within ~1 min, not 10 min')
case('bucket_empty_reduced', 'thick', 3, 7000, [Event(3, 'empty_midfill', at_s=70)], 'EMPTY fault within ~1-2 min')
case('bucket_empty_late', 'thick', 3, 7000, [Event(3, 'empty_midfill', at_s=85)], 'finishes from the residue or EMPTY fault')
case('clog_20s', 'thick', 3, 7000, [Event(3, 'medium_midfill', CLOG, at_s=25), Event(3, 'medium_midfill', 'thick', at_s=45)], 'continues, finishes ok')
case('clog_35s', 'thick', 3, 7000, [Event(3, 'medium_midfill', CLOG, at_s=25), Event(3, 'medium_midfill', 'thick', at_s=60)], '>30 s nothing at full open: EMPTY fault (by design)')
case('thicken_after_cut', 'dark_thin_honey', 3, 7000, [Event(3, 'medium_midfill', 'thick', at_s=8)], 'reopens wider, finishes in sane time')
case('thin_to_thick_midfill', 'dark_thin_honey', 3, 7000, [Event(3, 'medium_midfill', 'very_thick', at_s=5)], 'opens up, finishes')
case('thick_to_thin_midfill', 'thick', 3, 7000, [Event(3, 'medium_midfill', 'dark_thin_honey', at_s=20)], 'cuts at once, no overfill')
case('bucket_empty_start', 'thick', 3, 7000, [Event(3, 'empty_midfill', at_s=0.0)], 'opens to 100 then EMPTY fault')
