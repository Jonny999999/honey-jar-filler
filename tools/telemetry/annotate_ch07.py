#!/usr/bin/env python3
"""Draw the ch. 07 annotation layer on the synthetic plant-probe figure.

Replicates the annotations that were previously done by hand in Inkscape on
`2026.06.14_20-54-...__fill-03__plain_narrow__inkscape-edit.pdf`: the same eight
labels, the same shaded dead-time bands, the span arrow for the drip phase and
the ellipse around the Nachlauf mass.

Why scripted instead of by hand: the hand-annotated SVG could not follow the
data. When the run was replaced, all of that work was lost, which is exactly how
this figure ended up unannotated. Here the anchors are *computed* - from the gate
events in the telemetry and the plant parameters in session_meta.json - so they
stay correct when the run is regenerated, and the label for the last drip points
at the actual last drip rather than at a guess.

Placement is deliberately NOT automatic. Collision-free auto-layout is a hard
problem and not worth it for one figure. Instead each label carries a hand-tuned
offset in the ANNOTATIONS table below: the anchor comes from the data, the box
position is a fixed offset in axes fractions. Nudge the numbers in the table and
re-run; the PDF also keeps text as text, so Inkscape stays available for a final
polish.

Writes exactly one file: <session>/figures/<stem>__plain_narrow_annotated.pdf

Usage:
  python3 tools/telemetry/annotate_ch07.py <session_dir>
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import plot_runs
from analysis import resolve_session_dir

# Style, matching the thesis diagrams (pastel fills, #555555 borders).
BORDER = "#555555"
BAND_FC = "#f5e6a8"
BAND_EC = "#c9a227"
BOX = dict(boxstyle="round,pad=0.35", fc="white", ec=BORDER, lw=0.8, alpha=0.95)
# shrinkA is small on purpose: it is the gap between the box and the start of
# its leader, and at the default the arrow floats free of the box it belongs to.
ARROW = dict(arrowstyle="->", color=BORDER, lw=0.8,
             connectionstyle="arc3,rad=0.15", shrinkA=0.5, shrinkB=3)
FONT = 7.0

# The rate subplot is deliberately taller here than in the plain export. Four
# labels do not fit over a panel that short without either covering the trace or
# flattening it, and this figure is the one place the rate detail is the point.
# The figure grows a little taller with it, so the aspect ratio differs slightly
# from the plain variant - that is intended, not drift.
RATE_HEIGHT_SCALE = 1.5


def _events(records: list[dict]) -> dict:
    """Anchor times (relative to fill start) and levels, computed from the data."""
    samples = [r for r in records if r.get("kind") == "sample"]
    fill_start = next((r for r in records if r.get("kind") == "fill_start"), None)
    t0 = fill_start["ts_us"] if fill_start else samples[0]["ts_us"]

    def rel(r) -> float:
        return (r["ts_us"] - t0) / 1e6

    gates = [(rel(r), float(r.get("gate_pct", 0.0)))
             for r in records if r.get("kind") == "gate"]
    t_open = next((t for t, g in gates if g > 0), 0.0)
    t_close = next((t for t, g in gates if g <= 0), 0.0)
    # the intermediate step: a gate change between two flowing gates
    t_step = next((t for t, g in gates if g > 0 and t > t_open), t_open)

    ts = [rel(r) for r in samples]
    mass = [float(r.get("relative_fill_g", 0.0)) for r in samples]
    raw = [float(r.get("rate_raw_gps", 0.0) or 0.0) for r in samples]
    filt = [float(r.get("rate_filtered_gps", 0.0) or 0.0) for r in samples]

    def peak(a: float, b: float) -> tuple[float, float]:
        seg = [(t, v) for t, v, in zip(ts, filt) if a <= t <= b]
        return max(seg, key=lambda x: x[1]) if seg else (a, 0.0)

    return {
        "t_open": t_open,
        "t_step": t_step,
        "t_close": t_close,
        "t_end": ts[-1],
        "ts": ts, "mass": mass, "raw": raw, "filt": filt,
        "peak_open": peak(t_open, t_step),
        "peak_close": peak(t_close, t_close + 3.0),
        "mass_final": mass[-1],
    }


def _plant(session_dir: Path) -> dict:
    meta = json.loads((session_dir / "session_meta.json").read_text())
    return meta.get("generator", {}).get("plant_params", {})


def _last_drip(ev: dict) -> tuple[float, float]:
    """The last drip: the final raw-rate spike after the Nachlauf has died away."""
    late = [(t, v) for t, v in zip(ev["ts"], ev["filt"])
            if t > ev["t_close"] + 6.0 and v > 1.0]
    return late[-1] if late else (ev["t_end"], 0.0)


def _peak_flank(peak: tuple[float, float], frac: float = 0.68) -> tuple[float, float]:
    """A point in the upper third of a spike, rather than its tip.

    Aiming at the tip puts the arrowhead on top of the very feature it marks and
    leaves no gap between head and trace. Landing a third of the way down the
    flank still unambiguously points at the spike and keeps the tip readable.
    """
    px, py = peak
    return (px, py * frac)


def _noise_peak(ev: dict, t_react: float, t_step: float) -> tuple[float, float]:
    """A real spike inside the 30 % dwell, for the process-noise label.

    Starts two seconds past first contact so the anchor is one of the drip
    spikes rather than the first-contact peak, and takes the earliest of the
    tallest few so the arrow points left, away from the label.
    """
    seg = [(t, v) for t, v in zip(ev["ts"], ev["filt"])
           if t_react + 2.0 <= t <= t_step - 2.0]
    if not seg:
        return ((t_react + t_step) / 2, 0.0)
    top = sorted(seg, key=lambda x: x[1], reverse=True)[:5]
    return min(top, key=lambda x: x[0])


def _mass_at(ev: dict, at: float) -> float:
    return next((m for t, m in zip(ev["ts"], ev["mass"]) if t >= at), ev["mass_final"])


def _span_arrow(ax, x0: float, x1: float, y: float) -> None:
    """Double-headed arrow marking an interval on the time axis."""
    ax.annotate("", (x0, y), (x1, y),
                arrowprops=dict(arrowstyle="<->", color=BORDER, lw=0.9), zorder=7)


def _dimension_v(ax, x: float, y0: float, y1: float,
                 ext_x0: float, ext_x1: float) -> None:
    """Vertical dimension line with extension lines back to the feature.

    Smaller heads than the time-axis spans on purpose: this one covers ~20 g,
    only about 12 pt tall here, and heads at the default scale overlap into a
    blob. The extension lines run back to the curve so the arrow is measuring
    something the reader can see rather than floating next to it.
    """
    ax.annotate("", (x, y0), (x, y1),
                arrowprops=dict(arrowstyle="<->", color=BORDER, lw=0.9,
                                mutation_scale=7, shrinkA=0, shrinkB=0), zorder=7)
    for y, x_from in ((y0, ext_x0), (y1, ext_x1)):
        ax.plot([x_from, x + 1.5], [y, y],
                color=BORDER, lw=0.6, ls=(0, (2, 2)), zorder=7)


def _band(ax, x0: float, x1: float, y: float) -> None:
    """Shaded interval + double-headed arrow, as in the hand-made chart.

    Used only for the two short delay intervals. The drip phase gets the arrow
    without the shading: it runs a third of the chart, and shading it that wide
    reads as a highlighted region rather than as a measured duration.
    """
    ax.axvspan(x0, x1, color=BAND_FC, alpha=0.55, ec=BAND_EC, lw=0.6, zorder=1)
    _span_arrow(ax, x0, x1, y)


def _box(ax, text: str, xy: tuple[float, float], xytext: tuple[float, float],
         ha: str = "center", relpos: tuple[float, float] | None = None,
         rad: float = 0.15) -> None:
    """Label box at an axes-fraction position, arrow to a data-coordinate anchor.

    `relpos` picks the point on the box the arrow leaves from, in box fractions
    ((0,0) = bottom left). Without it matplotlib exits from whichever edge faces
    the anchor, which degenerates to a stub when the box sits directly above its
    target. `rad` bends the arrow so a short run still reads as a connector.
    """
    arrow = dict(ARROW)
    arrow["connectionstyle"] = f"arc3,rad={rad}"
    if relpos is not None:
        arrow["relpos"] = relpos
    ax.annotate(text, xy=xy, xycoords="data",
                xytext=xytext, textcoords="axes fraction",
                ha=ha, va="center", fontsize=FONT, color="#1f2937",
                bbox=BOX, arrowprops=arrow, zorder=8)


def annotate(fig, ax, rate_ax, fill_run, records, session_dir: Path) -> None:
    ev = _events(records)
    p = _plant(session_dir)
    dead = float(p.get("dead_time_s", 4.14))

    t_open, t_step, t_close = ev["t_open"], ev["t_step"], ev["t_close"]
    t_react = t_open + dead          # first mass change
    t_step_react = t_step + dead     # the step's effect becomes visible

    mass_top = ax.get_ylim()[1]

    # ---------------- mass panel ----------------------------------------
    # 1) dead time: gate opens -> first weight change
    _band(ax, t_open, t_react, y=mass_top * 0.13)
    _box(ax, "Totzeit bis erste\nGewichtsänderung",
         xy=((t_open + t_react) / 2, mass_top * 0.15), xytext=(0.15, 0.60))

    # 2) the same delay again, on a step between two flowing gates
    _band(ax, t_step, t_step_react, y=mass_top * 0.42)
    _box(ax, "verzögerte Wirkung der\nStellgrößenänderung",
         xy=((t_step + t_step_react) / 2, mass_top * 0.44), xytext=(0.42, 0.88))

    # 3) Nachlaufmasse: the mass still arriving after the gate shut, dimensioned
    # against the curve. Measured from *before* the close, because the flap shock
    # at t_close is apparent weight rather than delivered mass - taking the
    # spiked reading as the baseline would understate the Nachlauf by a few g.
    # The ellipse is gone: a dimension line that reaches the curve at both ends
    # states the same thing more precisely, and an ellipse big enough to contain
    # it would have swallowed half the panel.
    m_close = _mass_at(ev, t_close - 0.6)
    m_settled = ev["mass_final"]
    x_dim = t_close + 7.5
    _dimension_v(ax, x_dim, m_close, m_settled,
                 ext_x0=t_close, ext_x1=t_close + 5.0)
    # right of the dimension line: to its left the box would sit on the gate
    # line dropping to 0 % at t_close
    _box(ax, "Nachlaufmasse", xy=(x_dim, (m_close + m_settled) / 2),
         xytext=(0.88, 0.52), rad=0.2)

    # 4) the drip phase as a whole: arrow only, no shading
    _span_arrow(ax, t_close, ev["t_end"], y=mass_top * 0.06)
    _box(ax, "Nachlauf /\nAbtropfphase",
         xy=((t_close + ev["t_end"]) / 2, mass_top * 0.06), xytext=(0.76, 0.27))

    # ---------------- rate panel ----------------------------------------
    if rate_ax is None:
        return
    rate_top = rate_ax.get_ylim()[1]

    # Headroom for the labels. This pairs with RATE_HEIGHT_SCALE: the boxes are
    # sized in points, so a taller panel makes each one a smaller fraction of it
    # and buys back the room the labels need. That lets the headroom stay tight
    # here, which is what actually gives the trace its detail - the damped peaks
    # only help if the axis stops being set by them.
    rate_ax.set_ylim(rate_ax.get_ylim()[0], rate_top * 1.28)
    rate_top = rate_ax.get_ylim()[1]

    # 5) the peak when the first honey reaches the jar. Offset right so the box
    # no longer spans its own peak: while it does, the bbox clips the leader and
    # only a stub emerges from the bottom edge. This one lands on the tip rather
    # than the flank - the leader approaches from above, so the head sits on top
    # of the spike without covering it.
    _box(rate_ax, "Impulsbedingter Peak beim ersten\nAuftreffen des Honigstrahls",
         xy=ev["peak_open"], xytext=(0.36, 0.90), relpos=(0.0, 0.45), rad=0.5)

    # 6) the noisy plateau of the 30 % dwell. Anchored to a real spike in that
    # dwell rather than to its midpoint, and kept left of the 70 % ramp so the
    # box clears both the first-contact peak and the rising trace.
    _box(rate_ax, "Mess- und\nProzessschwankungen",
         xy=_noise_peak(ev, t_react, t_step), xytext=(0.345, 0.50), rad=0.2)

    # 7) the flap shock when the gate is shut. The arrow leaves the box's left
    # edge (relpos) and swings out below it before coming back into the peak.
    # The sign of rad matters: the text bbox clips the arrow, so an arc bending
    # back over the box is eaten and only a stub survives - it has to bulge
    # away. This box is too wide to clear its peak horizontally, so unlike (5)
    # it is lifted instead, keeping the spike's tip clear of the box edge.
    _box(rate_ax, "Impulsübertrag im Honigstrom durch\nabruptes Schließen der Klappe",
         xy=_peak_flank(ev["peak_close"]), xytext=(0.79, 0.93),
         relpos=(0.0, 0.45), rad=0.5)

    # 8) the final drip. Computed, so this really is the last one.
    dx, dy = _last_drip(ev)
    _box(rate_ax, "letztes Abtropfen", xy=(dx, dy), xytext=(0.87, 0.50))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir", type=Path)
    args = ap.parse_args()

    session_dir = resolve_session_dir(args.session_dir)
    session_dir, fill_runs, records_by_fill = plot_runs._load_input(session_dir, None, None)
    if not fill_runs:
        raise SystemExit(f"no fills found in {session_dir}")
    fill_run = fill_runs[0]
    records = records_by_fill[fill_run.fill_id]

    plot_runs.ANNOTATION_HOOK = (
        lambda fig, ax, rate_ax, fill_run, records:
        annotate(fig, ax, rate_ax, fill_run, records, session_dir)
    )

    out = plot_runs._render_fill_variant(
        session_dir, fill_run, records, session_dir / "figures", ["pdf"],
        debug=False,
        legend_placement="outside",
        state_style="none",
        show_rate="filtered",
        rate_layout="subplot",
        figure_profile="narrow",
        output_name="plain_narrow_annotated",
        include_control_panel=False,
        show_target=False,
        mass_y_max=None,
        rate_height_scale=RATE_HEIGHT_SCALE,
        command_text=None,
    )
    for path in out:
        print(f"Annotated: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
