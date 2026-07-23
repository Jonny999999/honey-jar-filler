#!/usr/bin/env python3
"""Cross-strategy comparison charts for thesis chapter 11 (Strategievergleich).

The honest scope, baked in so the thesis stays defensible:

  * Only runs on the SAME medium / jar / target are a controlled comparison.
    In this dataset that is the test6 pair (thin dummy, 300 g, minutes apart):
        data/telemetry/2026.07.19_11:43-x-test6-cascade
        data/telemetry/2026.07.19_11:49-x-test6-adaptive
    Those are this script's DEFAULT sessions and the only ones plotted without a
    "nicht kontrolliert" disclaimer.
  * The heuristic baseline only exists on a different medium (dark honey, 500 g,
    single fill), so it is NOT quantitatively comparable and is excluded from the
    controlled charts by default. Pooling other sessions is possible (--pool) but
    is always rendered with an explicit "indikativ, nicht kontrolliert" banner.

Learning strategies mislead on their first fills (cold start). Every stat is
reported twice: over ALL fills and over CONVERGED fills (after --warmup, default
2), so the section can state both.

Pooling more sessions (--all / --pool): the positional/default sessions are the
"controlled" set and drive the per-session accuracy strip. --all auto-discovers
EVERY session under data/telemetry (skipping any path containing 'del') and
--pool adds named ones; both feed only the cloud/tradeoff/throughput, always
with a "nicht kontrolliert" banner. NOTE --all also pulls in broken early
dev-tuning runs (huge errors) -- drop them with --exclude SUBSTR... or
--min-fills. flow-control/sequence/manual are dropped by default (not in the
thesis); --keep-flow-control re-adds them.

Chart variants (choose with --charts, default: all):
  accuracy    error-per-fill strip for the controlled set, target line +
              tolerance band, mean +/- std
  tradeoff    speed vs accuracy: mean duration (x) vs error-Streuung (y), one
              bubble per session, colour = strategy
  cloud       pooled point cloud of every fill's error, one column per strategy,
              mean bar + n. Emits BOTH absolute-gram and %-of-target versions
              (the % axis pools different target masses / honey types fairly).
  throughput  target-normalised speed: mean delivered flow rate [g/s] per
              strategy -- an intuitive, target-size-independent duration view.

Usage:
  python3 tools/telemetry/strategy_compare.py                 # test6 pair, all charts
  python3 tools/telemetry/strategy_compare.py --all           # every session pooled
  python3 tools/telemetry/strategy_compare.py --all --min-fills 2 \
      --exclude first-fix cascade-03 flowcontrol_rest dummy-cascade  # curated pool
  python3 tools/telemetry/strategy_compare.py --pool SESS1 SESS2 --charts cloud
  python3 tools/telemetry/strategy_compare.py -o /path/out --warmup 2
"""
from __future__ import annotations

import argparse
import json
import statistics as stat
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
TEL = REPO / "data" / "telemetry"

# Default controlled pair (same medium/jar/target, back-to-back).
DEFAULT_SESSIONS = [
    TEL / "2026.07.19_11:43-x-test6-cascade",
    TEL / "2026.07.19_11:49-x-test6-adaptive",
]

# German display names + a stable colour per strategy (pastels matching the
# thesis diagrams; strong marker edge for readability).
# German wording as used in the thesis ch.11 comparison table
# (Heuristisch / Adaptiv-heuristisch / Regelungsbasiert).
STRAT_LABELS = {
    "heuristic": "Heuristisch",
    "adaptive-heuristic": "Adaptiv-heuristisch",
    "flow-control": "Flussregelung",
    "flow-cascade": "Regelungsbasiert",
    "sequence": "Sequenz (Referenz)",
    "manual": "Manuell",
}
STRAT_COLORS = {
    "heuristic": "#94a3b8",
    "adaptive-heuristic": "#2563eb",
    "flow-control": "#0f766e",
    "flow-cascade": "#c81e1e",
    "sequence": "#7c3aed",
    "manual": "#ea580c",
}

# flow-control is not discussed in the thesis, so it is dropped from every chart
# by default (--keep-flow-control to re-enable).
DROP_STRATEGIES = {"flow-control", "sequence", "manual"}


@dataclass
class Fill:
    idx: int          # order within its session (0-based)
    error_g: float
    duration_s: float
    final_g: float
    target_g: float


@dataclass
class Group:
    """One strategy run (one session)."""
    key: str          # strategy_name
    label: str        # short session tag for the legend
    session: Path
    target_g: float
    tol_low: float
    tol_high: float
    fills: list[Fill]

    def errors(self, warmup: int = 0, relative: bool = False) -> list[float]:
        if relative:
            return [100.0 * f.error_g / f.target_g for f in self.fills[warmup:]
                    if f.target_g > 0]
        return [f.error_g for f in self.fills[warmup:]]

    def durations(self, warmup: int = 0) -> list[float]:
        return [f.duration_s for f in self.fills[warmup:]]

    def throughputs(self, warmup: int = 0) -> list[float]:
        """Average delivered flow rate [g/s] = final mass / fill time. This is
        the target-size-normalised 'speed' axis: a 500 g and a 150 g fill become
        comparable (how fast mass actually goes in), unlike raw duration."""
        return [f.final_g / f.duration_s for f in self.fills[warmup:]
                if f.duration_s > 0.1]


def _session_tag(session: Path) -> str:
    # ".../2026.07.19_11:43-x-test6-cascade" -> "test6-cascade"
    name = session.name
    return name.split("-x-", 1)[1] if "-x-" in name else name


def load_group(session: Path) -> Group | None:
    f = session / "telemetry.ndjson"
    if not f.exists():
        return None
    rows = [json.loads(l) for l in f.read_text().splitlines() if l.strip()]
    sums = [r for r in rows if r.get("kind") == "fill_summary"]
    if not sums:
        return None
    params = sums[0].get("params", {}) or {}

    def pv(key: str, default: float) -> float:
        v = params.get(f"VAR({key})", params.get(key, default))
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    target = float(sums[0].get("target_g", pv("target_grams", 0.0)))
    fills = [
        Fill(
            idx=i,
            error_g=float(s.get("fill_error_g", 0.0)),
            duration_s=float(s.get("fill_duration_s", 0.0)),
            final_g=float(s.get("final_mass_g", 0.0)),
            target_g=float(s.get("target_g", target)),
        )
        for i, s in enumerate(sums)
    ]
    return Group(
        key=sums[0].get("strategy_name", "?"),
        label=_session_tag(session),
        session=session,
        target_g=target,
        tol_low=pv("target_tol_low_g", 3.0),
        tol_high=pv("target_tol_high_g", 3.0),
        fills=fills,
    )


def synth_heuristic(n: int, target: float, tol_low: float, tol_high: float,
                    seed: int = 3) -> Group:
    """ILLUSTRATIVE (not measured) heuristic fills. The heuristic strategy has no
    learning, so with one fixed parameter set its behaviour is entirely dictated
    by how well those thresholds happen to match the current tank pressure. The
    modelled mix (Jonny's expectation):

      * ~1/3 gut getroffen  -> small error, fast fill
      * ~1/3 langsamer, knapp daneben -> error near the lower tolerance edge but
        still in band, slower (falling tank pressure), no refill
      * ~1/3 Nachfüll-getriggert -> refills correct the mass back into band but
        the fill takes very long

    -> a deliberately bimodal cloud with a visible gap (near-miss cluster vs.
    the ok/refill-corrected cluster) and a low-throughput tail. Clearly marked
    synthetic in the group label so it can never be mistaken for real telemetry.
    """
    import random
    rng = random.Random(seed)
    fills: list[Fill] = []
    for i in range(n):
        u = i / max(n - 1, 1)  # deterministic phase so fractions are stable
        r = (i * 0.61803 + 0.13) % 1.0  # low-discrepancy jitter
        if u < 0.34:                       # gut getroffen
            err = rng.uniform(-3.0, 3.0)
            dur = target / rng.uniform(11.0, 13.0)
        elif u < 0.67:                     # langsamer, knapp daneben (in band)
            err = -tol_low * rng.uniform(0.6, 0.95)
            dur = target / rng.uniform(5.5, 7.5)
        else:                              # Nachfüll-getriggert, sehr langsam
            err = rng.uniform(-2.0, 4.0)
            dur = target / rng.uniform(2.0, 3.2)
        _ = r
        fills.append(Fill(idx=i, error_g=err, duration_s=dur,
                          final_g=target + err, target_g=target))
    return Group(key="heuristic", label="heuristik(synth)", session=Path("SYNTH"),
                 target_g=target, tol_low=tol_low, tol_high=tol_high, fills=fills)


# --------------------------------------------------------------------------- #
# Stats
# --------------------------------------------------------------------------- #
def _in_tol(g: Group, warmup: int) -> float:
    fills = g.fills[warmup:]
    if not fills:
        return float("nan")
    ok = sum(1 for f in fills if -g.tol_low <= f.error_g <= g.tol_high)
    return 100.0 * ok / len(fills)


def _fmt(vals: list[float], fn) -> str:
    return f"{fn(vals):.2f}" if vals else "  -  "


def print_stats(groups: list[Group], warmup: int) -> None:
    print("\n=== Strategievergleich — Kennzahlen ===")
    print(f"(Streuung = Standardabweichung des Fehlers; Warmup = {warmup} "
          f"erste Füllungen als Kaltstart verworfen)\n")
    hdr = (f"{'Strategie':<20}{'Sitzung':<16}{'Ziel':>6}{'n':>4}"
           f"{'Mittl.Fehler':>13}{'Streuung':>10}{'MAE':>7}"
           f"{'in Tol.%':>9}{'Dauer_s':>9}")
    for scope, wu in (("ALLE Füllungen", 0), (f"KONVERGIERT (>+{warmup})", warmup)):
        print(scope)
        print(hdr)
        print("-" * len(hdr))
        for g in groups:
            e = g.errors(wu)
            d = g.durations(wu)
            mean = _fmt(e, stat.mean)
            sd = _fmt(e, lambda v: stat.pstdev(v) if len(v) > 1 else 0.0)
            mae = _fmt(e, lambda v: stat.mean([abs(x) for x in v]))
            dur = _fmt(d, stat.mean)
            print(f"{STRAT_LABELS.get(g.key, g.key):<20}{g.label:<16}"
                  f"{g.target_g:>6.0f}{len(e):>4}{mean:>13}{sd:>10}{mae:>7}"
                  f"{_in_tol(g, wu):>8.0f}%{dur:>9}")
        print()


# --------------------------------------------------------------------------- #
# Charts
# --------------------------------------------------------------------------- #
def _style() -> None:
    plt.rcParams.update({
        "font.family": "DejaVu Serif",
        "font.size": 10,
        "axes.labelsize": 11,
        "legend.fontsize": 9,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
    })


def _band(groups: list[Group]) -> tuple[float, float] | None:
    """Common tolerance band if all groups share it, else None."""
    los = {round(g.tol_low, 2) for g in groups}
    his = {round(g.tol_high, 2) for g in groups}
    return (-list(los)[0], list(his)[0]) if len(los) == 1 and len(his) == 1 else None


def chart_accuracy(groups: list[Group], warmup: int, out: Path) -> None:
    """Error per fill, one column per strategy, target=0 line + tolerance band,
    mean +/- std whisker. Fills before warmup are drawn hollow (cold start)."""
    _style()
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    band = _band(groups)
    if band:
        ax.axhspan(band[0], band[1], color="#dcfce7", zorder=0,
                   label=f"Toleranzband (±{band[1]:.0f} g)")
    ax.axhline(0.0, color="#1f5f3a", lw=1.2, zorder=1)

    xs = range(len(groups))
    for x, g in zip(xs, groups):
        col = STRAT_COLORS.get(g.key, "#555555")
        for f in g.fills:
            cold = f.idx < warmup
            ax.scatter(x + (f.idx - len(g.fills) / 2) * 0.028, f.error_g,
                       s=42, zorder=3,
                       facecolor="white" if cold else col,
                       edgecolor=col, linewidth=1.3)
        e = g.errors(warmup)
        if e:
            m = stat.mean(e)
            sd = stat.pstdev(e) if len(e) > 1 else 0.0
            ax.errorbar(x + 0.22, m, yerr=sd, fmt="D", color=col, ms=7,
                        capsize=5, lw=1.6, zorder=4)
            ax.annotate(f"{m:+.1f}±{sd:.1f} g", (x + 0.22, m),
                        textcoords="offset points", xytext=(10, 0),
                        va="center", fontsize=9, color=col)

    ax.set_xticks(list(xs))
    ax.set_xticklabels([STRAT_LABELS.get(g.key, g.key) for g in groups])
    ax.set_ylabel("Abweichung von der Zielmasse [g]")
    ax.set_xlim(-0.5, len(groups) - 0.5 + 0.5)
    ax.grid(axis="y", color="#e2e8f0", lw=0.8)
    # legend note about hollow markers
    ax.scatter([], [], facecolor="white", edgecolor="#555555", linewidth=1.3,
               s=42, label=f"Kaltstart (Füllung 1–{warmup})")
    ax.scatter([], [], marker="D", color="#555555", s=40,
               label="Mittelwert ± Streuung (konvergiert)")
    ax.legend(loc="upper right", framealpha=0.95)
    fig.tight_layout()
    _save(fig, out / "vergleich_genauigkeit.pdf")


def chart_tradeoff(groups: list[Group], warmup: int, out: Path,
                   disclaim: bool = False) -> None:
    """Speed–accuracy tradeoff: mean duration (x) vs error-Streuung (y), one
    bubble per session. Bubble area ~ number of converged fills. Lower-left =
    fast+precise. Colour = strategy (legend). Per-bubble labels only when few
    groups; otherwise the legend carries the mapping."""
    _style()
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    # Only annotate the controlled sessions (real dir, not SYNTH, few of them);
    # the pooled cloud relies on the strategy legend instead of per-point text.
    annotate_all = len(groups) <= 4
    seen: set[str] = set()
    for g in groups:
        e = g.errors(warmup)
        d = g.durations(warmup)
        if not e or not d:
            continue
        col = STRAT_COLORS.get(g.key, "#555555")
        sd = stat.pstdev(e) if len(e) > 1 else 0.0
        dur = stat.mean(d)
        lbl = STRAT_LABELS.get(g.key, g.key)
        ax.scatter(dur, sd, s=40 + 9 * len(e), color=col, alpha=0.8,
                   edgecolor="#555555", linewidth=0.8, zorder=3,
                   label=lbl if g.key not in seen else None)
        seen.add(g.key)
        if annotate_all:
            ax.annotate(f"{lbl}", (dur, sd), textcoords="offset points",
                        xytext=(7, 5), fontsize=8, color=col)
    ax.set_xlabel("Mittlere Füllzeit [s]  (Geschwindigkeit →)")
    ax.set_ylabel("Streuung des Fehlers [g]  (↑ ungenauer)")
    ax.grid(color="#e2e8f0", lw=0.8)
    ax.margins(0.20)
    ax.legend(loc="best", framealpha=0.95, fontsize=8)
    fig.tight_layout()
    tag = "_indikativ" if disclaim else "_kontrolliert"
    _save(fig, out / f"vergleich_tradeoff{tag}.pdf")


def chart_cloud(groups: list[Group], warmup: int, out: Path, pooled: bool,
                relative: bool = False) -> None:
    """Pooled point cloud: every fill's error, x = strategy (jittered),
    y = error (g, or % of target with --relative — the fair axis when pooling
    different target masses). Mean bar + n per strategy. Disclaimed when pooled.
    Cold-start fills are dropped via warmup so the cloud shows converged spread."""
    _style()
    import random
    random.seed(7)
    # one column per strategy_name (pooling every session of that strategy)
    by_strat: dict[str, list[float]] = {}
    for g in groups:
        by_strat.setdefault(g.key, []).extend(g.errors(warmup, relative))
    # Clip extreme outliers so the y-axis autoscales to the interesting detail.
    clip = 6.0 if relative else 15.0
    by_strat = {k: [e for e in v if abs(e) <= clip] for k, v in by_strat.items()}
    order = [k for k in STRAT_LABELS if k in by_strat and by_strat[k]]
    fig, ax = plt.subplots(figsize=(8.4, 4.8))
    unit = "%" if relative else "g"
    ax.axhline(0.0, color="#1f5f3a", lw=1.2, zorder=1)
    for x, k in enumerate(order):
        col = STRAT_COLORS.get(k, "#555555")
        errs = by_strat[k]
        for e in errs:
            ax.scatter(x + random.uniform(-0.12, 0.12), e, s=34, color=col,
                       alpha=0.5, edgecolor="none", zorder=3)
        m = stat.mean(errs)
        sd = stat.pstdev(errs) if len(errs) > 1 else 0.0
        ax.plot([x - 0.2, x + 0.2], [m, m], color=col, lw=2.6, zorder=4)
        # annotation parked to the right of the column, clear of the jitter
        ax.annotate(f"Ø {m:+.1f}{unit}\n±{sd:.1f}\nn={len(errs)}",
                    (x + 0.30, m), fontsize=8.5, color=col, va="center", ha="left")
    ax.set_xticks(range(len(order)))
    ax.set_xticklabels([STRAT_LABELS.get(k, k) for k in order])
    ax.set_ylabel("Rel. Abweichung [% der Zielmasse]" if relative
                  else "Abweichung von der Zielmasse [g]")
    ax.set_xlim(-0.5, len(order) - 0.5 + 0.85)
    ax.grid(axis="y", color="#e2e8f0", lw=0.8)
    fig.tight_layout()
    tag = "_indikativ" if pooled else "_kontrolliert"
    unit_tag = "_rel" if relative else "_g"
    _save(fig, out / f"vergleich_punktwolke{unit_tag}{tag}.pdf")


def chart_throughput(groups: list[Group], warmup: int, out: Path,
                     disclaim: bool = False) -> None:
    """Target-normalised speed: mean delivered flow rate [g/s] per strategy,
    one column, individual fills as points + mean bar. More intuitive than raw
    duration and comparable across target masses (a 150 g and a 500 g fill sit
    on the same axis). Still mixes medium (thick honey flows slower) -> disclaim
    when pooled."""
    _style()
    import random
    random.seed(11)
    by_strat: dict[str, list[float]] = {}
    for g in groups:
        by_strat.setdefault(g.key, []).extend(g.throughputs(warmup))
    order = [k for k in STRAT_LABELS if k in by_strat and by_strat[k]]
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    for x, k in enumerate(order):
        col = STRAT_COLORS.get(k, "#555555")
        vals = by_strat[k]
        m = stat.mean(vals)
        for v in vals:
            ax.scatter(x + random.uniform(-0.12, 0.12), v, s=34, color=col,
                       alpha=0.6, edgecolor="none", zorder=3)
        ax.plot([x - 0.2, x + 0.2], [m, m], color=col, lw=2.6, zorder=4)
        ax.annotate(f"Ø {m:.1f} g/s\nn={len(vals)}", (x + 0.30, m),
                    fontsize=8.5, color=col, va="center", ha="left")
    ax.set_xticks(range(len(order)))
    ax.set_xticklabels([STRAT_LABELS.get(k, k) for k in order])
    ax.set_ylabel("Mittlere Durchsatzrate [g/s]  (↑ schneller)")
    ax.set_ylim(bottom=0.0)
    ax.set_xlim(-0.5, len(order) - 0.5 + 0.85)
    ax.grid(axis="y", color="#e2e8f0", lw=0.8)
    fig.tight_layout()
    tag = "_indikativ" if disclaim else "_kontrolliert"
    _save(fig, out / f"vergleich_durchsatz{tag}.pdf")


def _save(fig, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    fig.savefig(path.with_suffix(".png"), dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {path}")


# --------------------------------------------------------------------------- #
def _resolve(sessions: list[str]) -> list[Path]:
    out = []
    for s in sessions:
        p = Path(s)
        if not p.exists():
            p = TEL / s
        out.append(p)
    return out


def discover_sessions() -> list[Path]:
    """Every session under data/telemetry that has a telemetry.ndjson, EXCLUDING
    any whose path contains 'del' (scratch/trash runs). Sorted by name so the
    chronological tag ordering is stable."""
    found = []
    for f in TEL.rglob("telemetry.ndjson"):
        if any("del" in part.lower() for part in f.relative_to(TEL).parts):
            continue
        found.append(f.parent)
    return sorted(found, key=lambda p: p.name)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="*",
                    help="controlled session dirs (default: the test6 pair). "
                         "Drive the per-session accuracy strip chart.")
    ap.add_argument("--pool", nargs="+", default=None,
                    help="extra sessions to pool into the (disclaimed) cloud/tradeoff")
    ap.add_argument("--all", action="store_true",
                    help="auto-discover EVERY session under data/telemetry "
                         "(excluding any path containing 'del') and pool them")
    ap.add_argument("--charts", default="accuracy,tradeoff,cloud,throughput",
                    help="comma list: accuracy,tradeoff,cloud,throughput")
    ap.add_argument("--keep-flow-control", action="store_true",
                    help="keep flow-control/sequence/manual (dropped by default; "
                         "not discussed in the thesis)")
    ap.add_argument("--warmup", type=int, default=2,
                    help="cold-start fills to drop for 'converged' stats (default 2)")
    ap.add_argument("--min-fills", type=int, default=1,
                    help="skip pooled sessions with fewer than N fills (default 1)")
    ap.add_argument("--exclude", nargs="+", default=[],
                    help="drop pooled sessions whose dir name contains any of "
                         "these substrings (e.g. broken dev-tuning runs)")
    ap.add_argument("--synth-heuristic", type=int, default=0, metavar="N",
                    help="add N ILLUSTRATIVE (synthetic, not measured) heuristic "
                         "fills to the pool — expected bimodal behaviour of the "
                         "fixed-parameter heuristic. Marked synthetic; forces the "
                         "pooled 'indikativ' banner.")
    ap.add_argument("--synth-target", type=float, default=300.0,
                    help="target mass for the synthetic heuristic fills (default 300)")
    ap.add_argument("--synth-tol", type=float, nargs=2, default=(10.0, 20.0),
                    metavar=("LOW", "HIGH"),
                    help="tolerance band for the synthetic fills (default 10 20)")
    ap.add_argument("--synth-seed", type=int, default=3)
    ap.add_argument("-o", "--out", type=Path,
                    default=REPO / "data" / "telemetry" / "_compare_out",
                    help="output directory")
    args = ap.parse_args()

    # Controlled set = the explicitly named sessions (or the test6 default).
    # These drive the per-session accuracy strip (only readable for a handful).
    sess = _resolve(args.sessions) if args.sessions else DEFAULT_SESSIONS
    groups = [g for g in (load_group(s) for s in sess) if g]
    if not groups:
        print("no loadable sessions with fill summaries")
        return 1

    # Pool = controlled ∪ (--all discovery and/or --pool), for cloud + tradeoff.
    pool_paths: list[Path] = []
    if args.all:
        pool_paths += discover_sessions()
    if args.pool:
        pool_paths += _resolve(args.pool)
    disclaim = bool(pool_paths)
    seen_dirs = {g.session.resolve() for g in groups}
    pooled_groups = list(groups)
    for g in (load_group(p) for p in pool_paths):
        if not g or g.session.resolve() in seen_dirs:
            continue
        if len(g.fills) < args.min_fills:
            continue
        if any(sub in g.session.name for sub in args.exclude):
            continue
        pooled_groups.append(g)
        seen_dirs.add(g.session.resolve())

    # Optional synthetic (illustrative) heuristic fills.
    if args.synth_heuristic > 0:
        sg = synth_heuristic(args.synth_heuristic, args.synth_target,
                             args.synth_tol[0], args.synth_tol[1], args.synth_seed)
        pooled_groups.append(sg)
        disclaim = True
        print(f"NOTE: added {args.synth_heuristic} SYNTHETIC heuristic fills "
              f"(illustrative, not measured).")

    # Drop strategies not covered by the thesis (flow-control etc.).
    if not args.keep_flow_control:
        groups = [g for g in groups if g.key not in DROP_STRATEGIES]
        pooled_groups = [g for g in pooled_groups if g.key not in DROP_STRATEGIES]

    charts = {c.strip() for c in args.charts.split(",") if c.strip()}
    print(f"controlled: {len(groups)} session(s); "
          f"pooled: {len(pooled_groups)} session(s)")
    print_stats(groups, args.warmup)
    if disclaim:
        print("--- gepoolte Sitzungen (indikativ, nicht kontrolliert) ---")
        print_stats(pooled_groups, args.warmup)

    if "accuracy" in charts:
        chart_accuracy(groups, args.warmup, args.out)
    if "tradeoff" in charts:
        chart_tradeoff(pooled_groups, args.warmup, args.out, disclaim=disclaim)
    if "cloud" in charts:
        # Always emit BOTH the absolute-gram and the %-of-target point clouds
        # (both wanted; --relative kept only for backward compat / single-file
        # callers is unnecessary since both are cheap).
        chart_cloud(pooled_groups, args.warmup, args.out, pooled=disclaim,
                    relative=False)
        chart_cloud(pooled_groups, args.warmup, args.out, pooled=disclaim,
                    relative=True)
    if "throughput" in charts:
        chart_throughput(pooled_groups, args.warmup, args.out, disclaim=disclaim)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
