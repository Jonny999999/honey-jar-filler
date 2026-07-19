#!/usr/bin/env bash
#
# One command for the thesis ch. 07 plant-response figure.
#
#   generate SIMULATED telemetry -> split into fills -> plot -> install into the
#   thesis Figures/strategies/plots/general/
#
# Why this exists: the figure needs a plant probe (gate 30 % -> 70 % -> close)
# that also *shows* the thick-honey process effects the chapter talks about -
# long dead time, low flow, drip noise, Nachlauf, jet-impact peaks. The measured
# 30/70/close run was recorded with a water-thin dummy medium and shows almost
# none of them, and there is no thick honey left to re-run it with. So the plant
# characteristics are identified from the measured thick-honey fill and replayed
# under the gate script we want.
#
# The output is SIMULATED. It is marked synthetic in session_meta.json, in the
# session name and in every figure filename. The caption must say so - see
# CAPTION below.
#
# Usage:
#   tools/telemetry/make_ch07_plant_figure.sh                  # generate + install
#   tools/telemetry/make_ch07_plant_figure.sh --dry-run        # identify only
#   SCRIPT=30:14,70:16,0:24 tools/telemetry/make_ch07_plant_figure.sh
#
# Any other argument is passed through to synth_plant_session.py, so plant
# parameters can be overridden, e.g. --dead-time-s 5.5 --flow-noise-frac 0.2

set -euo pipefail

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${TOOLS_DIR}/../.." && pwd)"

# The measured thick-honey fill the plant is identified from.
REFERENCE="${REFERENCE:-${REPO_ROOT}/data/telemetry/done-old/2026.06.14_20:54-x-heuristic-settings-changed}"
REFERENCE_FILL="${REFERENCE_FILL:-3}"

# The excitation we want to show: partial open, wider open, full close, then
# long enough to let the Nachlauf and the late drips play out. The close dwell
# stops at 18 s on purpose: that puts the right edge just after the last drip,
# so the tail stays readable instead of spending 12 s on empty axis. The run is
# seeded, so shortening the last step only drops samples off the end - it does
# not reshuffle anything before it.
SCRIPT="${SCRIPT:-30:18,70:22,0:18}"
POST_S="${POST_S:-0.6}"

# Presentational damping of the two transients, applied as explicit overrides so
# the identified values still show up in the report and in session_meta.json.
# At their identified size the first-contact and close peaks set the rate axis
# on their own and squash the 2.3 g/s plateau of the 30 % dwell into the
# baseline, which is one of the things the figure exists to show. Spreading the
# blob over 0.5 s instead of 0.2 s and trimming the flap shock keeps both peaks
# clearly present and labelable while letting the plateau read.
PEAK_OVERRIDES=(--onset-pool-s 0.35 --onset-impact-g 2.0 --close-impact-g 7.5)

SESSION_NAME="${SESSION_NAME:-2026.07.16_SYNTHETIC-plant-probe_30-70-close}"
OUT_DIR="${OUT_DIR:-${REPO_ROOT}/data/telemetry/${SESSION_NAME}}"

THESIS_FIG_DIR="${THESIS_FIG_DIR:-/home/jonny/HAW-LA/s8/latex/Figures/strategies/plots/general}"

CAPTION='Simulierter Streckenverlauf. Die Prozesskenngroessen wurden aus einer
gemessenen Fuellung mit zaehem Honig identifiziert und unter der gezeigten
Klappenansteuerung reproduziert. Als *simuliert* kennzeichnen.'

echo "==> Generating SIMULATED plant-probe session"
echo "    reference : ${REFERENCE} (fill ${REFERENCE_FILL})"
echo "    script    : ${SCRIPT}"
echo "    output    : ${OUT_DIR}"
echo

python3 "${TOOLS_DIR}/synth_plant_session.py" \
    --reference "${REFERENCE}" \
    --reference-fill "${REFERENCE_FILL}" \
    --script "${SCRIPT}" \
    --out "${OUT_DIR}" \
    --post-s "${POST_S}" \
    "${PEAK_OVERRIDES[@]}" \
    --plot \
    "$@"

# --dry-run writes nothing, so there is nothing to annotate or install.
for arg in "$@"; do
    if [ "${arg}" = "--dry-run" ]; then
        exit 0
    fi
done

echo
echo "==> Drawing the annotation layer (plain_narrow, PDF only)"
python3 "${TOOLS_DIR}/annotate_ch07.py" "${OUT_DIR}"

if [ ! -d "${THESIS_FIG_DIR}" ]; then
    echo
    echo "==> Thesis figure dir not found, skipping install:"
    echo "    ${THESIS_FIG_DIR}"
    echo "    Figures are in ${OUT_DIR}/figures"
    exit 0
fi

echo
echo "==> Installing into ${THESIS_FIG_DIR}"

# Drop earlier figures from *this* session first. The filename carries the run
# duration, so changing the gate script renames every file and would otherwise
# leave a stale set behind for \includegraphics to pick the wrong one out of.
# Scoped to this session's prefix - nothing else in the folder is touched.
for old in "${THESIS_FIG_DIR}/${SESSION_NAME}"__*; do
    [ -e "${old}" ] || continue
    rm -f "${old}"
    echo "    removed stale $(basename "${old}")"
done

installed=0
for f in "${OUT_DIR}"/figures/*plain_narrow_annotated.pdf \
         "${OUT_DIR}"/figures/*plain_narrow.pdf \
         "${OUT_DIR}"/figures/*plain_wide.pdf \
         "${OUT_DIR}"/figures/*plain_narrow.svg \
         "${OUT_DIR}"/figures/*debug.pdf; do
    [ -e "${f}" ] || continue
    cp "${f}" "${THESIS_FIG_DIR}/"
    echo "    $(basename "${f}")"
    installed=$((installed + 1))
done
echo "    ${installed} file(s)"

echo
echo "==> Use in Chapters/07_prozessanalyse_und_versuchsgrundlagen.tex:"
echo
for f in "${OUT_DIR}"/figures/*plain_narrow_annotated.pdf; do
    [ -e "${f}" ] || continue
    echo "    \\includegraphics[width=1\\textwidth]{Figures/strategies/plots/general/$(basename "${f}")}"
done
echo
echo "==> Caption MUST declare the data as simulated, e.g.:"
echo "${CAPTION}" | sed 's/^/    /'
