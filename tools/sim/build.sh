#!/usr/bin/env bash
# Build the host simulation library from the REAL firmware strategy sources.
# Output: tools/sim/build/libfillsim.so
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
fw="$here/../../firmware_honey-jar-filler"
out="$here/build"
mkdir -p "$out"

srcs=(
  "$fw/components/filler/filler_strategy_heuristic.c"
  "$fw/components/filler/filler_strategy_adaptive_heuristic.c"
  "$fw/components/filler/filler_strategy_flow_control.c"
  "$fw/components/filler/filler_strategy_flow_cascade.c"
)
defs=()
if [ -f "$fw/components/filler/filler_strategy_adaptive_gate.c" ]; then
  srcs+=("$fw/components/filler/filler_strategy_adaptive_gate.c")
  defs+=(-DSIM_HAVE_ADAPTIVE_GATE)
fi

cc -std=gnu11 -O2 -g -fPIC -shared -Wall -Wextra -Wno-unused-parameter \
  "${defs[@]}" \
  -I"$here/shim" \
  -I"$fw/components/filler" -I"$fw/components/app" -I"$fw/main" \
  "$here/harness.c" "${srcs[@]}" -lm -o "$out/libfillsim.so"
echo "built $out/libfillsim.so"
