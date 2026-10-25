#!/usr/bin/env bash
# Compare YOUR src/my_open with the reference solution of a step.
#
#   bash tools/compare-step.sh 3          # your tree against the tag step-3
#   bash tools/compare-step.sh 2 3        # what the reference changes between step-2 and step-3
#
# src/my_open is ignored on main, so git is told about it first (git add -f -N: "intent to add", nothing is staged).
set -euo pipefail
cd "$(dirname "$0")/.."
git rev-parse --verify --quiet "step-$1" >/dev/null || { echo "No existe la etiqueta step-$1: haz  git fetch --tags" >&2; exit 1; }
if [ $# -ge 2 ]; then
  git diff "step-$1" "step-$2" -- src/my_open
else
  [ -d src/my_open ] && git add -f -N src/my_open
  git diff "step-$1" -- src/my_open
fi
