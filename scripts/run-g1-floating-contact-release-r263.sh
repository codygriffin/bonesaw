#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PYTHON=${PYTHON:-python3}
OUTPUT=${OUTPUT:-"$ROOT/benchmarks/results/floating-g1-transfer-r263-release-cap8"}

cd "$ROOT"
PYTHONPATH="$ROOT/python/evals${PYTHONPATH:+:$PYTHONPATH}" \
  "$PYTHON" python/evals/floating_walk_corpus.py \
  --ticks 600 \
  --maximum-feasibility-projection-sweeps 8 \
  --output "$OUTPUT"

PYTHONPATH="$ROOT/python/evals${PYTHONPATH:+:$PYTHONPATH}" \
  "$PYTHON" python/evals/g1_floating_contact_release_r263.py \
  --candidate "$OUTPUT/floating-walk-raw.npz"
