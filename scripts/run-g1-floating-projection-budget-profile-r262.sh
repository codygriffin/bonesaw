#!/usr/bin/env bash
set -euo pipefail

python_bin="${BONESAW_EVAL_PYTHON:-/tmp/bonesaw-placo/bin/python}"
output_root="benchmarks/results"
corpus="python/evals/floating_walk_corpus.py"

run_trace() {
  local output="$1"
  shift
  PYTHONPATH=python/evals "${python_bin}" "${corpus}" \
    --ticks 600 \
    --output "${output_root}/${output}" \
    "$@"
}

run_trace floating-g1-transfer-latest
run_trace floating-g1-transfer-r262-cap8 \
  --maximum-feasibility-projection-sweeps 8
run_trace floating-g1-transfer-r262-cap16 \
  --maximum-feasibility-projection-sweeps 16
run_trace floating-g1-transfer-r262-cap32 \
  --maximum-feasibility-projection-sweeps 32
run_trace floating-g1-transfer-r262-cap64 \
  --maximum-feasibility-projection-sweeps 64

PYTHONPATH=python/evals "${python_bin}" \
  python/evals/g1_floating_projection_budget_profile_r262.py
