#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
python_bin="${BONESAW_EVAL_PYTHON:-${eval_venv}/bin/python}"
maturin_bin="${eval_venv}/bin/maturin"
timing_cpu="${BONESAW_TIMING_CPU:-4}"
output_root="benchmarks/results"
corpus="python/evals/floating_walk_corpus.py"

VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${maturin_bin}" develop --release

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

RUSTFLAGS='-C target-cpu=native' VIRTUAL_ENV="${eval_venv}" \
  PATH="${eval_venv}/bin:${PATH}" "${maturin_bin}" develop --release

for trial in 0 1 2 3 4; do
  env PYTHONPATH=python/evals taskset -c "${timing_cpu}" \
    "${python_bin}" "${corpus}" \
    --ticks 600 \
    --output "${output_root}/g1-floating-projection-budget-profile-r262/native-polish2-trial-${trial}" \
    --maximum-feasibility-projection-sweeps 8 \
    --maximum-feasibility-iterations 2
done

PYTHONPATH=python/evals "${python_bin}" \
  python/evals/g1_floating_projection_budget_profile_r262.py
