#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
python_bin="${BONESAW_EVAL_PYTHON:-${eval_venv}/bin/python}"
maturin_bin="${eval_venv}/bin/maturin"
reference="benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz"
witness="benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz"

VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${maturin_bin}" develop --release

run_trace() {
  local output="$1"
  local interval="$2"
  PYTHONPATH=python:python/evals "${python_bin}" python/evals/floating_walk_corpus.py \
    --reference-inputs "${reference}" \
    --initial-state-inputs "${witness}" \
    --ticks 2317 \
    --maximum-feasibility-projection-sweeps 8 \
    --morphology-posture-trace \
    --joint-posture-weight 0.05 \
    --output "benchmarks/results/${output}" \
    --normal-fallback-relock-probe-interval-ticks "${interval}" \
    --joint-velocity-envelope-weight 0.25 \
    --joint-velocity-envelope-activation-fraction 0.50 \
    --joint-velocity-envelope-phase-policy multi-support \
    --joint-velocity-envelope-lower-body-only \
    --joint-velocity-envelope-hard
}

run_trace floating-g1-r273-normal-fallback-relock-dormant-cap8 0
for interval in 1 4 8 16 32 64; do
  run_trace "floating-g1-r273-normal-fallback-relock-${interval}-cap8" "${interval}"
done

PYTHONPATH=python:python/evals "${python_bin}" \
  python/evals/g1_normal_fallback_relock_probe_r273.py
