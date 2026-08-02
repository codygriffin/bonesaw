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
  local weight="$2"
  local braking="$3"
  local reaction="$4"
  PYTHONPATH=python:python/evals "${python_bin}" python/evals/floating_walk_corpus.py \
    --reference-inputs "${reference}" \
    --initial-state-inputs "${witness}" \
    --ticks 2317 \
    --maximum-feasibility-projection-sweeps 8 \
    --morphology-posture-trace \
    --joint-posture-weight 0.05 \
    --output "benchmarks/results/${output}" \
    --joint-position-capture-weight "${weight}" \
    --joint-position-capture-assumed-braking-acceleration "${braking}" \
    --joint-position-capture-reaction-time-seconds "${reaction}" \
    --joint-velocity-envelope-weight 0.25 \
    --joint-velocity-envelope-activation-fraction 0.50 \
    --joint-velocity-envelope-phase-policy multi-support \
    --joint-velocity-envelope-lower-body-only \
    --joint-velocity-envelope-hard
}

run_trace floating-g1-r274-position-capture-dormant-cap8 0.0 50.0 0.02
run_trace floating-g1-r274-position-capture-w025-b25-r02-cap8 0.25 25.0 0.02
run_trace floating-g1-r274-position-capture-w025-b50-r02-cap8 0.25 50.0 0.02
run_trace floating-g1-r274-position-capture-w025-b50-r04-cap8 0.25 50.0 0.04
run_trace floating-g1-r274-position-capture-w05-b50-r02-cap8 0.50 50.0 0.02
run_trace floating-g1-r274-position-capture-w1-b50-r02-cap8 1.00 50.0 0.02

PYTHONPATH=python:python/evals "${python_bin}" \
  python/evals/g1_joint_position_capture_r274.py
