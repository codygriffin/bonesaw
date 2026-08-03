#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
python_bin="${BONESAW_EVAL_PYTHON:-${eval_venv}/bin/python}"
common=(
  --reference-inputs benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz
  --initial-state-inputs benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz
  --morphology-posture-trace --ticks 2317 --joint-posture-weight 0.05
  --joint-velocity-envelope-weight 0.25
  --joint-velocity-envelope-activation-fraction 0.5
  --joint-velocity-envelope-lower-body-only --joint-velocity-envelope-hard
  --joint-velocity-envelope-phase-policy multi-support
  --maximum-feasibility-projection-sweeps 8
)

VIRTUAL_ENV="${eval_venv}" "${eval_venv}/bin/maturin" develop --release
PYTHONPATH=python:python/evals "${python_bin}" python/evals/floating_walk_corpus.py \
  "${common[@]}" --output benchmarks/results/floating-g1-r277-dcm-dormant-cap8
PYTHONPATH=python:python/evals "${python_bin}" python/evals/floating_walk_corpus.py \
  "${common[@]}" --center-of-mass-controller dcm-zmp \
  --center-of-mass-task-weight 0.00005 --dcm-pre-liftoff-activation-ticks 10 \
  --output benchmarks/results/floating-g1-r277-dcm-gated-w0p00005-h10-cap8
PYTHONPATH=python:python/evals "${python_bin}" python/evals/floating_walk_corpus.py \
  "${common[@]}" --center-of-mass-controller dcm-zmp \
  --center-of-mass-task-weight 0.025 \
  --output benchmarks/results/floating-g1-r277-dcm-continuous-w0p025-cap8

screen=(
  "0.00001 5 w0p00001-h5"
  "0.000025 5 w0p000025-h5"
  "0.00005 5 w0p00005-h5"
  "0.0001 5 w0p0001-h5"
  "0.00001 10 w0p00001-h10"
  "0.000025 10 w0p000025-h10"
  "0.00005 10 w0p00005-h10"
  "0.0001 10 w0p0001-h10"
)
for specification in "${screen[@]}"; do
  read -r weight horizon slug <<<"${specification}"
  PYTHONPATH=python:python/evals "${python_bin}" python/evals/floating_walk_corpus.py \
    "${common[@]}" --center-of-mass-controller dcm-zmp \
    --center-of-mass-task-weight "${weight}" \
    --dcm-pre-liftoff-activation-ticks "${horizon}" \
    --output "benchmarks/results/floating-g1-r277-dcm-${slug}-ss-cap8"
done

PYTHONPATH=python:python/evals "${python_bin}" \
  python/evals/g1_dcm_transition_gate_r277.py
