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
  shift
  PYTHONPATH=python/evals "${python_bin}" python/evals/floating_walk_corpus.py \
    --reference-inputs "${reference}" \
    --initial-state-inputs "${witness}" \
    --ticks 2317 \
    --maximum-feasibility-projection-sweeps 8 \
    --morphology-posture-trace \
    --joint-posture-weight 0.05 \
    --output "benchmarks/results/${output}" \
    "$@"
}

run_trace floating-g1-r270-lower-body-dormant-r269-stack-cap8 \
  --continue-identical-exhausted-feasibility-prefix \
  --maximum-contact-solve-hold-ticks 12 \
  --localized-contact-fallback-target 1

run_trace floating-g1-r270-lower-body-soft-only-cap8 \
  --joint-velocity-envelope-weight 0.25 \
  --joint-velocity-envelope-activation-fraction 0.50 \
  --joint-velocity-envelope-phase-policy multi-support \
  --joint-velocity-envelope-lower-body-only

run_trace floating-g1-r270-lower-body-hard-only-cap8 \
  --joint-velocity-envelope-activation-fraction 0.50 \
  --joint-velocity-envelope-phase-policy multi-support \
  --joint-velocity-envelope-lower-body-only \
  --joint-velocity-envelope-hard

run_trace floating-g1-r270-lower-body-hard-cap8 \
  --joint-velocity-envelope-weight 0.25 \
  --joint-velocity-envelope-activation-fraction 0.50 \
  --joint-velocity-envelope-phase-policy multi-support \
  --joint-velocity-envelope-lower-body-only \
  --joint-velocity-envelope-hard

run_trace floating-g1-r270-lower-body-hard-only-r269-stack-cap8 \
  --continue-identical-exhausted-feasibility-prefix \
  --maximum-contact-solve-hold-ticks 12 \
  --localized-contact-fallback-target 1 \
  --joint-velocity-envelope-activation-fraction 0.50 \
  --joint-velocity-envelope-phase-policy multi-support \
  --joint-velocity-envelope-lower-body-only \
  --joint-velocity-envelope-hard

run_trace floating-g1-r270-lower-body-hard-r269-stack-cap8 \
  --continue-identical-exhausted-feasibility-prefix \
  --maximum-contact-solve-hold-ticks 12 \
  --localized-contact-fallback-target 1 \
  --joint-velocity-envelope-weight 0.25 \
  --joint-velocity-envelope-activation-fraction 0.50 \
  --joint-velocity-envelope-phase-policy multi-support \
  --joint-velocity-envelope-lower-body-only \
  --joint-velocity-envelope-hard
PYTHONPATH=python/evals "${python_bin}" \
  python/evals/g1_lower_body_velocity_envelope_r270.py
