#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
python_bin="${BONESAW_EVAL_PYTHON:-${eval_venv}/bin/python}"
maturin_bin="${eval_venv}/bin/maturin"
corpus="python/evals/floating_walk_corpus.py"
reference="benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz"
witness="benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz"

VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${maturin_bin}" develop --release

run_trace() {
  local output="$1"
  shift
  PYTHONPATH=python/evals "${python_bin}" "${corpus}" \
    --reference-inputs "${reference}" \
    --initial-state-inputs "${witness}" \
    --ticks 2317 \
    --maximum-feasibility-projection-sweeps 8 \
    --continue-identical-exhausted-feasibility-prefix \
    --maximum-contact-solve-hold-ticks 12 \
    --localized-contact-fallback-target 1 \
    --output "benchmarks/results/${output}" \
    "$@"
}

run_trace floating-g1-r269-low-gain-cross-tick-cap8x13 \
  --morphology-posture-trace \
  --joint-posture-weight 0.05

run_trace floating-g1-r269-local-handoff-centroidal-cap8x13 \
  --centroidal-angular-momentum-weight 1 \
  --centroidal-angular-momentum-priority 2

PYTHONPATH=python/evals "${python_bin}" \
  python/evals/g1_bounded_contact_continuation_r269.py
