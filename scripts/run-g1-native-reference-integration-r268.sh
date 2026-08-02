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
    --output "benchmarks/results/${output}" \
    "$@"
}

run_trace floating-g1-r268-native-reference-witness-cap8
run_trace floating-g1-r268-native-reference-oracle-stack-cap8 \
  --root-height-task-weight 1 \
  --root-horizontal-task-priority 1 \
  --joint-posture-weight 0.01 \
  --center-of-mass-task-weight 1 \
  --center-of-mass-task-priority 1 \
  --minimum-contact-cop-margin 0.005
run_trace floating-g1-r268-native-reference-morphology-jet-cap8 \
  --morphology-posture-trace
run_trace floating-g1-r268-native-reference-morphology-jet-low-gain-cap8 \
  --morphology-posture-trace \
  --joint-posture-weight 0.05

PYTHONPATH=python/evals "${python_bin}" \
  python/evals/g1_native_reference_integration_r268.py
