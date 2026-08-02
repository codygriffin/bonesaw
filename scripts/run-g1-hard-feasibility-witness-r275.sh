#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
python_bin="${BONESAW_EVAL_PYTHON:-${eval_venv}/bin/python}"
maturin_bin="${eval_venv}/bin/maturin"
reference="benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz"
witness="benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz"

VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${maturin_bin}" develop --release

base_args=(
  --reference-inputs "${reference}"
  --initial-state-inputs "${witness}"
  --ticks 2317
  --maximum-feasibility-projection-sweeps 8
  --morphology-posture-trace
  --joint-posture-weight 0.05
  --joint-velocity-envelope-weight 0.25
  --joint-velocity-envelope-activation-fraction 0.50
  --joint-velocity-envelope-phase-policy multi-support
  --joint-velocity-envelope-lower-body-only
  --joint-velocity-envelope-hard
)

run_trace() {
  local output="$1"
  shift
  PYTHONPATH=python:python/evals "${python_bin}" python/evals/floating_walk_corpus.py \
    "${base_args[@]}" --output "benchmarks/results/${output}" "$@"
}

run_trace floating-g1-r275-hard-witness-control-cap8
run_trace floating-g1-r275-hard-witness-cap16 --maximum-feasibility-projection-sweeps 16
run_trace floating-g1-r275-hard-witness-cap64 --maximum-feasibility-projection-sweeps 64
run_trace floating-g1-r275-position-capture-hard-w025-cap8 \
  --joint-position-capture-weight 0.25 --joint-position-capture-hard
run_trace floating-g1-r275-ablation-local0 --localized-contact-fallback-target 0
run_trace floating-g1-r275-ablation-local1 --localized-contact-fallback-target 1
run_trace floating-g1-r275-ablation-accel1000 --maximum-acceleration 1000
run_trace floating-g1-r275-ablation-torque20000 --maximum-torque 20000
run_trace floating-g1-r275-ablation-normal30 --maximum-normal-force-multiple 30
run_trace floating-g1-r275-ablation-friction2 --friction 2
run_trace floating-g1-r275-ablation-friction3 --friction 3
run_trace floating-g1-r275-ablation-friction5 --friction 5
run_trace floating-g1-r275-ablation-friction10 --friction 10
run_trace floating-g1-r275-ablation-point \
  --contact-patch-half-length 0 --contact-patch-half-width 0

PYTHONPATH=python:python/evals "${python_bin}" \
  python/evals/g1_hard_feasibility_witness_r275.py
