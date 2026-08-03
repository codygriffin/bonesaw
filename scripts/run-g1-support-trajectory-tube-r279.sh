#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
python_bin="${BONESAW_EVAL_PYTHON:-${eval_venv}/bin/python}"
corpus="python/evals/floating_walk_corpus.py"
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
tube=(
  --support-trajectory-tube-preview-ticks 5
  --support-trajectory-tube-margin 0.002
  --support-trajectory-tube-max-velocity 0.5
  --support-trajectory-tube-max-acceleration 20.0
)

VIRTUAL_ENV="${eval_venv}" "${eval_venv}/bin/maturin" develop --release
PYTHONPATH=python:python/evals "${python_bin}" "${corpus}" \
  "${common[@]}" \
  --output benchmarks/results/floating-g1-r279-dormant-cap8
PYTHONPATH=python:python/evals "${python_bin}" "${corpus}" \
  "${common[@]}" "${tube[@]}" --support-trajectory-tube \
  --output benchmarks/results/floating-g1-r279-local-position-h5-cap8
PYTHONPATH=python:python/evals "${python_bin}" "${corpus}" \
  "${common[@]}" "${tube[@]}" \
  --support-reachable-tube --support-reachable-tube-barrier-rate 2.0 \
  --output benchmarks/results/floating-g1-r279-reachable-observer-h5-rate2-cap8
PYTHONPATH=python:python/evals "${python_bin}" "${corpus}" \
  "${common[@]}" "${tube[@]}" \
  --support-reachable-tube --support-reachable-tube-hard \
  --support-reachable-tube-barrier-rate 2.0 \
  --output benchmarks/results/floating-g1-r279-reachable-hard-h5-rate2-cap8
for horizon in 10 25; do
  PYTHONPATH=python:python/evals "${python_bin}" "${corpus}" \
    "${common[@]}" \
    --support-trajectory-tube-preview-ticks "${horizon}" \
    --support-trajectory-tube-margin 0.002 \
    --support-trajectory-tube-max-velocity 0.5 \
    --support-trajectory-tube-max-acceleration 20.0 \
    --support-reachable-tube --support-reachable-tube-hard \
    --support-reachable-tube-barrier-rate 2.0 \
    --output "benchmarks/results/floating-g1-r279-reachable-hard-h${horizon}-rate2-cap8"
done
PYTHONPATH=python:python/evals "${python_bin}" "${corpus}" \
  "${common[@]}" "${tube[@]}" \
  --support-reachable-tube --support-reachable-tube-hard \
  --support-reachable-tube-barrier-rate 2.0 \
  --support-trajectory-tube-project-intent \
  --output benchmarks/results/floating-g1-r279-reachable-combined-h5-rate2-cap8
PYTHONPATH=python:python/evals "${python_bin}" \
  python/evals/g1_support_trajectory_tube_r279.py
