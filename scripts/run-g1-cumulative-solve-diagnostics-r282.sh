#!/usr/bin/env bash
set -euo pipefail

python_bin="${BONESAW_EVAL_PYTHON:-/tmp/bonesaw-placo/bin/python}"
python_path="${BONESAW_PYTHONPATH:-python:python/evals}"
corpus="python/evals/floating_walk_corpus.py"
output="benchmarks/results/floating-g1-r282-cumulative-solve-diagnostics"
cpu="${BONESAW_EVAL_CPU:-4}"

PYTHONPATH="${python_path}" taskset -c "${cpu}" "${python_bin}" "${corpus}" \
  --reference-inputs benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz \
  --initial-state-inputs benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz \
  --morphology-posture-trace --ticks 2317 --joint-posture-weight 0.05 \
  --joint-velocity-envelope-weight 0.25 \
  --joint-velocity-envelope-activation-fraction 0.5 \
  --joint-velocity-envelope-lower-body-only --joint-velocity-envelope-hard \
  --joint-velocity-envelope-phase-policy multi-support \
  --maximum-feasibility-projection-sweeps 8 \
  --support-trajectory-tube-preview-ticks 229 \
  --support-trajectory-tube-margin 0.002 \
  --support-trajectory-tube-max-velocity 0.5 \
  --support-trajectory-tube-max-acceleration 20.0 \
  --support-reachable-tube --support-reachable-tube-barrier-rate 80.0 \
  --support-trajectory-tube-project-intent \
  --center-of-mass-task-weight 0.01 --center-of-mass-task-priority 4 \
  --output "${output}"

PYTHONPATH="python/evals" "${python_bin}" \
  python/evals/g1_cumulative_solve_diagnostics_r282.py
