#!/usr/bin/env bash
set -euo pipefail

python_bin="${BONESAW_EVAL_PYTHON:-/tmp/bonesaw-placo/bin/python}"
eval_cpu="${BONESAW_EVAL_CPU:-4}"
model="${BONESAW_G1_URDF:-benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf}"
corpus="python/evals/floating_walk_corpus.py"

run_case() {
  local name="$1"
  shift
  PYTHONPATH=python:python/evals taskset -c "${eval_cpu}" "${python_bin}" "${corpus}" \
    --model "${model}" \
    --reference-inputs benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz \
    --initial-state-inputs benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz \
    --morphology-posture-trace --ticks 2317 --joint-posture-weight 0.05 \
    --joint-velocity-envelope-weight 0.25 \
    --joint-velocity-envelope-activation-fraction 0.5 \
    --joint-velocity-envelope-lower-body-only --joint-velocity-envelope-hard \
    --joint-velocity-envelope-phase-policy multi-support \
    --maximum-feasibility-projection-sweeps 8 \
    --maximum-feasibility-iterations 7 \
    --support-trajectory-tube-preview-ticks 229 \
    --support-trajectory-tube-margin 0.002 \
    --support-trajectory-tube-max-velocity 0.5 \
    --support-trajectory-tube-max-acceleration 20.0 \
    --support-reachable-tube --support-reachable-tube-barrier-rate 80.0 \
    --support-trajectory-tube-project-intent \
    --center-of-mass-task-weight 0.01 --center-of-mass-task-priority 4 \
    "$@" --output "benchmarks/results/floating-g1-r285-${name}"
}

run_case low-authority-unbounded
run_case low-authority-style1 --maximum-style-task-pseudoinverses 1
run_case low-authority-style2 --maximum-style-task-pseudoinverses 2
run_case low-authority-pref20-style2 \
  --maximum-preference-task-pseudoinverses 20 \
  --maximum-style-task-pseudoinverses 2

PYTHONPATH=python/evals "${python_bin}" python/evals/g1_low_authority_budget_r285.py \
  --unbounded benchmarks/results/floating-g1-r285-low-authority-unbounded \
  --style1 benchmarks/results/floating-g1-r285-low-authority-style1 \
  --style2 benchmarks/results/floating-g1-r285-low-authority-style2 \
  --pref20-style2 benchmarks/results/floating-g1-r285-low-authority-pref20-style2
PYTHONPATH=python/evals "${python_bin}" -m unittest \
  python/evals/test_g1_low_authority_budget_r285.py
