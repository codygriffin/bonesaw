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
  "${common[@]}" --output benchmarks/results/floating-g1-r276-auto-localization-dormant-cap8
PYTHONPATH=python:python/evals "${python_bin}" python/evals/floating_walk_corpus.py \
  "${common[@]}" --automatic-contact-fault-localization \
  --output benchmarks/results/floating-g1-r276-auto-localization-enabled-cap8
PYTHONPATH=python:python/evals "${python_bin}" \
  python/evals/g1_automatic_contact_localization_r276.py
