#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${eval_venv}/bin/maturin" develop --release
cargo test -p bonesaw-core robot_observation_admission_separates_causality_age_and_clock_uncertainty
PYTHONPATH=python:python/evals "${eval_venv}/bin/python" \
  python/evals/robot_observation_authority_report.py "$@"
