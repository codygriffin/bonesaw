#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${eval_venv}/bin/maturin" develop --release
cargo test -p bonesaw-core command_tracking_envelope_separates_nominal_contingency_and_reject
PYTHONPATH=python:python/evals "${eval_venv}/bin/python" \
  python/evals/command_tracking_authority_report.py "$@"
