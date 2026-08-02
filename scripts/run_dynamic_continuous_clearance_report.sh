#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${eval_venv}/bin/maturin" develop --release
cargo test -p bonesaw-core collision::tests
cargo test -p bonesaw-core dynamic_controller::tests
PYTHONPATH=python:python/evals "${eval_venv}/bin/python" \
  python/evals/dynamic_continuous_clearance_report.py "$@"
