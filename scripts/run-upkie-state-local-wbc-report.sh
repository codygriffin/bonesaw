#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${eval_venv}/bin/maturin" develop --release

PYTHONPATH=python:python/evals "${eval_venv}/bin/python" \
  python/evals/upkie_state_local_wbc_report.py "$@"
