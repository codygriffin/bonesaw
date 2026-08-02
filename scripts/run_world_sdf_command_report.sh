#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${eval_venv}/bin/maturin" develop --release
cargo test -p bonesaw-core world_collision
cargo test -p bonesaw-core dynamic_admission_withholds_world_colliding_primary_and_admits_clear_brake
PYTHONPATH=python:python/evals "${eval_venv}/bin/python" \
  python/evals/world_sdf_command_report.py "$@"
