#!/usr/bin/env bash
set -euo pipefail

plant_venv="${BONESAW_PLANT_VENV:-/tmp/bonesaw-mujoco}"
VIRTUAL_ENV="$plant_venv" "$plant_venv/bin/maturin" develop --release
PYTHONPATH=python/evals "$plant_venv/bin/python" \
  python/evals/upkie_viability_request_plant_ab.py \
  --planner-strategy multistep_budgeted \
  --confirmation-updates 2 "$@"
