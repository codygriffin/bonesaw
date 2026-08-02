#!/usr/bin/env bash
set -euo pipefail

BONESAW_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BONESAW_PLANT_VENV="${BONESAW_MUJOCO_VENV:-/tmp/bonesaw-mujoco}"

cd "$BONESAW_PROJECT_ROOT"
VIRTUAL_ENV="$BONESAW_PLANT_VENV" \
  "$BONESAW_PLANT_VENV/bin/maturin" develop --release
PYTHONPATH=python/evals "$BONESAW_PLANT_VENV/bin/python" \
  python/evals/upkie_zero_effort_realization_calibration.py "$@"
