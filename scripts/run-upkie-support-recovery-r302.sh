#!/usr/bin/env bash
set -euo pipefail

plant_venv="${BONESAW_MUJOCO_VENV:-/tmp/bonesaw-mujoco}"
VIRTUAL_ENV="${plant_venv}" "${plant_venv}/bin/maturin" develop --release
PYTHONPATH="python/evals${PYTHONPATH:+:${PYTHONPATH}}" \
  "${plant_venv}/bin/python" python/evals/upkie_live_support_recovery_r302.py --screen "$@"
