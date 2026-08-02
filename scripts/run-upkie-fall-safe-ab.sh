#!/usr/bin/env bash
set -euo pipefail

plant_venv="${BONESAW_PLANT_VENV:-/tmp/bonesaw-mujoco}"
if ! "${plant_venv}/bin/python" -c 'import bonesaw, mujoco, numpy' >/dev/null 2>&1; then
  echo "missing prepared Bonesaw MuJoCo environment at ${plant_venv}" >&2
  exit 1
fi

"${plant_venv}/bin/python" python/evals/upkie_fall_safe_ab.py "$@"
