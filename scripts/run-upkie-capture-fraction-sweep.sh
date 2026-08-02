#!/usr/bin/env bash
set -euo pipefail

capture_venv="${BONESAW_MUJOCO_VENV:-/tmp/bonesaw-mujoco}"
if ! "${capture_venv}/bin/python" -c 'import bonesaw, mujoco, numpy' >/dev/null 2>&1; then
  echo "missing prepared Bonesaw MuJoCo environment at ${capture_venv}" >&2
  exit 1
fi

VIRTUAL_ENV="${capture_venv}" "${capture_venv}/bin/maturin" develop --release
"${capture_venv}/bin/python" python/evals/upkie_capture_fraction_sweep.py "$@"
