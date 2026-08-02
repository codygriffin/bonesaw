#!/usr/bin/env bash
set -euo pipefail

envelope_venv="${BONESAW_MUJOCO_VENV:-/tmp/bonesaw-mujoco}"
if ! "${envelope_venv}/bin/python" -c 'import bonesaw, mujoco, numpy' >/dev/null 2>&1; then
  echo "missing prepared Bonesaw MuJoCo environment at ${envelope_venv}" >&2
  exit 1
fi

"${envelope_venv}/bin/python" python/evals/upkie_disturbance_envelope.py "$@"
