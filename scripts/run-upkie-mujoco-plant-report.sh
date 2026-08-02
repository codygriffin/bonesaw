#!/usr/bin/env bash
set -euo pipefail

plant_venv="${BONESAW_MUJOCO_VENV:-/tmp/bonesaw-mujoco}"
if ! "${plant_venv}/bin/python" -c 'import mujoco, maturin, numpy' >/dev/null 2>&1; then
  python3 -m venv "${plant_venv}"
  "${plant_venv}/bin/pip" install mujoco==3.3.7 maturin==1.9.4
fi

VIRTUAL_ENV="${plant_venv}" "${plant_venv}/bin/maturin" develop --release
"${plant_venv}/bin/python" python/evals/upkie_mujoco_plant_report.py "$@"
