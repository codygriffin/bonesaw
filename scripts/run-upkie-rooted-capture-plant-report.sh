#!/usr/bin/env bash
set -euo pipefail

capture_venv="${BONESAW_MUJOCO_VENV:-/tmp/bonesaw-mujoco}"
if ! "${capture_venv}/bin/python" -c 'import bonesaw, mujoco, numpy' >/dev/null 2>&1; then
  echo "missing prepared Bonesaw MuJoCo environment at ${capture_venv}" >&2
  exit 1
fi

VIRTUAL_ENV="${capture_venv}" "${capture_venv}/bin/maturin" develop --release
"${capture_venv}/bin/python" python/evals/upkie_mujoco_plant_report.py \
  --revision upkie-rooted-capture-plant-r130 \
  --balance-mode capture \
  --capture-velocity-fraction 0.2 \
  --duration 10 \
  --output benchmarks/results/upkie-rooted-capture-plant-r130 \
  --web-report web/UPKIE_ROOTED_CAPTURE_PLANT_R130.html \
  "$@"
