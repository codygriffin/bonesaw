#!/usr/bin/env bash
set -euo pipefail

BONESAW_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BONESAW_MUJOCO_VENV="${BONESAW_MUJOCO_VENV:-/tmp/bonesaw-mujoco}"
BONESAW_LIVE_URL="${BONESAW_LIVE_URL:-http://127.0.0.1:8777}"

if ! "${BONESAW_MUJOCO_VENV}/bin/python" -c 'import bonesaw, mujoco, numpy' >/dev/null 2>&1; then
  echo "missing prepared Bonesaw MuJoCo environment at ${BONESAW_MUJOCO_VENV}" >&2
  exit 1
fi

cd "$BONESAW_PROJECT_ROOT"
"${BONESAW_MUJOCO_VENV}/bin/python" python/evals/live_plant_gateway_report.py \
  --url "$BONESAW_LIVE_URL" \
  "$@"
