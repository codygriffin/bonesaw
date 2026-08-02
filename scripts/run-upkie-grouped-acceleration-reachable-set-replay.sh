#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

python_env="${BONESAW_MUJOCO_ENV:-/tmp/bonesaw-mujoco}"
if [[ ! -x "$python_env/bin/python" ]]; then
  echo "missing MuJoCo Python environment: $python_env" >&2
  exit 1
fi

PYTHONPATH=python:python/evals "$python_env/bin/python" \
  python/evals/upkie_grouped_acceleration_reachable_set_replay.py "$@"
