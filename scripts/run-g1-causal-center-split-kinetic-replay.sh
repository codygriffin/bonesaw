#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
python_env="${BONESAW_PYTHON_ENV:-/tmp/bonesaw-mujoco}"
if [[ ! -x "$python_env/bin/python" ]]; then
  echo "missing evaluation Python environment: $python_env" >&2
  exit 1
fi
PYTHONPATH=python:python/evals "$python_env/bin/python" \
  python/evals/g1_causal_center_split_kinetic_replay.py "$@"
