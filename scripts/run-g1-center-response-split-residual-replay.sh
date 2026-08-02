#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

python_env="${BONESAW_PYTHON_ENV:-/tmp/bonesaw-mujoco}"
PYTHONPATH=python:python/evals "$python_env/bin/python" \
  python/evals/g1_center_response_split_residual_replay.py "$@"
