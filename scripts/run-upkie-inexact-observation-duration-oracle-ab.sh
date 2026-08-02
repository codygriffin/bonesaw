#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv="${BONESAW_MUJOCO_VENV:-/tmp/bonesaw-mujoco}"

cd "$repo_root"
PYTHONPATH=python/evals "$venv/bin/python" \
  python/evals/upkie_inexact_observation_duration_oracle_ab.py "$@"
