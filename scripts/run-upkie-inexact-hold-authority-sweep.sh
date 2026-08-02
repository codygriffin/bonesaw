#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
plant_venv="${BONESAW_MUJOCO_VENV:-/tmp/bonesaw-mujoco}"

cd "$repo_root"
VIRTUAL_ENV="$plant_venv" "$plant_venv/bin/maturin" develop --release
PYTHONPATH=python/evals "$plant_venv/bin/python" \
  python/evals/upkie_inexact_hold_authority_sweep.py "$@"
