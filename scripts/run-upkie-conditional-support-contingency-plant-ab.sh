#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
plant_venv="${BONESAW_PLANT_VENV:-/tmp/bonesaw-mujoco}"
cd "$project_root"
RUSTFLAGS='-C target-cpu=native' VIRTUAL_ENV="$plant_venv" \
  "$plant_venv/bin/maturin" develop --release
PYTHONPATH=python/evals "$plant_venv/bin/python" \
  python/evals/upkie_conditional_support_contingency_plant_ab.py "$@"
