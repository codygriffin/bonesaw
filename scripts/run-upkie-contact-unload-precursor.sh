#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
plant_venv="${BONESAW_PLANT_VENV:-/tmp/bonesaw-mujoco}"

cd "$project_root"
VIRTUAL_ENV="$plant_venv" "$plant_venv/bin/maturin" develop --release
PYTHONPATH=python/evals "$plant_venv/bin/python" \
  python/evals/upkie_contact_unload_precursor.py "$@"
