#!/usr/bin/env bash
set -euo pipefail

plant_venv="${BONESAW_PLANT_VENV:-/tmp/bonesaw-mujoco}"
RUSTFLAGS="-C target-cpu=native" VIRTUAL_ENV="$plant_venv" \
  "$plant_venv/bin/maturin" develop --release
RUSTFLAGS="-C target-cpu=native" PYTHONPATH=python/evals \
  "$plant_venv/bin/python" \
  python/evals/upkie_cross_profile_feasibility_witness_ab.py "$@"
