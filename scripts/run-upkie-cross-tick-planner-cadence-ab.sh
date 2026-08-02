#!/usr/bin/env bash
set -euo pipefail

plant_venv="${BONESAW_PLANT_VENV:-/tmp/bonesaw-mujoco}"
export RUSTFLAGS="${BONESAW_TIMING_RUSTFLAGS:--C target-cpu=native}"
VIRTUAL_ENV="$plant_venv" "$plant_venv/bin/maturin" develop --release
PYTHONPATH=python/evals "$plant_venv/bin/python" \
  python/evals/upkie_cross_tick_planner_cadence_ab.py "$@"
