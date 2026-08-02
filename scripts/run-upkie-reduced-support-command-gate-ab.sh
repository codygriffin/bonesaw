#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
plant_venv="${BONESAW_PLANT_VENV:-/tmp/bonesaw-mujoco}"

cd "$project_root"
VIRTUAL_ENV="$plant_venv" "$plant_venv/bin/maturin" develop --release
PYTHONPATH=python/evals "$plant_venv/bin/python" \
  python/evals/upkie_contact_observed_controller_ab.py \
  --withhold-reduced-support \
  --output benchmarks/results/upkie-reduced-support-command-gate-ab-r141 \
  --web-report web/UPKIE_REDUCED_SUPPORT_COMMAND_GATE_AB_R141.html \
  "$@"
