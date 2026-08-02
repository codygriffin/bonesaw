#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_EVAL_VENV:-/tmp/bonesaw-mujoco}"
VIRTUAL_ENV="$eval_venv" "$eval_venv/bin/maturin" develop --release
"$eval_venv/bin/python" python/evals/g1_rk4_stage_force_cross_integrator_holdout.py "$@"
