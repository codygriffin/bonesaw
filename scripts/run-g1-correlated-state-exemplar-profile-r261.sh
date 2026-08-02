#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_EVAL_VENV:-/tmp/bonesaw-mujoco}"
PYTHONPATH="python/evals${PYTHONPATH:+:$PYTHONPATH}" \
  "$eval_venv/bin/python" python/evals/g1_correlated_state_exemplar_profile_r261.py "$@"
