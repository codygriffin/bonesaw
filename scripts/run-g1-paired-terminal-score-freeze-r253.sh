#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_EVAL_VENV:-/tmp/bonesaw-mujoco}"
VIRTUAL_ENV="$eval_venv" "$eval_venv/bin/maturin" develop --release
PYTHONPATH="python/evals${PYTHONPATH:+:$PYTHONPATH}" \
  "$eval_venv/bin/python" python/evals/g1_paired_terminal_score_freeze_r253.py "$@"
