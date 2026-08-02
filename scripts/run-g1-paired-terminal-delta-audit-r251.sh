#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_EVAL_VENV:-/tmp/bonesaw-mujoco}"
PYTHONPATH="python/evals${PYTHONPATH:+:$PYTHONPATH}" \
  "$eval_venv/bin/python" python/evals/g1_paired_terminal_delta_audit_r251.py "$@"
