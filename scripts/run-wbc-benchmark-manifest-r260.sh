#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_EVAL_VENV:-/tmp/bonesaw-mujoco}"
PYTHONPATH="python/evals${PYTHONPATH:+:$PYTHONPATH}" \
  "$eval_venv/bin/python" python/evals/wbc_benchmark_manifest_r260.py "$@"
