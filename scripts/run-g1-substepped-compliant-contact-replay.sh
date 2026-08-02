#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${BONESAW_MUJOCO_PYTHON:-/tmp/bonesaw-mujoco/bin/python}"

cd "$repo_dir"
PYTHONPATH="python:python/evals${PYTHONPATH:+:$PYTHONPATH}" \
  "$python_bin" python/evals/g1_substepped_compliant_contact_replay.py "$@"
