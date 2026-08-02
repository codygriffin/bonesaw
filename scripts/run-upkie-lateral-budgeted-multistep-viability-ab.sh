#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"

PYTHONPATH="python:python/evals:/tmp/bonesaw-mujoco/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}" \
  /tmp/bonesaw-mujoco/bin/python \
  python/evals/upkie_lateral_budgeted_multistep_viability_ab.py "$@"
