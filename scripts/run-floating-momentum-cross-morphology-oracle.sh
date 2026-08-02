#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

python_env="${BONESAW_PINOCCHIO_ENV:-/tmp/bonesaw-pinocchio}"
if [[ ! -x "$python_env/bin/python" ]]; then
  echo "missing Pinocchio Python environment: $python_env" >&2
  exit 1
fi

PYTHONPATH=python:python/evals "$python_env/bin/python" \
  python/evals/floating_momentum_cross_morphology_oracle.py "$@"
