#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

python_env="${BONESAW_PYTHON_ENV:-/tmp/bonesaw-mujoco}"
if [[ ! -x "$python_env/bin/python" ]]; then
  echo "missing Python environment: $python_env" >&2
  exit 1
fi

source "$python_env/bin/activate"
PYTHONPATH=python/evals python python/evals/upkie_contact_response_localization_audit.py "$@"
