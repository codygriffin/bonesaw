#!/usr/bin/env bash
set -euo pipefail

authority_venv="${BONESAW_AUTHORITY_VENV:-/tmp/bonesaw-mujoco}"
if ! "${authority_venv}/bin/python" -c 'import bonesaw, numpy' >/dev/null 2>&1; then
  echo "missing prepared Bonesaw Python environment at ${authority_venv}" >&2
  exit 1
fi

"${authority_venv}/bin/python" python/evals/upkie_state_local_authority.py "$@"
