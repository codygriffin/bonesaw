#!/usr/bin/env bash
set -euo pipefail

oracle_venv="${BONESAW_ORACLE_VENV:-/tmp/bonesaw-pinocchio}"
if ! "${oracle_venv}/bin/python" -c 'import pinocchio, numpy' >/dev/null 2>&1; then
  python3 -m venv "${oracle_venv}"
  "${oracle_venv}/bin/pip" install pin==4.0.0
fi

"${oracle_venv}/bin/python" python/evals/g1_pinocchio_fixed_effort_reference.py "$@"
