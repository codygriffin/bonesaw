#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_EVAL_VENV:-/tmp/bonesaw-resource-eval}"

if ! "${eval_venv}/bin/python" -c 'import bonesaw, numpy' >/dev/null 2>&1; then
  python3 -m venv "${eval_venv}"
  "${eval_venv}/bin/pip" install maturin
fi
VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${eval_venv}/bin/maturin" develop --release
"${eval_venv}/bin/python" python/evals/actuator_resource_authority.py "$@"
