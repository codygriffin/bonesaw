#!/usr/bin/env bash
set -euo pipefail

comparison_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"

"$(dirname "$0")/fetch-unitree-g1-reference.sh" >/dev/null
if ! "${comparison_venv}/bin/python" -c 'import numpy' >/dev/null 2>&1; then
  python3 -m venv "${comparison_venv}"
  "${comparison_venv}/bin/pip" install maturin
fi
# Always rebuild the Rust extension. Importability alone does not prove that a
# cached venv contains the current WBC/IK implementation.
VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
  "${comparison_venv}/bin/maturin" develop --release
"${comparison_venv}/bin/python" python/evals/g1_oracle_wbc_admission.py "$@"
