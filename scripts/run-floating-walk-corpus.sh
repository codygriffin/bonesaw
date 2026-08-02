#!/usr/bin/env bash
set -euo pipefail

comparison_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"

"$(dirname "$0")/fetch-cmu-walk-reference.sh" >/dev/null
"$(dirname "$0")/fetch-unitree-g1-reference.sh" >/dev/null

if ! "${comparison_venv}/bin/python" -c 'import bonesaw, numpy' >/dev/null 2>&1; then
  python3 -m venv "${comparison_venv}"
  "${comparison_venv}/bin/pip" install maturin
fi

VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
  "${comparison_venv}/bin/maturin" develop --release
"${comparison_venv}/bin/python" python/evals/floating_walk_corpus.py "$@"
