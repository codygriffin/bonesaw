#!/usr/bin/env bash
set -euo pipefail

comparison_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
source_name="${BONESAW_WPG_SOURCE:-g1-transfer-r39-retiming-release100-guard020-800}"
generator_output="${BONESAW_WPG_OUTPUT:-benchmarks/results/g1-placo-wpg-r41}"
contract_output="${BONESAW_WPG_CONTRACT_OUTPUT:-benchmarks/results/g1-reference-contract-r41}"

"$(dirname "$0")/fetch-unitree-g1-reference.sh" >/dev/null

if [[ ! -f "benchmarks/results/${source_name}/floating-walk-raw.npz" ]]; then
  echo "missing authored source artifact: benchmarks/results/${source_name}" >&2
  exit 2
fi

if ! "${comparison_venv}/bin/python" -c 'import numpy, placo' >/dev/null 2>&1; then
  python3 -m venv "${comparison_venv}"
  "${comparison_venv}/bin/pip" install 'numpy>=1.26' 'placo==0.9.23'
fi

"${comparison_venv}/bin/python" python/evals/placo_wpg_reference.py \
  --source "${source_name}" \
  --output "${generator_output}"
python3 python/evals/g1_reference_contract.py --output "${contract_output}"
