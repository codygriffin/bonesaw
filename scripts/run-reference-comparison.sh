#!/usr/bin/env bash
set -euo pipefail

comparison_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
comparison_output="benchmarks/results/reference-latest"
comparison_report_only=0
comparison_args=("$@")
for ((index = 0; index < ${#comparison_args[@]}; index++)); do
  if [[ "${comparison_args[index]}" == "--output" ]] &&
    ((index + 1 < ${#comparison_args[@]})); then
    comparison_output="${comparison_args[index + 1]}"
  fi
  if [[ "${comparison_args[index]}" == "--report-only" ]]; then
    comparison_report_only=1
  fi
done

"$(dirname "$0")/fetch-cmu-walk-reference.sh" >/dev/null

if ((comparison_report_only == 0)); then
  "$(dirname "$0")/run-upkie-controller-comparison.sh" \
    --ticks "${BONESAW_UPKIE_ORACLE_TICKS:-100000}" \
    --output "${comparison_output}"
fi

if ! "${comparison_venv}/bin/python" -c 'import bonesaw, numpy, placo, psutil' >/dev/null 2>&1; then
  python3 -m venv "${comparison_venv}"
  "${comparison_venv}/bin/pip" install maturin placo psutil
fi

VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
  "${comparison_venv}/bin/maturin" develop --release
if ((comparison_report_only == 0)) &&
  [[ "${BONESAW_REFERENCE_REUSE_G1:-0}" != "1" ]]; then
  "$(dirname "$0")/run-floating-walk-corpus.sh" \
    --ticks "${BONESAW_G1_LIFTOFF_TICKS:-260}" \
    --output benchmarks/results/floating-g1-liftoff-latest
  "$(dirname "$0")/run-floating-walk-corpus.sh" \
    --ticks "${BONESAW_G1_TRANSFER_TICKS:-600}" \
    --output benchmarks/results/floating-g1-transfer-latest
fi
if ((comparison_report_only == 0)); then
  "$(dirname "$0")/fetch-unitree-g1-reference.sh" >/dev/null
  "$(dirname "$0")/run-pinocchio-oracle.sh" \
    --model models/upkie/upkie.urdf \
    --samples "${BONESAW_PINOCCHIO_ORACLE_SAMPLES:-50}" \
    --output-json "${comparison_output}/pinocchio-upkie.json"
  "$(dirname "$0")/run-pinocchio-oracle.sh" \
    --model benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf \
    --samples "${BONESAW_PINOCCHIO_ORACLE_SAMPLES:-50}" \
    --output-json "${comparison_output}/pinocchio-g1.json"
fi
"${comparison_venv}/bin/python" python/evals/reference_comparison.py "$@"
