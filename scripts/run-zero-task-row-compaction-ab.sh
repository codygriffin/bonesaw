#!/usr/bin/env bash
set -euo pipefail

comparison_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
control_root="/tmp/bonesaw-r65-control"
candidate_root="/tmp/bonesaw-r65-zero"
extension_name="_bonesaw.cpython-312-x86_64-linux-gnu.so"

mkdir -p "${control_root}/bonesaw" "${candidate_root}/bonesaw"

VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
  "${comparison_venv}/bin/maturin" develop --release \
  --features bonesaw-core/resolved-task-row-compaction-control
cp python/bonesaw/__init__.py "${control_root}/bonesaw/__init__.py"
cp "python/bonesaw/${extension_name}" "${control_root}/bonesaw/${extension_name}"

VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
  "${comparison_venv}/bin/maturin" develop --release \
  --features bonesaw-core/resolved-task-row-compaction-control,bonesaw-core/zero-task-row-compaction-experiment
cp python/bonesaw/__init__.py "${candidate_root}/bonesaw/__init__.py"
cp "python/bonesaw/${extension_name}" "${candidate_root}/bonesaw/${extension_name}"

PYTHONPATH=python/evals "${comparison_venv}/bin/python" \
  python/evals/zero_task_row_compaction_ab.py \
  --control-pythonpath "${control_root}" \
  --candidate-pythonpath "${candidate_root}" \
  "$@"

# Leave the ordinary development environment on the production control build.
VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
  "${comparison_venv}/bin/maturin" develop --release
