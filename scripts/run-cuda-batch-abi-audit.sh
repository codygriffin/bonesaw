#!/usr/bin/env bash
set -euo pipefail

comparison_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
output="${1:-benchmarks/results/cuda-batch-abi-r75}"

VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
  "${comparison_venv}/bin/maturin" develop --release
PYTHONPATH=python:python/evals "${comparison_venv}/bin/python" \
  python/evals/cuda_batch_mirror_report.py \
  --output "${output}" --web-report web/CUDA_BATCH_ABI_R75.html
