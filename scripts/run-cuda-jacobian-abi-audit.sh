#!/usr/bin/env bash
set -euo pipefail

comparison_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
output="${1:-benchmarks/results/cuda-jacobian-abi-r76}"

VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
  "${comparison_venv}/bin/maturin" develop --release
PYTHONPATH=python:python/evals "${comparison_venv}/bin/python" \
  python/evals/cuda_jacobian_mirror_report.py \
  --output "${output}" --web-report web/CUDA_JACOBIAN_ABI_R76.html
