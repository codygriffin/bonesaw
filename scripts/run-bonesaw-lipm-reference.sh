#!/usr/bin/env bash
set -euo pipefail

cargo build --release -p bonesaw-tools --bin bonesaw-lipm-reference
python3 python/evals/bonesaw_lipm_reference.py "$@"
python3 python/evals/g1_reference_contract.py \
  --output benchmarks/results/g1-reference-contract-r44
