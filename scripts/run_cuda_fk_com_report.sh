#!/usr/bin/env bash
set -euo pipefail

cargo test -p bonesaw-cuda --lib cuda_fk_com::tests
cargo build --release -p bonesaw-tools --bin bonesaw-cuda-fk-com-audit
PYTHONPATH=python:python/evals python3 python/evals/cuda_fk_com_report.py "$@"
