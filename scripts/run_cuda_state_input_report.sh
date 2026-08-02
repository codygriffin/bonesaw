#!/usr/bin/env bash
set -euo pipefail

cargo test -p bonesaw-cuda --lib cuda_state_input::tests
cargo build --release -p bonesaw-tools --bin bonesaw-cuda-state-input-audit
PYTHONPATH=python:python/evals python3 python/evals/cuda_state_input_report.py "$@"
