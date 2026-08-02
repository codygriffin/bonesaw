#!/usr/bin/env bash
set -euo pipefail

cargo test -p bonesaw-cuda --lib cuda_fk_com::tests
cargo test -p bonesaw-cuda --lib cuda_kinematics::tests
cargo build --release -p bonesaw-tools --bin bonesaw-cuda-kinematics-audit
PYTHONPATH=python:python/evals python3 python/evals/cuda_kinematics_report.py "$@"
