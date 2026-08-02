#!/usr/bin/env bash
set -euo pipefail

cargo test -p bonesaw-cuda --lib cuda_fk_com::tests
cargo test -p bonesaw-cuda --lib cuda_kinematics::tests
cargo test -p bonesaw-cuda --lib cpu_mirror::tests::fixed_point_tasks_and_contact_locks_emit_stable_masked_rows_without_allocation
cargo build --release -p bonesaw-tools --bin bonesaw-cuda-emission-audit
PYTHONPATH=python:python/evals python3 python/evals/cuda_emission_report.py "$@"
