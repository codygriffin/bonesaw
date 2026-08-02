#!/usr/bin/env bash
set -euo pipefail

cargo test -p bonesaw-cuda --lib cuda_kinematics::tests::cuda_source_has_fixed_entry_no_allocation_and_no_atomics
cargo test -p bonesaw-cuda --lib cuda_fk_com::tests::device_solve_is_gated_then_matches_fixed_level_cpu_mirror
cargo test -p bonesaw-cuda --lib cpu_mirror_solve::tests
cargo build --release -p bonesaw-tools --bin bonesaw-cuda-solve-audit
PYTHONPATH=python:python/evals python3 python/evals/cuda_solve_report.py "$@"
