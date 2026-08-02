#!/usr/bin/env bash
set -euo pipefail

cargo test -p bonesaw-cuda --lib cuda_fk_com::tests
cargo test -p bonesaw-cuda --lib cuda_kinematics::tests
cargo test -p bonesaw-cuda --lib cpu_mirror::tests::cpu_mirror_dynamics_products_match_f64_core_and_physical_invariants
cargo test -p bonesaw-cuda --lib cpu_mirror::tests::repeated_dynamics_stage_is_bitwise_identical_and_allocation_free
cargo build --release -p bonesaw-tools --bin bonesaw-cuda-dynamics-audit
PYTHONPATH=python:python/evals python3 python/evals/cuda_dynamics_report.py "$@"
