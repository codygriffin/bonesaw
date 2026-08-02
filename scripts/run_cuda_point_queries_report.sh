#!/usr/bin/env bash
set -euo pipefail

cargo test -p bonesaw-cuda --lib cuda_fk_com::tests
cargo test -p bonesaw-cuda --lib cuda_kinematics::tests
cargo test -p bonesaw-cuda --lib cpu_mirror::tests::compiled_point_queries_match_f64_position_jacobian_and_jdot_v
cargo test -p bonesaw-cuda --lib cpu_mirror::tests::repeated_dynamics_stage_is_bitwise_identical_and_allocation_free
cargo build --release -p bonesaw-tools --bin bonesaw-cuda-point-queries-audit
PYTHONPATH=python:python/evals python3 python/evals/cuda_point_queries_report.py "$@"
