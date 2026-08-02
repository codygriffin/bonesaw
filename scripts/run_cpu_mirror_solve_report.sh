#!/usr/bin/env bash
set -euo pipefail

cargo test -p bonesaw-cuda --lib cpu_mirror_solve::tests
cargo build --release -p bonesaw-tools --bin bonesaw-cpu-mirror-solve-audit
PYTHONPATH=python:python/evals python3 python/evals/cpu_mirror_solve_report.py "$@"
