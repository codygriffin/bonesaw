#!/usr/bin/env bash
set -euo pipefail

cargo build --release -p bonesaw-tools --bin bonesaw-eval
python3 python/evals/native_wbc_counter.py "$@"
