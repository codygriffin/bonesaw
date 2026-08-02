#!/usr/bin/env bash
set -euo pipefail

python3 python/evals/cpu_counter_stability.py "$@"
