#!/usr/bin/env bash
set -euo pipefail

VIRTUAL_ENV=/tmp/bonesaw-placo /tmp/bonesaw-placo/bin/python \
  python/evals/cpu_exact_batch_solve_report.py "$@"
