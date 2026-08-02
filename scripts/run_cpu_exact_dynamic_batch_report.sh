#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python/evals /tmp/bonesaw-placo/bin/python python/evals/cpu_exact_dynamic_batch_report.py "$@"
