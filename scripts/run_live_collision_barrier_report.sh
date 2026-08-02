#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python:python/evals python3 \
  python/evals/live_collision_barrier_report.py "$@"
