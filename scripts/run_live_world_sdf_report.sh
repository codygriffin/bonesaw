#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python:python/evals python3 \
  python/evals/live_world_sdf_report.py "$@"
