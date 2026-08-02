#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python:python/evals python3 \
  python/evals/live_observation_uncertainty_report.py "$@"
