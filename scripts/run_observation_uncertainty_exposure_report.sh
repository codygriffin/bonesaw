#!/usr/bin/env bash
set -euo pipefail

cargo build --release -p bonesaw-tools --bin bonesaw-observation-uncertainty-audit
PYTHONPATH=python:python/evals python3 \
  python/evals/observation_uncertainty_exposure_report.py "$@"
