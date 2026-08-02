#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python/evals /tmp/bonesaw-mujoco/bin/python \
  python/evals/upkie_terminal_realization_calibration.py "$@"
