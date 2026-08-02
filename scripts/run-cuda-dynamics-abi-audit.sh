#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python:python/evals /tmp/bonesaw-placo/bin/python \
  python/evals/cuda_dynamics_mirror_report.py "$@"
