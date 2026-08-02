#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python:python/evals /tmp/bonesaw-placo/bin/python \
  python/evals/cuda_point_query_mirror_report.py "$@"
