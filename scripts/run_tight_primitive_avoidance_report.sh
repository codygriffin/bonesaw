#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python:python/evals python3 python/evals/tight_primitive_avoidance_report.py "$@"
