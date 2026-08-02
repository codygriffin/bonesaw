#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python:python/evals python3 python/evals/live_flat_foot_authority_report.py "$@"
