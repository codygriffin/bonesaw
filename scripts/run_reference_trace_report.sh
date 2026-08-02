#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python:python/evals python3 python/evals/reference_trace_report.py "$@"
