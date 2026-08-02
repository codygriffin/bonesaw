#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python:python/evals python3 python/evals/live_command_authority_report.py "$@"
