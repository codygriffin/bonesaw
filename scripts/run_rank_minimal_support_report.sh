#!/usr/bin/env bash
set -euo pipefail

source /tmp/bonesaw-placo/bin/activate
python python/evals/rank_minimal_support_report.py "$@"
