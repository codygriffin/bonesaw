#!/usr/bin/env bash
set -euo pipefail

source /tmp/bonesaw-placo/bin/activate
python python/evals/finite_support_patch_report.py "$@"
