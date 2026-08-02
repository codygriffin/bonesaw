#!/usr/bin/env bash
set -euo pipefail

VIRTUAL_ENV=/tmp/bonesaw-placo /tmp/bonesaw-placo/bin/python \
  python/evals/joint_viability_envelope_report.py "$@"
