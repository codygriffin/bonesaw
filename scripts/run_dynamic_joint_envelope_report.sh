#!/usr/bin/env bash
set -euo pipefail

PYTHONPATH=python/evals /tmp/bonesaw-placo/bin/python python/evals/dynamic_joint_envelope_composition_report.py "$@"
