#!/usr/bin/env bash
set -euo pipefail

source /tmp/bonesaw-placo/bin/activate
PYTHONPATH=python:python/evals python python/evals/auto_rigid_patch_basis_report.py "$@"
