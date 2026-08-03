#!/usr/bin/env bash
set -euo pipefail

python_bin="${BONESAW_EVAL_PYTHON:-/tmp/bonesaw-placo/bin/python}"
PYTHONPATH=python/evals "${python_bin}" python/evals/g1_support_reachable_tube_r280.py --check-only
PYTHONPATH=python/evals "${python_bin}" -m unittest python/evals/test_g1_support_reachable_tube_r280.py
