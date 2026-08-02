#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_MUJOCO_VENV:-/tmp/bonesaw-mujoco}"
python_bin="${BONESAW_EVAL_PYTHON:-${eval_venv}/bin/python}"
maturin_bin="${eval_venv}/bin/maturin"

VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${maturin_bin}" develop --release

PYTHONPATH=python/evals "${python_bin}" \
  python/evals/g1_residual_prototype_profile_r264.py

PYTHONPATH=python/evals "${python_bin}" -m unittest \
  python/evals/test_g1_residual_prototype_profile_r264.py
