#!/usr/bin/env bash
set -euo pipefail

# Reproduce the policy-/integration-/physics-free R319 exact RollingWheel
# comparison without polluting the project's runtime environment.  PlaCo
# 0.9.23 intentionally brings Pinocchio 3.8.0; the separate rigid-body oracle
# remains pinned to Pinocchio 4.0.0 in run-pinocchio-oracle.sh.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
oracle_venv="${BONESAW_R319_VENV:-/tmp/bonesaw-r319}"

if ! "${oracle_venv}/bin/python" -c 'import pinocchio, placo, psutil, numpy' >/dev/null 2>&1; then
  python3 -m venv "${oracle_venv}"
  "${oracle_venv}/bin/pip" install "placo==0.9.23" "psutil==7.2.2"
fi

# The evaluator imports the compiled PyO3 package through the repository's
# python/ tree. Build it only when the current checkout has not already been
# developed into this interpreter.
if ! PYTHONPATH="${repo_root}/python:${repo_root}/python/evals" \
  "${oracle_venv}/bin/python" -c 'import bonesaw' >/dev/null 2>&1; then
  maturin develop --release \
    --manifest-path "${repo_root}/crates/bonesaw-py/Cargo.toml" \
    --interpreter "${oracle_venv}/bin/python"
fi

cd "${repo_root}"
PYTHONPATH="${repo_root}/python:${repo_root}/python/evals" \
  "${oracle_venv}/bin/python" \
  "${repo_root}/python/evals/upkie_pinocchio_rolling_reference_r319.py" "$@"
