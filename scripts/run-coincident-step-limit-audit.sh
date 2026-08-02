#!/usr/bin/env bash
set -euo pipefail

comparison_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
control_root="/tmp/bonesaw-r67-control"
candidate_root="/tmp/bonesaw-r67-candidate"
output="${1:-benchmarks/results/g1-coincident-step-limit-r67}"
extension_name="_bonesaw.cpython-312-x86_64-linux-gnu.so"

mkdir -p "${control_root}/bonesaw" "${candidate_root}/bonesaw" "${output}/control" "${output}/candidate"

VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
  "${comparison_venv}/bin/maturin" develop --release
cp python/bonesaw/__init__.py "${control_root}/bonesaw/__init__.py"
cp "python/bonesaw/${extension_name}" "${control_root}/bonesaw/${extension_name}"

VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
  "${comparison_venv}/bin/maturin" develop --release \
  --features bonesaw-core/coincident-step-limit-freeze-experiment
cp python/bonesaw/__init__.py "${candidate_root}/bonesaw/__init__.py"
cp "python/bonesaw/${extension_name}" "${candidate_root}/bonesaw/${extension_name}"

common_args=(
  --reference-inputs benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz
  --loose-bound-control benchmarks/results/g1-multistep-oracle-r54/no-loose-control.npz
  --ik-minimum-iterations 2 --ik-posture-weight 0.001 --ik-center-of-mass-weight 0.5
  --jet-center-of-mass-weight 10 --jet-off-chain-regularization 1000
  --minimum-contact-transitions 8 --minimum-alternating-liftoffs 4
)
PYTHONPATH="${control_root}:python/evals" "${comparison_venv}/bin/python" \
  python/evals/g1_oracle_wbc_admission.py --output "${output}/control" "${common_args[@]}"
PYTHONPATH="${candidate_root}:python/evals" "${comparison_venv}/bin/python" \
  python/evals/g1_oracle_wbc_admission.py --output "${output}/candidate" "${common_args[@]}"
PYTHONPATH=python/evals "${comparison_venv}/bin/python" \
  python/evals/coincident_step_limit_audit.py \
  --control "${output}/control" --candidate "${output}/candidate" --output "${output}"

# Restore the ordinary development extension to the production r66 default.
VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
  "${comparison_venv}/bin/maturin" develop --release
