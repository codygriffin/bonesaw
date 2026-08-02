#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_root"

"${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}/bin/python" python/evals/g1_oracle_wbc_admission.py \
  --reference-inputs benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz \
  --output benchmarks/results/g1-multistep-oracle-r54-coupled-actuation \
  --loose-bound-control benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz \
  --ik-minimum-iterations 2 \
  --ik-posture-weight 0.001 \
  --ik-center-of-mass-weight 0.5 \
  --jet-center-of-mass-weight 10 \
  --jet-off-chain-regularization 1000 \
  --minimum-contact-transitions 8 \
  --minimum-alternating-liftoffs 4 \
  --coupled-actuation-pair left_ankle_pitch_joint right_ankle_pitch_joint \
  --coupled-pair-effort-limit-nm 14
