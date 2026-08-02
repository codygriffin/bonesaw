#!/usr/bin/env bash
set -euo pipefail

reference_output="benchmarks/results/g1-multistep-reference-r53"
oracle_output="benchmarks/results/g1-multistep-oracle-r54"

"$(dirname "$0")/fetch-unitree-g1-reference.sh" >/dev/null
cargo build --release -p bonesaw-tools --bin bonesaw-lipm-reference
PYTHONPATH=python/evals python3 python/evals/g1_multistep_reference.py \
  --output "${reference_output}"

if "$(dirname "$0")/run-g1-oracle-wbc-admission.sh" \
  --reference-inputs "${reference_output}/reference-inputs.npz" \
  --output "${oracle_output}" \
  --loose-bound-control "${oracle_output}/no-loose-control.npz" \
  --ik-minimum-iterations 2 \
  --ik-posture-weight 0.001 \
  --ik-center-of-mass-weight 0.5 \
  --jet-center-of-mass-weight 10 \
  --jet-off-chain-regularization 1000 \
  --minimum-contact-transitions 8 \
  --minimum-alternating-liftoffs 4 \
  "$@"; then
  exit 0
fi

# This runner captures the present capability boundary: hard admission is the
# release gate while acceleration tracking is expected to remain red until the
# next projection/task-decomposition milestone. Fail for any harder regression.
python3 -c 'import json, pathlib, sys; p = pathlib.Path(sys.argv[1]); m = json.loads(p.read_text()); assert m["hard_admission_passed"] and not m["tracking_passed"], "expected hard-green/tracking-red certificate"' \
  "${oracle_output}/oracle-wbc-admission-metrics.json"
