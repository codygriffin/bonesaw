#!/usr/bin/env bash
set -euo pipefail

"$(dirname "$0")/run-floating-walk-corpus.sh" \
  --reference-inputs benchmarks/results/g1-bonesaw-lipm-r43/reference-inputs.npz \
  --ticks 600 \
  --center-of-mass-task-weight 1.0 \
  --centroidal-angular-momentum-weight 1.0 \
  --centroidal-angular-momentum-priority 1 \
  --output benchmarks/results/g1-bonesaw-lipm-wbc-r43 \
  "$@"
