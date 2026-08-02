#!/usr/bin/env bash
set -euo pipefail

scripts/run-floating-walk-corpus.sh \
  --motion-profile synthetic-step \
  --ticks 160 \
  --synthetic-step-length 0.01 \
  --synthetic-step-clearance 0.01 \
  --synthetic-swing-ticks 40 \
  --precontact-ticks 40 \
  --precontact-maximum-acceleration 25 \
  --output benchmarks/results/g1-synthetic-step-latest \
  "$@"
