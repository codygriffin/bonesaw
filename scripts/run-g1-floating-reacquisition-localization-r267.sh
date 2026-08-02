#!/usr/bin/env bash
set -euo pipefail

eval_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
python_bin="${BONESAW_EVAL_PYTHON:-${eval_venv}/bin/python}"
maturin_bin="${eval_venv}/bin/maturin"
timing_cpu="${BONESAW_TIMING_CPU:-4}"
output_root="benchmarks/results"
corpus="python/evals/floating_walk_corpus.py"

VIRTUAL_ENV="${eval_venv}" PATH="${eval_venv}/bin:${PATH}" \
  "${maturin_bin}" develop --release

run_trace() {
  local ticks="$1"
  local output="$2"
  shift 2
  PYTHONPATH=python/evals "${python_bin}" "${corpus}" \
    --ticks "${ticks}" \
    --output "${output_root}/${output}" \
    "$@"
}

run_trace 600 floating-g1-transfer-r267-target-reacquisition-cap8 \
  --maximum-feasibility-projection-sweeps 8

for cap in 128 256 512; do
  run_trace 340 "floating-g1-r267-convergence-cap${cap}" \
    --maximum-feasibility-projection-sweeps "${cap}"
done
for cap in 768 1024; do
  run_trace 320 "floating-g1-r267-convergence-cap${cap}" \
    --maximum-feasibility-projection-sweeps "${cap}"
done

for iterations in 4 8 16; do
  run_trace 340 "floating-g1-r267-equality-repair-it${iterations}-cap8" \
    --maximum-feasibility-projection-sweeps 8 \
    --maximum-feasibility-iterations "${iterations}" \
    --repair-feasibility-equalities-before-inequalities
done
run_trace 340 floating-g1-r267-equality-repair-cap8 \
  --maximum-feasibility-projection-sweeps 8 \
  --repair-feasibility-equalities-before-inequalities

RUSTFLAGS='-C target-cpu=native' VIRTUAL_ENV="${eval_venv}" \
  PATH="${eval_venv}/bin:${PATH}" "${maturin_bin}" develop --release

env PYTHONPATH=python/evals taskset -c "${timing_cpu}" \
  "${python_bin}" "${corpus}" \
  --ticks 600 \
  --output "${output_root}/floating-g1-r267-equality-repair-native-it16-cap8" \
  --maximum-feasibility-projection-sweeps 8 \
  --maximum-feasibility-iterations 16 \
  --repair-feasibility-equalities-before-inequalities

PYTHONPATH=python/evals "${python_bin}" \
  python/evals/g1_floating_reacquisition_localization_r267.py
