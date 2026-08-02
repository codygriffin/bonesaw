#!/usr/bin/env bash
set -euo pipefail

comparison_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
control_root="/tmp/bonesaw-r72-control"
production_root="/tmp/bonesaw-r72-production"
control_binary="/tmp/bonesaw-eval-r72-control"
production_binary="/tmp/bonesaw-eval-r72-production"
output="${1:-benchmarks/results/g1-jacobi-column-slice-r72}"
extension_name="_bonesaw.cpython-312-x86_64-linux-gnu.so"

mkdir -p \
  "${control_root}/bonesaw" \
  "${production_root}/bonesaw" \
  "${output}/control" \
  "${output}/production" \
  "${output}/counter-ab" \
  "${output}/native-control" \
  "${output}/native-production"

build_variant() {
  local root="$1"
  shift
  VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
    "${comparison_venv}/bin/maturin" develop --release "$@"
  cp python/bonesaw/__init__.py "${root}/bonesaw/__init__.py"
  cp "python/bonesaw/${extension_name}" "${root}/bonesaw/${extension_name}"
}

build_variant "${control_root}" \
  --features bonesaw-core/jacobi-column-slice-control
build_variant "${production_root}"

common_args=(
  --reference-inputs benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz
  --loose-bound-control benchmarks/results/g1-multistep-oracle-r54/no-loose-control.npz
  --ik-minimum-iterations 2 --ik-posture-weight 0.001 --ik-center-of-mass-weight 0.5
  --jet-center-of-mass-weight 10 --jet-off-chain-regularization 1000
  --minimum-contact-transitions 8 --minimum-alternating-liftoffs 4
)
PYTHONPATH="${control_root}:python/evals" "${comparison_venv}/bin/python" \
  python/evals/g1_oracle_wbc_admission.py \
  --output "${output}/control" "${common_args[@]}"
PYTHONPATH="${production_root}:python/evals" "${comparison_venv}/bin/python" \
  python/evals/g1_oracle_wbc_admission.py \
  --output "${output}/production" "${common_args[@]}"

PYTHONPATH=python/evals "${comparison_venv}/bin/python" \
  python/evals/zero_task_row_compaction_ab.py \
  --control-pythonpath "${control_root}" \
  --candidate-pythonpath "${production_root}" \
  --repeats 5 \
  --output "${output}/counter-ab" \
  --web-report /tmp/JACOBI_COLUMN_SLICE_COUNTER_R72.html

cargo build --release -p bonesaw-tools --bin bonesaw-eval \
  --features bonesaw-core/jacobi-column-slice-control
cp target/release/bonesaw-eval "${control_binary}"
cargo build --release -p bonesaw-tools --bin bonesaw-eval
cp target/release/bonesaw-eval "${production_binary}"
python3 python/evals/native_wbc_counter.py \
  --binary "${control_binary}" --repeats 5 \
  --output "${output}/native-control" \
  --web-report /tmp/NATIVE_JACOBI_CONTROL_R72.html
python3 python/evals/native_wbc_counter.py \
  --binary "${production_binary}" --repeats 5 \
  --output "${output}/native-production" \
  --web-report /tmp/NATIVE_JACOBI_PRODUCTION_R72.html

PYTHONPATH=python/evals python3 python/evals/jacobi_column_slice_report.py \
  --control "${output}/control" \
  --production "${output}/production" \
  --counter "${output}/counter-ab" \
  --native-control "${output}/native-control" \
  --native-production "${output}/native-production" \
  --output "${output}"

# Restore the production extension and native binary after the A/B build.
build_variant "${production_root}"
cargo build --release -p bonesaw-tools --bin bonesaw-eval
