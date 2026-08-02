#!/usr/bin/env bash
set -euo pipefail

comparison_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
control_root="/tmp/bonesaw-r74-control"
production_root="/tmp/bonesaw-r74-production"
control_binary="/tmp/bonesaw-eval-r74-control"
production_binary="/tmp/bonesaw-eval-r74-production"
iterator_control_binary="/tmp/bonesaw-eval-r73-control"
iterator_candidate_binary="/tmp/bonesaw-eval-r73-candidate"
output="${1:-benchmarks/results/g1-dense-multiply-row-slice-r74}"
extension_name="_bonesaw.cpython-312-x86_64-linux-gnu.so"
model="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"

mkdir -p \
  "${control_root}/bonesaw" \
  "${production_root}/bonesaw" \
  "${output}/control" \
  "${output}/candidate" \
  "${output}/production" \
  "${output}/counter-ab" \
  "${output}/native-control" \
  "${output}/native-candidate" \
  "${output}/iterator-negative"

build_extension_variant() {
  local root="$1"
  shift
  VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
    "${comparison_venv}/bin/maturin" develop --release "$@"
  cp python/bonesaw/__init__.py "${root}/bonesaw/__init__.py"
  cp "python/bonesaw/${extension_name}" "${root}/bonesaw/${extension_name}"
}

common_args=(
  --reference-inputs benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz
  --loose-bound-control benchmarks/results/g1-multistep-oracle-r54/no-loose-control.npz
  --ik-minimum-iterations 2 --ik-posture-weight 0.001 --ik-center-of-mass-weight 0.5
  --jet-center-of-mass-weight 10 --jet-off-chain-regularization 1000
  --minimum-contact-transitions 8 --minimum-alternating-liftoffs 4
)

build_extension_variant "${control_root}" \
  --features bonesaw-core/dense-multiply-row-slice-control
build_extension_variant "${production_root}"

PYTHONPATH="${control_root}:python/evals" "${comparison_venv}/bin/python" \
  python/evals/g1_oracle_wbc_admission.py \
  --output "${output}/control" "${common_args[@]}"
PYTHONPATH="${production_root}:python/evals" "${comparison_venv}/bin/python" \
  python/evals/g1_oracle_wbc_admission.py \
  --output "${output}/candidate" "${common_args[@]}"
cp "${output}/candidate/oracle-wbc-admission-metrics.json" \
  "${output}/production/oracle-wbc-admission-metrics.json"
cp "${output}/candidate/oracle-wbc-admission-raw.npz" \
  "${output}/production/oracle-wbc-admission-raw.npz"
cp "${output}/candidate/ORACLE_WBC_ADMISSION.md" \
  "${output}/production/ORACLE_WBC_ADMISSION.md"

PYTHONPATH=python/evals "${comparison_venv}/bin/python" \
  python/evals/zero_task_row_compaction_ab.py \
  --control-pythonpath "${control_root}" \
  --candidate-pythonpath "${production_root}" \
  --repeats 5 \
  --output "${output}/counter-ab" \
  --web-report /tmp/DENSE_MULTIPLY_ROW_SLICE_COUNTER_R74.html

cargo build --release -p bonesaw-tools --bin bonesaw-eval \
  --features bonesaw-core/dense-multiply-row-slice-control
cp target/release/bonesaw-eval "${control_binary}"
cargo build --release -p bonesaw-tools --bin bonesaw-eval
cp target/release/bonesaw-eval "${production_binary}"
python3 python/evals/native_wbc_counter.py \
  --binary "${control_binary}" --repeats 5 \
  --output "${output}/native-control" \
  --web-report /tmp/NATIVE_DENSE_MULTIPLY_CONTROL_R74.html
python3 python/evals/native_wbc_counter.py \
  --binary "${production_binary}" --repeats 5 \
  --output "${output}/native-candidate" \
  --web-report /tmp/NATIVE_DENSE_MULTIPLY_PRODUCTION_R74.html

# Reproduce the r73 noise-floor negative with r74 explicitly disabled in both
# builds, so only Jacobi slice traversal differs.
cargo build --release -p bonesaw-tools --bin bonesaw-eval \
  --features bonesaw-core/dense-multiply-row-slice-control
cp target/release/bonesaw-eval "${iterator_control_binary}"
cargo build --release -p bonesaw-tools --bin bonesaw-eval \
  --features bonesaw-core/dense-multiply-row-slice-control,bonesaw-core/jacobi-column-iterator-experiment
cp target/release/bonesaw-eval "${iterator_candidate_binary}"
"${iterator_control_binary}" --model "${model}" --ticks 100 \
  --floating-wbc-only --json >"${output}/iterator-negative/control.json"
"${iterator_candidate_binary}" --model "${model}" --ticks 100 \
  --floating-wbc-only --json >"${output}/iterator-negative/candidate.json"
for repeat in 1 2 3 4 5; do
  if (( repeat % 2 )); then
    order=(control candidate)
  else
    order=(candidate control)
  fi
  for variant in "${order[@]}"; do
    binary="${iterator_control_binary}"
    if [[ "${variant}" == "candidate" ]]; then
      binary="${iterator_candidate_binary}"
    fi
    perf stat -x, -o "${output}/iterator-negative/${repeat}-${variant}.csv" \
      -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
      taskset -c 4 "${binary}" --model "${model}" --ticks 2000 \
      --floating-wbc-only --json >/dev/null
  done
done

PYTHONPATH=python/evals python3 python/evals/dense_multiply_row_slice_report.py \
  --output "${output}"

# Restore the production extension and native binary after every A/B build.
build_extension_variant "${production_root}"
cargo build --release -p bonesaw-tools --bin bonesaw-eval
