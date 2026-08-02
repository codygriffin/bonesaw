#!/usr/bin/env bash
set -euo pipefail

comparison_venv="${BONESAW_REFERENCE_VENV:-/tmp/bonesaw-placo}"
pinocchio_venv="${BONESAW_ORACLE_VENV:-/tmp/bonesaw-pinocchio}"
control_root="/tmp/bonesaw-r71-control"
row_gram_root="/tmp/bonesaw-r70-row-gram"
rank_one_root="/tmp/bonesaw-r71-rank-one"
output="${1:-benchmarks/results/g1-projected-factorization-r71}"
row_gram_output="${output}/row-gram"
rank_one_output="${output}/rank-one"
extension_name="_bonesaw.cpython-312-x86_64-linux-gnu.so"

mkdir -p \
  "${control_root}/bonesaw" \
  "${row_gram_root}/bonesaw" \
  "${rank_one_root}/bonesaw" \
  "${output}/control" \
  "${row_gram_output}/candidate" \
  "${row_gram_output}/realization" \
  "${row_gram_output}/pinocchio" \
  "${rank_one_output}/candidate" \
  "${rank_one_output}/counter-ab"

build_variant() {
  local root="$1"
  shift
  VIRTUAL_ENV="${comparison_venv}" PATH="${comparison_venv}/bin:${PATH}" \
    "${comparison_venv}/bin/maturin" develop --release "$@"
  cp python/bonesaw/__init__.py "${root}/bonesaw/__init__.py"
  cp "python/bonesaw/${extension_name}" "${root}/bonesaw/${extension_name}"
}

build_variant "${control_root}"
build_variant "${row_gram_root}" \
  --features bonesaw-core/row-gram-task-pseudoinverse-experiment
build_variant "${rank_one_root}" \
  --features bonesaw-core/rank-one-task-pseudoinverse-experiment

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
PYTHONPATH="${row_gram_root}:python/evals" "${comparison_venv}/bin/python" \
  python/evals/g1_oracle_wbc_admission.py \
  --output "${row_gram_output}/candidate" "${common_args[@]}"
PYTHONPATH="${rank_one_root}:python/evals" "${comparison_venv}/bin/python" \
  python/evals/g1_oracle_wbc_admission.py \
  --output "${rank_one_output}/candidate" "${common_args[@]}"

PYTHONPATH="${row_gram_root}:python/evals" "${comparison_venv}/bin/python" \
  python/evals/g1_constrained_acceleration_realization.py \
  --output "${row_gram_output}/realization" \
  --web-report /tmp/G1_CONSTRAINED_ACCELERATION_ROW_GRAM_R70.html
PYTHONPATH=python/evals "${pinocchio_venv}/bin/python" \
  python/evals/g1_pinocchio_fixed_effort_reference.py \
  --realization "${row_gram_output}/realization" \
  --output "${row_gram_output}/pinocchio" \
  --web-report /tmp/G1_PINOCCHIO_ROW_GRAM_R70.html

PYTHONPATH=python/evals "${comparison_venv}/bin/python" \
  python/evals/zero_task_row_compaction_ab.py \
  --control-pythonpath "${control_root}" \
  --candidate-pythonpath "${rank_one_root}" \
  --repeats 5 \
  --output "${rank_one_output}/counter-ab" \
  --web-report /tmp/RANK_ONE_PSEUDOINVERSE_COUNTER_R71.html

PYTHONPATH=python/evals "${comparison_venv}/bin/python" \
  python/evals/projected_factorization_audit.py \
  --control "${output}/control" \
  --row-gram "${row_gram_output}/candidate" \
  --row-gram-realization "${row_gram_output}/realization" \
  --row-gram-pinocchio "${row_gram_output}/pinocchio" \
  --rank-one "${rank_one_output}/candidate" \
  --rank-one-counter "${rank_one_output}/counter-ab" \
  --output "${output}"

# Restore the ordinary development extension to the production r66 default.
build_variant "${control_root}"
