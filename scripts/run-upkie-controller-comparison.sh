#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
revision="$(tr -d '[:space:]' < "${repo_root}/benchmarks/references/upkie-controller-revision.txt")"
upkie_source="${BONESAW_UPKIE_CONTROLLER_SOURCE:-/tmp/bonesaw-upkie-reference}"
bazelisk_home="${BONESAW_BAZELISK_HOME:-/tmp/bonesaw-bazelisk}"
bazel_cache="${BONESAW_BAZEL_CACHE:-/tmp/bonesaw-bazel-cache}"

if [[ ! -d "${upkie_source}/.git" ]]; then
  git clone https://github.com/upkie/upkie.git "${upkie_source}"
fi
if [[ "$(git -C "${upkie_source}" rev-parse HEAD)" != "${revision}" ]]; then
  git -C "${upkie_source}" fetch origin "${revision}"
  git -C "${upkie_source}" checkout --detach "${revision}"
fi

mkdir -p "${upkie_source}/bonesaw_oracle"
cp "${repo_root}/python/evals/upkie_wheel_balancer_oracle.cc" \
  "${upkie_source}/bonesaw_oracle/upkie_wheel_balancer_oracle.cc"
cp "${repo_root}/python/evals/upkie_wheel_balancer_oracle.BUILD" \
  "${upkie_source}/bonesaw_oracle/BUILD"

(cd "${upkie_source}" && \
  BAZELISK_HOME="${bazelisk_home}" \
  bazel --batch --output_user_root="${bazel_cache}" \
  build //bonesaw_oracle:upkie_wheel_balancer_oracle)
(cd "${repo_root}" && \
  cargo build --release -p bonesaw-tools --bin bonesaw-upkie-balance-worker)

cd "${repo_root}"
python3 python/evals/upkie_controller_comparison.py \
  --upkie-worker \
  "${upkie_source}/bazel-bin/bonesaw_oracle/upkie_wheel_balancer_oracle" \
  --upkie-source "${upkie_source}" \
  "$@"
