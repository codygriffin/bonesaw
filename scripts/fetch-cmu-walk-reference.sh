#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cache_dir="${BONESAW_CMU_CACHE:-${repository_root}/benchmarks/cache/cmu-37}"
mkdir -p "${cache_dir}"

skeleton="${cache_dir}/37.asf"
motion="${cache_dir}/37_01.amc"
skeleton_sha256="b1c80e8095954a86da9e2685a314d6fae258c905850044c03c4387757b2449a6"
motion_sha256="6864d7de5a09605769a30261b27387f42cf925a165c8b548cfdb798d275d0678"

fetch() {
  local url="$1"
  local destination="$2"
  local expected_sha256="$3"
  if [[ -f "${destination}" ]] &&
    echo "${expected_sha256}  ${destination}" | sha256sum --check --status; then
    return
  fi
  local temporary="${destination}.part"
  curl --fail --silent --show-error --location "${url}" --output "${temporary}"
  echo "${expected_sha256}  ${temporary}" | sha256sum --check --status
  mv "${temporary}" "${destination}"
}

fetch "http://mocap.cs.cmu.edu/subjects/37/37.asf" "${skeleton}" "${skeleton_sha256}"
fetch "http://mocap.cs.cmu.edu/subjects/37/37_01.amc" "${motion}" "${motion_sha256}"

printf '%s\n' "${cache_dir}"
