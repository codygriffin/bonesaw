#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cache_dir="${BONESAW_G1_CACHE:-${repository_root}/benchmarks/cache/unitree-g1}"
revision="f3772ce54c56ef2d34c6aee8100bc768896c7d19"
mkdir -p "${cache_dir}"

urdf="${cache_dir}/g1_23dof_mode_10.urdf"
license="${cache_dir}/LICENSE.unitree_ros"
urdf_sha256="9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43"
license_sha256="84aac59fd3246e3ddc49d1387644e8fcf43b0def4f0b9fe687f372e90446df2d"

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

raw_root="https://raw.githubusercontent.com/unitreerobotics/unitree_ros/${revision}"
fetch \
  "${raw_root}/robots/g1_description/g1_23dof_mode_10.urdf" \
  "${urdf}" \
  "${urdf_sha256}"
fetch "${raw_root}/LICENSE" "${license}" "${license_sha256}"

printf '%s\n' "${urdf}"
