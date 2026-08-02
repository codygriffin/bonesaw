#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
model_dir="${repo_root}/models/upkie"
upkie_revision="94735fbe6137276a41de0ff4cc04d2e533fa9e33"
mkdir -p "${model_dir}"

mesh_paths=(
  "add-ons/handle.stl"
  "ankle_stator.stl"
  "knee_stator.stl"
  "mjbots/mj5208_rotor.stl"
  "mjbots/mj5208_stator.stl"
  "mjbots/qdd100_horn.stl"
  "mjbots/qdd100_rotor.stl"
  "mjbots/qdd100_stator.stl"
  "wheel_hub.stl"
  "wheel_tire/wheel_tire.stl"
)

curl -fsSL \
  "https://raw.githubusercontent.com/tasts-robots/upkie_description/${upkie_revision}/urdf/upkie.urdf" \
  -o "${model_dir}/upkie.urdf"
curl -fsSL \
  "https://raw.githubusercontent.com/tasts-robots/upkie_description/${upkie_revision}/LICENSE" \
  -o "${model_dir}/LICENSE"

for mesh_path in "${mesh_paths[@]}"; do
  destination="${model_dir}/meshes/${mesh_path}"
  mkdir -p "$(dirname "${destination}")"
  curl -fsSL \
    "https://raw.githubusercontent.com/tasts-robots/upkie_description/${upkie_revision}/meshes/${mesh_path}" \
    -o "${destination}"
done

(
  cd "${model_dir}"
  sha256sum upkie.urdf LICENSE "${mesh_paths[@]/#/meshes/}" > MESHES.sha256
)

echo "Fetched Upkie ${upkie_revision} URDF, license, and ${#mesh_paths[@]} meshes into ${model_dir}"
