#!/usr/bin/env python3
"""Differential oracle for Bonesaw's fixed-base CPU model.

Requires Pinocchio 4.0.0 (`pip install pin==4.0.0`) and NumPy. The Rust fixture
binary is intentionally a separate process so the comparison also exercises
Bonesaw's public compiled-model path.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument(
        "--fixture-bin", default="target/release/bonesaw-oracle-fixture"
    )
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--json", action="store_true")
    parser.add_argument(
        "--output-json",
        help="write the complete machine-readable oracle result to this path",
    )
    parser.add_argument("--no-strict", action="store_true")
    return parser.parse_args()


def maximum_absolute(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.max(np.abs(a - b))) if a.size else 0.0


def main() -> int:
    args = parse_args()
    if args.samples <= 0:
        raise ValueError("--samples must be positive")
    model_path = Path(args.model).resolve()
    fixture_binary = Path(args.fixture_bin).resolve()
    fixture = json.loads(
        subprocess.check_output(
            [
                str(fixture_binary),
                "--model",
                str(model_path),
                "--samples",
                str(args.samples),
            ],
            text=True,
        )
    )

    model = pin.buildModelFromUrdf(str(model_path))
    data = model.createData()
    model.gravity.linear = np.asarray(fixture["gravity_world"], dtype=float)
    floating_model = pin.buildModelFromUrdf(str(model_path), pin.JointModelFreeFlyer())
    floating_data = floating_model.createData()
    floating_model.gravity.linear = np.asarray(fixture["gravity_world"], dtype=float)
    coordinate_names = fixture["coordinate_names"]
    joint_ids = [model.getJointId(name) for name in coordinate_names]
    if any(joint_id == 0 for joint_id in joint_ids):
        missing = [
            name
            for name, joint_id in zip(coordinate_names, joint_ids, strict=True)
            if joint_id == 0
        ]
        raise RuntimeError(f"Pinocchio is missing Bonesaw joints: {missing}")
    q_indices = [model.joints[joint_id].idx_q for joint_id in joint_ids]
    v_indices = [model.joints[joint_id].idx_v for joint_id in joint_ids]
    if any(model.joints[joint_id].nq != 1 or model.joints[joint_id].nv != 1 for joint_id in joint_ids):
        raise RuntimeError("this oracle currently requires scalar URDF joints")
    floating_joint_ids = [
        floating_model.getJointId(name) for name in coordinate_names
    ]
    floating_q_indices = [
        floating_model.joints[joint_id].idx_q for joint_id in floating_joint_ids
    ]
    floating_v_indices = [
        floating_model.joints[joint_id].idx_v for joint_id in floating_joint_ids
    ]
    # Pinocchio free-flyer tangent is [linear; angular]; Bonesaw is
    # [angular; linear].
    floating_permutation = [3, 4, 5, 0, 1, 2, *floating_v_indices]

    maxima = {
        "frame_translation_m": 0.0,
        "frame_rotation_matrix": 0.0,
        "frame_jacobian": 0.0,
        "center_of_mass": 0.0,
        "mass_matrix": 0.0,
        "generalized_gravity": 0.0,
        "inverse_dynamics": 0.0,
        "centroidal_map": 0.0,
        "centroidal_momentum": 0.0,
        "floating_mass_matrix": 0.0,
        "floating_bias": 0.0,
        "floating_inverse_dynamics": 0.0,
        "floating_centroidal_map": 0.0,
    }
    compared_frames = 0
    for sample in fixture["samples"]:
        q = pin.neutral(model)
        v = np.zeros(model.nv)
        acceleration = np.zeros(model.nv)
        for bonesaw_index, (q_index, v_index) in enumerate(
            zip(q_indices, v_indices, strict=True)
        ):
            q[q_index] = sample["q"][bonesaw_index]
            v[v_index] = sample["v"][bonesaw_index]
            acceleration[v_index] = sample["acceleration"][bonesaw_index]

        floating_q = pin.neutral(floating_model)
        floating_v = np.zeros(floating_model.nv)
        floating_acceleration = np.zeros(floating_model.nv)
        root_twist = np.asarray(sample["root_twist_angular_linear"], dtype=float)
        root_acceleration = np.asarray(
            sample["floating_acceleration_angular_linear_joint"], dtype=float
        )
        root_angular_velocity = root_twist[0:3]
        root_linear_velocity = root_twist[3:6]
        # Bonesaw stores the classical linear acceleration of the root origin.
        # Pinocchio's free-flyer tangent stores spatial acceleration, whose
        # linear component differs by ω × v at the identity pose used here.
        pin_root_linear_bias = -np.cross(
            root_angular_velocity, root_linear_velocity
        )
        floating_v[0:3] = root_twist[3:6]
        floating_v[3:6] = root_twist[0:3]
        floating_acceleration[0:3] = (
            root_acceleration[3:6] + pin_root_linear_bias
        )
        floating_acceleration[3:6] = root_acceleration[0:3]
        for bonesaw_index, (q_index, v_index) in enumerate(
            zip(floating_q_indices, floating_v_indices, strict=True)
        ):
            floating_q[q_index] = sample["q"][bonesaw_index]
            floating_v[v_index] = sample["v"][bonesaw_index]
            floating_acceleration[v_index] = root_acceleration[6 + bonesaw_index]

        pin.forwardKinematics(model, data, q, v, acceleration)
        pin.updateFramePlacements(model, data)
        pin.computeJointJacobians(model, data, q)
        mass = np.asarray(pin.crba(model, data, q), dtype=float)
        mass = 0.5 * (mass + mass.T)
        gravity = np.asarray(pin.computeGeneralizedGravity(model, data, q), dtype=float)
        inverse_dynamics = np.asarray(
            pin.rnea(model, data, q, v, acceleration), dtype=float
        )
        pin.ccrba(model, data, q, v)
        # A fixed-base Pinocchio model folds root-link inertia into `universe`,
        # then excludes that inertia from ccrba's moving-subtree CoM. Bonesaw's
        # canonical robot CoM includes the authored root body. Reconstruct the
        # full robot CoM and shift ccrba's angular momentum from the moving CoM
        # to that full-system point. The universe inertia has zero momentum.
        total_mass = sum(inertia.mass for inertia in model.inertias)
        center_of_mass = sum(
            inertia.mass * data.oMi[joint_id].act(inertia.lever)
            for joint_id, inertia in enumerate(model.inertias)
        ) / total_mass
        moving_center_of_mass = np.asarray(data.com[0], dtype=float)
        shift = moving_center_of_mass - center_of_mass
        shift_cross = np.array(
            [
                [0.0, -shift[2], shift[1]],
                [shift[2], 0.0, -shift[0]],
                [-shift[1], shift[0], 0.0],
            ]
        )
        raw_centroidal_map = np.asarray(data.Ag, dtype=float)
        linear_map = raw_centroidal_map[0:3, :]
        angular_map = raw_centroidal_map[3:6, :] + shift_cross @ linear_map
        centroidal_map = np.vstack((angular_map, linear_map))
        raw_centroidal_momentum = np.asarray(data.hg.vector, dtype=float)
        centroidal_momentum = np.concatenate(
            (
                raw_centroidal_momentum[3:6]
                + np.cross(shift, raw_centroidal_momentum[0:3]),
                raw_centroidal_momentum[0:3],
            )
        )

        floating_mass = np.asarray(
            pin.crba(floating_model, floating_data, floating_q), dtype=float
        )
        floating_mass = 0.5 * (floating_mass + floating_mass.T)
        floating_bias_acceleration = np.zeros(floating_model.nv)
        floating_bias_acceleration[0:3] = pin_root_linear_bias
        floating_bias = np.asarray(
            pin.rnea(
                floating_model,
                floating_data,
                floating_q,
                floating_v,
                floating_bias_acceleration,
            ),
            dtype=float,
        )
        floating_inverse_dynamics = np.asarray(
            pin.rnea(
                floating_model,
                floating_data,
                floating_q,
                floating_v,
                floating_acceleration,
            ),
            dtype=float,
        )
        pin.ccrba(floating_model, floating_data, floating_q, floating_v)
        floating_centroidal_map = np.asarray(floating_data.Ag, dtype=float)[
            np.r_[3:6, 0:3], :
        ][:, floating_permutation]

        selected = np.ix_(v_indices, v_indices)
        expected_mass = np.asarray(sample["mass_matrix_row_major"], dtype=float).reshape(
            len(coordinate_names), len(coordinate_names)
        )
        maxima["mass_matrix"] = max(
            maxima["mass_matrix"], maximum_absolute(expected_mass, mass[selected])
        )
        maxima["generalized_gravity"] = max(
            maxima["generalized_gravity"],
            maximum_absolute(
                np.asarray(sample["generalized_gravity"], dtype=float),
                gravity[v_indices],
            ),
        )
        maxima["inverse_dynamics"] = max(
            maxima["inverse_dynamics"],
            maximum_absolute(
                np.asarray(sample["inverse_dynamics"], dtype=float),
                inverse_dynamics[v_indices],
            ),
        )
        expected_centroidal_map = np.asarray(
            sample["centroidal_map_moment_force_row_major"], dtype=float
        ).reshape(6, len(coordinate_names))
        maxima["centroidal_map"] = max(
            maxima["centroidal_map"],
            maximum_absolute(expected_centroidal_map, centroidal_map[:, v_indices]),
        )
        maxima["centroidal_momentum"] = max(
            maxima["centroidal_momentum"],
            maximum_absolute(
                np.asarray(sample["centroidal_momentum_moment_force"], dtype=float),
                centroidal_momentum,
            ),
        )
        maxima["center_of_mass"] = max(
            maxima["center_of_mass"],
            maximum_absolute(
                np.asarray(sample["center_of_mass_world"], dtype=float),
                center_of_mass,
            ),
        )
        expected_floating_mass = np.asarray(
            sample["floating_mass_matrix_row_major"], dtype=float
        ).reshape(len(coordinate_names) + 6, len(coordinate_names) + 6)
        maxima["floating_mass_matrix"] = max(
            maxima["floating_mass_matrix"],
            maximum_absolute(
                expected_floating_mass,
                floating_mass[np.ix_(floating_permutation, floating_permutation)],
            ),
        )
        maxima["floating_bias"] = max(
            maxima["floating_bias"],
            maximum_absolute(
                np.asarray(sample["floating_bias_moment_force_joint"], dtype=float),
                floating_bias[floating_permutation],
            ),
        )
        maxima["floating_inverse_dynamics"] = max(
            maxima["floating_inverse_dynamics"],
            maximum_absolute(
                np.asarray(
                    sample["floating_inverse_dynamics_moment_force_joint"], dtype=float
                ),
                floating_inverse_dynamics[floating_permutation],
            ),
        )
        expected_floating_centroidal_map = np.asarray(
            sample["floating_centroidal_map_moment_force_row_major"], dtype=float
        ).reshape(6, len(coordinate_names) + 6)
        maxima["floating_centroidal_map"] = max(
            maxima["floating_centroidal_map"],
            maximum_absolute(
                expected_floating_centroidal_map, floating_centroidal_map
            ),
        )

        for frame in sample["frames"]:
            frame_id = model.getFrameId(frame["name"], pin.FrameType.BODY)
            if frame_id >= len(model.frames):
                raise RuntimeError(f"Pinocchio is missing BODY frame {frame['name']}")
            placement = data.oMf[frame_id]
            expected_translation = np.asarray(frame["translation"], dtype=float)
            expected_rotation = np.asarray(
                frame["rotation_matrix_row_major"], dtype=float
            ).reshape(3, 3)
            maxima["frame_translation_m"] = max(
                maxima["frame_translation_m"],
                maximum_absolute(expected_translation, placement.translation),
            )
            maxima["frame_rotation_matrix"] = max(
                maxima["frame_rotation_matrix"],
                maximum_absolute(expected_rotation, placement.rotation),
            )

            pin_jacobian = np.asarray(
                pin.getFrameJacobian(
                    model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                ),
                dtype=float,
            )
            # Pinocchio uses [linear; angular], Bonesaw [angular; linear].
            pin_jacobian = pin_jacobian[np.r_[3:6, 0:3], :][:, v_indices]
            expected_jacobian = np.asarray(
                frame["jacobian_angular_linear_row_major"], dtype=float
            ).reshape(6, len(coordinate_names))
            maxima["frame_jacobian"] = max(
                maxima["frame_jacobian"],
                maximum_absolute(expected_jacobian, pin_jacobian),
            )
            compared_frames += 1

    thresholds = {
        "frame_translation_m": 1e-10,
        "frame_rotation_matrix": 1e-10,
        "frame_jacobian": 1e-9,
        "center_of_mass": 1e-10,
        "mass_matrix": 1e-9,
        "generalized_gravity": 1e-9,
        "inverse_dynamics": 1e-8,
        "centroidal_map": 1e-9,
        "centroidal_momentum": 1e-9,
        "floating_mass_matrix": 1e-9,
        "floating_bias": 1e-8,
        "floating_inverse_dynamics": 1e-8,
        "floating_centroidal_map": 1e-9,
    }
    passed = all(maxima[name] <= thresholds[name] for name in thresholds)
    report = {
        "oracle": "Pinocchio",
        "pinocchio_version": pin.__version__,
        "model": fixture["model_name"],
        "samples": len(fixture["samples"]),
        "frames_compared": compared_frames,
        "coordinate_names": coordinate_names,
        "maximum_absolute_error": maxima,
        "thresholds": thresholds,
        "passed": passed,
    }
    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, indent=2) + "\n")
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(
            f"Bonesaw ↔ Pinocchio {pin.__version__} — {report['model']}, "
            f"{report['samples']} states, {compared_frames} frame samples"
        )
        for name, maximum in maxima.items():
            print(f"{name:28s} {maximum:12.3e}  limit {thresholds[name]:.1e}")
        print("PASS" if passed else "FAIL")
    return 0 if passed or args.no_strict else 1


if __name__ == "__main__":
    sys.exit(main())
