#!/usr/bin/env python3
"""R211 policy/physics-free momentum-query oracle across Upkie and G1."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np
import pinocchio as pin

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "floating-momentum-cross-morphology-oracle-r211"
DEFAULT_MODELS = (
    "models/upkie/upkie.urdf",
    "benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf",
)
MODEL_QUERY_FRAMES = {
    "upkie": "left_wheel_center",
    "g1_23dof_mode_10": "left_ankle_roll_link",
}
ABSOLUTE_TOLERANCE = 1.0e-9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--models", nargs="+", default=list(DEFAULT_MODELS))
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/FLOATING_MOMENTUM_CROSS_MORPHOLOGY_ORACLE_R211.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def pin_layout(model: pin.Model, joint_names: list[str]) -> tuple[list[int], np.ndarray]:
    q_indices: list[int] = []
    velocity_indices: list[int] = []
    for name in joint_names:
        joint_id = model.getJointId(name)
        if joint_id == 0:
            raise ValueError(f"Pinocchio is missing Bonesaw joint {name}")
        joint = model.joints[joint_id]
        if joint.nq != 1 or joint.nv != 1:
            raise ValueError(f"R211 requires scalar joint {name}")
        q_indices.append(joint.idx_q)
        velocity_indices.append(joint.idx_v)
    permutation = np.asarray([3, 4, 5, 0, 1, 2, *velocity_indices], np.int64)
    return q_indices, permutation


def interval_hull(matrix: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if matrix.ndim != 2 or matrix.shape[0] != matrix.shape[1]:
        raise ValueError("interval map must be square")
    if lower.shape != (matrix.shape[1],) or upper.shape != lower.shape or np.any(lower > upper):
        raise ValueError("invalid interval")
    center = 0.5 * (lower + upper)
    radius = 0.5 * (upper - lower)
    mapped_center = matrix @ center
    mapped_radius = np.abs(matrix) @ radius
    return mapped_center - mapped_radius, mapped_center + mapped_radius


def joint_state(sample: int, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    phase = 0.31 * sample + 0.47 * np.arange(len(lower), dtype=np.float64)
    state = 0.18 * np.sin(phase)
    finite = np.isfinite(lower) & np.isfinite(upper)
    midpoint = 0.5 * (lower[finite] + upper[finite])
    half_range = 0.22 * (upper[finite] - lower[finite])
    state[finite] = midpoint + half_range * np.sin(phase[finite])
    return state


def momentum_box(sample: int, generalized_dof: int) -> tuple[np.ndarray, np.ndarray]:
    coordinate = np.arange(generalized_dof, dtype=np.float64)
    scale = np.concatenate(
        (
            np.full(3, 0.025),
            np.full(3, 0.050),
            np.full(generalized_dof - 6, 0.012),
        )
    )
    center = scale * np.sin(0.23 * sample + 0.41 * coordinate)
    radius = scale * (0.05 + 0.35 * np.abs(np.cos(0.19 * sample - 0.37 * coordinate)))
    return center - radius, center + radius


def run_model(path: pathlib.Path, samples: int) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    import bonesaw

    frame_name = MODEL_QUERY_FRAMES.get(path.stem)
    if frame_name is None:
        raise ValueError(f"R211 has no frozen query frame for {path.stem}")
    session = bonesaw.ContactTransitionModelSession(str(path), [frame_name])
    joint_names = list(session.joint_names())
    dof = int(session.joint_dof())
    generalized_dof = int(session.generalized_dof())
    if generalized_dof != dof + 6 or len(joint_names) != dof:
        raise ValueError("Bonesaw floating layout is inconsistent")
    model = pin.buildModelFromUrdf(str(path), pin.JointModelFreeFlyer())
    q_indices, permutation = pin_layout(model, joint_names)
    if len(permutation) != generalized_dof:
        raise ValueError("Pinocchio/Bonesaw generalized layouts differ")
    position_lower = np.asarray(session.joint_position_lower_limits(), np.float64)
    position_upper = np.asarray(session.joint_position_upper_limits(), np.float64)
    root_position = np.zeros(3, np.float64)
    root_quaternion = np.asarray([1.0, 0.0, 0.0, 0.0], np.float64)
    velocity_lower = np.empty(generalized_dof, np.float64)
    velocity_upper = np.empty(generalized_dof, np.float64)
    singleton_lower = np.empty(generalized_dof, np.float64)
    singleton_upper = np.empty(generalized_dof, np.float64)
    residual = np.empty((3, generalized_dof), np.float64)
    interval_error = np.empty(samples, np.float64)
    singleton_error = np.empty(samples, np.float64)
    residual_error = np.empty(samples, np.float64)
    interval_timing = np.empty(samples, np.int64)
    singleton_timing = np.empty(samples, np.int64)
    residual_timing = np.empty(samples, np.int64)
    condition = np.empty(samples, np.float64)
    zero_rust_allocation = True
    for sample in range(samples):
        q_bonesaw = joint_state(sample, position_lower, position_upper)
        configuration = pin.neutral(model)
        configuration[:3] = root_position
        configuration[3:7] = (0.0, 0.0, 0.0, 1.0)
        for value, index in zip(q_bonesaw, q_indices, strict=True):
            configuration[index] = value
        data = model.createData()
        mass_pin = np.asarray(pin.crba(model, data, configuration), np.float64)
        mass_pin = 0.5 * (mass_pin + mass_pin.T)
        mass = mass_pin[np.ix_(permutation, permutation)]
        inverse_mass = np.linalg.inv(mass)
        condition[sample] = np.linalg.cond(mass)
        lower, upper = momentum_box(sample, generalized_dof)
        expected_lower, expected_upper = interval_hull(inverse_mass, lower, upper)
        timing = session.generalized_velocity_interval_from_momentum_box(
            root_position,
            root_quaternion,
            q_bonesaw,
            lower,
            upper,
            velocity_lower,
            velocity_upper,
        )
        interval_timing[sample] = timing[0]
        zero_rust_allocation &= timing[1:] == (0, 0)
        interval_error[sample] = max(
            np.max(np.abs(velocity_lower - expected_lower)),
            np.max(np.abs(velocity_upper - expected_upper)),
        )
        center = 0.5 * (lower + upper)
        expected_center = inverse_mass @ center
        timing = session.generalized_velocity_interval_from_momentum_box(
            root_position,
            root_quaternion,
            q_bonesaw,
            center,
            center,
            singleton_lower,
            singleton_upper,
        )
        singleton_timing[sample] = timing[0]
        zero_rust_allocation &= timing[1:] == (0, 0)
        singleton_error[sample] = max(
            np.max(np.abs(singleton_lower - expected_center)),
            np.max(np.abs(singleton_upper - expected_center)),
            np.max(np.abs(singleton_upper - singleton_lower)),
        )
        coordinate = np.arange(generalized_dof, dtype=np.float64)
        observed = 0.02 * np.sin(0.17 * sample + 0.29 * coordinate)
        predicted = np.stack(
            [
                0.015 * np.cos(0.13 * sample + shift + 0.31 * coordinate)
                for shift in (0.0, 0.4, 0.8)
            ]
        )
        expected_residual = (mass @ (observed[None, :] - predicted).T).T
        timing = session.generalized_momentum_impulse_residuals(
            root_position,
            root_quaternion,
            q_bonesaw,
            observed,
            predicted,
            residual,
        )
        residual_timing[sample] = timing[0]
        zero_rust_allocation &= timing[1:] == (0, 0)
        residual_error[sample] = np.max(np.abs(residual - expected_residual))
    maximum_error = max(
        float(np.max(interval_error)),
        float(np.max(singleton_error)),
        float(np.max(residual_error)),
    )
    result = {
        "model": path.stem,
        "path": str(path),
        "sha256": sha256(path),
        "joint_dof": dof,
        "generalized_dof": generalized_dof,
        "samples": samples,
        "root_orientation": "identity",
        "pinocchio_version": pin.__version__,
        "maximum_absolute_error": maximum_error,
        "interval_error": distribution(interval_error),
        "singleton_error": distribution(singleton_error),
        "momentum_residual_error": distribution(residual_error),
        "mass_condition": distribution(condition),
        "interval_timing_ns": distribution(interval_timing),
        "singleton_timing_ns": distribution(singleton_timing),
        "residual_timing_ns": distribution(residual_timing),
        "zero_rust_allocation": zero_rust_allocation,
        "passed": zero_rust_allocation and maximum_error <= ABSOLUTE_TOLERANCE,
    }
    arrays = {
        f"{path.stem}_interval_error": interval_error,
        f"{path.stem}_singleton_error": singleton_error,
        f"{path.stem}_residual_error": residual_error,
        f"{path.stem}_mass_condition": condition,
        f"{path.stem}_interval_timing_ns": interval_timing,
        f"{path.stem}_singleton_timing_ns": singleton_timing,
        f"{path.stem}_residual_timing_ns": residual_timing,
    }
    return result, arrays


def main() -> int:
    args = parse_args()
    if args.samples <= 0:
        raise SystemExit("samples must be positive")
    paths = [pathlib.Path(item).resolve() for item in args.models]
    for path in paths:
        if not path.is_file():
            raise SystemExit(f"missing model: {path}")
    results: list[dict[str, Any]] = []
    replay_arrays: dict[str, np.ndarray] = {}
    for path in paths:
        result, arrays = run_model(path, args.samples)
        results.append(result)
        replay_arrays.update(arrays)
    passed = all(result["passed"] for result in results)
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "absolute_tolerance": ABSOLUTE_TOLERANCE,
        "physics_steps": 0,
        "policy_or_controller_steps": 0,
        "morphology_count": len(results),
        "mechanism_passed": passed,
        "authority_admitted": False,
        "results": results,
    }
    rows = [
        [
            result["model"],
            f"{result['joint_dof']} / {result['generalized_dof']}",
            result["samples"],
            f"{result['maximum_absolute_error']:.3e}",
            f"{result['mass_condition']['p95']:.3e}",
            f"{result['interval_timing_ns']['p99'] / 1_000.0:.3f}",
            f"{result['residual_timing_ns']['p99'] / 1_000.0:.3f}",
            "yes" if result["zero_rust_allocation"] else "NO",
            "PASS" if result["passed"] else "FAIL",
        ]
        for result in results
    ]
    report = "\n".join(
        [
            "# Bonesaw floating momentum cross-morphology oracle · r211",
            "",
            f"> Pinocchio D1 oracle **{'PASS' if passed else 'FAIL'}** · morphologies **{len(results)}** · physics steps **0** · policy/controller steps **0** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- Rust's generic `ContactTransitionModelSession` now owns the same allocation-free generalized-momentum residual and exact full-inverse-mass interval queries previously exposed only through the Upkie example session.",
            "- Python supplies deterministic state/box corpora. Pinocchio independently assembles each floating mass matrix, reorders its tangent to `[root angular; root linear; joints]`, and computes the exact linear image of every box.",
            "- Root orientation is fixed to identity so world/root tangent conventions coincide exactly. This is a state-local model-product oracle: no contact labels, policy, controller, integration, or physics rollout are used.",
            "",
            "## Cross-morphology result",
            "",
            *markdown_table(
                [
                    "model",
                    "joint / gen dof",
                    "states",
                    "max abs error",
                    "mass cond p95",
                    "box p99 µs",
                    "residual p99 µs",
                    "zero alloc",
                    "gate",
                ],
                rows,
            ),
            "",
            f"The absolute D1 threshold is **{ABSOLUTE_TOLERANCE:.1e}**. Singleton covectors, signed boxes, and three-candidate residual batches are scored independently on every state.",
            "",
            "## Decision",
            "",
            "The generic Rust mechanism crosses from wheeled Upkie to the 23-DOF G1 morphology and agrees with an independent Pinocchio mass-matrix oracle. This admits model machinery only. It does not validate R204/R210 residual calibration on G1, a second contact law, terminal consequence, deadline composition, or hardware authority.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "floating-momentum-cross-morphology-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "FLOATING_MOMENTUM_CROSS_MORPHOLOGY_ORACLE.md").write_text(report)
    np.savez_compressed(
        destination / "floating-momentum-cross-morphology-replay.npz", **replay_arrays
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": passed,
                "morphology_count": len(results),
                "maximum_absolute_error": max(
                    result["maximum_absolute_error"] for result in results
                ),
                "physics_steps": 0,
                "policy_or_controller_steps": 0,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
