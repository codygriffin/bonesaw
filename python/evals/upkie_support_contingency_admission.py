#!/usr/bin/env python3
"""Policy- and physics-free admission of support-conditioned contingency actions."""

from __future__ import annotations

import argparse
import gc
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_mujoco_plant_report import allocate_outputs


REVISION = "upkie-support-contingency-admission-r175"
STATUS = {0: "Solved", 1: "SolvedWithSlack", 2: "PrimalInfeasible", 3: "Invalid", 4: "MaxIterations"}
SUPPORT = {0: "flight", 1: "left", 2: "right", 3: "double"}
JOINT_ORDER = (
    "left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"
)
CONTACT_FRAMES = ("left_wheel_center", "right_wheel_center")
ROLLING_COORDINATES = np.asarray([2, 5], np.int64)
ROLLING_COEFFICIENTS = np.asarray([-0.05, 0.05], np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--samples-per-support", type=int, default=64)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_SUPPORT_CONTINGENCY_ADMISSION_R175.html")
    return parser.parse_args()


def quaternion(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.column_stack((
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ))


def corpus(samples_per_support: int) -> dict[str, np.ndarray]:
    if samples_per_support < 16:
        raise ValueError("samples-per-support must be at least 16")
    rows = 4 * samples_per_support
    phase = np.tile(np.linspace(0.0, 2.0 * math.pi, samples_per_support, endpoint=False), 4)
    masks = np.repeat(np.asarray([3, 1, 2, 0], np.uint8), samples_per_support)
    side = np.where(masks == 1, 1.0, np.where(masks == 2, -1.0, 0.0))
    root_position = np.column_stack((
        0.03 * np.sin(phase),
        side * (0.025 + 0.012 * np.sin(phase)),
        0.55 + 0.015 * np.cos(phase),
    ))
    roll = side * (0.06 + 0.035 * np.sin(phase))
    pitch = 0.06 * np.cos(0.8 * phase)
    yaw = side * 0.08 * np.sin(0.5 * phase)
    root_quaternion = quaternion(roll, pitch, yaw)
    root_twist = np.column_stack((
        side * (0.3 + 0.2 * np.cos(phase)),
        0.25 * np.sin(phase),
        side * 0.2 * np.cos(0.5 * phase),
        0.25 * np.sin(0.6 * phase),
        side * (0.18 + 0.08 * np.sin(phase)),
        0.12 * np.cos(phase),
    ))
    q = np.zeros((rows, 6), np.float64)
    q[:, [0, 1, 3, 4]] = np.asarray([0.4, -0.625, -0.4, 0.625])
    q[:, 0] += 0.05 * np.sin(phase)
    q[:, 1] -= 0.08 * np.cos(phase)
    q[:, 3] -= 0.05 * np.sin(phase)
    q[:, 4] += 0.08 * np.cos(phase)
    velocity = np.zeros_like(q)
    velocity[:, 0] = 0.25 * np.cos(phase)
    velocity[:, 1] = 0.20 * np.sin(phase)
    velocity[:, 3] = -velocity[:, 0]
    velocity[:, 4] = -velocity[:, 1]
    velocity[:, 2] = -root_twist[:, 3] / ROLLING_COEFFICIENTS[0]
    velocity[:, 5] = -root_twist[:, 3] / ROLLING_COEFFICIENTS[1]
    contact_active = np.column_stack((masks & 1, (masks >> 1) & 1)).astype(np.uint8)
    return {
        "mask": masks,
        "root_position": np.ascontiguousarray(root_position),
        "root_quaternion": np.ascontiguousarray(root_quaternion),
        "root_twist": np.ascontiguousarray(root_twist),
        "q": np.ascontiguousarray(q),
        "v": np.ascontiguousarray(velocity),
        "contact_active": np.ascontiguousarray(contact_active),
    }


def author(model: pathlib.Path, states: dict[str, np.ndarray], order: np.ndarray | None = None) -> dict[str, np.ndarray]:
    import bonesaw

    session = bonesaw.UpkieBalanceSession(str(model))
    count = len(states["mask"])
    order = np.arange(count) if order is None else order
    diagnostics = np.empty((count, 17), np.float64)
    angular = np.empty((count, 3), np.float64)
    linear = np.empty((count, 3), np.float64)
    joint = np.empty((count, 6), np.float64)
    timing = np.empty(count, np.uint64)
    allocation_calls = np.empty(count, np.uint64)
    allocated_bytes = np.empty(count, np.uint64)
    for destination, source in enumerate(order):
        timing[destination], allocation_calls[destination], allocated_bytes[destination] = (
            session.write_support_contingency_from_state(
                int(states["mask"][source]), states["root_position"][source],
                states["root_quaternion"][source], states["root_twist"][source],
                states["q"][source], states["v"][source], diagnostics[destination],
                angular[destination], linear[destination], joint[destination]
            )
        )
    return {"diagnostics": diagnostics, "angular": angular, "linear": linear,
            "joint": joint, "step_ns": timing, "allocation_calls": allocation_calls,
            "allocated_bytes": allocated_bytes}


def new_wbc(model: pathlib.Path) -> Any:
    import bonesaw

    return bonesaw.FloatingWbcSession(
        str(model), maximum_contacts=2, friction_coefficient=0.8,
        maximum_acceleration=250.0, maximum_torque=2000.0,
        maximum_normal_force_multiple=3.0, maximum_feasibility_iterations=64,
        repair_feasibility_equalities_before_inequalities=True,
        use_feasibility_row_spans=True,
        joint_limit_braking=True, root_angular_task_weight=10.0,
        root_height_task_weight=10.0, root_horizontal_task_weight=10.0,
        root_horizontal_task_priority=1, joint_posture_weight=1.0,
        joint_posture_priority=1, center_of_mass_task_weight=0.0,
    )


def admit(model: pathlib.Path, states: dict[str, np.ndarray], target: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    session = new_wbc(model)
    count = len(states["mask"])
    out = allocate_outputs(session, count)
    frames = list(session.frame_names)
    frame_ids = np.asarray([frames.index(name) for name in CONTACT_FRAMES], np.int64)
    session.run_oracle_trace(
        states["root_position"], states["root_twist"][:, 3:], target["linear"],
        states["q"], states["v"], target["joint"], frame_ids,
        states["contact_active"], np.ones(2, np.uint8), np.ones(2, np.float64), 0, 0,
        out["generalized_acceleration"], out["actuator_torque"], out["contact_normal_force"],
        out["task_rms"], out["task_clipped"], out["dynamics_residual"],
        out["contact_residual"], out["minimum_friction_margin"], out["minimum_support_margin"],
        out["minimum_torque_margin"], out["maximum_constraint_violation"],
        out["minimum_bound_margin"], out["minimum_joint_margin_rad"],
        out["minimum_joint_headroom_fraction"], out["limiting_joint"],
        out["maximum_torque_utilization"], out["minimum_torque_headroom"],
        out["limiting_actuator"], out["witness_acceleration_rms"], out["step_ns"], out["status"],
        out["task_pseudoinverse_calls"], out["task_pseudoinverse_calls_by_priority"],
        out["clipped_steps"], out["clipped_steps_by_priority"], out["task_jacobi_sweeps"],
        out["task_jacobi_sweeps_by_priority"], out["feasibility_projection_sweeps"],
        out["feasibility_halfspace_projections"], out["feasibility_polish_iterations"],
        out["allocation_calls"], out["allocated_bytes"],
        contact_force_basis_out=out["contact_force_basis"],
        contact_modes=np.full(2, 3, np.uint8), rolling_coordinates=ROLLING_COORDINATES,
        rolling_velocity_coefficients=ROLLING_COEFFICIENTS,
        rolling_velocity_stabilization_gains=np.full(2, 2.0, np.float64),
        rolling_maximum_stabilization_accelerations=np.full(2, 3.0, np.float64),
        root_quaternions_wxyz=states["root_quaternion"],
        root_angular_velocities_world=states["root_twist"][:, :3],
        root_angular_accelerations_world=target["angular"],
        feasibility_seed_reused_out=out["feasibility_seed_reused"],
        feasibility_prefix_resumed_out=out["feasibility_prefix_resumed"],
    )
    return out


def exact(left: dict[str, np.ndarray], right: dict[str, np.ndarray], ignored: set[str]) -> bool:
    return all(np.array_equal(value, right[name]) for name, value in left.items() if name not in ignored)


def main() -> None:
    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    states = corpus(args.samples_per_support)
    gc.collect(); gc.disable()
    gc_before = np.asarray([item["collections"] for item in gc.get_stats()])
    authored = author(model, states)
    rng = np.random.default_rng(175)
    permutation = rng.permutation(len(states["mask"]))
    shuffled = author(model, states, permutation)
    inverse = np.argsort(permutation)
    order_independent = all(np.array_equal(authored[name], shuffled[name][inverse]) for name in ("diagnostics", "angular", "linear", "joint"))
    admitted = admit(model, states, authored)
    replay_authored = author(model, states)
    replay = admit(model, states, replay_authored)
    gc_after = np.asarray([item["collections"] for item in gc.get_stats()]); gc.enable()
    tracking = admitted["generalized_acceleration"] - np.column_stack((authored["angular"], authored["linear"], authored["joint"]))
    by_support: dict[str, Any] = {}
    table = []
    for mask, label in SUPPORT.items():
        selected = states["mask"] == mask
        status = admitted["status"][selected]
        root_error = tracking[selected, :6]
        row = {
            "samples": int(np.sum(selected)),
            "admitted": int(np.sum(np.isin(status, (0, 1)))),
            "status_counts": {STATUS[int(value)]: int(np.sum(status == value)) for value in np.unique(status)},
            "root_tracking_rms": float(np.sqrt(np.mean(root_error * root_error))),
            "root_tracking_max": float(np.max(np.abs(root_error))),
            "maximum_torque_utilization": float(np.max(admitted["maximum_torque_utilization"][selected])),
            "maximum_constraint_violation": float(np.max(admitted["maximum_constraint_violation"][selected])),
            "latency_us": distribution(admitted["step_ns"][selected].astype(np.float64) / 1000.0),
        }
        by_support[label] = row
        table.append([label, row["samples"], row["admitted"], f"{row['root_tracking_rms']:.3e}", f"{row['root_tracking_max']:.3e}", f"{row['maximum_torque_utilization']:.3f}", f"{row['latency_us']['p99']:.1f}"])
    left = states["mask"] == 1
    right = states["mask"] == 2
    mirror_action = bool(
        np.allclose(authored["linear"][left, 1], -authored["linear"][right, 1], atol=2e-2, rtol=2e-2)
        and np.allclose(authored["angular"][left, 0], -authored["angular"][right, 0], atol=0.15, rtol=2e-2)
    )
    flight = states["mask"] == 0
    supported = ~flight
    gain = np.where(states["mask"] == 3, 4.0, np.where(supported, 8.0, 0.0))
    com_error_xy = authored["diagnostics"][:, 3:5]
    horizontal_request = authored["linear"][:, :2]
    horizontal_pd_energy_rate = np.sum(
        states["root_twist"][:, 3:5]
        * (horizontal_request - gain[:, None] * com_error_xy),
        axis=1,
    )
    gates = {
        "all_support_modes_authored": set(np.unique(states["mask"]).tolist()) == {0, 1, 2, 3},
        "authoring_order_independent": order_independent,
        "mirrored_single_support_action": mirror_action,
        "flight_linear_request_exactly_ballistic": bool(np.array_equal(authored["linear"][flight], np.tile([0.0, 0.0, -9.81], (int(np.sum(flight)), 1)))),
        "supported_horizontal_pd_is_dissipative": bool(
            np.all(horizontal_pd_energy_rate[supported] <= 1.0e-12)
        ),
        "every_independent_wbc_action_admitted": bool(np.all(np.isin(admitted["status"], (0, 1)))),
        "hard_constraints_feasible": bool(
            float(np.max(admitted["maximum_constraint_violation"])) < 1.0e-8
        ),
        "author_replay_exact": exact(authored, replay_authored, {"step_ns"}),
        "candidate_replay_exact": exact(admitted, replay, {"step_ns"}),
        "zero_rust_allocation": bool(np.sum(authored["allocation_calls"]) == 0 and np.sum(authored["allocated_bytes"]) == 0 and np.sum(admitted["allocation_calls"]) == 0 and np.sum(admitted["allocated_bytes"]) == 0),
        "zero_python_gc": bool(np.sum(gc_after - gc_before) == 0),
        "finite_outputs": bool(all(np.isfinite(value).all() for value in (authored["diagnostics"], authored["angular"], authored["linear"], authored["joint"], admitted["generalized_acceleration"], admitted["actuator_torque"]))),
    }
    passed = all(gates.values())
    metrics = {
        "revision": REVISION, "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model), "passed": passed, "gates": gates, "by_support": by_support,
        "overall": {
            "samples": len(states["mask"]),
            "author_latency_us": distribution(authored["step_ns"].astype(np.float64) / 1000.0),
            "wbc_latency_us": distribution(admitted["step_ns"].astype(np.float64) / 1000.0),
            "maximum_constraint_violation": float(np.max(admitted["maximum_constraint_violation"])),
            "maximum_torque_utilization": float(np.max(admitted["maximum_torque_utilization"])),
        },
        "claim_boundary": {
            "policy_steps": 0, "plant_steps": 0, "recovery_claim": False,
            "meaning": "Each action is generated from the exact current support mask and admitted by a separate floating WBC solve. This proves instantaneous executability for the corpus, not causal recovery or hardware safety."
        },
    }
    report = "\n".join([
        "# Bonesaw observed-support contingency admission · r175", "",
        f"> Evaluation **{'PASS' if passed else 'FAIL'}**. This is a policy- and physics-free corpus: Rust authors a support-conditioned low-energy action, then an independent floating WBC either admits or rejects it under the same exact observed support mask.", "",
        "## Result", "",
        "Double support brakes motion while pulling the CoM toward the midpoint; single support pulls toward the one observed wheel; zero support requests exact ballistic gravity and never invents a ground impulse. Angular and joint damping remain requests until the second WBC satisfies dynamics, contact, friction, joint, acceleration, and effort bounds.", "",
        *markdown_table(["support", "rows", "admitted", "root RMS", "root max", "max torque util", "p99 µs"], table), "",
        "## Integrity gates", "", *markdown_table(["gate", "result"], [[name, "PASS" if value else "FAIL"] for name, value in gates.items()]), "",
        "## Architectural boundary", "",
        "- No policy, plant integration, contact estimator, command lease, hidden clock, or reset participates.",
        "- Reordering every query produces bit-identical authored requests, so prior support history and cache lifetime cannot select the action.",
        "- The WBC session is separate from request authoring; a target is executable only when its typed solve status is `Solved` or `SolvedWithSlack`.",
        "- Delay, noisy contact transitions, causal recovery, actuator bandwidth/thermal effects, and hardware calibration remain external gates.",
    ]) + "\n"
    destination = pathlib.Path(args.output); destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-support-contingency-admission-metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    (destination / "UPKIE_SUPPORT_CONTINGENCY_ADMISSION_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report); web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "gates": gates, "by_support": by_support}, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
