#!/usr/bin/env python3
"""State-local constrained-acceleration consequences of realized actuator effort."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from cpu_tail_stability import arrays_byte_exact, runs
from g1_oracle_wbc_admission import allocate_wbc_outputs, run_oracle


DT_SECONDS = 0.005
ACCELERATION_BOUND = 200.0
REPEAT_WITNESS_TICKS = 128
FIXED_EFFORT_SOLVER_SCHEMA = 5
STATUS_NAMES = ("solved", "solved_with_slack", "primal_infeasible", "numerical_failure")
REPLAY_ABSOLUTE_TOLERANCES = {
    "generalized_acceleration": 2e-6,
    "actuator_torque": 2e-7,
    "contact_normal_force": 2e-6,
    "task_rms": 5e-7,
    "dynamics_residual": 1e-10,
    "contact_residual": 1e-10,
    "minimum_friction_margin": 2e-6,
    "minimum_support_margin": 1e-9,
    "minimum_torque_margin": 2e-7,
    "maximum_constraint_violation": 1e-10,
    "maximum_torque_utilization": 1e-8,
    "minimum_torque_headroom": 1e-7,
    "witness_acceleration_rms": 5e-7,
    "task_jacobi_sweeps": 2.0,
    "task_jacobi_sweeps_by_priority": 2.0,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--oracle", default="benchmarks/results/g1-multistep-oracle-r54")
    parser.add_argument("--realization", default="benchmarks/results/g1-actuator-realization-r62")
    parser.add_argument("--output", default="benchmarks/results/g1-constrained-acceleration-r63")
    parser.add_argument("--web-report", default="web/G1_CONSTRAINED_ACCELERATION_R63.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def rms(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=np.float64)
    return float(np.sqrt(np.mean(values * values)))


def extended_distribution(values: np.ndarray) -> dict[str, Any]:
    """Retain unavailable infinities without feeding them into percentiles."""
    values = np.asarray(values, dtype=np.float64)
    finite = values[np.isfinite(values)]
    return {
        "finite": None if finite.size == 0 else distribution(finite),
        "positive_infinity": int(np.count_nonzero(np.isposinf(values))),
        "negative_infinity": int(np.count_nonzero(np.isneginf(values))),
        "nan": int(np.count_nonzero(np.isnan(values))),
    }


def longest_true_run(mask: np.ndarray) -> int:
    return max((stop - start for start, stop in runs(mask)), default=0)


def contact_edges(contacts: np.ndarray) -> np.ndarray:
    return np.flatnonzero(np.any(contacts[1:] != contacts[:-1], axis=1)) + 1


def edge_recovery_ticks(
    pressure: np.ndarray,
    edges: np.ndarray,
    threshold: float,
    dwell_ticks: int = 5,
) -> list[int | None]:
    recoveries: list[int | None] = []
    for edge_index, edge in enumerate(edges):
        stop = int(edges[edge_index + 1]) if edge_index + 1 < len(edges) else len(pressure)
        recovery = None
        for tick in range(int(edge), max(int(edge), stop - dwell_ticks + 1)):
            if np.all(pressure[tick : tick + dwell_ticks] <= threshold):
                recovery = tick - int(edge)
                break
        recoveries.append(recovery)
    return recoveries


def inequality_traces(
    generalized_acceleration: np.ndarray,
    contact_normal_force: np.ndarray,
    friction_margin: np.ndarray,
    foot_contacts: np.ndarray,
) -> dict[str, np.ndarray]:
    """Score inequalities omitted from the continuous equality surrogate."""
    ticks = len(generalized_acceleration)
    active_points = np.repeat(foot_contacts.astype(bool), 4, axis=1)
    minimum_normal_force = np.full(ticks, np.nan, dtype=np.float64)
    support_margin = np.full(ticks, np.nan, dtype=np.float64)
    patch_x = np.array([-0.05, -0.05, 0.12, 0.12], dtype=np.float64)
    patch_y = np.array([-0.0275, 0.0275, -0.0275, 0.0275], dtype=np.float64)
    for tick in range(ticks):
        active_force = contact_normal_force[tick, active_points[tick]]
        if active_force.size:
            minimum_normal_force[tick] = float(np.min(active_force))
        foot_margins = []
        for foot in range(foot_contacts.shape[1]):
            if not foot_contacts[tick, foot]:
                continue
            force = contact_normal_force[tick, foot * 4 : foot * 4 + 4]
            total = float(np.sum(force))
            if total <= 1e-9:
                continue
            cop_x = float(np.dot(force, patch_x) / total)
            cop_y = float(np.dot(force, patch_y) / total)
            foot_margins.append(min(cop_x + 0.05, 0.12 - cop_x, cop_y + 0.0275, 0.0275 - cop_y))
        if foot_margins:
            support_margin[tick] = min(foot_margins)
    return {
        "acceleration_bound_excess": np.maximum(
            np.max(np.abs(generalized_acceleration), axis=1) - ACCELERATION_BOUND,
            0.0,
        ),
        "minimum_normal_force": minimum_normal_force,
        "minimum_friction_margin": np.asarray(friction_margin, dtype=np.float64),
        "minimum_support_margin": support_margin,
    }


def make_session(bonesaw: Any, model: str, config: dict[str, Any]) -> Any:
    return bonesaw.FloatingWbcSession(
        model,
        maximum_contacts=8,
        friction_coefficient=config["wbc_friction_coefficient"],
        maximum_acceleration=ACCELERATION_BOUND,
        maximum_torque=config["global_torque_cap_nm"],
        maximum_normal_force_multiple=3.0,
        root_angular_task_weight=config["wbc_root_angular_weight"],
        root_height_task_weight=config["wbc_root_height_weight"],
        root_horizontal_task_weight=config["wbc_root_horizontal_weight"],
        root_horizontal_task_priority=1,
        joint_posture_weight=0.01,
        joint_posture_priority=3,
        center_of_mass_task_weight=config["wbc_center_of_mass_weight"],
        center_of_mass_task_priority=1,
        centroidal_angular_momentum_weight=config["wbc_centroidal_angular_momentum_weight"],
        centroidal_angular_momentum_priority=1,
        contact_patch_center_x=0.035,
        contact_patch_half_length=0.085,
        contact_patch_half_width=0.0275,
        contact_patch_z=-0.035,
        minimum_contact_cop_margin_m=config["minimum_contact_cop_margin_m"],
    )


def solve_case(
    bonesaw: Any,
    model: str,
    config: dict[str, Any],
    source: dict[str, np.ndarray],
    realized_effort: np.ndarray,
    reference_contact_force_basis: np.ndarray,
) -> tuple[dict[str, np.ndarray], list[str]]:
    session = make_session(bonesaw, model, config)
    frame_names = list(session.frame_names)
    frame_ids = np.asarray(
        [frame_names.index("left_ankle_roll_link"), frame_names.index("right_ankle_roll_link")],
        dtype=np.int64,
    )
    ticks = len(source["q"])
    outputs = allocate_wbc_outputs(ticks, session.dof, 8, session.task_diagnostic_capacity)
    outputs["contact_force_basis"] = np.empty((ticks, 8, 3), dtype=np.float64)
    run_oracle(
        session,
        source["root_positions"],
        source["root_velocities"],
        source["root_accelerations"],
        source["q"],
        source["v"],
        source["joint_accelerations"],
        frame_ids,
        source["contacts"],
        config["wbc_effector_weight"],
        config["wbc_root_angular_priority"],
        config["wbc_root_height_priority"],
        outputs,
        realized_effort,
        source["generalized_acceleration"],
        contact_force_basis_out=outputs["contact_force_basis"],
        realization_reference_contact_force_basis=reference_contact_force_basis,
    )
    return outputs, list(session.joint_names)


def reconstruct_reference_contact_forces(
    bonesaw: Any,
    model: str,
    config: dict[str, Any],
    source: dict[str, np.ndarray],
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    session = make_session(bonesaw, model, config)
    frame_names = list(session.frame_names)
    frame_ids = np.asarray(
        [frame_names.index("left_ankle_roll_link"), frame_names.index("right_ankle_roll_link")],
        dtype=np.int64,
    )
    ticks = len(source["q"])
    outputs = allocate_wbc_outputs(ticks, session.dof, 8, session.task_diagnostic_capacity)
    force_basis = np.empty((ticks, 8, 3), dtype=np.float64)
    run_oracle(
        session,
        source["root_positions"],
        source["root_velocities"],
        source["root_accelerations"],
        source["q"],
        source["v"],
        source["joint_accelerations"],
        frame_ids,
        source["contacts"],
        config["wbc_effector_weight"],
        config["wbc_root_angular_priority"],
        config["wbc_root_height_priority"],
        outputs,
        contact_force_basis_out=force_basis,
    )
    return force_basis, outputs


def replay_delta_contract(
    replay: dict[str, np.ndarray],
    source: dict[str, np.ndarray],
    fields: tuple[str, ...],
) -> tuple[bool, dict[str, dict[str, Any]]]:
    details: dict[str, dict[str, Any]] = {}
    passed = True
    for field in fields:
        left = replay[field]
        right = source[field]
        exact = arrays_byte_exact(left, right)
        tolerance = REPLAY_ABSOLUTE_TOLERANCES.get(field, 0.0)
        if exact:
            maximum = 0.0
            within = True
        elif left.shape != right.shape:
            maximum = float("inf")
            within = False
        elif np.issubdtype(left.dtype, np.number) and np.issubdtype(right.dtype, np.number):
            difference = np.abs(left.astype(np.float64) - right.astype(np.float64))
            finite = difference[np.isfinite(difference)]
            maximum = float(np.max(finite)) if finite.size else 0.0
            special_values_match = (
                np.array_equal(np.isnan(left), np.isnan(right))
                and np.array_equal(np.isposinf(left), np.isposinf(right))
                and np.array_equal(np.isneginf(left), np.isneginf(right))
            )
            within = special_values_match and maximum <= tolerance
        else:
            maximum = float("inf")
            within = False
        details[field] = {
            "exact": exact,
            "maximum_absolute_delta": maximum,
            "absolute_tolerance": tolerance,
            "within_contract": within,
        }
        passed &= within
    return passed, details


def main() -> None:
    args = parse_args()
    try:
        import bonesaw
    except ImportError as error:
        raise RuntimeError("build the release PyO3 extension before running this evaluator") from error

    oracle_dir = pathlib.Path(args.oracle)
    realization_dir = pathlib.Path(args.realization)
    oracle_raw_path = oracle_dir / "oracle-wbc-admission-raw.npz"
    oracle_metrics_path = oracle_dir / "oracle-wbc-admission-metrics.json"
    realization_raw_path = realization_dir / "actuator-realization-raw.npz"
    realization_metrics_path = realization_dir / "actuator-realization-metrics.json"
    source_paths = (oracle_raw_path, oracle_metrics_path, realization_raw_path, realization_metrics_path)
    source_hashes_before = {str(path): sha256(path) for path in source_paths}
    source_cache_key = hashlib.sha256(
        json.dumps(
            {"sources": source_hashes_before, "fixed_effort_solver_schema": FIXED_EFFORT_SOLVER_SCHEMA},
            sort_keys=True,
        ).encode()
    ).hexdigest()
    oracle_metrics = json.loads(oracle_metrics_path.read_text())
    realization_metrics = json.loads(realization_metrics_path.read_text())
    if not oracle_metrics["passed"] or not realization_metrics["passed"]:
        raise ValueError("r63 requires admitted r54 WBC and valid r62 actuator realization inputs")

    state_fields = (
        "root_positions", "root_velocities", "root_accelerations", "q", "v",
        "joint_accelerations", "contacts", "generalized_acceleration", "actuator_torque",
        "contact_normal_force", "task_rms",
    )
    with np.load(oracle_raw_path) as archive:
        replay_fields = tuple(
            allocate_wbc_outputs(
                1,
                archive["q"].shape[1],
                archive["contact_normal_force"].shape[1],
                archive["task_rms"].shape[1],
            )
        )
        source = {
            name: archive[name].copy()
            for name in dict.fromkeys((*state_fields, *replay_fields))
        }
    with np.load(realization_raw_path) as archive:
        realized = {
            case["name"]: archive[f"{case['name']}_realized"].copy()
            for case in realization_metrics["scenarios"]
        }

    ticks = len(source["q"])
    edges = contact_edges(source["contacts"])
    print("r63: reconstructing the immutable r54 full contact-force witness", flush=True)
    reference_contact_force_basis, reference_replay = reconstruct_reference_contact_forces(
        bonesaw, oracle_metrics["model"], oracle_metrics["configuration"], source
    )
    replay_comparison_fields = tuple(field for field in replay_fields if field != "step_ns")
    reference_replay_exact = all(
        arrays_byte_exact(reference_replay[key], source[key])
        for key in replay_comparison_fields
    )
    reference_replay_within_contract, reference_replay_deltas = replay_delta_contract(
        reference_replay, source, replay_comparison_fields
    )
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    cases: list[dict[str, Any]] = []
    raw_outputs: dict[str, np.ndarray] = {}
    exact_repeat = True
    joint_names: list[str] = []
    coordinate_names: list[str] = []
    for source_case in realization_metrics["scenarios"]:
        name = source_case["name"]
        cache_path = output / f".resume-{name}.npz"
        first: dict[str, np.ndarray]
        repeat: dict[str, np.ndarray]
        resumed = False
        if cache_path.exists():
            with np.load(cache_path) as archive:
                if (
                    str(archive["source_cache_key"].item()) == source_cache_key
                    and int(archive["repeat_witness_ticks"].item()) == REPEAT_WITNESS_TICKS
                ):
                    first = {
                        key.removeprefix("first__"): archive[key].copy()
                        for key in archive.files
                        if key.startswith("first__")
                    }
                    repeat = {
                        key.removeprefix("repeat__"): archive[key].copy()
                        for key in archive.files
                        if key.startswith("repeat__")
                    }
                    joint_names = archive["joint_names"].tolist()
                    resumed = True
        if resumed:
            print(f"r63: resumed completed {name} corpus", flush=True)
        else:
            print(f"r63: solving full {name} corpus", flush=True)
            first, joint_names = solve_case(
                bonesaw,
                oracle_metrics["model"],
                oracle_metrics["configuration"],
                source,
                realized[name],
                reference_contact_force_basis,
            )
            repeat_source = {key: value[:REPEAT_WITNESS_TICKS] for key, value in source.items()}
            print(f"r63: repeating {name} on {REPEAT_WITNESS_TICKS}-tick witness", flush=True)
            repeat, repeat_joint_names = solve_case(
                bonesaw,
                oracle_metrics["model"],
                oracle_metrics["configuration"],
                repeat_source,
                realized[name][:REPEAT_WITNESS_TICKS],
                reference_contact_force_basis[:REPEAT_WITNESS_TICKS],
            )
            if repeat_joint_names != joint_names:
                raise RuntimeError("repeat session coordinate names changed")
            np.savez_compressed(
                cache_path,
                source_cache_key=np.asarray(source_cache_key),
                repeat_witness_ticks=np.asarray(REPEAT_WITNESS_TICKS),
                joint_names=np.asarray(joint_names),
                **{f"first__{key}": value for key, value in first.items()},
                **{f"repeat__{key}": value for key, value in repeat.items()},
            )
            print(f"r63: checkpointed {name}", flush=True)
        repeat_fields = [key for key in first if key != "step_ns"]
        case_repeat = all(
            arrays_byte_exact(first[key][:REPEAT_WITNESS_TICKS], repeat[key])
            for key in repeat_fields
        )
        exact_repeat &= case_repeat

        coordinate_names = [
            "root_angular_x", "root_angular_y", "root_angular_z",
            "root_linear_x", "root_linear_y", "root_linear_z", *joint_names,
        ]
        acceleration_error = first["generalized_acceleration"] - source["generalized_acceleration"]
        absolute_error = np.abs(acceleration_error)
        pressure = np.max(absolute_error, axis=1) / ACCELERATION_BOUND
        fixed_effort_error = first["actuator_torque"] - realized[name]
        normal_force_shift = first["contact_normal_force"] - source["contact_normal_force"]
        inequality = inequality_traces(
            first["generalized_acceleration"],
            first["contact_normal_force"],
            first["minimum_friction_margin"],
            source["contacts"],
        )
        recoveries = edge_recovery_ticks(pressure, edges, 0.01)
        finite_recoveries = [value for value in recoveries if value is not None]
        per_coordinate = []
        for coordinate, coordinate_name in enumerate(coordinate_names):
            per_coordinate.append({
                "coordinate": coordinate,
                "name": coordinate_name,
                "rms": rms(acceleration_error[:, coordinate]),
                "p99_absolute": float(np.percentile(absolute_error[:, coordinate], 99)),
                "maximum_absolute": float(np.max(absolute_error[:, coordinate])),
            })
        per_coordinate.sort(key=lambda row: (-row["p99_absolute"], row["coordinate"]))
        dwell = {
            str(threshold): {
                "ticks": int(np.count_nonzero(pressure > threshold)),
                "longest_ticks": longest_true_run(pressure > threshold),
                "longest_ms": longest_true_run(pressure > threshold) * DT_SECONDS * 1_000.0,
            }
            for threshold in (0.001, 0.005, 0.01, 0.05)
        }
        status_counts = {
            status_name: int(np.count_nonzero(first["status"] == code))
            for code, status_name in enumerate(STATUS_NAMES)
        }
        admitted = first["status"] <= 1
        admitted_violation = first["maximum_constraint_violation"][admitted]
        case = {
            "name": name,
            "profile": source_case["profile"],
            "exact_repeat": case_repeat,
            "status_counts": status_counts,
            "acceleration_error": {
                "all_coordinates": {"rms": rms(acceleration_error), "absolute": distribution(absolute_error)},
                "root_angular_rms": rms(acceleration_error[:, :3]),
                "root_linear_rms": rms(acceleration_error[:, 3:6]),
                "joint_rms": rms(acceleration_error[:, 6:]),
                "bound_normalized_pressure": distribution(pressure),
                "worst_coordinates_by_p99": per_coordinate[:8],
            },
            "dwell": dwell,
            "contact_edge_recovery_at_1pct_bound": {
                "recoveries_ticks": recoveries,
                "unrecovered_edges": sum(value is None for value in recoveries),
                "maximum_recovery_ms": None if not finite_recoveries else max(finite_recoveries) * DT_SECONDS * 1_000.0,
            },
            "fixed_effort_equality_error_nm": {"rms": rms(fixed_effort_error), "absolute": distribution(np.abs(fixed_effort_error))},
            "admitted_fixed_effort_equality_error_nm": (
                None
                if not np.any(admitted)
                else distribution(np.abs(fixed_effort_error[admitted]))
            ),
            "contact_normal_force_shift_n": {"rms": rms(normal_force_shift), "absolute": distribution(np.abs(normal_force_shift))},
            "constraints": {
                "dynamics_residual_linf": distribution(first["dynamics_residual"]),
                "contact_residual_linf": distribution(first["contact_residual"]),
                "maximum_constraint_violation": extended_distribution(first["maximum_constraint_violation"]),
                "admitted_maximum_constraint_violation": (
                    None if admitted_violation.size == 0 else distribution(admitted_violation)
                ),
            },
            "scored_inequalities": {
                "acceleration_bound_excess": extended_distribution(inequality["acceleration_bound_excess"]),
                "minimum_normal_force_n": extended_distribution(inequality["minimum_normal_force"]),
                "minimum_friction_margin_n": extended_distribution(inequality["minimum_friction_margin"]),
                "minimum_support_margin_m": extended_distribution(inequality["minimum_support_margin"]),
                "violation_ticks": {
                    "acceleration_bound": int(np.count_nonzero(inequality["acceleration_bound_excess"] > 1e-7)),
                    "normal_force": int(np.count_nonzero(inequality["minimum_normal_force"] < -1e-7)),
                    "friction": int(np.count_nonzero(inequality["minimum_friction_margin"] < -1e-7)),
                    "support_5mm": int(np.count_nonzero(inequality["minimum_support_margin"] < 0.005 - 1e-7)),
                },
            },
            "task_residual": {
                "maximum_rms": float(np.max(first["task_rms"])),
                "maximum_increase_from_r54": float(np.max(first["task_rms"] - source["task_rms"])),
            },
            "runtime": {
                "step_us": distribution(first["step_ns"].astype(np.float64) / 1_000.0),
                "allocation_calls": int(np.sum(first["allocation_calls"], dtype=np.uint64)),
                "allocated_bytes": int(np.sum(first["allocated_bytes"], dtype=np.uint64)),
            },
        }
        cases.append(case)
        for key in (
            "generalized_acceleration", "actuator_torque", "contact_normal_force", "task_rms",
            "contact_force_basis",
            "dynamics_residual", "contact_residual", "maximum_constraint_violation", "status",
            "step_ns", "allocation_calls", "allocated_bytes",
        ):
            raw_outputs[f"{name}_{key}"] = first[key]
        raw_outputs[f"{name}_acceleration_error"] = acceleration_error
        raw_outputs[f"{name}_bound_normalized_pressure"] = pressure
        for inequality_name, trace in inequality.items():
            raw_outputs[f"{name}_{inequality_name}"] = trace

    ideal = cases[0]
    source_immutable = all(source_hashes_before[str(path)] == sha256(path) for path in source_paths)
    checks = {
        "source_artifacts_remain_immutable": source_immutable,
        f"r54_full_contact_force_replay_stays_within_declared_delta_contract_on_{len(replay_comparison_fields)}_non_timing_fields": reference_replay_within_contract,
        "ideal_fixed_effort_reconstructs_admitted_acceleration_within_1e-5": ideal["acceleration_error"]["all_coordinates"]["absolute"]["maximum"] <= 1e-5,
        "all_scenarios_repeat_128_tick_witness_bit_exactly_except_timing": exact_repeat,
        "all_admitted_fixed_effort_equalities_hold_within_1e-6_nm": all(
            case["admitted_fixed_effort_equality_error_nm"] is None
            or case["admitted_fixed_effort_equality_error_nm"]["maximum"] <= 1e-6
            for case in cases
        ),
        "all_scenarios_avoid_numerical_failure": all(case["status_counts"]["numerical_failure"] == 0 for case in cases),
        "all_admitted_ticks_keep_hard_constraint_violations_below_1e-7": all(
            case["constraints"]["admitted_maximum_constraint_violation"] is None
            or case["constraints"]["admitted_maximum_constraint_violation"]["maximum"] <= 1e-7
            for case in cases
        ),
        "all_rust_steps_are_allocation_free": all(case["runtime"]["allocation_calls"] == 0 and case["runtime"]["allocated_bytes"] == 0 for case in cases),
    }
    mechanism_passed = all(checks.values())
    raw_outputs["reference_contact_force_basis"] = reference_contact_force_basis
    np.savez_compressed(output / "constrained-acceleration-raw.npz", **raw_outputs)
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "passed": mechanism_passed,
        "checks": checks,
        "evaluation_boundary": {
            "oracle_state_source": "immutable admitted r54 q, v, root pose/twist, and contact schedule",
            "effort_source": "r62 realized actuator effort scenarios",
            "robot_policy": False,
            "robot_state_integration": False,
            "rigid_body_rollout": False,
            "contact_simulation": False,
            "state_local_floating_dynamics_and_contact_solve": True,
            "hard_equalities": "floating dynamics plus active locked-contact acceleration",
            "scored_not_enforced_inequalities": "acceleration bounds, unilateral normal force, friction pyramid, 5 mm finite-patch support margin",
            "persistent_state_inside_r63": False,
            "stability_or_hardware_claim": False,
        },
        "ticks": ticks,
        "repeat_witness_ticks": REPEAT_WITNESS_TICKS,
        "fixed_effort_solver_schema": FIXED_EFFORT_SOLVER_SCHEMA,
        "dt_seconds": DT_SECONDS,
        "acceleration_bound": ACCELERATION_BOUND,
        "contact_edges": edges.tolist(),
        "coordinate_names": coordinate_names,
        "reference_replay_exact": reference_replay_exact,
        "reference_replay_within_delta_contract": reference_replay_within_contract,
        "reference_replay_deltas": reference_replay_deltas,
        "reference_replay_non_timing_fields": list(replay_comparison_fields),
        "scenarios": cases,
        "source_sha256": source_hashes_before,
    }
    (output / "constrained-acceleration-metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    with (output / "constrained-acceleration-scenarios.csv").open("w", newline="") as stream:
        fields = (
            "name", "bandwidth_hz", "effort_rate_nm_s", "availability_fraction",
            "acceleration_rms", "root_angular_rms", "root_linear_rms", "joint_rms",
            "p99_bound_pressure", "maximum_bound_pressure", "longest_over_1pct_ms",
            "maximum_edge_recovery_ms", "infeasible_ticks", "maximum_constraint_violation",
            "acceleration_bound_violation_ticks", "normal_force_violation_ticks",
            "friction_violation_ticks", "support_violation_ticks",
            "p99_step_us", "allocation_calls", "allocated_bytes",
        )
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for case in cases:
            writer.writerow({
                "name": case["name"], **case["profile"],
                "acceleration_rms": case["acceleration_error"]["all_coordinates"]["rms"],
                "root_angular_rms": case["acceleration_error"]["root_angular_rms"],
                "root_linear_rms": case["acceleration_error"]["root_linear_rms"],
                "joint_rms": case["acceleration_error"]["joint_rms"],
                "p99_bound_pressure": case["acceleration_error"]["bound_normalized_pressure"]["p99"],
                "maximum_bound_pressure": case["acceleration_error"]["bound_normalized_pressure"]["maximum"],
                "longest_over_1pct_ms": case["dwell"]["0.01"]["longest_ms"],
                "maximum_edge_recovery_ms": case["contact_edge_recovery_at_1pct_bound"]["maximum_recovery_ms"],
                "infeasible_ticks": case["status_counts"]["primal_infeasible"] + case["status_counts"]["numerical_failure"],
                "maximum_constraint_violation": (
                    None
                    if case["constraints"]["admitted_maximum_constraint_violation"] is None
                    else case["constraints"]["admitted_maximum_constraint_violation"]["maximum"]
                ),
                "acceleration_bound_violation_ticks": case["scored_inequalities"]["violation_ticks"]["acceleration_bound"],
                "normal_force_violation_ticks": case["scored_inequalities"]["violation_ticks"]["normal_force"],
                "friction_violation_ticks": case["scored_inequalities"]["violation_ticks"]["friction"],
                "support_violation_ticks": case["scored_inequalities"]["violation_ticks"]["support_5mm"],
                "p99_step_us": case["runtime"]["step_us"]["p99"],
                "allocation_calls": case["runtime"]["allocation_calls"],
                "allocated_bytes": case["runtime"]["allocated_bytes"],
            })

    report = [
        "# Bonesaw G1 constrained-acceleration realization · r63", "",
        "## Result", "",
        "At every immutable r54 oracle state, a compact Rust solve imposes r62 realized actuator effort, floating dynamics, and active locked-contact acceleration as exact equalities, then minimizes departure from admitted r54 acceleration and contact force. Acceleration bounds, unilateral normal force, friction, and the 5 mm finite-patch support margin are scored continuously afterward rather than converted into an active-set timeout. This exposes instantaneous consequence without a policy, state integration, contact simulation, or rigid-body rollout. Every profile covers all 2,317 states; deterministic replay uses a declared 128-tick witness prefix per profile.", "",
        "> This is a state-local counterfactual, not forward simulation and not evidence of closed-loop stability. Synthetic actuator profiles remain sensitivity envelopes, not Unitree G1 calibration. Each signal stays separate; there is no aggregate health score.", "",
        "## Capability matrix", "",
    ]
    report += markdown_table(
        ["case", "bandwidth / rate / availability", "accel RMS", "root lin / joint RMS", "p99 / max bound pressure", "longest >1%", "edge recovery max", "A/N/F/S violation ticks", "p99 Rust solve"],
        [[
            case["name"],
            f"{case['profile']['bandwidth_hz']} Hz / {case['profile']['effort_rate_nm_s']} Nm/s / {100 * case['profile']['availability_fraction']:.0f}%",
            f"{case['acceleration_error']['all_coordinates']['rms']:.4f}",
            f"{case['acceleration_error']['root_linear_rms']:.4f} / {case['acceleration_error']['joint_rms']:.4f}",
            f"{100 * case['acceleration_error']['bound_normalized_pressure']['p99']:.3f}% / {100 * case['acceleration_error']['bound_normalized_pressure']['maximum']:.3f}%",
            f"{case['dwell']['0.01']['longest_ms']:.0f} ms",
            "N/A" if case["contact_edge_recovery_at_1pct_bound"]["maximum_recovery_ms"] is None else f"{case['contact_edge_recovery_at_1pct_bound']['maximum_recovery_ms']:.0f} ms",
            "/".join(str(case["scored_inequalities"]["violation_ticks"][key]) for key in ("acceleration_bound", "normal_force", "friction", "support_5mm")),
            f"{case['runtime']['step_us']['p99']:.1f} µs",
        ] for case in cases],
    )
    report += ["", "## Most exposed coordinates", ""]
    report += markdown_table(
        ["case", "coordinate", "p99 absolute acceleration error", "maximum", "RMS"],
        [[case["name"], row["name"], f"{row['p99_absolute']:.5f}", f"{row['maximum_absolute']:.5f}", f"{row['rms']:.5f}"]
         for case in cases for row in case["acceleration_error"]["worst_coordinates_by_p99"][:3]],
    )
    report += [
        "", "## Interpretation", "",
        "- r54 remains the admission certificate for commanded WBC acceleration and effort. r62 remains the causal command-response certificate. r63 only maps r62 effort into a state-local constrained acceleration consequence.",
        "- The reference-force reconstruction records byte exactness separately from a tight field-declared replay contract. This admits the promoted r66 removal of mathematically inert terminal Style rows without weakening dynamics, contact, task, effort, or solver-work bounds.",
        "- Bound pressure is the largest coordinate acceleration error divided by the declared 200-unit generalized acceleration bound. Root angular, root linear, and joint RMS values remain separate because their physical units differ.",
        "- Fixed effort, floating dynamics, and locked-contact acceleration are hard equalities. Acceleration bounds, unilateral force, friction, and support are independent continuous signals; crossing one is measured loss of authority, not hidden solver slack.",
        "- The compact equality solve deliberately removes the generic torque decision block and inequality projector. This makes the query continuous and allocation-free, while r54 remains the separate full hard-inequality admission certificate.",
        "- Dwell and edge recovery distinguish a brief mismatch from persistent loss of authority. An unrecovered edge means the 1%-of-bound threshold did not hold for five ticks before the next contact transition.",
        "- Hardware deployment still requires measured actuator bandwidth/rate/derating profiles, estimator error, delay, compliance, and an integrated closed-loop stability campaign.",
        "", "## Mechanism gates", "",
    ]
    report += [f"- {'PASS' if passed else 'FAIL'} `{name}`" for name, passed in checks.items()]
    report += [
        "", "## Artifacts", "",
        "`constrained-acceleration-raw.npz` retains every scenario's realized generalized acceleration, acceleration error, fixed effort, contact force, task residual, equality residual, timing, allocation trace, and all four scored-inequality traces. The JSON retains the full capability curve, dwell, edge recovery, worst coordinates, exact-equality evidence, separate inequality pressure, evaluation boundary, and source checksums. The CSV is the compact matrix.", "",
    ]
    report_text = "\n".join(report)
    (output / "G1_CONSTRAINED_ACCELERATION.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))
    print(json.dumps({"passed": mechanism_passed, "checks": checks, "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
