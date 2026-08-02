#!/usr/bin/env python3
"""Independent Pinocchio/NumPy oracle for r63 fixed-effort equality queries."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import time
from datetime import datetime, timezone
from typing import Any

import numpy as np

try:
    import pinocchio as pin
except ModuleNotFoundError:  # Unit-test the NumPy reference without the optional oracle wheel.
    pin = None  # type: ignore[assignment]

from cpu_reference_report import distribution, markdown_table, render_report_html


SCENARIOS = ("ideal_control", "fast_synthetic", "slow_synthetic", "quarter_available_slow")
POINTS = np.asarray(
    [
        [-0.05, -0.0275, -0.035],
        [-0.05, 0.0275, -0.035],
        [0.12, -0.0275, -0.035],
        [0.12, 0.0275, -0.035],
    ],
    dtype=np.float64,
)
FRAMES = ("left_ankle_roll_link", "right_ankle_roll_link")
SINGULAR_VALUE_TOLERANCE = 1e-12
TASK_DAMPING = 1e-8


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--oracle", default="benchmarks/results/g1-multistep-oracle-r54")
    parser.add_argument("--realization", default="benchmarks/results/g1-constrained-acceleration-r63")
    parser.add_argument("--samples", type=int, default=48)
    parser.add_argument("--output", default="benchmarks/results/g1-pinocchio-fixed-effort-r64")
    parser.add_argument("--web-report", default="web/G1_PINOCCHIO_FIXED_EFFORT_R64.html")
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


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)


def damped_pseudoinverse(
    matrix: np.ndarray,
    damping: float,
    tolerance: float = SINGULAR_VALUE_TOLERANCE,
) -> np.ndarray:
    rows, columns = matrix.shape
    if rows == 0 or columns == 0:
        return np.zeros((columns, rows), dtype=np.float64)
    left, singular, right_t = np.linalg.svd(matrix, full_matrices=False)
    maximum_singular = float(singular[0]) if singular.size else 0.0
    threshold = tolerance * max(maximum_singular, 1.0)
    factors = np.zeros_like(singular)
    retained = singular > threshold
    factors[retained] = singular[retained] / (
        singular[retained] * singular[retained] + damping * damping
    )
    return (right_t.T * factors) @ left.T


def normalized_rows(matrix: np.ndarray, target: np.ndarray, weight: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    matrix = np.asarray(matrix, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    norms = np.linalg.norm(matrix, axis=1)
    scale = np.sqrt(weight) / np.where(norms > 1e-12, norms, 1.0)
    return matrix * scale[:, None], target * scale


def lexicographic_equality_solve(
    equality: np.ndarray,
    equality_target: np.ndarray,
    levels: list[tuple[np.ndarray, np.ndarray]],
) -> np.ndarray:
    equality, equality_target = normalized_rows(equality, equality_target)
    equality_inverse = damped_pseudoinverse(equality, 0.0)
    solution = equality_inverse @ equality_target
    equality_nullspace = np.eye(equality.shape[1]) - equality_inverse @ equality
    nullspace = equality_nullspace.copy()
    for level_index, (matrix, target) in enumerate(levels):
        if matrix.shape[0] == 0:
            continue
        projected = matrix @ nullspace
        inverse = damped_pseudoinverse(projected, TASK_DAMPING)
        correction = nullspace @ inverse @ (target - matrix @ solution)
        correction = equality_nullspace @ correction
        solution += correction
        if level_index + 1 < len(levels):
            nullspace -= nullspace @ inverse @ projected
    return solution


def coordinate_layout(model: pin.Model, oracle_metrics: dict[str, Any]) -> dict[str, Any]:
    actuator_items = sorted(
        oracle_metrics["authority_evidence"]["actuator_effort"]["per_actuator"],
        key=lambda item: item["coordinate"],
    )
    names = [item["name"] for item in actuator_items]
    joint_ids = [model.getJointId(name) for name in names]
    if any(joint_id == 0 for joint_id in joint_ids):
        raise ValueError("Pinocchio is missing a Bonesaw coordinate")
    q_indices = [model.joints[joint_id].idx_q for joint_id in joint_ids]
    v_indices = [model.joints[joint_id].idx_v for joint_id in joint_ids]
    if any(model.joints[joint_id].nq != 1 or model.joints[joint_id].nv != 1 for joint_id in joint_ids):
        raise ValueError("r64 expects scalar G1 joints")
    return {
        "names": names,
        "q_indices": q_indices,
        "v_indices": v_indices,
        "permutation": np.asarray([3, 4, 5, 0, 1, 2, *v_indices], dtype=np.int64),
    }


def pin_configuration(
    model: pin.Model,
    layout: dict[str, Any],
    root_position: np.ndarray,
    root_velocity: np.ndarray,
    q_bonesaw: np.ndarray,
    v_bonesaw: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    configuration = pin.neutral(model)
    configuration[:3] = root_position
    velocity = np.zeros(model.nv, dtype=np.float64)
    velocity[:3] = root_velocity
    for value, index in zip(q_bonesaw, layout["q_indices"], strict=True):
        configuration[index] = value
    for value, index in zip(v_bonesaw, layout["v_indices"], strict=True):
        velocity[index] = value
    return configuration, velocity


def kinematic_products(
    model: pin.Model,
    configuration: np.ndarray,
    permutation: np.ndarray,
) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray], np.ndarray]:
    data = model.createData()
    pin.computeJointJacobians(model, data, configuration)
    pin.framesForwardKinematics(model, data, configuration)
    point_jacobians: list[np.ndarray] = []
    frame_linear_jacobians: list[np.ndarray] = []
    angular_jacobians: list[np.ndarray] = []
    for frame_name in FRAMES:
        frame_id = model.getFrameId(frame_name, pin.FrameType.BODY)
        frame_jacobian = np.asarray(
            pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED),
            dtype=np.float64,
        )
        frame_linear_jacobians.append(frame_jacobian[:3, :][:, permutation])
        angular_jacobians.append(frame_jacobian[3:6, :][:, permutation])
        for point in POINTS:
            point_world = data.oMf[frame_id].rotation @ point
            point_jacobians.append(
                (frame_jacobian[:3, :] - skew(point_world) @ frame_jacobian[3:6, :])[
                    :, permutation
                ]
            )
    center_of_mass_jacobian = np.asarray(
        pin.jacobianCenterOfMass(model, data, configuration), dtype=np.float64
    )[:, permutation]
    return (
        point_jacobians,
        frame_linear_jacobians,
        angular_jacobians,
        center_of_mass_jacobian,
    )


def pin_state_products(
    model: pin.Model,
    layout: dict[str, Any],
    root_position: np.ndarray,
    root_velocity: np.ndarray,
    q_bonesaw: np.ndarray,
    v_bonesaw: np.ndarray,
) -> dict[str, Any]:
    configuration, velocity = pin_configuration(
        model, layout, root_position, root_velocity, q_bonesaw, v_bonesaw
    )
    data = model.createData()
    pin.computeJointJacobians(model, data, configuration)
    pin.framesForwardKinematics(model, data, configuration)
    mass = np.asarray(pin.crba(model, data, configuration), dtype=np.float64)
    mass = 0.5 * (mass + mass.T)
    bias = np.asarray(
        pin.rnea(model, data, configuration, velocity, np.zeros(model.nv)), dtype=np.float64
    )
    point_jacobians, frame_linear_jacobians, angular_jacobians, com_jacobian = kinematic_products(
        model, configuration, layout["permutation"]
    )
    epsilon = 1e-7
    plus = pin.integrate(model, configuration, epsilon * velocity)
    minus = pin.integrate(model, configuration, -epsilon * velocity)
    plus_points, _, _, _ = kinematic_products(model, plus, layout["permutation"])
    minus_points, _, _, _ = kinematic_products(model, minus, layout["permutation"])
    point_bias = [
        ((plus_jacobian - minus_jacobian) / (2.0 * epsilon))
        @ velocity[layout["permutation"]]
        for plus_jacobian, minus_jacobian in zip(plus_points, minus_points, strict=True)
    ]
    return {
        "mass": mass[np.ix_(layout["permutation"], layout["permutation"])],
        "bias": bias[layout["permutation"]],
        "point_jacobians": point_jacobians,
        "point_bias": point_bias,
        "frame_linear_jacobians": frame_linear_jacobians,
        "angular_jacobians": angular_jacobians,
        "com_jacobian": com_jacobian,
    }


def equality_problem(
    products: dict[str, Any],
    torque: np.ndarray,
    contacts: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    generalized_dof = products["mass"].shape[0]
    variables = generalized_dof + 24
    dynamics = np.zeros((generalized_dof, variables), dtype=np.float64)
    dynamics[:, :generalized_dof] = products["mass"]
    target = -products["bias"].copy()
    target[6:] += torque
    active_force_slot = 0
    for foot in range(2):
        if not contacts[foot]:
            continue
        for point in range(4):
            point_index = foot * 4 + point
            dynamics[
                :, generalized_dof + active_force_slot * 3 : generalized_dof + active_force_slot * 3 + 3
            ] = -products["point_jacobians"][point_index].T
            active_force_slot += 1
    rows: list[np.ndarray] = [row.copy() for row in dynamics]
    targets: list[float] = target.tolist()
    for foot in range(2):
        if not contacts[foot]:
            continue
        for point, axes in ((0, (0, 1, 2)), (1, (2,)), (3, (1, 2))):
            slot = foot * 4 + point
            for axis in axes:
                row = np.zeros(variables, dtype=np.float64)
                row[:generalized_dof] = products["point_jacobians"][slot][axis]
                rows.append(row)
                targets.append(-products["point_bias"][slot][axis])
    # The Rust fixed-effort workspace retains eight statically allocated force
    # slots. Active contacts are packed first; remaining slots have exact-zero
    # bounds, represented as equalities by this independent solver.
    for slot in range(active_force_slot, 8):
        for axis in range(3):
            row = np.zeros(variables, dtype=np.float64)
            row[generalized_dof + slot * 3 + axis] = 1.0
            rows.append(row)
            targets.append(0.0)
    return np.vstack(rows), np.asarray(targets, dtype=np.float64)


def task_levels(
    products: dict[str, Any],
    reference_acceleration: np.ndarray,
    reference_force: np.ndarray,
    contacts: np.ndarray,
) -> list[tuple[np.ndarray, np.ndarray]]:
    generalized_dof = len(reference_acceleration)
    variables = generalized_dof + 24

    def direct(indices: list[int], weight: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
        matrix = np.zeros((len(indices), variables), dtype=np.float64)
        matrix[np.arange(len(indices)), indices] = 1.0
        return normalized_rows(matrix, reference_acceleration[indices], weight)

    invariant_parts = [direct([0, 1, 2]), direct([5])]
    invariant = (
        np.vstack([part[0] for part in invariant_parts]),
        np.concatenate([part[1] for part in invariant_parts]),
    )

    viability_matrices: list[np.ndarray] = []
    viability_targets: list[np.ndarray] = []
    root_horizontal = direct([3, 4])
    viability_matrices.append(root_horizontal[0])
    viability_targets.append(root_horizontal[1])
    com_matrix = np.zeros((3, variables), dtype=np.float64)
    com_matrix[:2, :generalized_dof] = products["com_jacobian"][:2]
    com_target = com_matrix @ np.pad(reference_acceleration, (0, 24))
    normalized_matrix, normalized_target = normalized_rows(com_matrix, com_target)
    viability_matrices.append(normalized_matrix)
    viability_targets.append(normalized_target)
    for foot in range(2):
        if contacts[foot]:
            continue
        point_matrix = np.zeros((3, variables), dtype=np.float64)
        # Oracle swing tasks use the ankle-roll frame origin, not a sole point.
        point_matrix[:, :generalized_dof] = products["frame_linear_jacobians"][foot]
        point_target = point_matrix @ np.pad(reference_acceleration, (0, 24))
        normalized_matrix, normalized_target = normalized_rows(point_matrix, point_target)
        viability_matrices.append(normalized_matrix)
        viability_targets.append(normalized_target)
        angular_matrix = np.zeros((3, variables), dtype=np.float64)
        angular_matrix[:, :generalized_dof] = products["angular_jacobians"][foot]
        angular_target = angular_matrix @ np.pad(reference_acceleration, (0, 24))
        normalized_matrix, normalized_target = normalized_rows(angular_matrix, angular_target)
        viability_matrices.append(normalized_matrix)
        viability_targets.append(normalized_target)
    viability = (np.vstack(viability_matrices), np.concatenate(viability_targets))

    preference = direct(list(range(6, generalized_dof)), weight=0.01)
    style_matrix = np.zeros((24, variables), dtype=np.float64)
    style_matrix[:, generalized_dof:] = np.eye(24)
    style = normalized_rows(style_matrix, reference_force.reshape(-1))
    return [invariant, viability, preference, style]


def selected_ticks(contacts: np.ndarray, pressure: np.ndarray, count: int) -> np.ndarray:
    ticks = len(contacts)
    edges = np.flatnonzero(np.any(contacts[1:] != contacts[:-1], axis=1)) + 1
    candidates = set(np.linspace(0, ticks - 1, count, dtype=int).tolist())
    for edge in edges:
        candidates.update(range(max(0, int(edge) - 2), min(ticks, int(edge) + 3)))
    leaders = np.argsort(pressure)[-count:]
    candidates.update(int(index) for index in leaders)
    ordered = sorted(candidates)
    if len(ordered) <= count:
        return np.asarray(ordered, dtype=np.int64)
    # Preserve every edge neighborhood, then fill remaining slots by even rank.
    mandatory = sorted(
        {
            tick
            for edge in edges
            for tick in range(max(0, int(edge) - 1), min(ticks, int(edge) + 2))
        }
    )
    remaining = [tick for tick in ordered if tick not in set(mandatory)]
    slots = max(0, count - len(mandatory))
    fill = [] if slots == 0 else [remaining[index] for index in np.linspace(0, len(remaining) - 1, slots, dtype=int)]
    return np.asarray(sorted(set(mandatory + fill))[:count], dtype=np.int64)


def main() -> None:
    if pin is None:
        raise RuntimeError(
            "Pinocchio is required; run scripts/run-g1-pinocchio-fixed-effort.sh"
        )
    args = parse_args()
    if args.samples < 24:
        raise ValueError("--samples must be at least 24 to retain all contact-edge neighborhoods")
    oracle_dir = pathlib.Path(args.oracle)
    realization_dir = pathlib.Path(args.realization)
    oracle_raw_path = oracle_dir / "oracle-wbc-admission-raw.npz"
    oracle_metrics_path = oracle_dir / "oracle-wbc-admission-metrics.json"
    realization_raw_path = realization_dir / "constrained-acceleration-raw.npz"
    realization_metrics_path = realization_dir / "constrained-acceleration-metrics.json"
    source_paths = (oracle_raw_path, oracle_metrics_path, realization_raw_path, realization_metrics_path)
    hashes_before = {str(path): sha256(path) for path in source_paths}
    oracle_metrics = json.loads(oracle_metrics_path.read_text())
    realization_metrics = json.loads(realization_metrics_path.read_text())
    with np.load(oracle_raw_path) as archive:
        source = {
            name: archive[name].copy()
            for name in ("root_positions", "root_velocities", "q", "v", "contacts", "generalized_acceleration")
        }
    with np.load(realization_raw_path) as archive:
        reference_force = archive["reference_contact_force_basis"].copy()
        realized = {
            scenario: {
                "acceleration": archive[f"{scenario}_generalized_acceleration"].copy(),
                "torque": archive[f"{scenario}_actuator_torque"].copy(),
                "force": archive[f"{scenario}_contact_force_basis"].copy(),
                "pressure": archive[f"{scenario}_bound_normalized_pressure"].copy(),
            }
            for scenario in SCENARIOS
        }
    ticks = selected_ticks(source["contacts"], realized["quarter_available_slow"]["pressure"], args.samples)
    model = pin.buildModelFromUrdf(oracle_metrics["model"], pin.JointModelFreeFlyer())
    model.gravity.linear = np.asarray([0.0, 0.0, -9.81])
    layout = coordinate_layout(model, oracle_metrics)
    generalized_dof = len(source["generalized_acceleration"][0])
    scenario_errors = {
        scenario: {"acceleration": [], "force": [], "solution": [], "solve_ns": []}
        for scenario in SCENARIOS
    }
    rust_hard_residuals = {scenario: {"dynamics": [], "contact": []} for scenario in SCENARIOS}
    product_ns: list[int] = []
    repeat_exact_by_scenario = {scenario: True for scenario in SCENARIOS}
    for sample_index, tick in enumerate(ticks):
        started = time.perf_counter_ns()
        products = pin_state_products(
            model,
            layout,
            source["root_positions"][tick],
            source["root_velocities"][tick],
            source["q"][tick],
            source["v"][tick],
        )
        product_ns.append(time.perf_counter_ns() - started)
        levels = task_levels(
            products,
            source["generalized_acceleration"][tick],
            reference_force[tick],
            source["contacts"][tick],
        )
        for scenario in SCENARIOS:
            equality, equality_target = equality_problem(
                products, realized[scenario]["torque"][tick], source["contacts"][tick]
            )
            rust_solution = np.concatenate(
                (
                    realized[scenario]["acceleration"][tick],
                    realized[scenario]["force"][tick].reshape(-1),
                )
            )
            hard_residual = equality @ rust_solution - equality_target
            rust_hard_residuals[scenario]["dynamics"].append(
                float(np.max(np.abs(hard_residual[:generalized_dof])))
            )
            rust_hard_residuals[scenario]["contact"].append(
                float(np.max(np.abs(hard_residual[generalized_dof:]), initial=0.0))
            )
            solve_started = time.perf_counter_ns()
            reference_solution = lexicographic_equality_solve(equality, equality_target, levels)
            scenario_errors[scenario]["solve_ns"].append(time.perf_counter_ns() - solve_started)
            if sample_index < 2:
                repeated = lexicographic_equality_solve(equality, equality_target, levels)
                repeat_exact_by_scenario[scenario] &= np.array_equal(reference_solution, repeated)
            difference = reference_solution - rust_solution
            scenario_errors[scenario]["solution"].append(difference)
            scenario_errors[scenario]["acceleration"].append(difference[:generalized_dof])
            scenario_errors[scenario]["force"].append(difference[generalized_dof:])

    scenario_rows = []
    all_repeat_exact = True
    for scenario in SCENARIOS:
        acceleration_error = np.asarray(scenario_errors[scenario]["acceleration"])
        force_error = np.asarray(scenario_errors[scenario]["force"])
        solution_error = np.asarray(scenario_errors[scenario]["solution"])
        dynamics_residual = np.asarray(rust_hard_residuals[scenario]["dynamics"])
        contact_residual = np.asarray(rust_hard_residuals[scenario]["contact"])
        repeat_exact = repeat_exact_by_scenario[scenario]
        all_repeat_exact &= repeat_exact
        scenario_rows.append(
            {
                "name": scenario,
                "acceleration_delta": {"rms": rms(acceleration_error), "absolute": distribution(np.abs(acceleration_error))},
                "contact_force_delta": {"rms": rms(force_error), "absolute": distribution(np.abs(force_error))},
                "complete_solution_delta": {"rms": rms(solution_error), "absolute": distribution(np.abs(solution_error))},
                "rust_under_pinocchio_dynamics_residual": distribution(dynamics_residual),
                "rust_under_pinocchio_contact_residual": distribution(contact_residual),
                "numpy_solve_us": distribution(np.asarray(scenario_errors[scenario]["solve_ns"], dtype=np.float64) / 1_000.0),
                "repeat_exact_on_two_states": repeat_exact,
            }
        )

    source_immutable = all(hashes_before[str(path)] == sha256(path) for path in source_paths)
    checks = {
        "source_artifacts_remain_immutable": source_immutable,
        "pinocchio_products_validate_every_rust_dynamics_row_below_2e-8": all(
            row["rust_under_pinocchio_dynamics_residual"]["maximum"] <= 2e-8 for row in scenario_rows
        ),
        "pinocchio_products_validate_every_rust_contact_row_below_2e-8": all(
            row["rust_under_pinocchio_contact_residual"]["maximum"] <= 2e-8 for row in scenario_rows
        ),
        "numpy_lexicographic_acceleration_matches_rust_within_2e-5": all(
            row["acceleration_delta"]["absolute"]["maximum"] <= 2e-5 for row in scenario_rows
        ),
        "numpy_lexicographic_contact_force_matches_rust_within_2e-4_n": all(
            row["contact_force_delta"]["absolute"]["maximum"] <= 2e-4 for row in scenario_rows
        ),
        "numpy_reference_repeats_bit_exactly_on_two_states_per_scenario": all_repeat_exact,
    }
    passed = all(checks.values())
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    raw_arrays: dict[str, np.ndarray] = {
        "ticks": ticks,
        "pinocchio_product_ns": np.asarray(product_ns, dtype=np.int64),
    }
    for scenario in SCENARIOS:
        raw_arrays[f"{scenario}_acceleration_delta"] = np.asarray(
            scenario_errors[scenario]["acceleration"], dtype=np.float64
        )
        raw_arrays[f"{scenario}_contact_force_delta"] = np.asarray(
            scenario_errors[scenario]["force"], dtype=np.float64
        ).reshape(len(ticks), 8, 3)
        raw_arrays[f"{scenario}_complete_solution_delta"] = np.asarray(
            scenario_errors[scenario]["solution"], dtype=np.float64
        )
        raw_arrays[f"{scenario}_rust_under_pinocchio_dynamics_residual"] = np.asarray(
            rust_hard_residuals[scenario]["dynamics"], dtype=np.float64
        )
        raw_arrays[f"{scenario}_rust_under_pinocchio_contact_residual"] = np.asarray(
            rust_hard_residuals[scenario]["contact"], dtype=np.float64
        )
        raw_arrays[f"{scenario}_numpy_solve_ns"] = np.asarray(
            scenario_errors[scenario]["solve_ns"], dtype=np.int64
        )
    np.savez_compressed(output / "pinocchio-fixed-effort-raw.npz", **raw_arrays)
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "passed": passed,
        "checks": checks,
        "evaluation_boundary": {
            "independent_dynamics_library": f"Pinocchio {pin.__version__}",
            "independent_solver": "NumPy SVD lexicographic equality projection",
            "sample_selection": "contact-edge neighborhoods plus evenly ranked and adverse-pressure states",
            "robot_policy": False,
            "state_integration": False,
            "contact_simulation": False,
            "rigid_body_rollout": False,
        },
        "sample_count": len(ticks),
        "ticks": ticks.tolist(),
        "scenarios": scenario_rows,
        "pinocchio_product_us": distribution(np.asarray(product_ns, dtype=np.float64) / 1_000.0),
        "source_sha256": hashes_before,
    }
    (output / "pinocchio-fixed-effort-metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    report = [
        "# Bonesaw G1 fixed-effort Pinocchio reference · r64", "",
        "## Result", "",
        f"An independent Pinocchio {pin.__version__} model reconstructs floating mass, bias, eight sole-point Jacobians, and contact acceleration bias at {len(ticks)} selected immutable r54 states. A separate NumPy SVD implementation rebuilds the four-level r63 lexicographic equality solve for four effort profiles. No Bonesaw model product or solver workspace is consumed by the reference.", "",
        "> This remains a state-local differential oracle: no policy, integration, contact simulation, or rigid-body rollout. It validates r63 equations and optimizer semantics, not calibrated actuator response or stability.", "",
        "## Differential matrix", "",
    ]
    report += markdown_table(
        ["scenario", "acceleration RMS / max", "contact-force RMS / max", "Pin dynamics max", "Pin contact max", "NumPy p99"],
        [[
            row["name"],
            f"{row['acceleration_delta']['rms']:.3e} / {row['acceleration_delta']['absolute']['maximum']:.3e}",
            f"{row['contact_force_delta']['rms']:.3e} / {row['contact_force_delta']['absolute']['maximum']:.3e}",
            f"{row['rust_under_pinocchio_dynamics_residual']['maximum']:.3e}",
            f"{row['rust_under_pinocchio_contact_residual']['maximum']:.3e}",
            f"{row['numpy_solve_us']['p99']:.1f} µs",
        ] for row in scenario_rows],
    )
    report += [
        "", "## Interpretation", "",
        "- Pinocchio independently owns inertial products and point Jacobians; NumPy independently owns equality seeding, damped task pseudoinverses, and nullspace freezing.",
        "- The sample set includes every contact edge neighborhood and additional evenly distributed/adverse-pressure states. It is a differential sentinel, not a replacement for r63's complete 2,317-state curve.",
        "- Dynamics/contact residuals evaluate the Rust solution directly under Pinocchio products. Solution deltas then test the independent lexicographic optimizer, so product and optimization disagreement cannot hide in one number.",
        "- Acceleration and contact-force comparisons remain separate because their units and operational meaning differ.",
        "", "## Gates", "",
    ]
    report += [f"- {'PASS' if value else 'FAIL'} `{name}`" for name, value in checks.items()]
    report += ["", "## Artifacts", "", "The raw NPZ retains every selected-state acceleration/contact-force/complete-solution delta, direct Pinocchio hard residual, and reference product/solve timing. The JSON retains selected ticks, per-scenario distributions, evaluation boundaries, and source checksums. Markdown and public HTML are human-readable mirrors.", ""]
    report_text = "\n".join(report)
    (output / "G1_PINOCCHIO_FIXED_EFFORT.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))
    print(json.dumps({"passed": passed, "checks": checks, "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
