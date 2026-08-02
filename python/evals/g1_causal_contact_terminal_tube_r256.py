#!/usr/bin/env python3
"""R256 spent-evidence causal contact/state tube composition audit.

This is deliberately not a controller.  Rust bounds a finite, explicitly
authored contact-law/acceleration family, Python lifts the resulting
generalized velocity interval to a complete terminal-state interval, and the
candidate-major hypothesis rows are scored by the Rust state-box boundary.
No completed plant labels are used to fit or widen the tube, and no policy,
MuJoCo step, plant action, or authority is exercised. The paired selector is
queried only as an evaluation boundary and emits no command.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import distribution, render_report_html
from g1_compliant_terminal_consequence_audit import (
    ROOT_IMPACT_PLANE_M,
    joint_limits,
)
from g1_contact_law_momentum_holdout import (
    FOOT_FRAMES,
    directional_witnesses,
    point_velocities_world,
    quaternion_from_rpy,
    sha256,
    standing_posture,
)
from g1_paired_terminal_score_plant_holdout_r254 import (
    FRESH_PLANT_LAWS,
    build_plant,
    plant_layout,
)


REVISION = "g1-causal-contact-terminal-tube-r256"
# The source artifact is the completed R254 replay, not the R253 family that
# R254 itself consumed.  Keeping this gate on the artifact revision makes the
# spent evidence boundary explicit and prevents accidentally opening a newer
# or differently-shaped replay.
SOURCE_REVISION = "g1-paired-terminal-score-plant-holdout-r254"
SOURCE_METRICS = pathlib.Path(
    "benchmarks/results/g1-paired-terminal-score-plant-holdout-r254/"
    "g1-paired-terminal-score-plant-holdout-metrics.json"
)
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-paired-terminal-score-plant-holdout-r254/"
    "g1-paired-terminal-score-plant-holdout.npz"
)
CANDIDATES = 3
HYPOTHESES = 4
CONTROL_DT_S = 0.020
CONTACT_FRAME_NAMES = (FOOT_FRAMES[0], FOOT_FRAMES[0], FOOT_FRAMES[1], FOOT_FRAMES[1])

# These are authored before any R254 output is opened.  Each row varies one
# contact/estimator reserve while sharing the exact hypothesis index across
# all three candidates; no residual fit, quantile, or completed label enters.
CONTACT_HYPOTHESES = (
    ("nominal", 0.80, 0.40, 0.010, 0.75),
    ("mid", 1.00, 0.70, 0.015, 1.00),
    ("high_restitution", 1.10, 1.00, 0.020, 1.25),
    ("wide_estimator", 1.20, 1.00, 0.020, 1.50),
)
RESERVE_FRACTION = np.asarray([50.0] * 3 + [10.0] * 3 + [50.0] * 23, np.float64)
UPPER_DIAGNOSTIC_NAMES = (
    "time_to_impact_s",
    "vertical_specific_impact_energy_j_kg",
    "terminal_tilt_rad",
    "terminal_angular_rate_rad_s",
    "maximum_terminal_joint_velocity_utilization",
    "impact_speed_pressure",
    "tilt_pressure",
    "angular_rate_pressure",
    "joint_position_pressure",
    "joint_velocity_pressure",
    "actuator_effort_pressure",
    "maximum_terminal_harm_pressure",
    "aggregate_score",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-metrics", default=str(SOURCE_METRICS))
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_CAUSAL_CONTACT_TERMINAL_TUBE_R256.html"
    )
    return parser.parse_args()


def interval_displacement(
    velocity: float, delta_lower: float, delta_upper: float, duration_s: float
) -> tuple[float, float]:
    """Outer-bound displacement when an interval velocity jump can occur."""

    endpoint_lower = velocity + delta_lower
    endpoint_upper = velocity + delta_upper
    return (
        min(velocity, endpoint_lower, endpoint_upper) * duration_s,
        max(velocity, endpoint_lower, endpoint_upper) * duration_s,
    )


def set_mujoco_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    root_qpos: int,
    root_qvel: int,
    joint_qpos: list[int],
    joint_qvel: list[int],
    root_state: np.ndarray,
    q: np.ndarray,
    velocity: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    roll, pitch = float(root_state[1]), float(root_state[2])
    quaternion = quaternion_from_rpy(roll, pitch, 0.0)
    data.qpos.fill(0.0)
    data.qpos[root_qpos : root_qpos + 7] = [
        0.0,
        0.0,
        float(root_state[0]) + ROOT_IMPACT_PLANE_M,
        *quaternion,
    ]
    data.qpos[joint_qpos] = q
    data.qvel.fill(0.0)
    # Bonesaw tangent order is [root angular, root linear, joints], while
    # MuJoCo's floating joint stores [root linear, root angular, joints].
    data.qvel[root_qvel : root_qvel + 3] = velocity[3:6]
    data.qvel[root_qvel + 3 : root_qvel + 6] = velocity[:3]
    data.qvel[joint_qvel] = velocity[6:]
    mujoco.mj_forward(model, data)
    return (
        np.asarray(data.qpos[root_qpos : root_qpos + 3], np.float64).copy(),
        np.asarray(data.qpos[root_qpos + 3 : root_qpos + 7], np.float64).copy(),
    )


def main() -> int:
    import bonesaw

    args = parse_args()
    model_path = pathlib.Path(args.model).resolve()
    metrics_path = pathlib.Path(args.source_metrics).resolve()
    replay_path = pathlib.Path(args.source_replay).resolve()
    if not all(path.is_file() for path in (model_path, metrics_path, replay_path)):
        raise SystemExit("R256 requires the pinned model and immutable R254 evidence")
    source_metrics = json.loads(metrics_path.read_text())
    if source_metrics.get("revision") != SOURCE_REVISION:
        raise ValueError("R256 source metrics revision is not the frozen R254 gate")
    source_hashes = {"metrics": sha256(metrics_path), "replay": sha256(replay_path)}
    with np.load(replay_path, allow_pickle=False) as replay:
        required = {
            "root_state",
            "joint_position",
            "initial_velocity",
            "candidate_acceleration",
            "candidate_effort_utilization",
            "actual_diagnostics",
            "fixed_status",
            "law_index",
        }
        missing = required - set(replay.files)
        if missing:
            raise ValueError(f"R254 replay is missing {sorted(missing)}")
        root_state = np.asarray(replay["root_state"], np.float64)
        joint_position = np.asarray(replay["joint_position"], np.float64)
        initial_velocity = np.asarray(replay["initial_velocity"], np.float64)
        candidate_acceleration = np.asarray(replay["candidate_acceleration"], np.float64)
        candidate_effort = np.asarray(replay["candidate_effort_utilization"], np.float64)
        actual_diagnostics = np.asarray(replay["actual_diagnostics"], np.float64)
        fixed_status = np.asarray(replay["fixed_status"], np.uint8)
        law_index = np.asarray(replay["law_index"], np.uint8)

    samples = len(root_state)
    if (
        root_state.shape != (samples, 3)
        or initial_velocity.shape[0] != samples
        or candidate_acceleration.shape[:2] != (samples, CANDIDATES)
        or candidate_effort.shape != (samples, CANDIDATES)
        or actual_diagnostics.shape != (samples, CANDIDATES, 17)
        or fixed_status.shape != (samples, CANDIDATES)
    ):
        raise ValueError("R254 replay has an unexpected terminal-tube shape")

    generic = bonesaw.ContactTransitionModelSession(str(model_path), CONTACT_FRAME_NAMES)
    joint_names = list(generic.joint_names())
    joints = int(generic.joint_dof())
    generalized = int(generic.generalized_dof())
    if generalized != 6 + joints or RESERVE_FRACTION.shape != (generalized,):
        raise ValueError("R256 reserve profile does not match the G1 tangent layout")
    q_nominal = standing_posture(joint_names)
    position_limits_lower, position_limits_upper, velocity_limits = joint_limits(
        model_path, joint_names
    )

    models: list[tuple[mujoco.MjModel, mujoco.MjData, tuple[Any, ...]]] = []
    for law in FRESH_PLANT_LAWS:
        model, data = build_plant(model_path, law)
        layout = plant_layout(model, joint_names)
        models.append((model, data, layout))
    total_masses = [float(np.sum(model.body_mass)) for model, _, _ in models]
    bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 4, axis=0)
    response = np.empty((generalized, 4, 3), np.float64)
    effective_mass = np.empty((4, 3), np.float64)
    delassus = np.empty((12, 12), np.float64)
    impulse_upper = np.empty((4, 3), np.float64)
    delta_lower = np.empty(generalized, np.float64)
    delta_upper = np.empty(generalized, np.float64)
    transition_time = np.empty(2, np.float64)

    root_lower = np.empty((CANDIDATES, HYPOTHESES, 6), np.float64)
    root_upper = np.empty_like(root_lower)
    q_lower = np.empty((CANDIDATES, HYPOTHESES, joints), np.float64)
    q_upper = np.empty_like(q_lower)
    v_lower = np.empty_like(q_lower)
    v_upper = np.empty_like(q_lower)
    available = np.empty((CANDIDATES, HYPOTHESES), np.uint8)
    root_acceleration = np.empty((CANDIDATES, HYPOTHESES, 2), np.float64)
    joint_acceleration = np.empty((CANDIDATES, HYPOTHESES, joints), np.float64)
    effort = np.empty((CANDIDATES, HYPOTHESES), np.float64)
    diagnostics = np.empty((CANDIDATES, HYPOTHESES, 17), np.float64)
    repeated = np.empty_like(diagnostics)

    tube_diagnostics = np.empty((samples, CANDIDATES, HYPOTHESES, 17), np.float64)
    tube_available = np.empty((samples, CANDIDATES, HYPOTHESES), np.uint8)
    paired_hypothesis_diagnostics = np.empty(
        (samples, CANDIDATES, HYPOTHESES, 14), np.float64
    )
    paired_envelope_diagnostics = np.empty((samples, CANDIDATES, 14), np.float64)
    paired_selection = np.empty((samples, 6), np.float64)
    paired_hypothesis_repeat = np.empty_like(paired_hypothesis_diagnostics)
    paired_envelope_repeat = np.empty_like(paired_envelope_diagnostics)
    paired_selection_repeat = np.empty_like(paired_selection)
    contact_timing_ns: list[int] = []
    state_box_timing_ns: list[int] = []
    paired_timing_ns: list[int] = []
    contact_zero_allocation = True
    state_box_zero_allocation = True
    paired_zero_allocation = True
    semantic_repeat = True
    paired_semantic_repeat = True
    kinematic_forwards = 0
    selector_queries = 0

    for sample in range(samples):
        law_slot = int(law_index[sample])
        if law_slot >= len(models):
            raise ValueError("R254 law index is outside the authored law table")
        model, data, layout = models[law_slot]
        root_qpos, joint_qpos, joint_qvel, root_qvel, foot_geoms, _ = layout
        root_position, quaternion = set_mujoco_state(
            model,
            data,
            root_qpos,
            root_qvel,
            joint_qpos,
            joint_qvel,
            root_state[sample],
            joint_position[sample],
            initial_velocity[sample],
        )
        kinematic_forwards += 1
        # The primitive fixture exposes exactly four probes (two per foot),
        # matching CONTACT_FRAME_NAMES' left/left/right/right ordering.
        selected_geoms = foot_geoms
        points = np.asarray(data.geom_xpos[selected_geoms], np.float64).copy()
        timing = generic.point_impulse_velocity_response(
            root_position,
            quaternion,
            joint_position[sample],
            points,
            bases,
            response,
            effective_mass,
            delassus,
        )
        contact_timing_ns.append(int(timing[0]))
        contact_zero_allocation &= tuple(timing[1:]) == (0, 0)
        prospective_velocity = point_velocities_world(model, data, selected_geoms)
        # Build one shared state tube per candidate/hypothesis, then make the
        # ballistic clearance/vertical-velocity columns common across all
        # candidates so R253's impact-speed separation remains explicit.
        candidate_clearance_lower = np.empty((CANDIDATES, HYPOTHESES), np.float64)
        candidate_clearance_upper = np.empty_like(candidate_clearance_lower)
        candidate_vertical_lower = np.empty_like(candidate_clearance_lower)
        candidate_vertical_upper = np.empty_like(candidate_clearance_lower)
        for candidate_index in range(CANDIDATES):
            acceleration = candidate_acceleration[sample, candidate_index]
            safe_acceleration = np.where(np.isfinite(acceleration), acceleration, 0.0)
            for hypothesis_index, (_name, friction_scale, restitution, duration, reserve_scale) in enumerate(
                CONTACT_HYPOTHESES
            ):
                transition_time[:] = [0.0, duration]
                witnesses = directional_witnesses(
                    prospective_velocity,
                    effective_mass,
                    total_masses[law_slot],
                    max(0.0, FRESH_PLANT_LAWS[law_slot].friction * friction_scale),
                )
                timing = generic.bound_directional_contact_transition_velocity_jump(
                    transition_time,
                    restitution,
                    witnesses,
                    safe_acceleration - reserve_scale * RESERVE_FRACTION,
                    safe_acceleration + reserve_scale * RESERVE_FRACTION,
                    response,
                    impulse_upper,
                    delta_lower,
                    delta_upper,
                )
                contact_timing_ns.append(int(timing[0]))
                contact_zero_allocation &= tuple(timing[1:]) == (0, 0)
                row = candidate_index * HYPOTHESES + hypothesis_index
                effort[candidate_index, hypothesis_index] = candidate_effort[
                    sample, candidate_index
                ]
                root_acceleration[candidate_index, hypothesis_index] = safe_acceleration[:2]
                joint_acceleration[candidate_index, hypothesis_index] = safe_acceleration[6:]
                root_velocity = initial_velocity[sample]
                tilt = root_state[sample, 1:3]
                tilt_lower = np.empty(2, np.float64)
                tilt_upper = np.empty(2, np.float64)
                for axis in range(2):
                    shift_lower, shift_upper = interval_displacement(
                        root_velocity[axis],
                        delta_lower[axis],
                        delta_upper[axis],
                        duration,
                    )
                    tilt_lower[axis] = tilt[axis] + shift_lower
                    tilt_upper[axis] = tilt[axis] + shift_upper
                angular_lower = root_velocity[:2] + delta_lower[:2]
                angular_upper = root_velocity[:2] + delta_upper[:2]
                clearance_shift = interval_displacement(
                    root_velocity[5], delta_lower[5], delta_upper[5], duration
                )
                candidate_clearance_lower[candidate_index, hypothesis_index] = (
                    root_state[sample, 0] + clearance_shift[0]
                )
                candidate_clearance_upper[candidate_index, hypothesis_index] = (
                    root_state[sample, 0] + clearance_shift[1]
                )
                candidate_vertical_lower[candidate_index, hypothesis_index] = (
                    root_velocity[5] + delta_lower[5]
                )
                candidate_vertical_upper[candidate_index, hypothesis_index] = (
                    root_velocity[5] + delta_upper[5]
                )
                joint_shift_lower = np.empty(joints, np.float64)
                joint_shift_upper = np.empty(joints, np.float64)
                for joint in range(joints):
                    shift_lower, shift_upper = interval_displacement(
                        root_velocity[6 + joint],
                        delta_lower[6 + joint],
                        delta_upper[6 + joint],
                        duration,
                    )
                    joint_shift_lower[joint] = shift_lower
                    joint_shift_upper[joint] = shift_upper
                q_lower[candidate_index, hypothesis_index] = (
                    joint_position[sample] + joint_shift_lower
                )
                q_upper[candidate_index, hypothesis_index] = (
                    joint_position[sample] + joint_shift_upper
                )
                v_lower[candidate_index, hypothesis_index] = (
                    root_velocity[6:] + delta_lower[6:]
                )
                v_upper[candidate_index, hypothesis_index] = (
                    root_velocity[6:] + delta_upper[6:]
                )
                root_lower[candidate_index, hypothesis_index] = [
                    0.0,
                    0.0,
                    tilt_lower[0],
                    tilt_lower[1],
                    angular_lower[0],
                    angular_lower[1],
                ]
                root_upper[candidate_index, hypothesis_index] = [
                    0.0,
                    0.0,
                    tilt_upper[0],
                    tilt_upper[1],
                    angular_upper[0],
                    angular_upper[1],
                ]
                available[candidate_index, hypothesis_index] = np.uint8(
                    fixed_status[sample, candidate_index] <= 1
                )
                # The two ballistic columns are filled from a shared union
                # below.  Zero is a valid temporary state for the preflight.
                _ = row

        common_clearance_lower = float(np.min(candidate_clearance_lower))
        common_clearance_upper = float(np.max(candidate_clearance_upper))
        common_vertical_lower = float(np.min(candidate_vertical_lower))
        common_vertical_upper = float(np.max(candidate_vertical_upper))
        invalid_clearance = common_clearance_lower < 0.0 or common_clearance_upper < 0.0
        common_clearance_lower = max(common_clearance_lower, 0.0)
        common_clearance_upper = max(common_clearance_upper, common_clearance_lower)
        for candidate_index in range(CANDIDATES):
            for hypothesis_index in range(HYPOTHESES):
                root_lower[candidate_index, hypothesis_index, 0] = common_clearance_lower
                root_upper[candidate_index, hypothesis_index, 0] = common_clearance_upper
                root_lower[candidate_index, hypothesis_index, 1] = common_vertical_lower
                root_upper[candidate_index, hypothesis_index, 1] = common_vertical_upper
                if invalid_clearance:
                    available[candidate_index, hypothesis_index] = 0

        timing = generic.score_terminal_impact_state_box_hypotheses(
            root_lower,
            root_upper,
            q_lower,
            q_upper,
            v_lower,
            v_upper,
            position_limits_lower,
            position_limits_upper,
            velocity_limits,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            diagnostics,
        )
        state_box_timing_ns.append(int(timing[0]))
        state_box_zero_allocation &= tuple(timing[1:]) == (0, 0)
        timing = generic.score_terminal_impact_state_box_hypotheses(
            root_lower,
            root_upper,
            q_lower,
            q_upper,
            v_lower,
            v_upper,
            position_limits_lower,
            position_limits_upper,
            velocity_limits,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            repeated,
        )
        state_box_zero_allocation &= tuple(timing[1:]) == (0, 0)
        semantic_repeat &= np.array_equal(diagnostics, repeated)
        tube_diagnostics[sample] = diagnostics
        tube_available[sample] = available

        # Candidate zero is the shared baseline state box.  For the other
        # candidates, endpoint subtraction gives a conservative delta tube;
        # the hypothesis index is retained on both sides.  The paired Rust
        # boundary owns the six-component lower/upper envelope and its
        # fail-closed selector.  This remains evaluation-only: no WBC
        # proposal or plant command is emitted.
        paired_baseline_root_lower = root_lower[0, :, 2:6].copy()
        paired_baseline_root_upper = root_upper[0, :, 2:6].copy()
        paired_candidate_root_delta_lower = np.zeros(
            (CANDIDATES, HYPOTHESES, 4), np.float64
        )
        paired_candidate_root_delta_upper = np.zeros_like(
            paired_candidate_root_delta_lower
        )
        paired_baseline_position_lower = q_lower[0].copy()
        paired_baseline_position_upper = q_upper[0].copy()
        paired_candidate_position_delta_lower = np.zeros(
            (CANDIDATES, HYPOTHESES, joints), np.float64
        )
        paired_candidate_position_delta_upper = np.zeros_like(
            paired_candidate_position_delta_lower
        )
        paired_baseline_velocity_lower = v_lower[0].copy()
        paired_baseline_velocity_upper = v_upper[0].copy()
        paired_candidate_velocity_delta_lower = np.zeros(
            (CANDIDATES, HYPOTHESES, joints), np.float64
        )
        paired_candidate_velocity_delta_upper = np.zeros_like(
            paired_candidate_velocity_delta_lower
        )
        for candidate_index in range(1, CANDIDATES):
            paired_candidate_root_delta_lower[candidate_index] = (
                root_lower[candidate_index, :, 2:6] - paired_baseline_root_upper
            )
            paired_candidate_root_delta_upper[candidate_index] = (
                root_upper[candidate_index, :, 2:6] - paired_baseline_root_lower
            )
            paired_candidate_position_delta_lower[candidate_index] = (
                q_lower[candidate_index] - paired_baseline_position_upper
            )
            paired_candidate_position_delta_upper[candidate_index] = (
                q_upper[candidate_index] - paired_baseline_position_lower
            )
            paired_candidate_velocity_delta_lower[candidate_index] = (
                v_lower[candidate_index] - paired_baseline_velocity_upper
            )
            paired_candidate_velocity_delta_upper[candidate_index] = (
                v_upper[candidate_index] - paired_baseline_velocity_lower
            )
        paired_baseline_effort = effort[0].copy()
        paired_candidate_effort = effort.copy()
        paired_timing = generic.bound_terminal_impact_paired_state_tubes(
            paired_baseline_root_lower,
            paired_baseline_root_upper,
            paired_candidate_root_delta_lower,
            paired_candidate_root_delta_upper,
            paired_baseline_position_lower,
            paired_baseline_position_upper,
            paired_candidate_position_delta_lower,
            paired_candidate_position_delta_upper,
            paired_baseline_velocity_lower,
            paired_baseline_velocity_upper,
            paired_candidate_velocity_delta_lower,
            paired_candidate_velocity_delta_upper,
            position_limits_lower,
            position_limits_upper,
            velocity_limits,
            available,
            paired_baseline_effort,
            paired_candidate_effort,
            0,
            0.0,
            0.01,
            paired_hypothesis_diagnostics[sample],
            paired_envelope_diagnostics[sample],
            paired_selection[sample],
        )
        paired_timing_ns.append(int(paired_timing[0]))
        paired_zero_allocation &= tuple(paired_timing[1:]) == (0, 0)
        selector_queries += 1
        paired_repeat_timing = generic.bound_terminal_impact_paired_state_tubes(
            paired_baseline_root_lower,
            paired_baseline_root_upper,
            paired_candidate_root_delta_lower,
            paired_candidate_root_delta_upper,
            paired_baseline_position_lower,
            paired_baseline_position_upper,
            paired_candidate_position_delta_lower,
            paired_candidate_position_delta_upper,
            paired_baseline_velocity_lower,
            paired_baseline_velocity_upper,
            paired_candidate_velocity_delta_lower,
            paired_candidate_velocity_delta_upper,
            position_limits_lower,
            position_limits_upper,
            velocity_limits,
            available,
            paired_baseline_effort,
            paired_candidate_effort,
            0,
            0.0,
            0.01,
            paired_hypothesis_repeat[sample],
            paired_envelope_repeat[sample],
            paired_selection_repeat[sample],
        )
        paired_zero_allocation &= tuple(paired_repeat_timing[1:]) == (0, 0)
        paired_semantic_repeat &= (
            np.array_equal(
                paired_hypothesis_diagnostics[sample], paired_hypothesis_repeat[sample]
            )
            and np.array_equal(
                paired_envelope_diagnostics[sample], paired_envelope_repeat[sample]
            )
            and np.array_equal(paired_selection[sample], paired_selection_repeat[sample])
        )

    source_immutable = source_hashes == {
        "metrics": sha256(metrics_path),
        "replay": sha256(replay_path),
    }
    upper_names = tuple(UPPER_DIAGNOSTIC_NAMES)
    names = tuple(generic.terminal_impact_state_diagnostic_names)
    upper_indices = np.asarray([names.index(name) for name in upper_names], np.int64)
    headroom_index = names.index("minimum_terminal_joint_headroom_fraction")
    envelope_upper = np.max(tube_diagnostics[:, :, :, :][:, :, :, upper_indices], axis=2)
    envelope_headroom_lower = np.min(tube_diagnostics[:, :, :, headroom_index], axis=2)
    upper_excess = np.maximum(
        actual_diagnostics[:, :, upper_indices] - envelope_upper,
        0.0,
    )
    headroom_shortfall = np.maximum(
        envelope_headroom_lower - actual_diagnostics[:, :, headroom_index],
        0.0,
    )
    upper_covered = np.all(upper_excess <= 1.0e-12, axis=2)
    headroom_covered = headroom_shortfall <= 1.0e-12
    terminal_coverage = upper_covered & headroom_covered
    mechanism_passed = bool(
        source_immutable
        and contact_zero_allocation
        and state_box_zero_allocation
        and paired_zero_allocation
        and semantic_repeat
        and paired_semantic_repeat
        and np.all(np.isfinite(tube_diagnostics))
        and np.all(np.isfinite(paired_hypothesis_diagnostics))
        and np.all(np.isfinite(paired_envelope_diagnostics))
        and np.all(np.isfinite(paired_selection))
        and np.all(np.isfinite(upper_excess))
        and np.all(np.isfinite(headroom_shortfall))
    )
    metrics = {
        "revision": REVISION,
        "source_revision": SOURCE_REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "source_hashes": source_hashes,
        "sample_count": samples,
        "candidate_count": CANDIDATES,
        "hypothesis_count": HYPOTHESES,
        "hypotheses": [name for name, *_ in CONTACT_HYPOTHESES],
        "kinematic_forwards": kinematic_forwards,
        "physics_steps": 0,
        "policy_steps": 0,
        "selector_queries": selector_queries,
        "plant_actions": 0,
        "mechanism_passed": mechanism_passed,
        "source_immutable": source_immutable,
        "contact_zero_rust_allocation": contact_zero_allocation,
        "state_box_zero_rust_allocation": state_box_zero_allocation,
        "paired_zero_rust_allocation": paired_zero_allocation,
        "semantic_repeat": semantic_repeat,
        "paired_semantic_repeat": paired_semantic_repeat,
        "terminal_candidate_coverage": int(np.count_nonzero(terminal_coverage)),
        "terminal_candidate_count": int(terminal_coverage.size),
        "terminal_candidate_coverage_fraction": float(np.mean(terminal_coverage)),
        "upper_component_coverage": float(np.mean(upper_excess <= 1.0e-12)),
        "headroom_coverage": float(np.mean(headroom_shortfall <= 1.0e-12)),
        "maximum_upper_excess": float(np.max(upper_excess)),
        "maximum_headroom_shortfall": float(np.max(headroom_shortfall)),
        "contact_timing_ns": distribution(np.asarray(contact_timing_ns, np.float64)),
        "state_box_timing_ns": distribution(np.asarray(state_box_timing_ns, np.float64)),
        "paired_timing_ns": distribution(np.asarray(paired_timing_ns, np.float64)),
        "profile_frozen": False,
        "authority_admitted": False,
    }
    report = "\n".join(
        [
            "# Bonesaw causal contact/state tube replay · r256",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · terminal coverage **{metrics['terminal_candidate_coverage']}/{metrics['terminal_candidate_count']} ({metrics['terminal_candidate_coverage_fraction'] * 100.0:.3f}%)** · profile **NOT FROZEN** · authority **NOT ADMITTED**.",
            "",
            "R256 consumes immutable R254 states and constructs four authored directional contact-law/estimator hypotheses per candidate. Rust owns the point response, directional velocity-jump outer bound, complete terminal-state-box scorer, and paired candidate-minus-baseline delta envelope; Python only lifts the finite velocity tube into q/clearance/attitude intervals and reports coverage. The ballistic clearance/vertical-speed columns are unioned across candidates so impact-speed deltas remain explicitly candidate-invariant; candidate/baseline rows retain the same hypothesis index.",
            "",
            f"The audit performs {kinematic_forwards} MuJoCo kinematic forwards, zero physics steps, zero policy steps, {selector_queries} evaluation-only paired selectors, and zero plant actions. Contact/state-box/paired timed Rust allocation is {'zero' if contact_zero_allocation and state_box_zero_allocation and paired_zero_allocation else 'nonzero'}; semantic replay is {'exact' if semantic_repeat and paired_semantic_repeat else 'not exact'}. Upper component coverage is {metrics['upper_component_coverage'] * 100.0:.3f}% and headroom coverage is {metrics['headroom_coverage'] * 100.0:.3f}%, with maximum excess/shortfall {metrics['maximum_upper_excess']:.6g}/{metrics['maximum_headroom_shortfall']:.6g}.",
            "",
            "This is a causal shared-hypothesis construction replay, not a calibrated transfer certificate. The finite hypothesis family, q-drift lift, and absolute consequence coverage must be frozen on spent evidence, then tested once on new contact laws/offsets with paired lower/upper deltas before any action or authority can be considered.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-causal-contact-terminal-tube-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-causal-contact-terminal-tube.npz",
        root_lower=root_lower,
        root_upper=root_upper,
        tube_diagnostics=tube_diagnostics,
        tube_available=tube_available,
        paired_hypothesis_diagnostics=paired_hypothesis_diagnostics,
        paired_envelope_diagnostics=paired_envelope_diagnostics,
        paired_selection=paired_selection,
        actual_diagnostics=actual_diagnostics,
        upper_excess=upper_excess,
        headroom_shortfall=headroom_shortfall,
    )
    (output / "G1_CAUSAL_CONTACT_TERMINAL_TUBE.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw causal contact tube · r256"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "terminal_candidate_coverage": metrics["terminal_candidate_coverage"],
                "terminal_candidate_count": metrics["terminal_candidate_count"],
                "profile_frozen": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
