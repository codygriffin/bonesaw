#!/usr/bin/env python3
"""Multidimensional closed-loop disturbance envelope for the Upkie example.

This is deliberately a Python-owned plant evaluation. MuJoCo owns contact,
integration, and the external wrench. Persistent Rust sessions own the capture
reference, state-local floating WBC, hierarchy, torque, and allocation witness.
The report admits the *evaluation* when its matrix is finite, repeatable, and
discriminating; individual controller rows remain honestly green or red.
"""

from __future__ import annotations

import argparse
import gc
import json
import math
import pathlib
import platform
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
import upkie_mujoco_plant_report as plant


REVISION = "upkie-disturbance-envelope-r133"
PUSH_START_S = 1.0
RECOVERY_WINDOW_S = 0.4
FALL_HEIGHT_M = 0.35
FALL_TILT_RAD = math.radians(45.0)
MUJOCO_WARNING_NAMES = tuple(mujoco.mjtWarning(index).name for index in range(8))


@dataclass(frozen=True)
class CaseSpec:
    name: str
    family: str
    force_world_n: tuple[float, float, float]
    duration_s: float
    body: str = "base"
    point_body_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    friction: float = 1.0
    repetitions: int = 1
    repeat_interval_s: float = 0.0

    @property
    def force_magnitude_n(self) -> float:
        return math.sqrt(sum(value * value for value in self.force_world_n))

    @property
    def impulse_ns(self) -> float:
        return self.force_magnitude_n * self.duration_s * self.repetitions

    @property
    def final_push_end_s(self) -> float:
        return (
            PUSH_START_S
            + (self.repetitions - 1) * self.repeat_interval_s
            + self.duration_s
        )


def case_matrix() -> tuple[CaseSpec, ...]:
    """Stable, predeclared rows; ordering is part of the retained artifact."""
    return (
        CaseSpec("nominal", "nominal", (0.0, 0.0, 0.0), 0.0),
        CaseSpec("forward_2n", "axis_x", (2.0, 0.0, 0.0), 0.1),
        CaseSpec("forward_4n_reference", "axis_x", (4.0, 0.0, 0.0), 0.1),
        CaseSpec("forward_6n_overload", "axis_x", (6.0, 0.0, 0.0), 0.1),
        CaseSpec("backward_2n", "axis_x", (-2.0, 0.0, 0.0), 0.1),
        CaseSpec("backward_4n", "axis_x", (-4.0, 0.0, 0.0), 0.1),
        CaseSpec("left_1n", "axis_y", (0.0, 1.0, 0.0), 0.1),
        CaseSpec("left_2n", "axis_y", (0.0, 2.0, 0.0), 0.1),
        CaseSpec("left_4n", "axis_y", (0.0, 4.0, 0.0), 0.1),
        CaseSpec("right_2n", "axis_y", (0.0, -2.0, 0.0), 0.1),
        CaseSpec("right_4n", "axis_y", (0.0, -4.0, 0.0), 0.1),
        CaseSpec("diagonal_4n", "axis_xy", (2.82842712474619, 2.82842712474619, 0.0), 0.1),
        CaseSpec("up_4n", "axis_z", (0.0, 0.0, 4.0), 0.1),
        CaseSpec("down_4n", "axis_z", (0.0, 0.0, -4.0), 0.1),
        CaseSpec("handle_forward_4n", "application_point", (4.0, 0.0, 0.0), 0.1, body="handle"),
        CaseSpec("short_8n_50ms", "duration", (8.0, 0.0, 0.0), 0.05),
        CaseSpec("long_2n_200ms", "duration", (2.0, 0.0, 0.0), 0.2),
        CaseSpec(
            "forward_2n_three_pulses",
            "repeated",
            (2.0, 0.0, 0.0),
            0.1,
            repetitions=3,
            repeat_interval_s=1.0,
        ),
        CaseSpec("forward_4n_friction_0p1", "friction", (4.0, 0.0, 0.0), 0.1, friction=0.1),
        CaseSpec("forward_4n_friction_0p03", "friction", (4.0, 0.0, 0.0), 0.1, friction=0.03),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument(
        "--controller",
        choices=(
            "capture",
            "planar_capture",
            "viability_capture",
            "viability_verified",
            "viability_support_capture",
            "viability_coordinate",
        ),
        default="capture",
        help="frozen sagittal baseline or differential-wheel planar candidate",
    )
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_DISTURBANCE_ENVELOPE_R133.html")
    parser.add_argument(
        "--cases",
        help="comma-separated case names; default runs the frozen complete matrix",
    )
    return parser.parse_args()


def validate_cases(cases: tuple[CaseSpec, ...]) -> None:
    if not cases or len({case.name for case in cases}) != len(cases):
        raise ValueError("case names must be non-empty and unique")
    for case in cases:
        values = (
            *case.force_world_n,
            *case.point_body_m,
            case.duration_s,
            case.friction,
            case.repeat_interval_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{case.name}: all case values must be finite")
        if case.duration_s < 0.0 or case.friction <= 0.0:
            raise ValueError(f"{case.name}: duration and friction must be positive/bounded")
        if case.repetitions < 1 or (
            case.repetitions > 1 and case.repeat_interval_s < case.duration_s
        ):
            raise ValueError(
                f"{case.name}: repetitions require a non-overlapping interval"
            )
        if case.force_magnitude_n > 8.0 + 1.0e-12:
            raise ValueError(f"{case.name}: force exceeds the live 8 N contract")


def prepare_case(
    model_path: pathlib.Path,
    case: CaseSpec,
    balance_mode: str = "capture",
    planar_config: tuple[float, float, float, float, float, float, float] | None = None,
    fall_safe_enabled: bool = False,
    fall_safe_primary_blend: bool = True,
    execute_reduced_support: bool = True,
    support_contingency_enabled: bool = False,
    support_contingency_execute: bool = False,
    support_contingency_preserve_primary_support: bool = False,
    support_contingency_query_every_tick: bool = False,
    support_contingency_flight_only: bool = False,
    support_contingency_single_evaluation_lease: bool = False,
    support_contingency_forecast_guard: bool = False,
    support_contingency_project_primary: bool = False,
    support_contingency_realize_primary_torque: bool = False,
    contact_program_authority_ticks: int | None = None,
    contact_program_inexact_hold_ticks: int = 0,
    contact_program_inexact_hold_authority: float = 1.0,
    contact_program_inexact_hold_forecast_selector: bool = False,
    contact_program_inexact_hold_forecast_minimum_improvement: float = 0.0,
    contact_program_inexact_support_free_brake: bool = False,
    contact_program_inexact_terminal_chooser: bool = False,
    contact_program_inexact_terminal_zero_effort_baseline: bool = False,
    contact_program_inexact_terminal_support_hypothesis_envelope: bool = False,
    contact_program_inexact_terminal_maximum_component_regression: float = 0.0,
    contact_program_inexact_terminal_minimum_component_improvement: float = 0.01,
    contact_command_lease_ticks: int | None = None,
    contact_command_lease_fallback_to_freshness: bool = False,
    lateral_viability_config: tuple[float, ...] | None = None,
    viability_planner_enabled: bool = False,
    viability_planner_activation_pressure: float = 0.10,
    viability_planner_release_pressure: float = 0.05,
    viability_support_requires_active_request: bool = False,
    viability_planner_strategy: str = "coordinate",
    viability_planner_update_period_ticks: int | None = None,
    viability_confirmation_updates: int = 1,
    execution_residual_veto: bool = False,
    maximum_feasibility_iterations: int = 64,
    maximum_feasibility_projection_sweeps: int | None = None,
    repair_feasibility_equalities_before_inequalities: bool = False,
    use_feasibility_row_spans: bool = False,
    viability_planner_maximum_feasibility_projection_sweeps: int | None = None,
    viability_planner_projection_continuation_violation_threshold: float | None = None,
    viability_planner_reuse_identical_hard_feasibility_seed: bool = False,
    viability_planner_transfer_hard_feasibility_witness: bool = False,
) -> tuple[mujoco.MjModel, mujoco.MjData, plant.RustWbcAdapter, np.ndarray, int]:
    import bonesaw

    model, data = plant.make_plant(model_path, sliding_friction=case.friction)
    balance = bonesaw.UpkieBalanceSession(str(model_path))
    if planar_config is not None:
        balance.configure_planar_capture(*planar_config)
    if lateral_viability_config is not None:
        balance.configure_lateral_viability(*lateral_viability_config)
    root_position, _, _, q, _ = plant.read_state(model, data)
    balanced_root = np.empty(3, np.float64)
    balanced_q = np.empty(6, np.float64)
    balance_error = balance.balanced_standing(
        root_position, q, balanced_root, balanced_q
    )
    if abs(balance_error) > 1.0e-6:
        raise RuntimeError(f"balanced projection retained {balance_error:.3e} m error")
    root_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    root_qpos = model.jnt_qposadr[root_joint]
    data.qpos[root_qpos : root_qpos + 3] = balanced_root
    for coordinate, name in enumerate(plant.JOINT_ORDER):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint]] = balanced_q[coordinate]
    mujoco.mj_forward(model, data)
    nominal_root, _, _, _, _ = plant.read_state(model, data)
    wheel_bodies = np.asarray(
        [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in ("left_ankle_mj5208_rotor", "right_ankle_mj5208_rotor")
        ],
        np.int64,
    )
    target_ground = float(np.mean(data.xpos[wheel_bodies, 0]))
    controller = plant.RustWbcAdapter(
        model_path,
        nominal_root,
        target_ground,
        balance,
        balance_mode,
        0.2,
        # This matrix is a frozen pre-R309 consequence fixture. New live
        # construction uses the qualified priority-0 posture hierarchy.
        joint_posture_priority=1,
        fall_safe_enabled=fall_safe_enabled,
        fall_safe_primary_blend=fall_safe_primary_blend,
        execute_reduced_support=execute_reduced_support,
        support_contingency_enabled=support_contingency_enabled,
        support_contingency_execute=support_contingency_execute,
        support_contingency_preserve_primary_support=(
            support_contingency_preserve_primary_support
        ),
        support_contingency_query_every_tick=support_contingency_query_every_tick,
        support_contingency_flight_only=support_contingency_flight_only,
        support_contingency_single_evaluation_lease=(
            support_contingency_single_evaluation_lease
        ),
        support_contingency_forecast_guard=support_contingency_forecast_guard,
        support_contingency_project_primary=support_contingency_project_primary,
        support_contingency_realize_primary_torque=(
            support_contingency_realize_primary_torque
        ),
        contact_program_authority_ticks=contact_program_authority_ticks,
        contact_program_inexact_hold_ticks=contact_program_inexact_hold_ticks,
        contact_program_inexact_hold_authority=contact_program_inexact_hold_authority,
        contact_program_inexact_hold_forecast_selector=(
            contact_program_inexact_hold_forecast_selector
        ),
        contact_program_inexact_hold_forecast_minimum_improvement=(
            contact_program_inexact_hold_forecast_minimum_improvement
        ),
        contact_program_inexact_support_free_brake=(
            contact_program_inexact_support_free_brake
        ),
        contact_program_inexact_terminal_chooser=(
            contact_program_inexact_terminal_chooser
        ),
        contact_program_inexact_terminal_zero_effort_baseline=(
            contact_program_inexact_terminal_zero_effort_baseline
        ),
        contact_program_inexact_terminal_support_hypothesis_envelope=(
            contact_program_inexact_terminal_support_hypothesis_envelope
        ),
        contact_program_inexact_terminal_maximum_component_regression=(
            contact_program_inexact_terminal_maximum_component_regression
        ),
        contact_program_inexact_terminal_minimum_component_improvement=(
            contact_program_inexact_terminal_minimum_component_improvement
        ),
        contact_command_lease_ticks=contact_command_lease_ticks,
        contact_command_lease_fallback_to_freshness=contact_command_lease_fallback_to_freshness,
        viability_planner_enabled=viability_planner_enabled,
        viability_planner_activation_pressure=viability_planner_activation_pressure,
        viability_planner_release_pressure=viability_planner_release_pressure,
        viability_support_requires_active_request=viability_support_requires_active_request,
        viability_planner_strategy=viability_planner_strategy,
        viability_planner_update_period_ticks=viability_planner_update_period_ticks,
        viability_confirmation_updates=viability_confirmation_updates,
        execution_residual_veto=execution_residual_veto,
        maximum_feasibility_iterations=maximum_feasibility_iterations,
        maximum_feasibility_projection_sweeps=maximum_feasibility_projection_sweeps,
        repair_feasibility_equalities_before_inequalities=repair_feasibility_equalities_before_inequalities,
        use_feasibility_row_spans=use_feasibility_row_spans,
        viability_planner_maximum_feasibility_projection_sweeps=viability_planner_maximum_feasibility_projection_sweeps,
        viability_planner_projection_continuation_violation_threshold=viability_planner_projection_continuation_violation_threshold,
        viability_planner_reuse_identical_hard_feasibility_seed=viability_planner_reuse_identical_hard_feasibility_seed,
        viability_planner_transfer_hard_feasibility_witness=viability_planner_transfer_hard_feasibility_witness,
    )
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, case.body)
    if body < 0:
        raise ValueError(f"{case.name}: unknown application body {case.body!r}")
    return model, data, controller, wheel_bodies, body


def run_case(
    model_path: pathlib.Path,
    case: CaseSpec,
    duration_s: float,
    balance_mode: str = "capture",
    planar_config: tuple[float, float, float, float, float, float, float] | None = None,
    fall_safe_enabled: bool = False,
    fall_safe_primary_blend: bool = True,
    measured_contact_admission: bool = False,
    execute_reduced_support: bool = True,
    support_contingency_enabled: bool = False,
    support_contingency_execute: bool = False,
    support_contingency_preserve_primary_support: bool = False,
    support_contingency_query_every_tick: bool = False,
    support_contingency_flight_only: bool = False,
    support_contingency_single_evaluation_lease: bool = False,
    support_contingency_forecast_guard: bool = False,
    support_contingency_project_primary: bool = False,
    support_contingency_realize_primary_torque: bool = False,
    contact_program_authority_ticks: int | None = None,
    contact_program_inexact_hold_ticks: int = 0,
    contact_program_inexact_hold_authority: float = 1.0,
    contact_program_inexact_hold_forecast_selector: bool = False,
    contact_program_inexact_hold_forecast_minimum_improvement: float = 0.0,
    contact_program_inexact_support_free_brake: bool = False,
    contact_program_inexact_terminal_chooser: bool = False,
    contact_program_inexact_terminal_zero_effort_baseline: bool = False,
    contact_program_inexact_terminal_support_hypothesis_envelope: bool = False,
    contact_program_inexact_terminal_maximum_component_regression: float = 0.0,
    contact_program_inexact_terminal_minimum_component_improvement: float = 0.01,
    contact_observation_prestart_samples: int = 0,
    contact_observation_prestart_age_ticks: int = 0,
    contact_observation_delay_ticks: int = 0,
    contact_observation_dropout_period_ticks: int = 0,
    contact_observation_dropout_burst_ticks: int = 0,
    contact_observation_dropout_start_tick: int = 0,
    contact_observation_flip_period_ticks: int = 0,
    contact_observation_flip_burst_ticks: int = 0,
    contact_observation_flip_contact: int = -1,
    record_physical_contact_impulses: bool = False,
    record_physical_contact_prestate: bool = False,
    record_physical_prospective_contact_state: bool = False,
    contact_command_lease_ticks: int | None = None,
    contact_command_lease_fallback_to_freshness: bool = False,
    lateral_viability_config: tuple[float, ...] | None = None,
    viability_planner_enabled: bool = False,
    viability_planner_activation_pressure: float = 0.10,
    viability_planner_release_pressure: float = 0.05,
    viability_support_requires_active_request: bool = False,
    viability_planner_strategy: str = "coordinate",
    viability_planner_update_period_ticks: int | None = None,
    viability_confirmation_updates: int = 1,
    execution_residual_veto: bool = False,
    maximum_feasibility_iterations: int = 64,
    maximum_feasibility_projection_sweeps: int | None = None,
    repair_feasibility_equalities_before_inequalities: bool = False,
    use_feasibility_row_spans: bool = False,
    viability_planner_maximum_feasibility_projection_sweeps: int | None = None,
    viability_planner_projection_continuation_violation_threshold: float | None = None,
    viability_planner_reuse_identical_hard_feasibility_seed: bool = False,
    viability_planner_transfer_hard_feasibility_witness: bool = False,
) -> dict[str, Any]:
    model, data, controller, wheel_bodies, application_body = prepare_case(
        model_path,
        case,
        balance_mode,
        planar_config,
        fall_safe_enabled,
        fall_safe_primary_blend,
        execute_reduced_support,
        support_contingency_enabled,
        support_contingency_execute,
        support_contingency_preserve_primary_support,
        support_contingency_query_every_tick,
        support_contingency_flight_only,
        support_contingency_single_evaluation_lease,
        support_contingency_forecast_guard,
        support_contingency_project_primary,
        support_contingency_realize_primary_torque,
        contact_program_authority_ticks,
        contact_program_inexact_hold_ticks,
        contact_program_inexact_hold_authority,
        contact_program_inexact_hold_forecast_selector,
        contact_program_inexact_hold_forecast_minimum_improvement,
        contact_program_inexact_support_free_brake,
        contact_program_inexact_terminal_chooser,
        contact_program_inexact_terminal_zero_effort_baseline,
        contact_program_inexact_terminal_support_hypothesis_envelope,
        contact_program_inexact_terminal_maximum_component_regression,
        contact_program_inexact_terminal_minimum_component_improvement,
        contact_command_lease_ticks,
        contact_command_lease_fallback_to_freshness,
        lateral_viability_config,
        viability_planner_enabled,
        viability_planner_activation_pressure,
        viability_planner_release_pressure,
        viability_support_requires_active_request,
        viability_planner_strategy,
        viability_planner_update_period_ticks,
        viability_confirmation_updates,
        execution_residual_veto,
        maximum_feasibility_iterations,
        maximum_feasibility_projection_sweeps,
        repair_feasibility_equalities_before_inequalities,
        use_feasibility_row_spans,
        viability_planner_maximum_feasibility_projection_sweeps,
        viability_planner_projection_continuation_violation_threshold,
        viability_planner_reuse_identical_hard_feasibility_seed,
        viability_planner_transfer_hard_feasibility_witness,
    )
    ticks = int(round(duration_s / plant.CONTROL_DT))
    substeps = int(round(plant.CONTROL_DT / plant.PHYSICS_DT))
    time_s = np.arange(ticks, dtype=np.float64) * plant.CONTROL_DT
    traces: dict[str, Any] = {
        "time_s": time_s,
        "root_position": np.empty((ticks, 3), np.float64),
        "root_quaternion_wxyz": np.empty((ticks, 4), np.float64),
        "root_twist": np.empty((ticks, 6), np.float64),
        "post_root_twist": np.empty((ticks, 6), np.float64),
        "rotation_vector": np.empty((ticks, 3), np.float64),
        "q": np.empty((ticks, 6), np.float64),
        "v": np.empty((ticks, 6), np.float64),
        "post_v": np.empty((ticks, 6), np.float64),
        "torque": np.empty((ticks, 6), np.float64),
        "generalized_acceleration": np.empty((ticks, 12), np.float64),
        "wbc_normal_force": np.empty((ticks, 2), np.float64),
        "physical_wheel_contact_impulse_ns": np.zeros(
            (ticks, 2, 2), np.float64
        ),
        "physical_wheel_contact_impulse_world_ns": np.zeros(
            (ticks, 2, 3), np.float64
        ),
        "physical_wheel_contact_position_m_ns": np.zeros(
            (ticks, 2, 3), np.float64
        ),
        "physical_wheel_contact_moment_world_origin_nms": np.zeros(
            (ticks, 2, 3), np.float64
        ),
        "physical_constraint_generalized_impulse_ns": np.zeros(
            (ticks, model.nv), np.float64
        ),
        "physical_wheel_contact_prestate_available": np.zeros(
            (ticks, 2), np.uint8
        ),
        "physical_wheel_contact_distance_m": np.full(
            (ticks, 2), np.inf, np.float64
        ),
        "physical_wheel_contact_relative_velocity_m_s": np.zeros(
            (ticks, 2, 3), np.float64
        ),
        "physical_wheel_prospective_contact_distance_m": np.zeros(
            (ticks, 2), np.float64
        ),
        "physical_wheel_prospective_contact_point_world_m": np.zeros(
            (ticks, 2, 3), np.float64
        ),
        "physical_wheel_prospective_contact_velocity_m_s": np.zeros(
            (ticks, 2, 3), np.float64
        ),
        "status": np.empty(ticks, np.uint8),
        "primary_status": np.empty(ticks, np.uint8),
        "primary_wbc_step_ns": np.empty(ticks, np.uint64),
        "primary_task_pseudoinverse_calls": np.empty(ticks, np.uint16),
        "primary_clipped_steps": np.empty(ticks, np.uint16),
        "primary_task_jacobi_sweeps": np.empty(ticks, np.uint16),
        "primary_feasibility_projection_sweeps": np.empty(ticks, np.uint16),
        "primary_feasibility_halfspace_projections": np.empty(
            ticks, np.uint32
        ),
        "support_contingency_admitted": np.empty(ticks, np.uint8),
        "support_contingency_armed": np.empty(ticks, np.uint8),
        "support_contingency_requested": np.empty(ticks, np.uint8),
        "support_contingency_selected": np.empty(ticks, np.uint8),
        "support_contingency_realization_fallback": np.empty(ticks, np.uint8),
        "support_contingency_status": np.empty(ticks, np.uint8),
        "support_contingency_mode": np.empty(ticks, np.uint8),
        "support_contingency_support_mask": np.empty(ticks, np.uint8),
        "support_contingency_maximum_constraint_violation": np.empty(
            ticks, np.float64
        ),
        "support_contingency_author_step_ns": np.empty(ticks, np.uint64),
        "support_contingency_wbc_step_ns": np.empty(ticks, np.uint64),
        "support_contingency_diagnostics": np.empty((ticks, 17), np.float64),
        "support_contingency_candidate_generalized_acceleration": np.empty(
            (ticks, 12), np.float64
        ),
        "support_contingency_candidate_torque": np.empty((ticks, 6), np.float64),
        "support_contingency_primary_torque": np.empty((ticks, 6), np.float64),
        "support_contingency_primary_source_fresh": np.empty(ticks, np.uint8),
        "support_contingency_candidate_power_w": np.empty(ticks, np.float64),
        "support_contingency_primary_power_w": np.empty(ticks, np.float64),
        "support_contingency_incremental_power_w": np.empty(ticks, np.float64),
        "support_contingency_forecast_guard_passed": np.empty(ticks, np.uint8),
        "support_contingency_forecast_baseline_score": np.empty(ticks, np.float64),
        "support_contingency_forecast_candidate_score": np.empty(ticks, np.float64),
        "support_contingency_forecast_step_ns": np.empty(ticks, np.uint64),
        "command_age_steps": np.empty(ticks, np.uint32),
        "contact_count": np.empty(ticks, np.uint16),
        "physical_contact_active": np.empty((ticks, 2), np.uint8),
        "observed_contact_active": np.empty((ticks, 2), np.uint8),
        "contact_observation_available": np.empty(ticks, np.uint8),
        "contact_observation_status": np.empty(ticks, np.uint8),
        "contact_observation_provenance": np.empty(ticks, np.uint8),
        "contact_observation_age_ns": np.empty(ticks, np.int64),
        "contact_observation_flags": np.empty(ticks, np.uint32),
        "admitted_contact_active": np.empty((ticks, 2), np.uint8),
        "support_transition_count": np.empty(ticks, np.uint32),
        "contact_program_authority_selection": np.empty(ticks, np.uint8),
        "contact_program_authority_executable": np.empty(ticks, np.uint8),
        "contact_program_authority_transition_pending": np.empty(ticks, np.uint8),
        "contact_program_authority_activation_pending": np.empty(ticks, np.uint8),
        "contact_program_authority_deactivation_pending": np.empty(ticks, np.uint8),
        "contact_program_authority_masks_consistent": np.empty(ticks, np.uint8),
        "contact_program_authority_lease_status": np.empty(ticks, np.uint8),
        "contact_program_authority_lease_provenance": np.empty(ticks, np.uint8),
        "contact_program_authority_authoring_mask": np.empty(ticks, np.uint8),
        "contact_program_authority_stable_mask": np.empty(ticks, np.uint8),
        "contact_program_authority_hard_mask": np.empty(ticks, np.uint8),
        "contact_program_authority_transition_count": np.empty(ticks, np.uint32),
        "contact_program_authority_transition_current_enabled": np.empty(
            ticks, np.uint8
        ),
        "contact_program_authority_age_ticks": np.empty(ticks, np.uint32),
        "contact_program_authority_remaining_ticks": np.empty(ticks, np.uint32),
        "contact_program_authority_flags": np.empty(ticks, np.uint32),
        "inexact_observation_authority_selector_queried": np.empty(
            ticks, np.uint8
        ),
        "inexact_observation_authority_selector_selected_index": np.empty(
            ticks, np.uint8
        ),
        "inexact_observation_authority_selector_authority_q15": np.empty(
            ticks, np.uint16
        ),
        "inexact_observation_authority_selector_selected_score": np.empty(
            ticks, np.float64
        ),
        "inexact_observation_authority_selector_zero_score": np.empty(
            ticks, np.float64
        ),
        "inexact_observation_authority_selector_full_score": np.empty(
            ticks, np.float64
        ),
        "inexact_observation_authority_selector_improvement": np.empty(
            ticks, np.float64
        ),
        "inexact_observation_authority_selector_candidate_scores": np.empty(
            (ticks, 5), np.float64
        ),
        "inexact_observation_authority_selector_step_ns": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_authority_selector_allocation_calls": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_authority_selector_allocated_bytes": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_selector_queried": np.empty(
            ticks, np.uint8
        ),
        "inexact_observation_terminal_zero_effort_available": np.empty(
            ticks, np.uint8
        ),
        "inexact_observation_terminal_zero_effort_acceleration": np.empty(
            (ticks, 12), np.float64
        ),
        "inexact_observation_terminal_zero_effort_step_ns": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_zero_effort_allocation_calls": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_zero_effort_allocated_bytes": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_hypothesis_acceleration": np.empty(
            (ticks, 4, 3, 12), np.float64
        ),
        "inexact_observation_terminal_hypothesis_available": np.empty(
            (ticks, 4, 3), np.uint8
        ),
        "inexact_observation_terminal_hypothesis_diagnostics": np.empty(
            (ticks, 3, 4, 17), np.float64
        ),
        "inexact_observation_terminal_hypothesis_envelopes": np.empty(
            (ticks, 3, 17), np.float64
        ),
        "inexact_observation_terminal_hypothesis_torque": np.empty(
            (ticks, 3, 6), np.float64
        ),
        "inexact_observation_terminal_hypothesis_effort_utilization": np.empty(
            (ticks, 3), np.float64
        ),
        "inexact_observation_terminal_hypothesis_query_step_ns": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_hypothesis_query_allocation_calls": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_hypothesis_query_allocated_bytes": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_hypothesis_aggregate_step_ns": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_hypothesis_aggregate_allocation_calls": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_hypothesis_aggregate_allocated_bytes": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_selector_action": np.empty(
            ticks, np.uint8
        ),
        "inexact_observation_terminal_selector_retained_available": np.empty(
            ticks, np.uint8
        ),
        "inexact_observation_terminal_selector_support_free_available": np.empty(
            ticks, np.uint8
        ),
        "inexact_observation_terminal_selector_time_to_impact_s": np.empty(
            ticks, np.float64
        ),
        "inexact_observation_terminal_selector_vertical_specific_energy_j_kg": np.empty(
            ticks, np.float64
        ),
        "inexact_observation_terminal_selector_candidate_diagnostics": np.empty(
            (ticks, 3, 17), np.float64
        ),
        "inexact_observation_terminal_selector_state": np.empty(
            (ticks, 6), np.float64
        ),
        "inexact_observation_terminal_selector_root_acceleration": np.empty(
            (ticks, 3, 2), np.float64
        ),
        "inexact_observation_terminal_selector_joint_acceleration": np.empty(
            (ticks, 3, 6), np.float64
        ),
        "inexact_observation_terminal_selector_effort_utilization": np.empty(
            (ticks, 3), np.float64
        ),
        "inexact_observation_terminal_selector_selection_diagnostics": np.empty(
            (ticks, 6), np.float64
        ),
        "inexact_observation_terminal_selector_maximum_harm_pressures": np.empty(
            (ticks, 3), np.float64
        ),
        "inexact_observation_terminal_selector_aggregate_scores": np.empty(
            (ticks, 3), np.float64
        ),
        "inexact_observation_terminal_selector_step_ns": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_selector_allocation_calls": np.empty(
            ticks, np.uint64
        ),
        "inexact_observation_terminal_selector_allocated_bytes": np.empty(
            ticks, np.uint64
        ),
        "contact_command_lease_status": np.empty(ticks, np.uint8),
        "contact_command_lease_executable": np.empty(ticks, np.uint8),
        "contact_command_lease_age_ticks": np.empty(ticks, np.uint32),
        "contact_command_lease_remaining_ticks": np.empty(ticks, np.uint32),
        "contact_command_lease_flags": np.empty(ticks, np.uint32),
        "capture_pressure": np.empty(ticks, np.float64),
        "station_authority": np.empty(ticks, np.float64),
        "station_error": np.empty(ticks, np.float64),
        "heading": np.empty(ticks, np.float64),
        "lateral_capture_error": np.empty(ticks, np.float64),
        "planar_capture_pressure": np.empty(ticks, np.float64),
        "commanded_yaw_rate": np.empty(ticks, np.float64),
        "lateral_support_margin": np.empty(ticks, np.float64),
        "lateral_dcm": np.empty(ticks, np.float64),
        "viability_margin": np.empty(ticks, np.float64),
        "commanded_zmp": np.empty(ticks, np.float64),
        "commanded_lateral_acceleration": np.empty(ticks, np.float64),
        "commanded_bank_angle": np.empty(ticks, np.float64),
        "commanded_roll_acceleration": np.empty(ticks, np.float64),
        "viability_activation_pressure": np.empty(ticks, np.float64),
        "viability_zmp_was_saturated": np.empty(ticks, np.uint8),
        "viability_verified_scale": np.empty(ticks, np.float64),
        "viability_verification_queries": np.empty(ticks, np.uint8),
        "viability_support_pressure": np.empty(ticks, np.float64),
        "viability_support_active": np.empty(ticks, np.uint8),
        "viability_coordinate_queries": np.empty(ticks, np.uint8),
        "viability_coordinate_score": np.empty(ticks, np.float64),
        "viability_coordinate_target": np.empty((ticks, 3), np.float64),
        "viability_coordinate_request": np.empty((ticks, 3), np.float64),
        "viability_request_status": np.empty(ticks, np.uint8),
        "viability_request_active": np.empty(ticks, np.uint8),
        "viability_request_executable": np.empty(ticks, np.uint8),
        "viability_request_slew_limited": np.empty(ticks, np.uint8),
        "viability_request_age_ticks": np.empty(ticks, np.uint32),
        "viability_request_remaining_ticks": np.empty(ticks, np.uint32),
        "viability_planner_update": np.empty(ticks, np.uint8),
        "viability_confirmation_status": np.empty(ticks, np.uint8),
        "viability_confirmation_executable": np.empty(ticks, np.uint8),
        "viability_confirmation_has_shadow": np.empty(ticks, np.uint8),
        "viability_confirmation_support_mask": np.empty(ticks, np.uint8),
        "viability_confirmation_consistent_updates": np.empty(ticks, np.uint32),
        "viability_confirmation_alignment": np.empty(ticks, np.float64),
        "viability_confirmation_score_improvement": np.empty(ticks, np.float64),
        "viability_confirmation_flags": np.empty(ticks, np.uint32),
        "viability_confirmation_shadow": np.empty((ticks, 3), np.float64),
        "viability_hybrid_guard_status": np.empty(ticks, np.uint8),
        "viability_hybrid_guard_executable": np.empty(ticks, np.uint8),
        "viability_hybrid_guard_shadow_admissible": np.empty(ticks, np.uint8),
        "viability_hybrid_guard_support_age_ticks": np.empty(ticks, np.uint32),
        "viability_hybrid_guard_minimum_load_fraction": np.empty(
            ticks, np.float64
        ),
        "viability_hybrid_guard_signed_roll_capture_pressure": np.empty(
            ticks, np.float64
        ),
        "viability_hybrid_guard_flags": np.empty(ticks, np.uint32),
        "viability_planner_query_count": np.empty(ticks, np.uint16),
        "viability_planner_feasibility_seed_reuses": np.empty(ticks, np.uint16),
        "viability_planner_feasibility_projection_sweeps": np.empty(
            ticks, np.uint32
        ),
        "viability_planner_feasibility_halfspace_projections": np.empty(
            ticks, np.uint64
        ),
        "viability_hard_feasibility_witness_transferred": np.empty(ticks, np.uint8),
        "final_feasibility_seed_reused": np.empty(ticks, np.uint8),
        "final_feasibility_prefix_resumed": np.empty(ticks, np.uint8),
        "viability_hard_feasibility_witness_transfer_ns": np.empty(ticks, np.uint64),
        "viability_hard_feasibility_witness_transfer_allocation_calls": np.empty(
            ticks, np.uint64
        ),
        "viability_hard_feasibility_witness_transfer_allocated_bytes": np.empty(
            ticks, np.uint64
        ),
        "viability_planner_zero_pressure": np.empty(ticks, np.float64),
        "viability_planner_candidate_pressure": np.empty(ticks, np.float64),
        "viability_forecast_peak_capture_pressure": np.empty(ticks, np.float64),
        "viability_forecast_peak_sagittal_pressure": np.empty(ticks, np.float64),
        "viability_forecast_terminal_capture_pressure": np.empty(ticks, np.float64),
        "viability_forecast_terminal_sagittal_pressure": np.empty(ticks, np.float64),
        "viability_forecast_terminal_rate_pressure": np.empty(ticks, np.float64),
        "viability_forecast_yaw_pressure": np.empty(ticks, np.float64),
        "viability_forecast_resource_pressure": np.empty(ticks, np.float64),
        "viability_forecast_action_pressure": np.empty(ticks, np.float64),
        "viability_forecast_action_delta_pressure": np.empty(ticks, np.float64),
        "viability_forecast_support_pressure": np.empty(ticks, np.float64),
        "viability_forecast_minimum_capture_margin": np.empty(ticks, np.float64),
        "viability_forecast_path_valid": np.empty(ticks, np.uint8),
        "viability_forecast_path": np.empty((ticks, 8, 9), np.float64),
        "execution_forecast_path_valid": np.empty(ticks, np.uint8),
        "execution_forecast_support_mask": np.empty(ticks, np.uint8),
        "execution_forecast_reduced_state": np.empty((ticks, 8), np.float64),
        "execution_forecast_achieved_acceleration": np.empty(
            (ticks, 4), np.float64
        ),
        "execution_forecast_path": np.empty((ticks, 8, 9), np.float64),
        "execution_forecast_step_ns": np.empty(ticks, np.uint64),
        "execution_forecast_allocation_calls": np.empty(ticks, np.uint64),
        "execution_forecast_allocated_bytes": np.empty(ticks, np.uint64),
        "execution_residual_veto_active": np.empty(ticks, np.uint8),
        "execution_residual_status": np.empty(ticks, np.uint8),
        "execution_residual_certificate_valid": np.empty(ticks, np.uint8),
        "execution_residual_sample_count": np.empty(ticks, np.uint32),
        "execution_residual_maximum_error_ratio": np.empty(ticks, np.float64),
        "execution_residual_flags": np.empty(ticks, np.uint32),
        "execution_residual_step_ns": np.empty(ticks, np.uint64),
        "execution_residual_allocation_calls": np.empty(ticks, np.uint64),
        "execution_residual_allocated_bytes": np.empty(ticks, np.uint64),
        "viability_request": np.empty((ticks, 3), np.float64),
        "fall_safe_mode": np.empty(ticks, np.uint8),
        "fall_safe_primary_authority": np.empty(ticks, np.float64),
        "fall_safe_fresh_command_authority": np.empty(ticks, np.float64),
        "fall_safe_risk": np.empty(ticks, np.float64),
        "fall_safe_reason_flags": np.empty(ticks, np.uint32),
        "torque_utilization": np.empty(ticks, np.float64),
        "controller_step_ns": np.empty(ticks, np.uint64),
        "viability_planner_step_ns": np.empty(ticks, np.uint64),
        "task_pseudoinverse_calls": np.empty(ticks, np.uint16),
        "clipped_steps": np.empty(ticks, np.uint16),
        "task_jacobi_sweeps": np.empty(ticks, np.uint16),
        "feasibility_projection_sweeps": np.empty(ticks, np.uint16),
        "feasibility_halfspace_projections": np.empty(ticks, np.uint32),
        "feasibility_polish_iterations": np.empty(ticks, np.uint16),
        "loop_ns": np.empty(ticks, np.uint64),
        "allocation_calls": np.empty(ticks, np.uint64),
        "allocated_bytes": np.empty(ticks, np.uint64),
        "maximum_abs_qacc": np.empty(ticks, np.float64),
        "external_force_world": np.zeros((ticks, 3), np.float64),
        "application_point_world": np.empty((ticks, 3), np.float64),
    }
    actuator_ids = np.asarray(
        [
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_motor"
            )
            for name in plant.JOINT_ORDER
        ],
        np.int64,
    )
    zero_torque = np.zeros(3, np.float64)
    observed_contact_active = np.ones(2, np.uint8)
    physical_contact_active = np.ones(2, np.uint8)
    wheel_tire_geoms = tuple(
            next(
                geom
                for geom in range(model.ngeom)
                if mujoco.mj_id2name(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    int(model.geom_bodyid[geom]),
                )
                == name
            )
            for name in ("left_wheel_tire", "right_wheel_tire")
    )
    ground_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    contact_force_scratch = np.empty(6, np.float64)
    contact_force_world_scratch = np.empty(3, np.float64)
    contact_torque_world_scratch = np.empty(3, np.float64)
    constraint_impulse_scratch = np.empty(model.nv, np.float64)
    prospective_point_scratch = np.empty(3, np.float64)
    prospective_radial_scratch = np.empty(3, np.float64)
    prospective_linear_jacobian = np.empty((3, model.nv), np.float64)
    prospective_angular_jacobian = np.empty((3, model.nv), np.float64)
    if contact_observation_prestart_samples < 0:
        raise ValueError("contact observation prestart samples must be nonnegative")
    if contact_observation_prestart_age_ticks < 0:
        raise ValueError("contact observation prestart age ticks must be nonnegative")
    if contact_observation_delay_ticks < 0:
        raise ValueError("contact observation delay ticks must be nonnegative")
    for name, value in (
        ("dropout period", contact_observation_dropout_period_ticks),
        ("dropout burst", contact_observation_dropout_burst_ticks),
        ("dropout start tick", contact_observation_dropout_start_tick),
        ("flip period", contact_observation_flip_period_ticks),
        ("flip burst", contact_observation_flip_burst_ticks),
    ):
        if value < 0:
            raise ValueError(f"contact observation {name} must be nonnegative")
    if contact_observation_dropout_burst_ticks > contact_observation_dropout_period_ticks:
        raise ValueError("contact dropout burst cannot exceed its period")
    if contact_observation_flip_burst_ticks > contact_observation_flip_period_ticks:
        raise ValueError("contact flip burst cannot exceed its period")
    if contact_observation_flip_contact not in (-1, 0, 1):
        raise ValueError("contact flip index must be -1, 0, or 1")
    if contact_observation_flip_period_ticks and contact_observation_flip_contact < 0:
        raise ValueError("contact flip profile requires a left/right contact index")
    if measured_contact_admission and contact_observation_prestart_samples:
        observed_contact_active.fill(0)
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            if contact.geom[0] == ground_geom:
                other = contact.geom[1]
            elif contact.geom[1] == ground_geom:
                other = contact.geom[0]
            else:
                continue
            if other == wheel_tire_geoms[0]:
                observed_contact_active[0] = 1
            elif other == wheel_tire_geoms[1]:
                observed_contact_active[1] = 1
        controller.prime_contact_observation(
            observed_contact_active,
            contact_observation_prestart_samples,
            contact_observation_prestart_age_ticks,
        )
    contact_history = np.tile(
        observed_contact_active,
        (contact_observation_delay_ticks + 1, 1),
    )
    force = np.asarray(case.force_world_n, np.float64)
    point_body = np.asarray(case.point_body_m, np.float64)
    gc.collect()
    gc_before = np.asarray([item["collections"] for item in gc.get_stats()])
    rss_before = plant.rss_bytes()
    completed_ticks = ticks
    termination_reason: str | None = None
    terminal_time_s: float | None = None
    terminal_root = np.full(3, np.nan, np.float64)
    terminal_rotation = np.full(3, np.nan, np.float64)
    for tick, timestamp in enumerate(time_s):
        loop_started = time.perf_counter_ns()
        root_position, root_quaternion, root_twist, q, v = plant.read_state(
            model, data
        )
        ground_position = float(np.mean(data.xpos[wheel_bodies, 0]))
        ground_height = float(np.mean(data.xpos[wheel_bodies, 2]))
        if record_physical_prospective_contact_state:
            for wheel, (wheel_body, wheel_geom) in enumerate(
                zip(wheel_bodies, wheel_tire_geoms, strict=True)
            ):
                geom_rotation = data.geom_xmat[wheel_geom].reshape(3, 3)
                cylinder_axis = geom_rotation[:, 2]
                prospective_radial_scratch[:] = (0.0, 0.0, -1.0)
                prospective_radial_scratch -= (
                    float(np.dot(prospective_radial_scratch, cylinder_axis))
                    * cylinder_axis
                )
                radial_norm = float(np.linalg.norm(prospective_radial_scratch))
                if radial_norm <= 1.0e-12:
                    prospective_radial_scratch[:] = (0.0, 0.0, -1.0)
                    radial_norm = 1.0
                prospective_radial_scratch *= (
                    float(model.geom_size[wheel_geom, 0]) / radial_norm
                )
                prospective_point_scratch[:] = data.geom_xpos[wheel_geom]
                prospective_point_scratch += prospective_radial_scratch
                mujoco.mj_jac(
                    model,
                    data,
                    prospective_linear_jacobian,
                    prospective_angular_jacobian,
                    prospective_point_scratch,
                    int(wheel_body),
                )
                traces["physical_wheel_prospective_contact_distance_m"][
                    tick, wheel
                ] = prospective_point_scratch[2]
                traces["physical_wheel_prospective_contact_point_world_m"][
                    tick, wheel
                ] = prospective_point_scratch
                np.dot(
                    prospective_linear_jacobian,
                    data.qvel,
                    out=traces[
                        "physical_wheel_prospective_contact_velocity_m_s"
                    ][tick, wheel],
                )
        physical_contact_active.fill(0)
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            if contact.geom[0] == ground_geom:
                other = contact.geom[1]
            elif contact.geom[1] == ground_geom:
                other = contact.geom[0]
            else:
                continue
            if other == wheel_tire_geoms[0]:
                wheel = 0
            elif other == wheel_tire_geoms[1]:
                wheel = 1
            else:
                continue
            physical_contact_active[wheel] = 1
            if record_physical_contact_prestate:
                traces["physical_wheel_contact_prestate_available"][tick, wheel] = 1
                traces["physical_wheel_contact_distance_m"][tick, wheel] = min(
                    traces["physical_wheel_contact_distance_m"][tick, wheel],
                    float(contact.dist),
                )
                address = int(contact.efc_address)
                dimensions = min(int(contact.dim), 3)
                if address >= 0 and dimensions > 0:
                    traces[
                        "physical_wheel_contact_relative_velocity_m_s"
                    ][tick, wheel, :dimensions] = data.efc_vel[
                        address : address + dimensions
                    ]
        history_slot = tick % len(contact_history)
        contact_history[history_slot] = physical_contact_active
        delayed_tick = tick - contact_observation_delay_ticks
        if delayed_tick >= 0:
            observed_contact_active[:] = contact_history[
                delayed_tick % len(contact_history)
            ]
        if (
            contact_observation_flip_period_ticks
            and tick % contact_observation_flip_period_ticks
            < contact_observation_flip_burst_ticks
        ):
            observed_contact_active[contact_observation_flip_contact] ^= 1
        contact_observation_available = not (
            contact_observation_dropout_period_ticks
            and tick >= contact_observation_dropout_start_tick
            and (tick - contact_observation_dropout_start_tick)
            % contact_observation_dropout_period_ticks
            < contact_observation_dropout_burst_ticks
        )
        result = controller.solve(
            root_position,
            root_quaternion,
            root_twist,
            q,
            v,
            ground_position,
            ground_height,
            observed_contact_active if measured_contact_admission else None,
            observed_contact_available=contact_observation_available,
            observed_contact_age_ticks=contact_observation_delay_ticks,
        )
        data.ctrl[actuator_ids] = result["torque"]
        data.qfrc_applied.fill(0.0)
        data.xfrc_applied.fill(0.0)
        rotation = data.xmat[application_body].reshape(3, 3)
        point_world = data.xpos[application_body] + rotation @ point_body
        disturbed = any(
            PUSH_START_S + pulse * case.repeat_interval_s
            <= timestamp
            < PUSH_START_S + pulse * case.repeat_interval_s + case.duration_s
            for pulse in range(case.repetitions)
        )
        if disturbed and case.force_magnitude_n > 0.0:
            mujoco.mj_applyFT(
                model,
                data,
                force,
                zero_torque,
                point_world,
                application_body,
                data.qfrc_applied,
            )
            traces["external_force_world"][tick] = force
        traces["application_point_world"][tick] = point_world
        physical_wheel_contact_impulse = traces[
            "physical_wheel_contact_impulse_ns"
        ][tick]
        physical_wheel_contact_impulse_world = traces[
            "physical_wheel_contact_impulse_world_ns"
        ][tick]
        physical_wheel_contact_position_m_ns = traces[
            "physical_wheel_contact_position_m_ns"
        ][tick]
        physical_wheel_contact_moment_world_origin = traces[
            "physical_wheel_contact_moment_world_origin_nms"
        ][tick]
        physical_constraint_impulse = traces[
            "physical_constraint_generalized_impulse_ns"
        ][tick]
        for _ in range(substeps):
            mujoco.mj_step(model, data)
            if record_physical_contact_impulses:
                np.multiply(
                    data.qfrc_constraint,
                    plant.PHYSICS_DT,
                    out=constraint_impulse_scratch,
                )
                np.add(
                    physical_constraint_impulse,
                    constraint_impulse_scratch,
                    out=physical_constraint_impulse,
                )
                for contact_index in range(data.ncon):
                    contact = data.contact[contact_index]
                    if contact.geom[0] == ground_geom:
                        other = contact.geom[1]
                    elif contact.geom[1] == ground_geom:
                        other = contact.geom[0]
                    else:
                        continue
                    if other == wheel_tire_geoms[0]:
                        wheel = 0
                    elif other == wheel_tire_geoms[1]:
                        wheel = 1
                    else:
                        continue
                    mujoco.mj_contactForce(
                        model,
                        data,
                        contact_index,
                        contact_force_scratch,
                    )
                    normal_impulse_ns = (
                        abs(float(contact_force_scratch[0]))
                        * plant.PHYSICS_DT
                    )
                    physical_wheel_contact_impulse[wheel, 0] += normal_impulse_ns
                    for world_axis in range(3):
                        physical_wheel_contact_position_m_ns[
                            wheel, world_axis
                        ] += float(contact.pos[world_axis]) * normal_impulse_ns
                    physical_wheel_contact_impulse[wheel, 1] += (
                        math.hypot(
                            float(contact_force_scratch[1]),
                            float(contact_force_scratch[2]),
                        )
                        * plant.PHYSICS_DT
                    )
                    # MuJoCo stores contact-frame axes as matrix rows. Convert
                    # the signed force to world coordinates before summing the
                    # complete control-interval impulse; componentwise
                    # magnitudes are taken only by the evaluator.
                    for world_axis in range(3):
                        contact_force_world_scratch[world_axis] = (
                            float(contact.frame[world_axis])
                            * float(contact_force_scratch[0])
                            + float(contact.frame[3 + world_axis])
                            * float(contact_force_scratch[1])
                            + float(contact.frame[6 + world_axis])
                            * float(contact_force_scratch[2])
                        )
                        physical_wheel_contact_impulse_world[
                            wheel, world_axis
                        ] += (
                            contact_force_world_scratch[world_axis]
                            * plant.PHYSICS_DT
                        )
                        contact_torque_world_scratch[world_axis] = (
                            float(contact.frame[world_axis])
                            * float(contact_force_scratch[3])
                            + float(contact.frame[3 + world_axis])
                            * float(contact_force_scratch[4])
                            + float(contact.frame[6 + world_axis])
                            * float(contact_force_scratch[5])
                        )
                    # Preserve the complete distributed-contact wrench about
                    # a fixed world origin. A consumer can translate it to any
                    # declared point with M_p = M_0 - p x J. This includes the
                    # contact solver's free torque instead of silently
                    # collapsing the patch to one force-only centroid.
                    px = float(contact.pos[0])
                    py = float(contact.pos[1])
                    pz = float(contact.pos[2])
                    fx = float(contact_force_world_scratch[0])
                    fy = float(contact_force_world_scratch[1])
                    fz = float(contact_force_world_scratch[2])
                    physical_wheel_contact_moment_world_origin[wheel, 0] += (
                        py * fz - pz * fy + contact_torque_world_scratch[0]
                    ) * plant.PHYSICS_DT
                    physical_wheel_contact_moment_world_origin[wheel, 1] += (
                        pz * fx - px * fz + contact_torque_world_scratch[1]
                    ) * plant.PHYSICS_DT
                    physical_wheel_contact_moment_world_origin[wheel, 2] += (
                        px * fy - py * fx + contact_torque_world_scratch[2]
                    ) * plant.PHYSICS_DT
        traces["maximum_abs_qacc"][tick] = np.max(np.abs(data.qacc))
        traces["root_position"][tick] = root_position
        traces["root_quaternion_wxyz"][tick] = root_quaternion
        traces["root_twist"][tick] = root_twist
        traces["rotation_vector"][tick] = plant.quaternion_rotation_vector(
            root_quaternion
        )
        traces["q"][tick] = q
        traces["v"][tick] = v
        traces["torque"][tick] = result["torque"]
        traces["generalized_acceleration"][tick] = result[
            "generalized_acceleration"
        ]
        traces["wbc_normal_force"][tick] = result["normal_force"]
        traces["status"][tick] = result["status"]
        traces["primary_status"][tick] = result["primary_status"]
        for field in (
            "primary_wbc_step_ns",
            "primary_task_pseudoinverse_calls",
            "primary_clipped_steps",
            "primary_task_jacobi_sweeps",
            "primary_feasibility_projection_sweeps",
            "primary_feasibility_halfspace_projections",
        ):
            traces[field][tick] = result[field]
        traces["support_contingency_admitted"][tick] = result[
            "support_contingency_admitted"
        ]
        traces["support_contingency_armed"][tick] = result[
            "support_contingency_armed"
        ]
        traces["support_contingency_requested"][tick] = result[
            "support_contingency_requested"
        ]
        traces["support_contingency_selected"][tick] = result[
            "support_contingency_selected"
        ]
        traces["support_contingency_realization_fallback"][tick] = result[
            "support_contingency_realization_fallback"
        ]
        traces["support_contingency_status"][tick] = result[
            "support_contingency_status"
        ]
        traces["support_contingency_mode"][tick] = result[
            "support_contingency_mode"
        ]
        traces["support_contingency_support_mask"][tick] = result[
            "support_contingency_support_mask"
        ]
        traces["support_contingency_maximum_constraint_violation"][tick] = result[
            "support_contingency_maximum_constraint_violation"
        ]
        traces["support_contingency_author_step_ns"][tick] = result[
            "support_contingency_author_step_ns"
        ]
        traces["support_contingency_wbc_step_ns"][tick] = result[
            "support_contingency_wbc_step_ns"
        ]
        traces["support_contingency_diagnostics"][tick] = result[
            "support_contingency_diagnostics"
        ]
        traces["support_contingency_candidate_generalized_acceleration"][tick] = result[
            "support_contingency_candidate_generalized_acceleration"
        ]
        traces["support_contingency_candidate_torque"][tick] = result[
            "support_contingency_candidate_torque"
        ]
        traces["support_contingency_candidate_power_w"][tick] = result[
            "support_contingency_candidate_power_w"
        ]
        traces["support_contingency_primary_torque"][tick] = result[
            "support_contingency_primary_torque"
        ]
        traces["support_contingency_primary_source_fresh"][tick] = result[
            "support_contingency_primary_source_fresh"
        ]
        traces["support_contingency_primary_power_w"][tick] = result[
            "support_contingency_primary_power_w"
        ]
        traces["support_contingency_incremental_power_w"][tick] = result[
            "support_contingency_incremental_power_w"
        ]
        traces["support_contingency_forecast_guard_passed"][tick] = result[
            "support_contingency_forecast_guard_passed"
        ]
        traces["support_contingency_forecast_baseline_score"][tick] = result[
            "support_contingency_forecast_baseline_score"
        ]
        traces["support_contingency_forecast_candidate_score"][tick] = result[
            "support_contingency_forecast_candidate_score"
        ]
        traces["support_contingency_forecast_step_ns"][tick] = result[
            "support_contingency_forecast_step_ns"
        ]
        traces["command_age_steps"][tick] = result["command_age_steps"]
        traces["contact_count"][tick] = data.ncon
        traces["physical_contact_active"][tick] = physical_contact_active
        traces["observed_contact_active"][tick] = observed_contact_active
        traces["contact_observation_available"][tick] = result[
            "contact_observation_available"
        ]
        traces["contact_observation_status"][tick] = result[
            "contact_observation_status"
        ]
        traces["contact_observation_provenance"][tick] = result[
            "contact_observation_provenance"
        ]
        traces["contact_observation_age_ns"][tick] = result[
            "contact_observation_age_ns"
        ]
        traces["contact_observation_flags"][tick] = result[
            "contact_observation_flags"
        ]
        traces["admitted_contact_active"][tick] = (
            result["support_active_left"],
            result["support_active_right"],
        )
        traces["support_transition_count"][tick] = result[
            "support_transition_count"
        ]
        for field in (
            "contact_program_authority_selection",
            "contact_program_authority_executable",
            "contact_program_authority_transition_pending",
            "contact_program_authority_activation_pending",
            "contact_program_authority_deactivation_pending",
            "contact_program_authority_masks_consistent",
            "contact_program_authority_lease_status",
            "contact_program_authority_lease_provenance",
            "contact_program_authority_authoring_mask",
            "contact_program_authority_stable_mask",
            "contact_program_authority_hard_mask",
            "contact_program_authority_transition_count",
            "contact_program_authority_transition_current_enabled",
            "contact_program_authority_age_ticks",
            "contact_program_authority_remaining_ticks",
            "contact_program_authority_flags",
            "inexact_observation_authority_selector_queried",
            "inexact_observation_authority_selector_selected_index",
            "inexact_observation_authority_selector_authority_q15",
            "inexact_observation_authority_selector_selected_score",
            "inexact_observation_authority_selector_zero_score",
            "inexact_observation_authority_selector_full_score",
            "inexact_observation_authority_selector_improvement",
            "inexact_observation_authority_selector_candidate_scores",
            "inexact_observation_authority_selector_step_ns",
            "inexact_observation_authority_selector_allocation_calls",
            "inexact_observation_authority_selector_allocated_bytes",
            "inexact_observation_terminal_selector_queried",
            "inexact_observation_terminal_zero_effort_available",
            "inexact_observation_terminal_zero_effort_acceleration",
            "inexact_observation_terminal_zero_effort_step_ns",
            "inexact_observation_terminal_zero_effort_allocation_calls",
            "inexact_observation_terminal_zero_effort_allocated_bytes",
            "inexact_observation_terminal_hypothesis_acceleration",
            "inexact_observation_terminal_hypothesis_available",
            "inexact_observation_terminal_hypothesis_diagnostics",
            "inexact_observation_terminal_hypothesis_envelopes",
            "inexact_observation_terminal_hypothesis_torque",
            "inexact_observation_terminal_hypothesis_effort_utilization",
            "inexact_observation_terminal_hypothesis_query_step_ns",
            "inexact_observation_terminal_hypothesis_query_allocation_calls",
            "inexact_observation_terminal_hypothesis_query_allocated_bytes",
            "inexact_observation_terminal_hypothesis_aggregate_step_ns",
            "inexact_observation_terminal_hypothesis_aggregate_allocation_calls",
            "inexact_observation_terminal_hypothesis_aggregate_allocated_bytes",
            "inexact_observation_terminal_selector_action",
            "inexact_observation_terminal_selector_retained_available",
            "inexact_observation_terminal_selector_support_free_available",
            "inexact_observation_terminal_selector_time_to_impact_s",
            "inexact_observation_terminal_selector_vertical_specific_energy_j_kg",
            "inexact_observation_terminal_selector_candidate_diagnostics",
            "inexact_observation_terminal_selector_state",
            "inexact_observation_terminal_selector_root_acceleration",
            "inexact_observation_terminal_selector_joint_acceleration",
            "inexact_observation_terminal_selector_effort_utilization",
            "inexact_observation_terminal_selector_selection_diagnostics",
            "inexact_observation_terminal_selector_maximum_harm_pressures",
            "inexact_observation_terminal_selector_aggregate_scores",
            "inexact_observation_terminal_selector_step_ns",
            "inexact_observation_terminal_selector_allocation_calls",
            "inexact_observation_terminal_selector_allocated_bytes",
        ):
            traces[field][tick] = result[field]
        traces["contact_command_lease_status"][tick] = result[
            "contact_command_lease_status"
        ]
        traces["contact_command_lease_executable"][tick] = result[
            "contact_command_lease_executable"
        ]
        traces["contact_command_lease_age_ticks"][tick] = result[
            "contact_command_lease_age_ticks"
        ]
        traces["contact_command_lease_remaining_ticks"][tick] = result[
            "contact_command_lease_remaining_ticks"
        ]
        traces["contact_command_lease_flags"][tick] = result[
            "contact_command_lease_flags"
        ]
        traces["capture_pressure"][tick] = result["capture_pressure"]
        traces["station_authority"][tick] = result["station_authority"]
        traces["station_error"][tick] = result["station_error"]
        traces["heading"][tick] = result["heading"]
        traces["lateral_capture_error"][tick] = result["lateral_capture_error"]
        traces["planar_capture_pressure"][tick] = result["planar_capture_pressure"]
        traces["commanded_yaw_rate"][tick] = result["commanded_yaw_rate"]
        traces["lateral_support_margin"][tick] = result["lateral_support_margin"]
        traces["lateral_dcm"][tick] = result["lateral_dcm"]
        traces["viability_margin"][tick] = result["viability_margin"]
        traces["commanded_zmp"][tick] = result["commanded_zmp"]
        traces["commanded_lateral_acceleration"][tick] = result[
            "commanded_lateral_acceleration"
        ]
        traces["commanded_bank_angle"][tick] = result["commanded_bank_angle"]
        traces["commanded_roll_acceleration"][tick] = result[
            "commanded_roll_acceleration"
        ]
        traces["viability_activation_pressure"][tick] = result[
            "viability_activation_pressure"
        ]
        traces["viability_zmp_was_saturated"][tick] = result[
            "viability_zmp_was_saturated"
        ]
        traces["viability_verified_scale"][tick] = result[
            "viability_verified_scale"
        ]
        traces["viability_verification_queries"][tick] = result[
            "viability_verification_queries"
        ]
        traces["viability_support_pressure"][tick] = result[
            "viability_support_pressure"
        ]
        traces["viability_support_active"][tick] = result[
            "viability_support_active"
        ]
        traces["viability_coordinate_queries"][tick] = result[
            "viability_coordinate_queries"
        ]
        traces["viability_coordinate_score"][tick] = result[
            "viability_coordinate_score"
        ]
        traces["viability_coordinate_target"][tick] = (
            result["viability_coordinate_target_roll"],
            result["viability_coordinate_target_lateral"],
            result["viability_coordinate_target_yaw"],
        )
        traces["viability_coordinate_request"][tick] = (
            result["viability_coordinate_request_roll"],
            result["viability_coordinate_request_lateral"],
            result["viability_coordinate_request_yaw"],
        )
        traces["viability_request_status"][tick] = result["viability_request_status"]
        traces["viability_request_active"][tick] = result["viability_request_active"]
        traces["viability_request_executable"][tick] = result[
            "viability_request_executable"
        ]
        traces["viability_request_slew_limited"][tick] = result[
            "viability_request_slew_limited"
        ]
        traces["viability_request_age_ticks"][tick] = result[
            "viability_request_age_ticks"
        ]
        traces["viability_request_remaining_ticks"][tick] = result[
            "viability_request_remaining_ticks"
        ]
        traces["viability_planner_update"][tick] = result[
            "viability_planner_update"
        ]
        for field in (
            "viability_confirmation_status",
            "viability_confirmation_executable",
            "viability_confirmation_has_shadow",
            "viability_confirmation_support_mask",
            "viability_confirmation_consistent_updates",
            "viability_confirmation_alignment",
            "viability_confirmation_score_improvement",
            "viability_confirmation_flags",
            "viability_confirmation_shadow",
            "viability_hybrid_guard_status",
            "viability_hybrid_guard_executable",
            "viability_hybrid_guard_shadow_admissible",
            "viability_hybrid_guard_support_age_ticks",
            "viability_hybrid_guard_minimum_load_fraction",
            "viability_hybrid_guard_signed_roll_capture_pressure",
            "viability_hybrid_guard_flags",
        ):
            traces[field][tick] = result[field]
        traces["viability_forecast_path_valid"][tick] = result[
            "viability_forecast_path_valid"
        ]
        traces["viability_forecast_path"][tick] = result[
            "viability_forecast_path"
        ]
        for field in (
            "execution_forecast_path_valid",
            "execution_forecast_support_mask",
            "execution_forecast_reduced_state",
            "execution_forecast_achieved_acceleration",
            "execution_forecast_path",
            "execution_forecast_step_ns",
            "execution_forecast_allocation_calls",
            "execution_forecast_allocated_bytes",
            "execution_residual_veto_active",
            "execution_residual_status",
            "execution_residual_certificate_valid",
            "execution_residual_sample_count",
            "execution_residual_maximum_error_ratio",
            "execution_residual_flags",
            "execution_residual_step_ns",
            "execution_residual_allocation_calls",
            "execution_residual_allocated_bytes",
        ):
            traces[field][tick] = result[field]
        traces["viability_planner_query_count"][tick] = result[
            "viability_planner_query_count"
        ]
        traces["viability_planner_feasibility_seed_reuses"][tick] = result[
            "viability_planner_feasibility_seed_reuses"
        ]
        traces["viability_planner_feasibility_projection_sweeps"][tick] = result[
            "viability_planner_feasibility_projection_sweeps"
        ]
        traces["viability_planner_feasibility_halfspace_projections"][tick] = result[
            "viability_planner_feasibility_halfspace_projections"
        ]
        traces["viability_hard_feasibility_witness_transferred"][tick] = result[
            "viability_hard_feasibility_witness_transferred"
        ]
        traces["final_feasibility_seed_reused"][tick] = result[
            "final_feasibility_seed_reused"
        ]
        traces["final_feasibility_prefix_resumed"][tick] = result[
            "final_feasibility_prefix_resumed"
        ]
        traces["viability_hard_feasibility_witness_transfer_ns"][tick] = result[
            "viability_hard_feasibility_witness_transfer_ns"
        ]
        traces[
            "viability_hard_feasibility_witness_transfer_allocation_calls"
        ][tick] = result[
            "viability_hard_feasibility_witness_transfer_allocation_calls"
        ]
        traces["viability_hard_feasibility_witness_transfer_allocated_bytes"][
            tick
        ] = result[
            "viability_hard_feasibility_witness_transfer_allocated_bytes"
        ]
        traces["viability_planner_zero_pressure"][tick] = result[
            "viability_planner_zero_pressure"
        ]
        traces["viability_planner_candidate_pressure"][tick] = result[
            "viability_planner_candidate_pressure"
        ]
        for field in (
            "viability_forecast_peak_capture_pressure",
            "viability_forecast_peak_sagittal_pressure",
            "viability_forecast_terminal_capture_pressure",
            "viability_forecast_terminal_sagittal_pressure",
            "viability_forecast_terminal_rate_pressure",
            "viability_forecast_yaw_pressure",
            "viability_forecast_resource_pressure",
            "viability_forecast_action_pressure",
            "viability_forecast_action_delta_pressure",
            "viability_forecast_support_pressure",
            "viability_forecast_minimum_capture_margin",
        ):
            traces[field][tick] = result[field]
        traces["viability_request"][tick] = result["viability_request"]
        traces["fall_safe_mode"][tick] = result["fall_safe_mode"]
        traces["fall_safe_primary_authority"][tick] = result[
            "fall_safe_primary_authority"
        ]
        traces["fall_safe_fresh_command_authority"][tick] = result[
            "fall_safe_fresh_command_authority"
        ]
        traces["fall_safe_risk"][tick] = result["fall_safe_risk"]
        traces["fall_safe_reason_flags"][tick] = result["fall_safe_reason_flags"]
        traces["torque_utilization"][tick] = result["torque_utilization"]
        traces["controller_step_ns"][tick] = result["step_ns"]
        traces["viability_planner_step_ns"][tick] = result[
            "viability_planner_step_ns"
        ]
        for field in (
            "task_pseudoinverse_calls",
            "clipped_steps",
            "task_jacobi_sweeps",
            "feasibility_projection_sweeps",
            "feasibility_halfspace_projections",
            "feasibility_polish_iterations",
        ):
            traces[field][tick] = result[field]
        traces["allocation_calls"][tick] = result["allocation_calls"]
        traces["allocated_bytes"][tick] = result["allocated_bytes"]
        traces["loop_ns"][tick] = time.perf_counter_ns() - loop_started
        post_root, post_quaternion, post_root_twist, _, post_v = plant.read_state(
            model, data
        )
        traces["post_root_twist"][tick] = post_root_twist
        traces["post_v"][tick] = post_v
        post_rotation = plant.quaternion_rotation_vector(post_quaternion)
        warning_count = sum(warning.number for warning in data.warning)
        post_finite = bool(
            np.all(np.isfinite(data.qpos))
            and np.all(np.isfinite(data.qvel))
            and np.all(np.isfinite(data.qacc))
        )
        post_fallen = bool(
            float(post_root[2]) < FALL_HEIGHT_M
            or float(np.linalg.norm(post_rotation[:2])) > FALL_TILT_RAD
        )
        if post_fallen or not post_finite or warning_count:
            completed_ticks = tick + 1
            termination_reason = "fall" if post_fallen else "numeric_fault"
            terminal_time_s = float(timestamp + plant.CONTROL_DT)
            terminal_root[:] = post_root
            terminal_rotation[:] = post_rotation
            break
    if completed_ticks != ticks:
        for field, value in tuple(traces.items()):
            if isinstance(value, np.ndarray) and value.shape[0] == ticks:
                traces[field] = value[:completed_ticks].copy()
    else:
        final_root, final_quaternion, _, _, _ = plant.read_state(model, data)
        terminal_time_s = duration_s
        terminal_root[:] = final_root
        terminal_rotation[:] = plant.quaternion_rotation_vector(final_quaternion)
    gc_after = np.asarray([item["collections"] for item in gc.get_stats()])
    traces["gc_collections"] = int(np.sum(gc_after - gc_before))
    traces["rss_delta_bytes"] = plant.rss_bytes() - rss_before
    traces["mujoco_warning_counts"] = np.asarray(
        [warning.number for warning in data.warning], np.uint64
    )
    traces["mujoco_warning_lastinfo"] = np.asarray(
        [warning.lastinfo for warning in data.warning], np.uint64
    )
    traces["configured_ticks"] = ticks
    traces["termination_reason"] = termination_reason
    traces["terminal_time_s"] = terminal_time_s
    traces["terminal_root_position"] = terminal_root
    traces["terminal_rotation_vector"] = terminal_rotation
    return traces


def first_stable_window(mask: np.ndarray, start: int, count: int) -> int | None:
    for index in range(start, len(mask) - count + 1):
        if np.all(mask[index : index + count]):
            return index
    return None


def summarize(
    case: CaseSpec, trace: dict[str, Any], duration_s: float
) -> dict[str, Any]:
    time_s = np.asarray(trace["time_s"])
    root = np.asarray(trace["root_position"])
    twist = np.asarray(trace["root_twist"])
    rotation = np.asarray(trace["rotation_vector"])
    status = np.asarray(trace["status"])
    tilt = np.linalg.norm(rotation[:, :2], axis=1)
    angular_speed = np.linalg.norm(twist[:, :3], axis=1)
    displacement = root - root[0]
    translation = np.linalg.norm(displacement, axis=1)
    # The optional contact-prestate trace uses +inf as an explicit
    # unavailable-distance sentinel.  Validate that typed sentinel against its
    # availability mask instead of collapsing missing evidence into a numeric
    # plant fault.
    prestate_available = np.asarray(
        trace["physical_wheel_contact_prestate_available"], np.bool_
    )
    prestate_distance = np.asarray(
        trace["physical_wheel_contact_distance_m"], np.float64
    )
    finite = all(
        np.all(np.isfinite(value))
        for name, value in trace.items()
        if isinstance(value, np.ndarray)
        and value.dtype.kind == "f"
        and name != "physical_wheel_contact_distance_m"
    ) and bool(
        np.all(np.isfinite(prestate_distance[prestate_available]))
        and np.all(np.isposinf(prestate_distance[~prestate_available]))
    )
    warning_counts = np.asarray(trace["mujoco_warning_counts"])
    warning_lastinfo = np.asarray(trace["mujoco_warning_lastinfo"])
    warning_count = int(np.sum(warning_counts))
    termination_reason = trace["termination_reason"]
    terminal_root = np.asarray(trace["terminal_root_position"])
    terminal_rotation = np.asarray(trace["terminal_rotation_vector"])
    terminal_available = bool(np.all(np.isfinite(terminal_root)))
    terminal_tilt = (
        float(np.linalg.norm(terminal_rotation[:2])) if terminal_available else 0.0
    )
    terminal_translation = (
        float(np.linalg.norm(terminal_root - root[0])) if terminal_available else 0.0
    )
    numeric_fault = bool(
        termination_reason == "numeric_fault" or not finite or warning_count > 0
    )
    fallen = bool(
        termination_reason == "fall"
        or np.min(root[:, 2]) < FALL_HEIGHT_M
        or np.max(tilt) > FALL_TILT_RAD
    )
    admitted = np.isin(status, [0, 1])
    post_startup = time_s > plant.STARTUP_TRANSIENT_S
    contacts = np.asarray(trace["contact_count"])
    contactless = contacts == 0
    push_end = case.final_push_end_s
    settle_start = int(np.searchsorted(time_s, push_end, side="left"))
    window = int(round(RECOVERY_WINDOW_S / plant.CONTROL_DT))
    recovered_mask = (
        (tilt < math.radians(2.0))
        & (angular_speed < 0.15)
        & (np.abs(displacement[:, 0]) < 0.05)
        & (np.abs(displacement[:, 1]) < 0.03)
    )
    recovered_index = first_stable_window(recovered_mask, settle_start, window)
    recovery_time = (
        None if recovered_index is None else float(time_s[recovered_index] - push_end)
    )
    if fallen:
        recovery_time = None
    observable = case.force_magnitude_n == 0.0 or bool(
        np.max(translation) > 1.0e-3 or np.max(tilt) > math.radians(0.1)
    )
    longest_flight = plant.longest_true_run(contactless) * plant.CONTROL_DT
    allocation_free = bool(
        np.sum(np.asarray(trace["allocation_calls"])) == 0
        and np.sum(np.asarray(trace["allocated_bytes"])) == 0
    )
    loop_overruns = int(
        np.sum(np.asarray(trace["loop_ns"]) > int(plant.CONTROL_DT * 1.0e9))
    )
    post_startup_nonadmitted_steps = int(np.sum(~admitted & post_startup))
    warnings = {
        name: {"count": int(count), "lastinfo": int(lastinfo)}
        for name, count, lastinfo in zip(
            MUJOCO_WARNING_NAMES, warning_counts, warning_lastinfo, strict=True
        )
        if count
    }
    qualified = bool(
        not numeric_fault
        and not fallen
        and observable
        and (case.force_magnitude_n == 0.0 or recovery_time is not None)
        and post_startup_nonadmitted_steps == 0
        and longest_flight <= 0.25
        and allocation_free
        and loop_overruns == 0
    )
    outcome = (
        "NUMERIC_FAULT"
        if numeric_fault
        else "FALL"
        if fallen
        else "RECOVERED"
        if recovery_time is not None or case.force_magnitude_n == 0.0
        else "UNSETTLED"
    )
    qualification_blockers: list[str] = []
    if numeric_fault:
        qualification_blockers.append(
            "numeric:" + ",".join(warnings) if warnings else "numeric:nonfinite"
        )
    if fallen:
        qualification_blockers.append("fall")
    if case.force_magnitude_n > 0.0 and recovery_time is None:
        qualification_blockers.append("no recovery")
    if post_startup_nonadmitted_steps:
        qualification_blockers.append(
            f"WBC nonadmitted×{post_startup_nonadmitted_steps}"
        )
    if longest_flight > 0.25:
        qualification_blockers.append(f"contactless {longest_flight:.3f}s")
    if not allocation_free:
        qualification_blockers.append("Rust allocation")
    if loop_overruns:
        qualification_blockers.append(f"loop overrun×{loop_overruns}")
    return {
        "case": asdict(case),
        "force_magnitude_n": case.force_magnitude_n,
        "impulse_ns": case.impulse_ns,
        "finite": finite,
        "numeric_fault": numeric_fault,
        "mujoco_warning_count": warning_count,
        "mujoco_warning_counts": warning_counts.tolist(),
        "mujoco_warning_lastinfo": warning_lastinfo.tolist(),
        "mujoco_warnings": warnings,
        "maximum_abs_qacc": float(np.max(np.asarray(trace["maximum_abs_qacc"]))),
        "qualification_blockers": qualification_blockers,
        "observable": observable,
        "outcome": outcome,
        "qualified": qualified,
        "fell": fallen,
        "termination_reason": termination_reason,
        "terminal_time_s": trace["terminal_time_s"],
        "executed_ticks": len(time_s),
        "configured_ticks": int(trace["configured_ticks"]),
        "recovery_time_s": recovery_time,
        "maximum_translation_m": max(float(np.max(translation)), terminal_translation),
        "maximum_abs_x_m": float(np.max(np.abs(displacement[:, 0]))),
        "maximum_abs_y_m": float(np.max(np.abs(displacement[:, 1]))),
        "maximum_tilt_deg": math.degrees(max(float(np.max(tilt)), terminal_tilt)),
        "maximum_abs_roll_deg": math.degrees(float(np.max(np.abs(rotation[:, 0])))),
        "maximum_abs_pitch_deg": math.degrees(float(np.max(np.abs(rotation[:, 1])))),
        "minimum_root_height_m": min(
            float(np.min(root[:, 2])),
            float(terminal_root[2]) if terminal_available else math.inf,
        ),
        "maximum_capture_pressure": float(np.max(np.asarray(trace["capture_pressure"]))),
        "minimum_station_authority": float(np.min(np.asarray(trace["station_authority"]))),
        "final_station_error_m": float(np.asarray(trace["station_error"])[-1]),
        "maximum_torque_utilization": float(np.max(np.asarray(trace["torque_utilization"]))),
        "post_startup_nonadmitted_steps": post_startup_nonadmitted_steps,
        "maximum_contactless_duration_s": longest_flight,
        "allocation_free": allocation_free,
        "python_gc_collections": int(trace["gc_collections"]),
        "rss_delta_bytes": int(trace["rss_delta_bytes"]),
        "loop_overruns": loop_overruns,
        "controller_step_ns": distribution(np.asarray(trace["controller_step_ns"])),
        "viability_planner_step_ns": distribution(
            np.asarray(trace["viability_planner_step_ns"])
        ),
        "final_wbc_step_ns": distribution(
            np.asarray(trace["controller_step_ns"])
            - np.asarray(trace["viability_planner_step_ns"])
        ),
        "loop_ns": distribution(np.asarray(trace["loop_ns"])),
        "duration_s": duration_s,
    }


def semantic_trace_equal(
    left: dict[str, Any], right: dict[str, Any]
) -> bool:
    fields = (
        "time_s",
        "root_position",
        "root_quaternion_wxyz",
        "root_twist",
        "post_root_twist",
        "rotation_vector",
        "q",
        "v",
        "post_v",
        "torque",
        "wbc_normal_force",
        "physical_wheel_contact_impulse_ns",
        "physical_wheel_contact_impulse_world_ns",
        "physical_wheel_contact_position_m_ns",
        "physical_wheel_contact_moment_world_origin_nms",
        "physical_constraint_generalized_impulse_ns",
        "physical_wheel_contact_prestate_available",
        "physical_wheel_contact_distance_m",
        "physical_wheel_contact_relative_velocity_m_s",
        "physical_wheel_prospective_contact_distance_m",
        "physical_wheel_prospective_contact_point_world_m",
        "physical_wheel_prospective_contact_velocity_m_s",
        "status",
        "primary_status",
        "primary_task_pseudoinverse_calls",
        "primary_clipped_steps",
        "primary_task_jacobi_sweeps",
        "primary_feasibility_projection_sweeps",
        "primary_feasibility_halfspace_projections",
        "support_contingency_admitted",
        "support_contingency_armed",
        "support_contingency_requested",
        "support_contingency_selected",
        "support_contingency_realization_fallback",
        "support_contingency_status",
        "support_contingency_mode",
        "support_contingency_support_mask",
        "support_contingency_maximum_constraint_violation",
        "support_contingency_diagnostics",
        "support_contingency_candidate_generalized_acceleration",
        "support_contingency_candidate_torque",
        "support_contingency_primary_torque",
        "support_contingency_primary_source_fresh",
        "support_contingency_candidate_power_w",
        "support_contingency_primary_power_w",
        "support_contingency_incremental_power_w",
        "support_contingency_forecast_guard_passed",
        "support_contingency_forecast_baseline_score",
        "support_contingency_forecast_candidate_score",
        "command_age_steps",
        "contact_count",
        "physical_contact_active",
        "observed_contact_active",
        "contact_observation_available",
        "contact_observation_status",
        "contact_observation_provenance",
        "contact_observation_age_ns",
        "contact_observation_flags",
        "admitted_contact_active",
        "support_transition_count",
        "contact_program_authority_selection",
        "contact_program_authority_executable",
        "contact_program_authority_transition_pending",
        "contact_program_authority_activation_pending",
        "contact_program_authority_deactivation_pending",
        "contact_program_authority_masks_consistent",
        "contact_program_authority_lease_status",
        "contact_program_authority_lease_provenance",
        "contact_program_authority_authoring_mask",
        "contact_program_authority_stable_mask",
        "contact_program_authority_hard_mask",
        "contact_program_authority_transition_count",
        "contact_program_authority_transition_current_enabled",
        "contact_program_authority_age_ticks",
        "contact_program_authority_remaining_ticks",
        "contact_program_authority_flags",
        "inexact_observation_authority_selector_queried",
        "inexact_observation_authority_selector_selected_index",
        "inexact_observation_authority_selector_authority_q15",
        "inexact_observation_authority_selector_selected_score",
        "inexact_observation_authority_selector_zero_score",
        "inexact_observation_authority_selector_full_score",
        "inexact_observation_authority_selector_improvement",
        "inexact_observation_authority_selector_candidate_scores",
        "inexact_observation_authority_selector_allocation_calls",
        "inexact_observation_authority_selector_allocated_bytes",
        "inexact_observation_terminal_selector_queried",
        "inexact_observation_terminal_selector_action",
        "inexact_observation_terminal_selector_retained_available",
        "inexact_observation_terminal_selector_support_free_available",
        "inexact_observation_terminal_selector_time_to_impact_s",
        "inexact_observation_terminal_selector_vertical_specific_energy_j_kg",
        "inexact_observation_terminal_selector_candidate_diagnostics",
        "inexact_observation_terminal_selector_state",
        "inexact_observation_terminal_selector_root_acceleration",
        "inexact_observation_terminal_selector_joint_acceleration",
        "inexact_observation_terminal_selector_effort_utilization",
        "inexact_observation_terminal_selector_selection_diagnostics",
        "inexact_observation_terminal_selector_maximum_harm_pressures",
        "inexact_observation_terminal_selector_aggregate_scores",
        "inexact_observation_terminal_selector_allocation_calls",
        "inexact_observation_terminal_selector_allocated_bytes",
        "contact_command_lease_status",
        "contact_command_lease_executable",
        "contact_command_lease_age_ticks",
        "contact_command_lease_remaining_ticks",
        "contact_command_lease_flags",
        "capture_pressure",
        "station_authority",
        "station_error",
        "heading",
        "lateral_capture_error",
        "planar_capture_pressure",
        "commanded_yaw_rate",
        "lateral_support_margin",
        "lateral_dcm",
        "viability_margin",
        "commanded_zmp",
        "commanded_lateral_acceleration",
        "commanded_bank_angle",
        "commanded_roll_acceleration",
        "viability_activation_pressure",
        "viability_zmp_was_saturated",
        "viability_verified_scale",
        "viability_verification_queries",
        "viability_support_pressure",
        "viability_support_active",
        "viability_coordinate_queries",
        "viability_coordinate_score",
        "viability_coordinate_target",
        "viability_coordinate_request",
        "viability_request_status",
        "viability_request_active",
        "viability_request_executable",
        "viability_request_slew_limited",
        "viability_request_age_ticks",
        "viability_request_remaining_ticks",
        "viability_confirmation_status",
        "viability_confirmation_executable",
        "viability_confirmation_has_shadow",
        "viability_confirmation_support_mask",
        "viability_confirmation_consistent_updates",
        "viability_confirmation_alignment",
        "viability_confirmation_score_improvement",
        "viability_confirmation_flags",
        "viability_confirmation_shadow",
        "viability_hybrid_guard_status",
        "viability_hybrid_guard_executable",
        "viability_hybrid_guard_shadow_admissible",
        "viability_hybrid_guard_support_age_ticks",
        "viability_hybrid_guard_minimum_load_fraction",
        "viability_hybrid_guard_signed_roll_capture_pressure",
        "viability_hybrid_guard_flags",
        "viability_planner_query_count",
        "viability_planner_zero_pressure",
        "viability_planner_candidate_pressure",
        "viability_forecast_peak_capture_pressure",
        "viability_forecast_peak_sagittal_pressure",
        "viability_forecast_terminal_capture_pressure",
        "viability_forecast_terminal_sagittal_pressure",
        "viability_forecast_terminal_rate_pressure",
        "viability_forecast_yaw_pressure",
        "viability_forecast_resource_pressure",
        "viability_forecast_action_pressure",
        "viability_forecast_action_delta_pressure",
        "viability_forecast_support_pressure",
        "viability_forecast_minimum_capture_margin",
        "viability_forecast_path_valid",
        "viability_forecast_path",
        "execution_forecast_path_valid",
        "execution_forecast_support_mask",
        "execution_forecast_reduced_state",
        "execution_forecast_achieved_acceleration",
        "execution_forecast_path",
        "execution_forecast_allocation_calls",
        "execution_forecast_allocated_bytes",
        "viability_request",
        "fall_safe_mode",
        "fall_safe_primary_authority",
        "fall_safe_fresh_command_authority",
        "fall_safe_risk",
        "fall_safe_reason_flags",
        "torque_utilization",
        "task_pseudoinverse_calls",
        "clipped_steps",
        "task_jacobi_sweeps",
        "feasibility_projection_sweeps",
        "feasibility_halfspace_projections",
        "feasibility_polish_iterations",
        "allocation_calls",
        "allocated_bytes",
        "maximum_abs_qacc",
        "external_force_world",
        "application_point_world",
    )
    return (
        all(
            np.array_equal(np.asarray(left[field]), np.asarray(right[field]))
            for field in fields
        )
        and left["termination_reason"] == right["termination_reason"]
        and left["terminal_time_s"] == right["terminal_time_s"]
        and np.array_equal(
            left["terminal_root_position"], right["terminal_root_position"]
        )
        and np.array_equal(
            left["terminal_rotation_vector"], right["terminal_rotation_vector"]
        )
    )


def make_report(metrics: dict[str, Any]) -> str:
    rows = []
    for result in metrics["results"]:
        case = result["case"]
        rows.append(
            [
                case["name"],
                case["family"],
                "/".join(f"{value:g}" for value in case["force_world_n"]),
                f'{result["impulse_ns"]:.3f}',
                case["repetitions"],
                f'{case["friction"]:.2f}',
                result["outcome"],
                result["qualified"],
                "—" if result["qualified"] else "; ".join(result["qualification_blockers"]),
                f'{result["maximum_translation_m"] * 1000:.1f}',
                f'{result["maximum_tilt_deg"]:.2f}',
                "—" if result["recovery_time_s"] is None else f'{result["recovery_time_s"]:.3f}',
                f'{result["maximum_torque_utilization"]:.3f}',
                result["mujoco_warning_count"],
                f'{result["maximum_abs_qacc"]:.2e}',
                f'{result["controller_step_ns"]["p99"] / 1e3:.1f}',
                f'{result["terminal_time_s"]:.3f}',
            ]
        )
    gate_rows = [
        [name, gate["observed"], gate["pass"]]
        for name, gate in metrics["gates"].items()
    ]
    return "\n".join(
        [
            f'# Upkie multidimensional disturbance envelope · {metrics["revision"]}',
            "",
            f'**Evaluation admission: {"PASS" if metrics["admission"] else "FAIL"}.** This report freezes a discriminating plant-consequence matrix; it does not require every controller row to recover. Python owns MuJoCo integration, case construction, external wrench application, friction variation, and scoring. Persistent Rust sessions own rooted capture/station state, floating WBC, contact/rolling equations, hierarchy, torque, and allocation counters.',
            "",
            "The matrix separates all three force axes, sign, impulse duration, repeated impulses, application body, and plant friction. The live controller remains intentionally sagittal: red lateral or low-friction rows are measured capability boundaries, not evaluator failures. Every row terminates at its first declared 45°/350 mm fall boundary or pre-boundary numeric fault, so post-fall solver behavior cannot pollute control latency or be mislabeled as recovery.",
            "",
            "## Case matrix",
            "",
            *markdown_table(
                [
                    "case",
                    "family",
                    "force xyz N",
                    "impulse N·s",
                    "pulses",
                    "friction",
                    "outcome",
                    "qualified",
                    "qualification blockers",
                    "peak Δp mm",
                    "peak tilt °",
                    "recovery s",
                    "torque use",
                    "warnings",
                    "peak qacc abs",
                    "Rust p99 µs",
                    "terminal s",
                ],
                rows,
            ),
            "",
            "## Evaluation gates",
            "",
            *markdown_table(["gate", "observed", "pass"], gate_rows),
            "",
            "## Interpretation",
            "",
            f'- Qualified controller cases: **{metrics["qualified_cases"]}**.',
            f'- Failed/unsettled controller cases: **{metrics["failed_cases"]}**.',
            f'- Exact semantic repeat of the canonical 4 N row: **{metrics["canonical_replay_exact"]}**.',
            f'- The ±2 N lateral rows both end in FALL at **{metrics["lateral_fall_times_s"][0]:.3f}/{metrics["lateral_fall_times_s"][1]:.3f} s** (**{metrics["lateral_fall_time_relative_error"]:.3e}** relative timing difference). Their first-boundary peak translations differ by **{metrics["lateral_translation_relative_error"]:.3e}**, retained as real path asymmetry rather than called symmetric.',
            f'- Canonical/low-friction discrimination: **{metrics["friction_discriminating"]}**; μ=0.10 physically recovers but exposes later WBC non-admission, while μ=0.03 crosses the fall boundary.',
            f'- MuJoCo numeric warnings captured across the matrix: **{metrics["mujoco_warning_count"]}**.',
            f'- Failure rows stopped at their first declared boundary: **{metrics["terminal_failures_stopped"]}**.',
            "- `qualified=false` is not collapsed into one reason: FALL, UNSETTLED, and NUMERIC_FAULT remain distinct, alongside contact loss, later WBC nonadmission, effort use, loop budget, and recovery timing.",
            "",
            "## Deliberate limits",
            "",
            "This is still an ideal-observation soft-contact MuJoCo consequence test, not a learned policy and not hardware. It does not add lateral capture logic to make the chart greener. Plant friction is varied while the Rust contact model retains its compiled coefficient, deliberately exposing model mismatch. Repeated sagittal impulse tolerance is now measured, but terrain slope, delayed/noisy observation, simultaneous contacts, motor bandwidth, thermal derating, and a measured contact-mode estimator remain separate future axes. The policy-/physics-free state-local contact replay remains the semantic gate beneath this plant matrix.",
            "",
        ]
    )


def main() -> int:
    args = parse_args()
    if args.duration < PUSH_START_S + 1.0:
        raise ValueError("duration must leave at least one second after the push")
    all_cases = case_matrix()
    validate_cases(all_cases)
    if args.cases:
        requested = tuple(name for name in args.cases.split(",") if name)
        available = {case.name: case for case in all_cases}
        unknown = set(requested) - set(available)
        if unknown:
            raise ValueError(f"unknown cases: {sorted(unknown)}")
        cases = tuple(available[name] for name in requested)
    else:
        cases = all_cases
    if any(case.final_push_end_s + 1.0 > args.duration for case in cases):
        raise ValueError("duration must leave at least one second after every pulse")
    model_path = pathlib.Path(args.model).resolve()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, Any]] = []
    raw: dict[str, np.ndarray] = {}
    traces: dict[str, dict[str, Any]] = {}
    for case in cases:
        trace = run_case(model_path, case, args.duration, args.controller)
        traces[case.name] = trace
        results.append(summarize(case, trace, args.duration))
        for field in (
            "time_s",
            "root_position",
            "root_twist",
            "rotation_vector",
            "status",
            "contact_count",
            "capture_pressure",
            "station_authority",
            "station_error",
            "heading",
            "lateral_capture_error",
            "planar_capture_pressure",
            "commanded_yaw_rate",
            "lateral_support_margin",
            "torque_utilization",
            "external_force_world",
            "application_point_world",
            "maximum_abs_qacc",
        ):
            raw[f"{case.name}_{field}"] = np.asarray(trace[field])

    complete_matrix = cases == all_cases
    replay_case = next(
        (case for case in cases if case.name == "forward_4n_reference"), None
    )
    canonical_replay_exact = False
    if replay_case is not None:
        replay_trace = run_case(model_path, replay_case, args.duration, args.controller)
        canonical_replay_exact = semantic_trace_equal(
            traces[replay_case.name], replay_trace
        )
        raw["forward_4n_reference_repeat_root_position"] = np.asarray(
            replay_trace["root_position"]
        )
    by_name = {result["case"]["name"]: result for result in results}
    lateral_left = by_name.get("left_2n")
    lateral_right = by_name.get("right_2n")
    lateral_translation_error = (
        math.nan
        if lateral_left is None or lateral_right is None
        else abs(
            lateral_left["maximum_translation_m"]
            - lateral_right["maximum_translation_m"]
        )
        / max(
            lateral_left["maximum_translation_m"],
            lateral_right["maximum_translation_m"],
            1.0e-12,
        )
    )
    lateral_fall_times = (
        math.nan if lateral_left is None else lateral_left["terminal_time_s"],
        math.nan if lateral_right is None else lateral_right["terminal_time_s"],
    )
    lateral_fall_time_error = (
        math.nan
        if not all(math.isfinite(value) for value in lateral_fall_times)
        else abs(lateral_fall_times[0] - lateral_fall_times[1])
        / max(lateral_fall_times)
    )
    lateral_pair_classified = bool(
        lateral_left
        and lateral_right
        and lateral_left["outcome"] == "FALL"
        and lateral_right["outcome"] == "FALL"
    )
    canonical = by_name.get("forward_4n_reference")
    low_friction = by_name.get("forward_4n_friction_0p03")
    friction_discriminating = bool(
        canonical
        and low_friction
        and (
            canonical["outcome"] != low_friction["outcome"]
            or low_friction["maximum_translation_m"]
            > 2.0 * canonical["maximum_translation_m"]
        )
    )
    warnings_classified = all(
        result["mujoco_warning_count"] == 0
        or result["outcome"] == "NUMERIC_FAULT"
        for result in results
    )
    terminal_failures_stopped = all(
        not (result["fell"] or result["numeric_fault"])
        or (
            result["termination_reason"] in ("fall", "numeric_fault")
            and result["executed_ticks"] < result["configured_ticks"]
            and result["terminal_time_s"] < args.duration
        )
        for result in results
    )
    qualified = [result["case"]["name"] for result in results if result["qualified"]]
    failed = [result["case"]["name"] for result in results if not result["qualified"]]
    families = {result["case"]["family"] for result in results}
    gates = {
        "complete frozen matrix": {
            "observed": f"{len(cases)}/{len(all_cases)} cases · {sorted(families)}",
            "pass": complete_matrix,
        },
        "all traces finite": {
            "observed": all(result["finite"] for result in results),
            "pass": all(result["finite"] for result in results),
        },
        "canonical forward recovery retained": {
            "observed": by_name.get("forward_4n_reference", {}).get("outcome"),
            "pass": by_name.get("forward_4n_reference", {}).get("qualified", False),
        },
        "exact canonical semantic replay": {
            "observed": canonical_replay_exact,
            "pass": canonical_replay_exact,
        },
        "matrix is discriminating": {
            "observed": f"{len(qualified)} qualified / {len(failed)} failed",
            "pass": bool(qualified and failed),
        },
        "lateral sign pair has a repeatable boundary": {
            "observed": (
                f"FALL at {lateral_fall_times[0]:.3f}/{lateral_fall_times[1]:.3f} s; "
                f"relative timing difference {lateral_fall_time_error:.3e}; "
                f"path difference {lateral_translation_error:.3e}"
            ),
            "pass": lateral_pair_classified
            and math.isfinite(lateral_fall_time_error)
            and lateral_fall_time_error < 0.02,
        },
        "friction boundary is discriminating": {
            "observed": None
            if canonical is None or low_friction is None
            else f'{canonical["outcome"]} @ 1.00 / {low_friction["outcome"]} @ 0.03',
            "pass": friction_discriminating,
        },
        "MuJoCo warnings are explicitly classified": {
            "observed": sum(result["mujoco_warning_count"] for result in results),
            "pass": warnings_classified,
        },
        "no pre-boundary numeric fault": {
            "observed": sum(result["numeric_fault"] for result in results),
            "pass": not any(result["numeric_fault"] for result in results),
        },
        "failure rows stop at first boundary": {
            "observed": terminal_failures_stopped,
            "pass": terminal_failures_stopped,
        },
        "Rust timed region allocation-free": {
            "observed": all(result["allocation_free"] for result in results),
            "pass": all(result["allocation_free"] for result in results),
        },
    }
    metrics = {
        "schema_version": 1,
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "machine": platform.machine()},
        "mujoco_version": mujoco.__version__,
        "model": str(model_path),
        "model_sha256": plant.sha256(model_path),
        "control_dt_s": plant.CONTROL_DT,
        "physics_dt_s": plant.PHYSICS_DT,
        "duration_s": args.duration,
        "controller": args.controller,
        "push_start_s": PUSH_START_S,
        "results": results,
        "qualified_cases": qualified,
        "failed_cases": failed,
        "canonical_replay_exact": canonical_replay_exact,
        "lateral_fall_times_s": lateral_fall_times,
        "lateral_fall_time_relative_error": lateral_fall_time_error,
        "lateral_translation_relative_error": lateral_translation_error,
        "friction_discriminating": friction_discriminating,
        "mujoco_warning_count": sum(
            result["mujoco_warning_count"] for result in results
        ),
        "terminal_failures_stopped": terminal_failures_stopped,
        "gates": gates,
        "admission": all(gate["pass"] for gate in gates.values()),
    }
    np.savez_compressed(output / "upkie-disturbance-envelope-raw.npz", **raw)
    metrics_path = output / "upkie-disturbance-envelope-metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")
    report = make_report(metrics)
    report_path = output / "UPKIE_DISTURBANCE_ENVELOPE_AUDIT.md"
    report_path.write_text(report)
    pathlib.Path(args.web_report).write_text(
        render_report_html(report, title="Upkie disturbance envelope")
    )
    print(
        json.dumps(
            {
                "admission": metrics["admission"],
                "qualified": qualified,
                "failed": failed,
                "metrics": str(metrics_path),
                "report": str(report_path),
            },
            indent=2,
        )
    )
    return 0 if metrics["admission"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
