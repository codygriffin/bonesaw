#!/usr/bin/env python3
"""Probe the live Bonesaw WebSocket authority contract and execution transition."""

from __future__ import annotations

import argparse
import json
import math
from typing import Any

from websocket import create_connection


BASE_CAPABILITIES = {
    "hard_rows": "measured",
    "joint_position": "measured",
    "joint_stopping": "measured",
    "self_collision_avoidance": "measured",
    "world_collision_avoidance": "measured",
    "world_scene_snapshot": "measured",
    "robot_observation_history": "measured",
    "robot_observation_authority": "measured",
    "command_tracking_authority": "measured",
    "actuator_effort": "measured",
    "command_sampled_geometry": "measured",
    "command_continuous_clearance": "measured",
    "command_world_sampled_geometry": "measured",
    "command_root_prediction": "measured",
    "command_root_prediction_error": "measured",
    "command_world_continuous_clearance": "measured",
    "command_selection": "measured",
    "actuator_realization": "unmodeled",
    "acceleration_realization": "unavailable",
    "solver_budget": "measured",
    "thermal_reliability": "unmodeled",
    "task_residuals": "measured",
}


def receive_kind(socket: Any, expected: str, attempts: int = 100) -> dict[str, Any]:
    for _ in range(attempts):
        message = json.loads(socket.recv())
        if message.get("type") == "error":
            raise RuntimeError(message.get("message", "server error"))
        if message.get("type") == expected:
            return message
    raise RuntimeError(f"did not receive {expected!r} within {attempts} messages")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="ws://127.0.0.1:8787/ws")
    parser.add_argument(
        "--expected-support",
        choices=("measured", "unavailable"),
        default="unavailable",
    )
    args = parser.parse_args()

    socket = create_connection(args.url, timeout=5)
    try:
        hello = receive_kind(socket, "hello", 1)
        contract = hello["authority_contract"]
        capabilities = {
            signal["stable_id"]: signal["availability"]
            for signal in contract["signals"]
        }
        assert contract["schema"] == 1
        expected_capabilities = {
            **BASE_CAPABILITIES,
            "finite_support": args.expected_support,
        }
        assert capabilities == expected_capabilities, (
            capabilities,
            expected_capabilities,
        )
        assert len(capabilities) == len(contract["signals"])
        assert "health" not in capabilities

        idle = receive_kind(socket, "state")
        assert idle["metrics"]["authority_profile"] == "kinematic_controller"
        assert idle["metrics"]["minimum_support_margin_m"] is None
        assert idle["metrics"]["limiting_support_patch"] is None
        assert math.isfinite(idle["metrics"]["minimum_joint_margin_rad"])

        handle = hello["interaction_handles"][0]
        frame_id = hello["frame_names"].index(handle["frame"])
        frame = next(frame for frame in idle["frames"] if frame["id"] == frame_id)
        target = list(frame["translation"])
        target[2] -= 0.01
        socket.send(json.dumps({"type": "drag", "frame": handle["frame"], "target": target}))

        dynamic = None
        for _ in range(100):
            candidate = receive_kind(socket, "state")
            if candidate["metrics"]["authority_profile"] == "floating_dynamic_wbc_with_command_admission":
                dynamic = candidate
                break
        assert dynamic is not None
        metrics = dynamic["metrics"]
        assert math.isfinite(metrics["solve_us"])
        assert math.isfinite(metrics["dynamics_residual_linf"])
        assert math.isfinite(metrics["contact_residual_linf"])
        assert math.isfinite(metrics["maximum_torque_utilization"])
        assert math.isfinite(metrics["minimum_joint_stopping_margin_rad_s2"])
        assert math.isfinite(metrics["collision_barrier_minimum_distance_m"])
        assert math.isfinite(metrics["collision_barrier_minimum_margin_m"])
        assert isinstance(metrics["collision_barrier_closest_pair"], int)
        assert isinstance(metrics["collision_barrier_active_pairs"], int)
        assert isinstance(metrics["collision_barrier_unsupported_shapes"], int)
        assert math.isfinite(metrics["world_collision_minimum_distance_m"])
        assert math.isfinite(metrics["world_collision_minimum_margin_m"])
        assert isinstance(metrics["world_collision_closest_probe"], int)
        assert isinstance(metrics["world_collision_closest_body"], int)
        assert isinstance(metrics["world_collision_active_probes"], int)
        assert metrics["world_collision_field_source"] in {
            "trilinear_grid",
            "occupied_boundary",
        }
        assert math.isfinite(metrics["world_collision_gradient_norm"])
        assert metrics["world_collision_outside_policy"] in {
            "reject",
            "occupied_boundary",
        }
        assert isinstance(metrics["world_collision_unsupported_shapes"], int)
        assert isinstance(metrics["limiting_joint_stopping"], int)
        assert metrics["command_selection"] in {"primary", "contingency", "rejected"}
        assert isinstance(metrics["command_admission_flags"], int)
        assert math.isfinite(metrics["primary_sampled_clearance_m"])
        assert math.isfinite(metrics["primary_continuous_clearance_m"])
        assert isinstance(metrics["primary_continuous_limiting_pair"], int)
        assert isinstance(metrics["primary_continuous_limiting_body_a"], int)
        assert isinstance(metrics["primary_continuous_limiting_body_b"], int)
        assert math.isfinite(metrics["primary_continuous_relative_speed_m_s"])
        assert isinstance(metrics["primary_continuity_leaf_intervals"], int)
        assert isinstance(metrics["primary_refinement_pair_samples"], int)
        assert isinstance(metrics["primary_continuity_unresolved_intervals"], int)
        assert isinstance(metrics["primary_continuity_maximum_subdivision_depth"], int)
        assert isinstance(metrics["primary_minimum_collision_pair"], int)
        assert isinstance(metrics["primary_minimum_collision_body_a"], int)
        assert isinstance(metrics["primary_minimum_collision_body_b"], int)
        first_witness = (
            metrics["primary_first_collision_pair"],
            metrics["primary_first_collision_time_ns"],
            metrics["primary_first_collision_body_a"],
            metrics["primary_first_collision_body_b"],
        )
        assert all(value is None for value in first_witness) or all(
            isinstance(value, int) for value in first_witness
        )
        assert math.isfinite(metrics["contingency_sampled_clearance_m"])
        assert math.isfinite(metrics["contingency_continuous_clearance_m"])
        assert isinstance(metrics["contingency_continuous_limiting_pair"], int)
        assert isinstance(metrics["contingency_continuous_limiting_body_a"], int)
        assert isinstance(metrics["contingency_continuous_limiting_body_b"], int)
        assert math.isfinite(metrics["contingency_continuous_relative_speed_m_s"])
        assert isinstance(metrics["contingency_continuity_leaf_intervals"], int)
        assert isinstance(metrics["contingency_refinement_pair_samples"], int)
        assert isinstance(metrics["contingency_continuity_unresolved_intervals"], int)
        assert isinstance(metrics["contingency_continuity_maximum_subdivision_depth"], int)
        for plan in ("primary", "contingency"):
            assert math.isfinite(metrics[f"{plan}_world_sampled_clearance_m"])
            assert isinstance(metrics[f"{plan}_world_minimum_probe"], int)
            assert isinstance(metrics[f"{plan}_world_minimum_body"], int)
            assert metrics[f"{plan}_world_field_source"] in {
                "trilinear_grid",
                "occupied_boundary",
            }
            assert math.isfinite(metrics[f"{plan}_world_continuous_clearance_m"])
            assert isinstance(metrics[f"{plan}_world_continuous_limiting_probe"], int)
            assert isinstance(metrics[f"{plan}_world_continuous_limiting_body"], int)
            assert math.isfinite(metrics[f"{plan}_world_distance_rate_bound_m_s"])
            assert isinstance(metrics[f"{plan}_world_leaf_intervals"], int)
            assert isinstance(metrics[f"{plan}_world_refinement_probe_samples"], int)
            assert isinstance(metrics[f"{plan}_world_unresolved_intervals"], int)
            assert isinstance(metrics[f"{plan}_world_maximum_subdivision_depth"], int)
            for witness in ("first_violation", "first_unknown"):
                fields = (
                    metrics[f"{plan}_world_{witness}_probe"],
                    metrics[f"{plan}_world_{witness}_body"],
                    metrics[f"{plan}_world_{witness}_time_ns"],
                )
                assert all(value is None for value in fields) or all(
                    isinstance(value, int) for value in fields
                )
        assert math.isfinite(metrics["command_admission_us"])
        assert math.isfinite(metrics["command_admission_batch_us"])
        assert metrics["command_tracking_action"] in {
            "nominal",
            "contingency",
            "rejected",
        }
        assert math.isfinite(metrics["command_tracking_position_error"])
        assert math.isfinite(metrics["command_tracking_velocity_error"])
        assert isinstance(metrics["command_tracking_limiting_position_actuator"], int)
        assert isinstance(metrics["command_tracking_limiting_velocity_actuator"], int)
        assert metrics["robot_observation_causal"] is True
        assert metrics["robot_observation_age_valid"] is True
        assert metrics["robot_observation_synchronization_valid"] is True
        assert isinstance(metrics["robot_observation_source_time_ns"], int)
        assert isinstance(metrics["robot_observation_mapped_time_ns"], int)
        assert isinstance(metrics["robot_observation_source_sequence"], int)
        assert isinstance(metrics["robot_observation_source_id"], int)
        assert metrics["robot_observation_age_headroom_ns"] >= 0
        assert metrics["robot_observation_synchronization_headroom_ns"] >= 0
        assert metrics["robot_observation_reconstruction_provenance"] == "exact"
        assert metrics["robot_observation_reconstruction_hard_eligible"] is True
        assert metrics["robot_observation_error_exposure_ns"] == 0
        for field in (
            "robot_observation_joint_position_error_rad",
            "robot_observation_joint_velocity_error_rad_s",
            "robot_observation_root_translation_error_m",
            "robot_observation_root_rotation_error_rad",
            "robot_observation_point_position_error_m",
            "robot_observation_center_of_mass_position_error_m",
        ):
            assert metrics[field] == 0.0
        for raw, robust in (
            ("raw_minimum_joint_margin_rad", "minimum_joint_margin_rad"),
            (
                "raw_minimum_joint_stopping_margin_rad_s2",
                "minimum_joint_stopping_margin_rad_s2",
            ),
            (
                "raw_collision_barrier_minimum_margin_m",
                "collision_barrier_minimum_margin_m",
            ),
            (
                "raw_world_collision_minimum_margin_m",
                "world_collision_minimum_margin_m",
            ),
        ):
            assert metrics[raw] == metrics[robust]
        assert 0 < metrics["robot_observation_history_len"] <= 64
        assert metrics["robot_observation_history_capacity"] == 64
        assert metrics["robot_observation_ingest_accepted"] == 1
        assert metrics["robot_observation_ingest_ignored"] == 0
        assert metrics["robot_observation_ingest_rejected"] == 0
        assert metrics["robot_observation_transport_mode"] == "exact"
        assert metrics["robot_observation_transport_sample_emitted"] is True
        assert metrics["robot_observation_frame_exact_queries"] == 5
        assert metrics["robot_observation_frame_interpolated_queries"] == 0
        assert metrics["robot_observation_frame_predicted_queries"] == 0
        assert metrics["robot_observation_frame_held_queries"] == 0
        assert (
            metrics["robot_observation_reconstruction_lower_time_ns"]
            == metrics["robot_observation_reconstruction_upper_time_ns"]
            == metrics["robot_observation_transport_query_time_ns"]
        )
        assert len(metrics["center_of_mass_world"]) == 3
        if args.expected_support == "measured":
            assert math.isfinite(metrics["minimum_support_margin_m"])
            assert (
                metrics["raw_minimum_support_margin_m"]
                == metrics["minimum_support_margin_m"]
            )
            assert isinstance(metrics["limiting_support_patch"], int)
        else:
            assert metrics["minimum_support_margin_m"] is None
            assert metrics["raw_minimum_support_margin_m"] is None
            assert metrics["limiting_support_patch"] is None
        active_tasks = [task for task in metrics["task_residuals"] if task["active"]]
        assert active_tasks

        socket.send(json.dumps({"type": "release"}))
        print(json.dumps({
            "pass": True,
            "contract_source": contract["source"],
            "capability_count": len(capabilities),
            "idle_profile": idle["metrics"]["authority_profile"],
            "dynamic_profile": metrics["authority_profile"],
            "dynamic_solve_us": metrics["solve_us"],
            "command_tracking_action": metrics["command_tracking_action"],
            "command_tracking_position_error": metrics[
                "command_tracking_position_error"
            ],
            "command_tracking_velocity_error": metrics[
                "command_tracking_velocity_error"
            ],
            "robot_observation_age_ns": metrics["robot_observation_age_ns"],
            "robot_observation_age_headroom_ns": metrics[
                "robot_observation_age_headroom_ns"
            ],
            "dynamics_residual_linf": metrics["dynamics_residual_linf"],
            "contact_residual_linf": metrics["contact_residual_linf"],
            "maximum_torque_utilization": metrics["maximum_torque_utilization"],
            "minimum_joint_stopping_margin_rad_s2": metrics[
                "minimum_joint_stopping_margin_rad_s2"
            ],
            "limiting_joint_stopping": metrics["limiting_joint_stopping"],
            "minimum_support_margin_m": metrics["minimum_support_margin_m"],
            "limiting_support_patch": metrics["limiting_support_patch"],
            "collision_barrier_minimum_distance_m": metrics[
                "collision_barrier_minimum_distance_m"
            ],
            "collision_barrier_minimum_margin_m": metrics[
                "collision_barrier_minimum_margin_m"
            ],
            "collision_barrier_closest_pair": metrics[
                "collision_barrier_closest_pair"
            ],
            "collision_barrier_active_pairs": metrics[
                "collision_barrier_active_pairs"
            ],
            "collision_barrier_limiting_pair": metrics[
                "collision_barrier_limiting_pair"
            ],
            "collision_barrier_quality": metrics["collision_barrier_quality"],
            "collision_barrier_residual_mps2": metrics[
                "collision_barrier_residual_mps2"
            ],
            "collision_barrier_unsupported_shapes": metrics[
                "collision_barrier_unsupported_shapes"
            ],
            "world_collision_minimum_distance_m": metrics[
                "world_collision_minimum_distance_m"
            ],
            "world_collision_minimum_margin_m": metrics[
                "world_collision_minimum_margin_m"
            ],
            "world_collision_closest_probe": metrics[
                "world_collision_closest_probe"
            ],
            "world_collision_closest_body": hello["body_names"][
                metrics["world_collision_closest_body"]
            ],
            "world_collision_active_probes": metrics[
                "world_collision_active_probes"
            ],
            "world_collision_limiting_probe": metrics[
                "world_collision_limiting_probe"
            ],
            "world_collision_field_source": metrics[
                "world_collision_field_source"
            ],
            "world_collision_gradient_norm": metrics[
                "world_collision_gradient_norm"
            ],
            "world_collision_residual_mps2": metrics[
                "world_collision_residual_mps2"
            ],
            "world_collision_outside_policy": metrics[
                "world_collision_outside_policy"
            ],
            "active_task_count": len(active_tasks),
            "command_selection": metrics["command_selection"],
            "command_admission_flags": metrics["command_admission_flags"],
            "primary_sampled_clearance_m": metrics["primary_sampled_clearance_m"],
            "primary_continuous_clearance_m": metrics[
                "primary_continuous_clearance_m"
            ],
            "primary_continuous_limiting_pair": metrics[
                "primary_continuous_limiting_pair"
            ],
            "primary_continuous_limiting_bodies": [
                hello["body_names"][metrics["primary_continuous_limiting_body_a"]],
                hello["body_names"][metrics["primary_continuous_limiting_body_b"]],
            ],
            "primary_continuous_relative_speed_m_s": metrics[
                "primary_continuous_relative_speed_m_s"
            ],
            "primary_continuity_leaf_intervals": metrics[
                "primary_continuity_leaf_intervals"
            ],
            "primary_refinement_pair_samples": metrics[
                "primary_refinement_pair_samples"
            ],
            "primary_continuity_unresolved_intervals": metrics[
                "primary_continuity_unresolved_intervals"
            ],
            "primary_continuity_maximum_subdivision_depth": metrics[
                "primary_continuity_maximum_subdivision_depth"
            ],
            "primary_first_collision_pair": metrics["primary_first_collision_pair"],
            "primary_minimum_collision_pair": metrics[
                "primary_minimum_collision_pair"
            ],
            "primary_minimum_collision_bodies": [
                hello["body_names"][metrics["primary_minimum_collision_body_a"]],
                hello["body_names"][metrics["primary_minimum_collision_body_b"]],
            ],
            "primary_first_collision_bodies": None
            if metrics["primary_first_collision_body_a"] is None
            else [
                hello["body_names"][metrics["primary_first_collision_body_a"]],
                hello["body_names"][metrics["primary_first_collision_body_b"]],
            ],
            "primary_first_collision_time_ns": metrics[
                "primary_first_collision_time_ns"
            ],
            "contingency_continuous_clearance_m": metrics[
                "contingency_continuous_clearance_m"
            ],
            "primary_world_sampled_clearance_m": metrics[
                "primary_world_sampled_clearance_m"
            ],
            "primary_world_minimum_probe": metrics["primary_world_minimum_probe"],
            "primary_world_minimum_body": hello["body_names"][
                metrics["primary_world_minimum_body"]
            ],
            "primary_world_field_source": metrics["primary_world_field_source"],
            "primary_world_continuous_clearance_m": metrics[
                "primary_world_continuous_clearance_m"
            ],
            "primary_world_continuous_limiting_probe": metrics[
                "primary_world_continuous_limiting_probe"
            ],
            "primary_world_continuous_limiting_body": hello["body_names"][
                metrics["primary_world_continuous_limiting_body"]
            ],
            "primary_world_distance_rate_bound_m_s": metrics[
                "primary_world_distance_rate_bound_m_s"
            ],
            "primary_world_leaf_intervals": metrics[
                "primary_world_leaf_intervals"
            ],
            "primary_world_refinement_probe_samples": metrics[
                "primary_world_refinement_probe_samples"
            ],
            "primary_world_unresolved_intervals": metrics[
                "primary_world_unresolved_intervals"
            ],
            "primary_world_maximum_subdivision_depth": metrics[
                "primary_world_maximum_subdivision_depth"
            ],
            "contingency_world_sampled_clearance_m": metrics[
                "contingency_world_sampled_clearance_m"
            ],
            "contingency_world_continuous_clearance_m": metrics[
                "contingency_world_continuous_clearance_m"
            ],
            "command_admission_us": metrics["command_admission_us"],
            "command_admission_batch_us": metrics["command_admission_batch_us"],
            "support_availability": capabilities["finite_support"],
            "joint_stopping_availability": capabilities["joint_stopping"],
        }, indent=2, sort_keys=True))
    finally:
        socket.close()


if __name__ == "__main__":
    main()
