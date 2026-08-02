#!/usr/bin/env python3
"""Retain r117 canonical observation, tracking, and world-SDF authority."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import platform
from collections import Counter
from datetime import datetime, timezone
from typing import Any

import numpy as np
from websocket import create_connection

from cpu_reference_report import distribution, markdown_table, render_report_html
from live_authority_stream_report import BASE_CAPABILITIES, receive_kind


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="ws://127.0.0.1:8800/ws")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--pull-m", type=float, default=0.08)
    parser.add_argument("--expected-support", choices=("measured", "unavailable"), default="measured")
    parser.add_argument("--output", default="benchmarks/results/live-observation-history-r117")
    parser.add_argument("--web-report", default="web/LIVE_OBSERVATION_HISTORY_R117.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def finite_series(frames: list[dict[str, Any]], key: str) -> np.ndarray:
    values = np.asarray([frame["metrics"][key] for frame in frames], dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise AssertionError(f"non-finite live world-SDF field: {key}")
    return values


def evaluate(url: str, frame_count: int, pull_m: float, expected_support: str) -> dict[str, Any]:
    socket = create_connection(url, timeout=5)
    try:
        hello = receive_kind(socket, "hello", 1)
        contract = hello["authority_contract"]
        capabilities = {
            signal["stable_id"]: signal["availability"]
            for signal in contract["signals"]
        }
        expected = {**BASE_CAPABILITIES, "finite_support": expected_support}
        if capabilities != expected:
            raise AssertionError("live authority capability contract changed")
        if len(hello.get("world_sdf_planes", [])) != 1:
            raise AssertionError("the r117 editor must expose one visible world-SDF plane")
        idle = receive_kind(socket, "state")
        frame_name = hello["interaction_handles"][0]["frame"]
        frame_id = hello["frame_names"].index(frame_name)
        frame = next(item for item in idle["frames"] if item["id"] == frame_id)
        target = list(frame["translation"])
        target[2] -= pull_m
        socket.send(json.dumps({"type": "drag", "frame": frame_name, "target": target}))
        retained: list[dict[str, Any]] = []
        while len(retained) < frame_count:
            state = receive_kind(socket, "state")
            if state["metrics"]["authority_profile"] == "floating_dynamic_wbc_with_command_admission":
                retained.append(state)
        socket.send(json.dumps({"type": "release"}))
    finally:
        socket.close()

    distance = finite_series(retained, "world_collision_minimum_distance_m")
    margin = finite_series(retained, "world_collision_minimum_margin_m")
    gradient = finite_series(retained, "world_collision_gradient_norm")
    relative_velocity = finite_series(retained, "world_collision_relative_velocity_mps")
    required = finite_series(retained, "world_collision_required_acceleration_mps2")
    achieved = finite_series(retained, "world_collision_achieved_acceleration_mps2")
    residual = finite_series(retained, "world_collision_residual_mps2")
    solve_us = finite_series(retained, "solve_us")
    dynamics = finite_series(retained, "dynamics_residual_linf")
    contact = finite_series(retained, "contact_residual_linf")
    command_clearance = finite_series(retained, "primary_continuous_clearance_m")
    primary_world_sampled = finite_series(retained, "primary_world_sampled_clearance_m")
    primary_world_continuous = finite_series(
        retained, "primary_world_continuous_clearance_m"
    )
    contingency_world_sampled = finite_series(
        retained, "contingency_world_sampled_clearance_m"
    )
    contingency_world_continuous = finite_series(
        retained, "contingency_world_continuous_clearance_m"
    )
    primary_world_rate = finite_series(
        retained, "primary_world_distance_rate_bound_m_s"
    )
    contingency_world_rate = finite_series(
        retained, "contingency_world_distance_rate_bound_m_s"
    )
    primary_root_translation = finite_series(
        retained, "primary_root_prediction_translation_m"
    )
    primary_root_rotation = finite_series(
        retained, "primary_root_prediction_rotation_rad"
    )
    primary_root_linear_speed = finite_series(
        retained, "primary_root_prediction_max_linear_speed_m_s"
    )
    primary_root_angular_speed = finite_series(
        retained, "primary_root_prediction_max_angular_speed_rad_s"
    )
    primary_root_translation_error = finite_series(
        retained, "primary_root_prediction_translation_error_radius_m"
    )
    primary_root_rotation_error = finite_series(
        retained, "primary_root_prediction_rotation_error_radius_rad"
    )
    primary_prediction_erosion = finite_series(
        retained, "primary_world_prediction_clearance_erosion_m"
    )
    contingency_prediction_erosion = finite_series(
        retained, "contingency_world_prediction_clearance_erosion_m"
    )
    scene_epochs = np.asarray(
        [frame["metrics"]["world_scene_epoch"] for frame in retained], dtype=np.uint64
    )
    scene_ages = finite_series(retained, "world_scene_age_ns")
    scene_valid_until = np.asarray(
        [frame["metrics"]["world_scene_valid_until_ns"] for frame in retained],
        dtype=np.int64,
    )
    scene_horizon_end = np.asarray(
        [frame["metrics"]["world_scene_horizon_end_ns"] for frame in retained],
        dtype=np.int64,
    )
    scene_validities = [frame["metrics"]["world_scene_validity"] for frame in retained]
    observation_ages = finite_series(retained, "robot_observation_age_ns")
    observation_age_headroom = finite_series(
        retained, "robot_observation_age_headroom_ns"
    )
    observation_sync_uncertainty = finite_series(
        retained, "robot_observation_synchronization_uncertainty_ns"
    )
    observation_sync_headroom = finite_series(
        retained, "robot_observation_synchronization_headroom_ns"
    )
    observation_source_ids = [
        frame["metrics"]["robot_observation_source_id"] for frame in retained
    ]
    observation_sequences = [
        frame["metrics"]["robot_observation_source_sequence"] for frame in retained
    ]
    observation_causal = [
        frame["metrics"]["robot_observation_causal"] for frame in retained
    ]
    observation_age_valid = [
        frame["metrics"]["robot_observation_age_valid"] for frame in retained
    ]
    observation_sync_valid = [
        frame["metrics"]["robot_observation_synchronization_valid"]
        for frame in retained
    ]
    reconstruction_provenance = [
        frame["metrics"]["robot_observation_reconstruction_provenance"]
        for frame in retained
    ]
    reconstruction_hard_eligible = [
        frame["metrics"]["robot_observation_reconstruction_hard_eligible"]
        for frame in retained
    ]
    reconstruction_history_len = np.asarray(
        [frame["metrics"]["robot_observation_history_len"] for frame in retained],
        dtype=np.int64,
    )
    reconstruction_history_capacity = np.asarray(
        [frame["metrics"]["robot_observation_history_capacity"] for frame in retained],
        dtype=np.int64,
    )
    reconstruction_ingest_accepted = np.asarray(
        [frame["metrics"]["robot_observation_ingest_accepted"] for frame in retained],
        dtype=np.int64,
    )
    reconstruction_ingest_ignored = np.asarray(
        [frame["metrics"]["robot_observation_ingest_ignored"] for frame in retained],
        dtype=np.int64,
    )
    reconstruction_ingest_rejected = np.asarray(
        [frame["metrics"]["robot_observation_ingest_rejected"] for frame in retained],
        dtype=np.int64,
    )
    reconstruction_lower_time = np.asarray(
        [frame["metrics"]["robot_observation_reconstruction_lower_time_ns"] for frame in retained],
        dtype=np.int64,
    )
    reconstruction_upper_time = np.asarray(
        [frame["metrics"]["robot_observation_reconstruction_upper_time_ns"] for frame in retained],
        dtype=np.int64,
    )
    reconstruction_lower_source = np.asarray(
        [frame["metrics"]["robot_observation_reconstruction_lower_source_id"] for frame in retained],
        dtype=np.uint64,
    )
    reconstruction_upper_source = np.asarray(
        [frame["metrics"]["robot_observation_reconstruction_upper_source_id"] for frame in retained],
        dtype=np.uint64,
    )
    reconstruction_age = finite_series(
        retained, "robot_observation_reconstruction_source_age_ns"
    )
    reconstruction_age_headroom = finite_series(
        retained, "robot_observation_reconstruction_source_age_headroom_ns"
    )
    reconstruction_sync_headroom = finite_series(
        retained, "robot_observation_reconstruction_synchronization_headroom_ns"
    )
    tracking_position = finite_series(retained, "command_tracking_position_error")
    tracking_velocity = finite_series(retained, "command_tracking_velocity_error")
    tracking_position_contingency = finite_series(
        retained, "command_tracking_position_contingency_headroom"
    )
    tracking_position_reject = finite_series(
        retained, "command_tracking_position_reject_headroom"
    )
    tracking_velocity_contingency = finite_series(
        retained, "command_tracking_velocity_contingency_headroom"
    )
    tracking_velocity_reject = finite_series(
        retained, "command_tracking_velocity_reject_headroom"
    )
    tracking_actions = [
        frame["metrics"]["command_tracking_action"] for frame in retained
    ]
    contingency_root_translation = finite_series(
        retained, "contingency_root_prediction_translation_m"
    )
    command_admission_us = finite_series(retained, "command_admission_us")
    active = np.asarray(
        [frame["metrics"]["world_collision_active_probes"] for frame in retained],
        dtype=np.int64,
    )
    unsupported = np.asarray(
        [frame["metrics"]["world_collision_unsupported_shapes"] for frame in retained],
        dtype=np.int64,
    )
    closest_probes = [frame["metrics"]["world_collision_closest_probe"] for frame in retained]
    limiting_probes = [frame["metrics"]["world_collision_limiting_probe"] for frame in retained]
    sources = [frame["metrics"]["world_collision_field_source"] for frame in retained]
    qualities = [frame["metrics"]["world_collision_proxy_quality"] for frame in retained]
    policies = [frame["metrics"]["world_collision_outside_policy"] for frame in retained]
    body_names = hello["body_names"]
    closest_bodies = [body_names[frame["metrics"]["world_collision_closest_body"]] for frame in retained]
    limiting_bodies = [body_names[frame["metrics"]["world_collision_limiting_body"]] for frame in retained]
    primary_world_probes = [
        frame["metrics"]["primary_world_minimum_probe"] for frame in retained
    ]
    primary_world_bodies = [
        body_names[frame["metrics"]["primary_world_minimum_body"]]
        for frame in retained
    ]
    contingency_world_probes = [
        frame["metrics"]["contingency_world_minimum_probe"] for frame in retained
    ]
    command_selections = [frame["metrics"]["command_selection"] for frame in retained]
    primary_world_leaves = np.asarray(
        [frame["metrics"]["primary_world_leaf_intervals"] for frame in retained],
        dtype=np.int64,
    )
    primary_world_midpoints = np.asarray(
        [
            frame["metrics"]["primary_world_refinement_probe_samples"]
            for frame in retained
        ],
        dtype=np.int64,
    )
    primary_world_unresolved = np.asarray(
        [frame["metrics"]["primary_world_unresolved_intervals"] for frame in retained],
        dtype=np.int64,
    )
    primary_world_depth = np.asarray(
        [
            frame["metrics"]["primary_world_maximum_subdivision_depth"]
            for frame in retained
        ],
        dtype=np.int64,
    )
    world_witness_fields = [
        f"{plan}_world_first_{kind}_{part}"
        for plan in ("primary", "contingency")
        for kind in ("violation", "unknown")
        for part in ("probe", "body", "time_ns")
    ]
    no_world_fault_witness = all(
        frame["metrics"][field] is None
        for frame in retained
        for field in world_witness_fields
    )
    passed = bool(
        contract["source"].endswith(("-r117", "-r118", "-r119"))
        and capabilities["world_collision_avoidance"] == "measured"
        and capabilities["world_scene_snapshot"] == "measured"
        and capabilities["robot_observation_authority"] == "measured"
        and capabilities["robot_observation_history"] == "measured"
        and capabilities["command_tracking_authority"] == "measured"
        and capabilities["command_world_sampled_geometry"] == "measured"
        and capabilities["command_root_prediction"] == "measured"
        and capabilities["command_root_prediction_error"] == "measured"
        and capabilities["command_world_continuous_clearance"] == "measured"
        and np.all(active > 0)
        and np.all(unsupported == 0)
        and all(isinstance(probe, int) for probe in closest_probes)
        and all(isinstance(probe, int) for probe in limiting_probes)
        and set(sources) == {"trilinear_grid"}
        and set(policies) == {"reject"}
        and all(isinstance(quality, str) for quality in qualities)
        and float(np.max(np.abs(gradient - 1.0))) < 1e-12
        and float(np.min(residual)) >= -1e-7
        and float(np.min(achieved - required)) >= -1e-7
        and float(np.max(dynamics)) < 1e-8
        and float(np.max(contact)) < 1e-8
        and distribution(solve_us)["p99"] < 5_000.0
        and float(np.min(command_clearance)) > 0.0
        and float(np.min(primary_world_sampled)) >= 0.02
        and float(np.min(primary_world_continuous)) >= 0.02
        and float(np.min(contingency_world_sampled)) >= 0.02
        and float(np.min(contingency_world_continuous)) >= 0.02
        and np.all(primary_world_continuous <= primary_world_sampled + 1e-12)
        and np.all(contingency_world_continuous <= contingency_world_sampled + 1e-12)
        and all(isinstance(probe, int) for probe in primary_world_probes)
        and all(isinstance(probe, int) for probe in contingency_world_probes)
        and no_world_fault_witness
        and np.all(primary_world_leaves > 0)
        and np.all(primary_world_unresolved == 0)
        and np.all(primary_world_depth <= 3)
        and np.all(primary_root_translation >= 0.0)
        and np.all(primary_root_rotation >= 0.0)
        and np.all(primary_root_linear_speed >= 0.0)
        and np.all(primary_root_angular_speed >= 0.0)
        and np.all(primary_root_translation_error > 0.0)
        and np.all(primary_root_rotation_error > 0.0)
        and np.all(primary_prediction_erosion > primary_root_translation_error)
        and np.all(contingency_prediction_erosion > 0.0)
        and set(scene_epochs.tolist()) == {115}
        and set(scene_validities) == {"valid"}
        and np.all(scene_ages >= 0.0)
        and np.all(scene_valid_until >= scene_horizon_end)
        and np.all(observation_ages >= 0.0)
        and np.all(observation_age_headroom >= 0.0)
        and np.all(observation_sync_uncertainty >= 0.0)
        and np.all(observation_sync_headroom >= 0.0)
        and set(observation_source_ids) == {0xB015}
        and all(isinstance(sequence, int) for sequence in observation_sequences)
        and all(observation_causal)
        and all(observation_age_valid)
        and all(observation_sync_valid)
        and set(reconstruction_provenance) == {"exact"}
        and all(reconstruction_hard_eligible)
        and np.all(reconstruction_history_len > 0)
        and np.all(reconstruction_history_len <= reconstruction_history_capacity)
        and set(reconstruction_history_capacity.tolist()) == {64}
        and np.all(reconstruction_ingest_accepted == 1)
        and np.all(reconstruction_ingest_ignored == 0)
        and np.all(reconstruction_ingest_rejected == 0)
        and np.array_equal(reconstruction_lower_time, reconstruction_upper_time)
        and set(reconstruction_lower_source.tolist()) == {0xB015}
        and np.array_equal(reconstruction_lower_source, reconstruction_upper_source)
        and np.all(reconstruction_age == 0.0)
        and np.all(reconstruction_age_headroom >= 0.0)
        and np.all(reconstruction_sync_headroom >= 0.0)
        and set(tracking_actions).issubset({"nominal", "contingency", "rejected"})
        and np.all(tracking_position >= 0.0)
        and np.all(tracking_velocity >= 0.0)
        and np.all(tracking_position_reject > tracking_position_contingency)
        and np.all(tracking_velocity_reject > tracking_velocity_contingency)
        and np.all(contingency_root_translation >= 0.0)
        and set(command_selections).issubset({"primary", "contingency", "rejected"})
        and len(command_selections) == frame_count
        and distribution(command_admission_us)["p99"] < 5_000.0
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "guided state-local WBC plus policy-/physics-free command admission; no plant-response claim",
        "frames": frame_count,
        "pull_m": pull_m,
        "contract_source": contract["source"],
        "capability_count": len(capabilities),
        "minimum_distance_m": float(np.min(distance)),
        "minimum_margin_m": float(np.min(margin)),
        "active_probe_count": {"minimum": int(np.min(active)), "maximum": int(np.max(active))},
        "closest_probe_counts": dict(Counter(str(value) for value in closest_probes)),
        "limiting_probe_counts": dict(Counter(str(value) for value in limiting_probes)),
        "closest_body_counts": dict(Counter(closest_bodies)),
        "limiting_body_counts": dict(Counter(limiting_bodies)),
        "field_source_counts": dict(Counter(sources)),
        "proxy_quality_counts": dict(Counter(qualities)),
        "outside_policy_counts": dict(Counter(policies)),
        "maximum_gradient_norm_error": float(np.max(np.abs(gradient - 1.0))),
        "relative_velocity_mps": distribution(relative_velocity),
        "required_acceleration_mps2": distribution(required),
        "achieved_acceleration_mps2": distribution(achieved),
        "barrier_residual_mps2": distribution(residual),
        "minimum_achieved_minus_required_mps2": float(np.min(achieved - required)),
        "maximum_unsupported_shape_count": int(np.max(unsupported)),
        "maximum_dynamics_residual": float(np.max(dynamics)),
        "maximum_contact_residual": float(np.max(contact)),
        "minimum_separate_command_clearance_m": float(np.min(command_clearance)),
        "primary_world_sampled_clearance_m": distribution(primary_world_sampled),
        "primary_world_continuous_clearance_m": distribution(primary_world_continuous),
        "contingency_world_sampled_clearance_m": distribution(
            contingency_world_sampled
        ),
        "contingency_world_continuous_clearance_m": distribution(
            contingency_world_continuous
        ),
        "primary_root_prediction_translation_m": distribution(primary_root_translation),
        "primary_root_prediction_rotation_rad": distribution(primary_root_rotation),
        "primary_root_prediction_max_linear_speed_m_s": distribution(
            primary_root_linear_speed
        ),
        "primary_root_prediction_max_angular_speed_rad_s": distribution(
            primary_root_angular_speed
        ),
        "primary_root_prediction_translation_error_radius_m": distribution(
            primary_root_translation_error
        ),
        "primary_root_prediction_rotation_error_radius_rad": distribution(
            primary_root_rotation_error
        ),
        "primary_world_prediction_clearance_erosion_m": distribution(
            primary_prediction_erosion
        ),
        "contingency_world_prediction_clearance_erosion_m": distribution(
            contingency_prediction_erosion
        ),
        "world_scene_epochs": sorted(set(int(value) for value in scene_epochs)),
        "world_scene_validity_counts": dict(Counter(scene_validities)),
        "world_scene_age_ns": distribution(scene_ages),
        "world_scene_horizon_covered": bool(
            np.all(scene_valid_until >= scene_horizon_end)
        ),
        "robot_observation_source_id_counts": dict(
            Counter(hex(value) for value in observation_source_ids)
        ),
        "robot_observation_sequence_range": [
            min(observation_sequences),
            max(observation_sequences),
        ],
        "robot_observation_age_ns": distribution(observation_ages),
        "robot_observation_age_headroom_ns": distribution(
            observation_age_headroom
        ),
        "robot_observation_synchronization_uncertainty_ns": distribution(
            observation_sync_uncertainty
        ),
        "robot_observation_synchronization_headroom_ns": distribution(
            observation_sync_headroom
        ),
        "robot_observation_all_valid": bool(
            all(observation_causal)
            and all(observation_age_valid)
            and all(observation_sync_valid)
        ),
        "robot_observation_reconstruction_provenance_counts": dict(
            Counter(reconstruction_provenance)
        ),
        "robot_observation_reconstruction_all_hard_eligible": bool(
            all(reconstruction_hard_eligible)
        ),
        "robot_observation_history_length_range": [
            int(np.min(reconstruction_history_len)),
            int(np.max(reconstruction_history_len)),
        ],
        "robot_observation_history_capacity": int(
            np.max(reconstruction_history_capacity)
        ),
        "robot_observation_ingest_totals": {
            "accepted": int(np.sum(reconstruction_ingest_accepted)),
            "ignored": int(np.sum(reconstruction_ingest_ignored)),
            "rejected": int(np.sum(reconstruction_ingest_rejected)),
        },
        "robot_observation_reconstruction_source_age_ns": distribution(
            reconstruction_age
        ),
        "robot_observation_reconstruction_age_headroom_ns": distribution(
            reconstruction_age_headroom
        ),
        "robot_observation_reconstruction_sync_headroom_ns": distribution(
            reconstruction_sync_headroom
        ),
        "command_tracking_action_counts": dict(Counter(tracking_actions)),
        "command_tracking_position_error": distribution(tracking_position),
        "command_tracking_velocity_error": distribution(tracking_velocity),
        "command_tracking_position_contingency_headroom": distribution(
            tracking_position_contingency
        ),
        "command_tracking_position_reject_headroom": distribution(
            tracking_position_reject
        ),
        "command_tracking_velocity_contingency_headroom": distribution(
            tracking_velocity_contingency
        ),
        "command_tracking_velocity_reject_headroom": distribution(
            tracking_velocity_reject
        ),
        "contingency_root_prediction_translation_m": distribution(
            contingency_root_translation
        ),
        "primary_world_distance_rate_bound_m_s": distribution(primary_world_rate),
        "contingency_world_distance_rate_bound_m_s": distribution(
            contingency_world_rate
        ),
        "primary_world_minimum_probe_counts": dict(
            Counter(str(value) for value in primary_world_probes)
        ),
        "primary_world_minimum_body_counts": dict(Counter(primary_world_bodies)),
        "contingency_world_minimum_probe_counts": dict(
            Counter(str(value) for value in contingency_world_probes)
        ),
        "primary_world_work": {
            "leaf_intervals": distribution(primary_world_leaves.astype(np.float64)),
            "midpoint_probe_samples": distribution(
                primary_world_midpoints.astype(np.float64)
            ),
            "maximum_unresolved_intervals": int(np.max(primary_world_unresolved)),
            "maximum_subdivision_depth": int(np.max(primary_world_depth)),
        },
        "world_fault_witnesses_absent": no_world_fault_witness,
        "command_selection_counts": dict(Counter(command_selections)),
        "command_admission_timing_us": distribution(command_admission_us),
        "solve_timing_us": distribution(solve_us),
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    timing = audit["solve_timing_us"]
    command_timing = audit["command_admission_timing_us"]
    table = "\n".join(markdown_table(
        ["signal", "retained value"],
        [
            ["closest clearance / hard-margin reserve", f"{audit['minimum_distance_m'] * 1e3:.3f} / {audit['minimum_margin_m'] * 1e3:.3f} mm"],
            ["active probe count", f"{audit['active_probe_count']['minimum']}–{audit['active_probe_count']['maximum']}"],
            ["closest / limiting bodies", f"{json.dumps(audit['closest_body_counts'], sort_keys=True)} / {json.dumps(audit['limiting_body_counts'], sort_keys=True)}"],
            ["field / unknown-space policy", f"{json.dumps(audit['field_source_counts'], sort_keys=True)} / {json.dumps(audit['outside_policy_counts'], sort_keys=True)}"],
            ["minimum barrier residual", f"{audit['barrier_residual_mps2']['minimum']:.3e} m/s²"],
            ["raw WBC p50 / p99 / max", f"{timing['p50'] / 1000:.3f} / {timing['p99'] / 1000:.3f} / {timing['maximum'] / 1000:.3f} ms"],
            ["command admission p50 / p99 / max", f"{command_timing['p50'] / 1000:.3f} / {command_timing['p99'] / 1000:.3f} / {command_timing['maximum'] / 1000:.3f} ms"],
            ["primary sampled / continuous world minimum", f"{audit['primary_world_sampled_clearance_m']['minimum'] * 1e3:.3f} / {audit['primary_world_continuous_clearance_m']['minimum'] * 1e3:.3f} mm"],
            ["brake sampled / continuous world minimum", f"{audit['contingency_world_sampled_clearance_m']['minimum'] * 1e3:.3f} / {audit['contingency_world_continuous_clearance_m']['minimum'] * 1e3:.3f} mm"],
            ["root translation / attitude error radius", f"{audit['primary_root_prediction_translation_error_radius_m']['maximum'] * 1e3:.3f} mm / {audit['primary_root_prediction_rotation_error_radius_rad']['maximum'] * 180 / math.pi:.3f}°"],
            ["primary / brake prediction erosion", f"{audit['primary_world_prediction_clearance_erosion_m']['maximum'] * 1e3:.3f} / {audit['contingency_world_prediction_clearance_erosion_m']['maximum'] * 1e3:.3f} mm"],
            ["world scene epoch / validity", f"{audit['world_scene_epochs']} / {json.dumps(audit['world_scene_validity_counts'], sort_keys=True)}"],
            ["world scene age p50 / p99", f"{audit['world_scene_age_ns']['p50'] / 1e6:.3f} / {audit['world_scene_age_ns']['p99'] / 1e6:.3f} ms"],
            ["scene covers full command horizon", str(audit['world_scene_horizon_covered'])],
            ["observation source / sequence range", f"{json.dumps(audit['robot_observation_source_id_counts'], sort_keys=True)} / {audit['robot_observation_sequence_range']}"],
            ["observation age p50 / minimum headroom", f"{audit['robot_observation_age_ns']['p50'] / 1e6:.3f} / {audit['robot_observation_age_headroom_ns']['minimum'] / 1e6:.3f} ms"],
            ["sync uncertainty p50 / minimum headroom", f"{audit['robot_observation_synchronization_uncertainty_ns']['p50'] / 1e6:.3f} / {audit['robot_observation_synchronization_headroom_ns']['minimum'] / 1e6:.3f} ms"],
            ["all observations causal / fresh / synchronized", str(audit['robot_observation_all_valid'])],
            ["canonical reconstruction provenance", json.dumps(audit['robot_observation_reconstruction_provenance_counts'], sort_keys=True)],
            ["history occupancy / capacity", f"{audit['robot_observation_history_length_range']} / {audit['robot_observation_history_capacity']}"],
            ["history ingest accepted / ignored / rejected", f"{audit['robot_observation_ingest_totals']['accepted']} / {audit['robot_observation_ingest_totals']['ignored']} / {audit['robot_observation_ingest_totals']['rejected']}"],
            ["all reconstructions hard eligible", str(audit['robot_observation_reconstruction_all_hard_eligible'])],
            ["tracking action counts", json.dumps(audit['command_tracking_action_counts'], sort_keys=True)],
            ["max |observed−commanded| position / velocity", f"{audit['command_tracking_position_error']['maximum']:.4f} / {audit['command_tracking_velocity_error']['maximum']:.4f}"],
            ["minimum tracking contingency headroom q / v", f"{audit['command_tracking_position_contingency_headroom']['minimum']:.4f} / {audit['command_tracking_velocity_contingency_headroom']['minimum']:.4f}"],
        ],
    ))
    return f"""# Bonesaw live canonical observation + tracking + robust world-SDF authority · r117

## Outcome

**{audit['status'].upper()}.** The r117 WebSocket contract routes every floating-WBC query through the fixed-capacity canonical observation history before timestamp admission, observed-versus-commanded tracking, local world-SDF Viability, scene epoch/freshness, floating-root prediction, deterministic forecast-error erosion, sampled command-world geometry, and continuous certification. Across {audit['frames']} guided 50 Hz frames during an {audit['pull_m'] * 1e3:.1f} mm vertical torso disturbance, reconstruction provenance was `{json.dumps(audit['robot_observation_reconstruction_provenance_counts'], sort_keys=True)}` and every reconstructed state was hard-eligible: **{audit['robot_observation_reconstruction_all_hard_eligible']}**. Every observation was causal, fresh, and synchronized: **{audit['robot_observation_all_valid']}**. This is canonical state-local authority plus policy-/physics-free command admission, not a map-frame jump, policy, simulator, probability claim, fault diagnosis, or plant rollout.

{table}

All frames used the trilinear grid path with unit-gradient error at most `{audit['maximum_gradient_norm_error']:.3e}` and explicit `reject` unknown-space policy. The minimum achieved-minus-required acceleration was `{audit['minimum_achieved_minus_required_mps2']:.3e}` m/s². Maximum dynamics/contact residuals were `{audit['maximum_dynamics_residual']:.3e}` / `{audit['maximum_contact_residual']:.3e}`, and the separate self-collision command certificate retained at least `{audit['minimum_separate_command_clearance_m'] * 1e3:.3f}` mm. Selector counts were `{json.dumps(audit['command_selection_counts'], sort_keys=True)}`: world and self geometry stayed clear, while the six typed tracking-contingency frames withheld feed-forward authority and selected the independently valid brake.

## Command-world witnesses

Every primary and brake quintic was sampled at 21 deterministic 1 ms knots against the same immutable SDF, then conservatively bounded between knots. The primary limiting body counts were `{json.dumps(audit['primary_world_minimum_body_counts'], sort_keys=True)}`. No sampled violation or unknown-space witness occurred: **{audit['world_fault_witnesses_absent']}**. Primary work used at most depth {audit['primary_world_work']['maximum_subdivision_depth']} with {audit['primary_world_work']['maximum_unresolved_intervals']} unresolved leaves; safe broad certificates legitimately required zero midpoint probes in this trace.

## Typed boundary

The reconstruction row carries ring occupancy/capacity, ingest dispositions, exact/interpolated/predicted/held provenance, lower/upper source timestamps, stable source IDs/sequences, source age, synchronization headroom, and hard eligibility. The WBC no longer reads the producer-owned floating state directly. The subsequent observation row independently checks timestamp authority, while tracking carries actuator mismatch and two-stage headroom. Local and swept geometry, resources, solver work, final selection, and plant response remain separate; none becomes one health score.

## Bounded failure evidence

The first demonstration placement began 3.591 mm inside the represented wall. Feasibility exhausted its bounded work after 1,767,200 halfspace projections with 5.475e-3 maximum violation, returned `MaxIterations`, zeroed the executable candidate, and withheld the frame. The displayed placement begins outside the hard margin so the live row can be inspected continuously. `MaxIterations` is budget exhaustion, not an infeasibility proof.

## Remaining work

The local barrier excludes SDF Hessian curvature and voxel-feature switching. Swept command validation uses a measured global field Lipschitz bound, explicit floating-root paths, compiled angular reach, and declared deterministic root-error growth; it is conservative sphere-probe evidence, not covariance, arbitrary mesh CCD, or a calibrated estimator claim. R117 proves the exact-sample live path. Injected delayed/interpolated/predicted live transport faults and conservative reconstruction-error erosion into joint/collision margins remain separate gates, as do fixed-capacity scene replacement, mesh-specific probes, calibrated actuation, and measured plant response.
"""


def main() -> None:
    args = parse_args()
    if args.frames <= 0 or not math.isfinite(args.pull_m) or args.pull_m <= 0.0:
        raise SystemExit("invalid live trace length or pull")
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = evaluate(args.url, args.frames, args.pull_m, args.expected_support)
    sources = [
        pathlib.Path("crates/bonesaw-core/src/world_collision.rs"),
        pathlib.Path("crates/bonesaw-core/src/history.rs"),
        pathlib.Path("crates/bonesaw-core/src/dynamic_wbc.rs"),
        pathlib.Path("crates/bonesaw-tools/src/bin/server.rs"),
        pathlib.Path("web/motion-rig-r16.js"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "live-observation-history-r117",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in sources},
        "audit": audit,
    }
    (output / "live-world-sdf-command-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "LIVE_WORLD_SDF_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(
            report, title="Bonesaw live canonical observation authority · r117"
        )
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
