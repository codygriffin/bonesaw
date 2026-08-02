from __future__ import annotations

import pytest

from live_command_observation_uncertainty_report import (
    COMMAND_MARGIN_PAIRS,
    assert_command_uncertainty_mode,
    summarize_mode,
)
from live_observation_uncertainty_report import ERROR_FIELDS, EXPECTED


def command_state(mode: str) -> dict:
    expected = EXPECTED[mode]
    metrics = {
        "robot_observation_error_exposure_ns": expected["exposure_ns"],
        "robot_observation_reconstruction_hard_eligible": True,
        "robot_observation_reconstruction_provenance": mode,
        "robot_observation_transport_query_time_ns": 123,
        "robot_observation_mapped_time_ns": 123,
        "command_tracking_action": "nominal",
        "command_selection": "primary",
        "primary_root_prediction_translation_error_radius_m": 0.001,
        "primary_root_prediction_rotation_error_radius_rad": 0.002,
        "primary_world_prediction_clearance_erosion_m": 0.003,
        "command_admission_us": 8.0,
        "solve_us": 40.0,
    }
    for name, field in ERROR_FIELDS.items():
        metrics[field] = expected[name]
    for index, (_, (raw_field, robust_field, multiplier)) in enumerate(
        COMMAND_MARGIN_PAIRS.items()
    ):
        raw = 0.1 + 0.01 * index
        metrics[raw_field] = raw
        metrics[robust_field] = raw - multiplier * expected["point_position_m"]
    return {"metrics": metrics}


def test_command_uncertainty_checks_primary_and_brake_identities() -> None:
    for mode in ("exact", "interpolated", "predicted"):
        assert_command_uncertainty_mode(mode, [command_state(mode)])


def test_command_uncertainty_rejects_missing_world_erosion() -> None:
    state = command_state("predicted")
    state["metrics"]["primary_world_robust_sampled_clearance_m"] += 1e-3
    with pytest.raises(AssertionError):
        assert_command_uncertainty_mode("predicted", [state])


def test_command_uncertainty_summary_keeps_tracking_and_selection_separate() -> None:
    states = [command_state("interpolated"), command_state("interpolated")]
    states[1]["metrics"]["command_selection"] = "contingency"
    states[1]["metrics"]["command_tracking_action"] = "contingency"
    summary = summarize_mode(states)
    assert summary["selection_counts"] == {
        "primary": 1,
        "contingency": 1,
        "rejected": 0,
    }
    assert summary["tracking_action_counts"] == {
        "nominal": 1,
        "contingency": 1,
        "rejected": 0,
    }
    assert summary["command_margin_erosion"]["self_sampled_m"]["minimum"] == pytest.approx(
        2.0 * EXPECTED["interpolated"]["point_position_m"]
    )
