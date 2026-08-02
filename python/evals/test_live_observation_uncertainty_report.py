from __future__ import annotations

import pytest

from live_observation_uncertainty_report import (
    ERROR_FIELDS,
    EXPECTED,
    assert_uncertainty_mode,
    summarize_mode,
)


def uncertainty_state(mode: str, stopping_erosion: float = 0.0) -> dict:
    expected = EXPECTED[mode]
    metrics = {
        "robot_observation_error_exposure_ns": expected["exposure_ns"],
        "robot_observation_reconstruction_hard_eligible": True,
        "robot_observation_reconstruction_provenance": mode,
        "raw_minimum_joint_margin_rad": 0.2,
        "minimum_joint_margin_rad": 0.2 - expected["joint_position_rad"],
        "raw_minimum_support_margin_m": 0.05,
        "minimum_support_margin_m": (
            0.05 - expected["center_of_mass_position_m"]
        ),
        "raw_collision_barrier_minimum_margin_m": 0.04,
        "collision_barrier_minimum_margin_m": (
            0.04 - 2.0 * expected["point_position_m"]
        ),
        "raw_world_collision_minimum_margin_m": 0.03,
        "world_collision_minimum_margin_m": (
            0.03 - expected["point_position_m"]
        ),
        "raw_minimum_joint_stopping_margin_rad_s2": 10.0,
        "minimum_joint_stopping_margin_rad_s2": 10.0 - stopping_erosion,
        "command_selection": "primary",
        "solve_us": 40.0,
        "command_admission_us": 8.0,
    }
    for name, field in ERROR_FIELDS.items():
        metrics[field] = expected[name]
    return {"metrics": metrics}


def test_assert_uncertainty_mode_checks_exact_and_robust_margin_identities() -> None:
    assert_uncertainty_mode("exact", [uncertainty_state("exact")])
    assert_uncertainty_mode(
        "interpolated", [uncertainty_state("interpolated", 0.25)]
    )
    assert_uncertainty_mode("predicted", [uncertainty_state("predicted", 0.5)])


def test_assert_uncertainty_mode_rejects_missing_margin_erosion() -> None:
    state = uncertainty_state("predicted", 0.5)
    state["metrics"]["minimum_support_margin_m"] += 1e-3
    with pytest.raises(AssertionError):
        assert_uncertainty_mode("predicted", [state])


def test_summary_preserves_error_margin_and_timing_axes() -> None:
    states = [uncertainty_state("interpolated", value) for value in (0.2, 0.3)]
    states[1]["metrics"]["command_selection"] = "contingency"
    summary = summarize_mode(states)
    assert summary["exposure_ns"] == 2_500_000
    assert summary["margin_erosion"]["self_collision_m"]["minimum"] == pytest.approx(
        2.0 * EXPECTED["interpolated"]["point_position_m"]
    )
    assert summary["selection_counts"] == {
        "primary": 1,
        "contingency": 1,
        "rejected": 0,
    }
    assert summary["solve"]["p50_us"] == 40.0
