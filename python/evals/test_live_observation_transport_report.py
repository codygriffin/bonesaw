from __future__ import annotations

from live_observation_transport_report import assert_mode, mode_summary, percentile


def state(
    mode: str,
    provenance: str,
    lower: int,
    query: int,
    upper: int,
    source_age: int,
    observation_age: int,
    exact: int,
    interpolated: int,
    predicted: int,
) -> dict:
    return {
        "metrics": {
            "robot_observation_transport_mode": mode,
            "robot_observation_reconstruction_provenance": provenance,
            "robot_observation_reconstruction_hard_eligible": True,
            "robot_observation_frame_exact_queries": exact,
            "robot_observation_frame_interpolated_queries": interpolated,
            "robot_observation_frame_predicted_queries": predicted,
            "robot_observation_frame_held_queries": 0,
            "robot_observation_ingest_rejected": 0,
            "robot_observation_causal": True,
            "robot_observation_age_valid": True,
            "robot_observation_synchronization_valid": True,
            "robot_observation_transport_query_time_ns": query,
            "robot_observation_mapped_time_ns": query,
            "robot_observation_reconstruction_lower_time_ns": lower,
            "robot_observation_reconstruction_upper_time_ns": upper,
            "robot_observation_reconstruction_source_age_ns": source_age,
            "robot_observation_age_ns": observation_age,
            "solve_us": 100.0,
            "command_admission_us": 20.0,
            "command_selection": "primary",
        }
    }


def test_assert_mode_covers_exact_interpolated_and_predicted_contracts() -> None:
    exact = state("exact", "exact", 10_000_000, 10_000_000, 10_000_000, 0, 0, 4, 0, 0)
    interpolated = state(
        "interpolated",
        "interpolated",
        10_000_000,
        12_500_000,
        15_000_000,
        2_500_000,
        2_500_000,
        0,
        4,
        0,
    )
    predicted = state(
        "predicted",
        "predicted",
        15_000_000,
        20_000_000,
        15_000_000,
        5_000_000,
        0,
        2,
        0,
        2,
    )
    assert_mode("exact", [exact])
    assert_mode("interpolated", [interpolated])
    assert_mode("predicted", [predicted])


def test_mode_summary_keeps_timing_provenance_and_selection_separate() -> None:
    states = [
        state("exact", "exact", index, index, index, 0, 0, 4, 0, 0)
        for index in (1, 2, 3)
    ]
    states[1]["metrics"]["command_selection"] = "contingency"
    summary = mode_summary(states)
    assert summary["streamed_command_provenance"] == {"exact": 3}
    assert summary["command_query_counts"]["exact"] == 12
    assert summary["selection_counts"] == {
        "primary": 2,
        "contingency": 1,
        "rejected": 0,
    }
    assert summary["solve"]["p50_us"] == 100.0
    assert percentile([3.0, 1.0, 2.0], 0.99) == 3.0
