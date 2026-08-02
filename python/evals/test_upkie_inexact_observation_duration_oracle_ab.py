from __future__ import annotations

import numpy as np

from upkie_inexact_observation_duration_oracle_ab import (
    ONE_Q15,
    gain_name,
    profiles,
    scaled_hold_contract,
)


def test_scaled_hold_contract_requires_unavailable_q15_exact_effort() -> None:
    trace = {
        "contact_program_authority_selection": np.asarray([1, 4], np.uint8),
        "contact_observation_available": np.asarray([1, 0], np.uint8),
        "torque": np.asarray([[4.0, 8.0], [1.0, 2.0]], np.float64),
    }
    result = scaled_hold_contract(trace, 0.25)
    assert result["configured_q15"] == ONE_Q15 // 4
    assert result["typed_hold_ticks"] == 1
    assert result["typed_hold_only_when_unavailable"]
    assert result["typed_hold_matches_scaled_preceding_effort"]

    trace["torque"][1, 1] = 2.5
    assert not scaled_hold_contract(trace, 0.25)[
        "typed_hold_matches_scaled_preceding_effort"
    ]


def test_duration_oracle_profiles_keep_hold_zero_distinct_from_typed_zero_gain() -> None:
    matrix = profiles()
    assert gain_name(0.45) == "g045"
    assert matrix["drop5_hold0"][0] == 0
    assert matrix["drop5_g000"][:2] == (1, 0.0)
    assert matrix["drop10_g100"][:2] == (1, 1.0)
