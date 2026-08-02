from __future__ import annotations

import numpy as np

from upkie_current_support_realization_plant_ab import (
    PLANT_COMMAND_FIELDS,
    exact_terminal_outcome,
    plant_command_trace_equal,
)


def test_plant_command_trace_equal_checks_authoritative_command() -> None:
    left = {field: np.zeros(2) for field in PLANT_COMMAND_FIELDS}
    left.update(
        termination_reason="completed",
        terminal_time_s=1.0,
        terminal_root_position=np.zeros(3),
        terminal_rotation_vector=np.zeros(3),
    )
    right = {
        key: value.copy() if isinstance(value, np.ndarray) else value
        for key, value in left.items()
    }
    assert plant_command_trace_equal(left, right)
    right["torque"][0] = 1.0
    assert not plant_command_trace_equal(left, right)


def test_exact_terminal_outcome_rejects_boundary_change() -> None:
    left = {
        "fell": True,
        "numeric_fault": False,
        "termination_reason": "fall",
        "terminal_time_s": 2.0,
        "qualified": False,
    }
    right = dict(left)
    assert exact_terminal_outcome(left, right)
    right["terminal_time_s"] = 2.005
    assert not exact_terminal_outcome(left, right)
