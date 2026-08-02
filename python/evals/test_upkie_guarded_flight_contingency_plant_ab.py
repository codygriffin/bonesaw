from __future__ import annotations

import numpy as np

from upkie_guarded_flight_contingency_plant_ab import command_trace_equal


def test_command_trace_equal_checks_every_authoritative_field() -> None:
    from upkie_guarded_flight_contingency_plant_ab import COMMAND_FIELDS

    left = {field: np.zeros(2) for field in COMMAND_FIELDS}
    left.update(termination_reason="completed", terminal_time_s=1.0)
    right = {
        key: value.copy() if isinstance(value, np.ndarray) else value
        for key, value in left.items()
    }
    assert command_trace_equal(left, right)
    right["torque"][0] = 1.0
    assert not command_trace_equal(left, right)
