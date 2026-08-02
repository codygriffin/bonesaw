import pathlib

import numpy as np
import pytest

from upkie_state_local_authority import (
    AXES,
    CONTACT_MODES,
    DEMANDS,
    build_probe_rows,
    run_probe,
    summarize,
)


def test_probe_matrix_covers_modes_axes_signs_and_saturation_curve() -> None:
    rows = build_probe_rows()
    assert len(rows) == len(CONTACT_MODES) * (1 + len(AXES) * len(DEMANDS) * 2)
    for mode in CONTACT_MODES:
        selected = [row for row in rows if row["mode"] == mode]
        assert sum(row["axis"] == "baseline" for row in selected) == 1
        for axis in AXES:
            demands = {
                row["demand"] for row in selected if row["axis"] == axis
            }
            assert demands == {
                sign * magnitude
                for sign in (-1.0, 1.0)
                for magnitude in DEMANDS
            }


def test_state_local_rust_probe_is_hard_feasible_and_allocation_free() -> None:
    pytest.importorskip("bonesaw")
    model = pathlib.Path("models/upkie/upkie.urdf").resolve()
    rows, raw = run_probe(model)
    metrics = summarize(rows, raw)
    assert metrics["allocation_calls"] == 0
    assert metrics["allocated_bytes"] == 0
    assert metrics["maximum_hard_violation"] < 1.0e-8
    assert metrics["maximum_dynamics_residual"] < 1.0e-8
    assert metrics["maximum_contact_residual"] < 1.0e-8
    assert set(metrics["status_counts"]) <= {"Solved", "SolvedWithSlack"}
    assert np.all(np.isfinite(raw["generalized_acceleration"]))
    assert len(metrics["envelope_250"]) == len(CONTACT_MODES) * len(AXES)

    rolling_roll = next(
        row
        for row in metrics["envelope_250"]
        if row["mode"] == "rolling_wheel" and row["axis"] == "roll"
    )
    assert 0.0 < rolling_roll["minimum_small_signal_fraction"] < 1.0
    assert rolling_roll["minimum_signed_fraction"] < 0.1
