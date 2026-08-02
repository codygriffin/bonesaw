from __future__ import annotations

import pathlib

import numpy as np

from upkie_support_contingency_admission import admit, author, corpus


MODEL = pathlib.Path(__file__).resolve().parents[2] / "models/upkie/upkie.urdf"


def test_four_support_modes_author_and_admit_end_to_end() -> None:
    states = corpus(16)
    authored = author(MODEL, states)
    assert set(np.unique(authored["diagnostics"][:, 0]).tolist()) == {0.0, 1.0, 2.0}
    assert set(np.unique(authored["diagnostics"][:, 1]).tolist()) == {0.0, 1.0, 2.0, 3.0}
    flight = states["mask"] == 0
    assert np.array_equal(
        authored["linear"][flight], np.tile([0.0, 0.0, -9.81], (16, 1))
    )
    assert int(np.sum(authored["allocation_calls"])) == 0
    assert int(np.sum(authored["allocated_bytes"])) == 0

    admitted = admit(MODEL, states, authored)
    assert np.all(np.isin(admitted["status"], (0, 1)))
    assert float(np.max(admitted["maximum_constraint_violation"])) < 1.0e-8
    assert int(np.sum(admitted["allocation_calls"])) == 0
    assert int(np.sum(admitted["allocated_bytes"])) == 0
