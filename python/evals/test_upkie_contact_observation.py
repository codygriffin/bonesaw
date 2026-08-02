from __future__ import annotations

import pathlib

import numpy as np
import pytest


def test_contact_binding_debounces_and_missing_evidence_is_nonhard() -> None:
    bonesaw = pytest.importorskip("bonesaw")
    session = bonesaw.UpkieBalanceSession(
        str(pathlib.Path("models/upkie/upkie.urdf").resolve())
    )
    names = list(session.contact_observation_diagnostic_names)
    index = {name: slot for slot, name in enumerate(names)}
    raw = np.asarray([1, 1], np.uint8)
    debounced = np.empty(2, np.uint8)
    hard = np.empty(2, np.uint8)
    diagnostics = np.empty(len(names), np.int64)
    for sequence in range(1, 4):
        timestamp = sequence * 5_000_000
        session.step_contact_observation_from_mask(
            timestamp,
            True,
            timestamp,
            sequence,
            1,
            100,
            raw,
            debounced,
            hard,
            diagnostics,
        )
    np.testing.assert_array_equal(debounced, [1, 1])
    np.testing.assert_array_equal(hard, [1, 1])
    session.step_contact_observation_from_mask(
        20_000_000,
        False,
        0,
        0,
        0,
        0,
        raw,
        debounced,
        hard,
        diagnostics,
    )
    assert diagnostics[index["status"]] == 1
    assert diagnostics[index["provenance"]] == 1
    assert diagnostics[index["hard_constraint_eligible"]] == 0
    np.testing.assert_array_equal(hard, [0, 0])


def test_contact_command_lease_is_bounded_and_does_not_resurrect() -> None:
    bonesaw = pytest.importorskip("bonesaw")
    session = bonesaw.UpkieBalanceSession(
        str(pathlib.Path("models/upkie/upkie.urdf").resolve())
    )
    session.configure_contact_command_lease(2)
    names = list(session.contact_command_lease_diagnostic_names)
    index = {name: slot for slot, name in enumerate(names)}
    stable = np.asarray([1, 1], np.uint8)
    hard = np.asarray([1, 1], np.uint8)
    candidate = np.arange(6, dtype=np.float64) + 1.0
    command = np.empty(6, np.float64)
    diagnostics = np.empty(len(names), np.int64)

    session.step_contact_command_lease_from_masks(
        1, True, stable, hard, True, candidate, command, diagnostics
    )
    assert diagnostics[index["status"]] == 1
    assert diagnostics[index["provenance"]] == 1
    assert diagnostics[index["executable"]] == 1
    np.testing.assert_array_equal(command, candidate)

    hard[1] = 0
    for tick in (2, 3):
        session.step_contact_command_lease_from_masks(
            tick, True, stable, hard, False, candidate, command, diagnostics
        )
        assert diagnostics[index["status"]] == 2
        assert diagnostics[index["provenance"]] == 2
        assert diagnostics[index["executable"]] == 1
        np.testing.assert_array_equal(command, candidate)
        stable[:] = hard

    session.step_contact_command_lease_from_masks(
        4, True, stable, hard, False, candidate, command, diagnostics
    )
    assert diagnostics[index["status"]] == 3
    assert diagnostics[index["executable"]] == 0
    np.testing.assert_array_equal(command, np.zeros(6))

    session.step_contact_command_lease_from_masks(
        5, True, stable, hard, False, candidate, command, diagnostics
    )
    assert diagnostics[index["status"]] == 0
    assert diagnostics[index["executable"]] == 0
