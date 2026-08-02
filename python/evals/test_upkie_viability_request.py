from __future__ import annotations

import pathlib

import numpy as np
import pytest


MODEL = pathlib.Path("models/upkie/upkie.urdf").resolve()


def configured_session():
    bonesaw = pytest.importorskip("bonesaw")
    session = bonesaw.UpkieBalanceSession(str(MODEL))
    session.configure_viability_request(
        0.10,
        0.05,
        2,
        np.asarray([100.0, 100.0, 50.0], np.float64),
        np.asarray([25.0, 20.0, 10.0], np.float64),
    )
    names = list(session.viability_request_diagnostic_names)
    return session, {name: slot for slot, name in enumerate(names)}


def step(
    session,
    tick: int,
    *,
    exact: bool = True,
    update: bool = False,
    pressure: float = 0.0,
    available: bool = False,
    candidate: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[np.ndarray, np.ndarray]:
    request = np.empty(3, np.float64)
    diagnostics = np.empty(len(session.viability_request_diagnostic_names), np.float64)
    session.step_viability_request(
        tick,
        exact,
        update,
        pressure,
        available,
        np.asarray(candidate, np.float64),
        request,
        diagnostics,
    )
    return request, diagnostics


def test_viability_request_hysteresis_hold_and_release_are_bounded() -> None:
    session, index = configured_session()

    request, diagnostics = step(
        session,
        1,
        update=True,
        pressure=0.11,
        available=True,
        candidate=(80.0, -60.0, 30.0),
    )
    assert diagnostics[index["status"]] == 1
    assert diagnostics[index["provenance"]] == 1
    assert diagnostics[index["active"]] == 1
    assert diagnostics[index["executable"]] == 1
    np.testing.assert_array_equal(request, [25.0, -20.0, 10.0])

    request, diagnostics = step(session, 2)
    assert diagnostics[index["status"]] == 2
    assert diagnostics[index["candidate_age_ticks"]] == 1
    np.testing.assert_array_equal(request, [50.0, -40.0, 20.0])

    request, diagnostics = step(session, 3, update=True, pressure=0.04)
    assert diagnostics[index["status"]] == 3
    assert diagnostics[index["active"]] == 0
    np.testing.assert_array_equal(request, [25.0, -20.0, 10.0])

    request, diagnostics = step(session, 4)
    assert diagnostics[index["status"]] == 0
    assert diagnostics[index["executable"]] == 0
    np.testing.assert_array_equal(request, np.zeros(3))


def test_viability_request_expiry_and_evidence_loss_revoke_without_tail() -> None:
    session, index = configured_session()
    step(
        session,
        10,
        update=True,
        pressure=0.2,
        available=True,
        candidate=(80.0, 0.0, 0.0),
    )
    step(session, 11)
    step(session, 12)
    request, diagnostics = step(session, 13)
    assert diagnostics[index["status"]] == 4
    assert diagnostics[index["executable"]] == 0
    np.testing.assert_array_equal(request, np.zeros(3))

    step(
        session,
        14,
        update=True,
        pressure=0.2,
        available=True,
        candidate=(80.0, 0.0, 0.0),
    )
    request, diagnostics = step(session, 15, exact=False)
    assert diagnostics[index["status"]] == 5
    assert diagnostics[index["executable"]] == 0
    np.testing.assert_array_equal(request, np.zeros(3))


def test_viability_request_reordered_tick_is_atomic_and_bad_input_fails_closed() -> None:
    session, index = configured_session()
    step(
        session,
        20,
        update=True,
        pressure=0.2,
        available=True,
        candidate=(80.0, 0.0, 0.0),
    )
    request, diagnostics = step(session, 20)
    assert diagnostics[index["status"]] == 6
    assert diagnostics[index["executable"]] == 0
    np.testing.assert_array_equal(request, np.zeros(3))

    request, diagnostics = step(session, 21)
    assert diagnostics[index["status"]] == 2
    assert diagnostics[index["candidate_age_ticks"]] == 1
    np.testing.assert_array_equal(request, [50.0, 0.0, 0.0])

    request, diagnostics = step(
        session,
        22,
        update=True,
        pressure=0.2,
        available=True,
        candidate=(101.0, 0.0, 0.0),
    )
    assert diagnostics[index["status"]] == 7
    assert diagnostics[index["executable"]] == 0
    np.testing.assert_array_equal(request, np.zeros(3))


def test_viability_request_configuration_rejects_bad_threshold_order() -> None:
    bonesaw = pytest.importorskip("bonesaw")
    session = bonesaw.UpkieBalanceSession(str(MODEL))
    with pytest.raises(ValueError):
        session.configure_viability_request(
            0.05,
            0.10,
            2,
            np.ones(3, np.float64),
            np.ones(3, np.float64),
        )


def test_planner_feasibility_iteration_budget_is_bounded_at_construction() -> None:
    bonesaw = pytest.importorskip("bonesaw")
    with pytest.raises(ValueError):
        bonesaw.FloatingWbcSession(str(MODEL), maximum_feasibility_iterations=0)
    bonesaw.FloatingWbcSession(str(MODEL), maximum_feasibility_iterations=8)


def test_multistep_forecast_batch_is_separate_repeatable_and_allocation_free() -> None:
    bonesaw = pytest.importorskip("bonesaw")
    session = bonesaw.UpkieBalanceSession(str(MODEL))
    names = list(session.viability_forecast_diagnostic_names)
    index = {name: slot for slot, name in enumerate(names)}
    state = np.asarray(
        [0.12, 0.40, 0.04, 0.10, 0.025, 0.20, 0.05, 0.10, 0.55, 3.0]
    )
    achieved = np.asarray([[0.0, 0.0, 0.0, 0.0], [-8.0, -4.0, 0.0, -1.0]])
    request = achieved[:, :3].copy()
    previous = np.zeros(3, np.float64)
    maximum = np.asarray([250.0, 250.0, 80.0])
    torque = np.asarray([0.40, 0.40])
    joint = np.asarray([0.80, 0.80])
    output = np.empty((2, len(names)), np.float64)
    replay = np.empty_like(output)

    _, allocation_calls, allocated_bytes = session.score_viability_forecast_batch(
        state, achieved, request, previous, maximum, torque, joint, output
    )
    session.score_viability_forecast_batch(
        state, achieved, request, previous, maximum, torque, joint, replay
    )

    assert allocation_calls == 0
    assert allocated_bytes == 0
    np.testing.assert_array_equal(output, replay)
    assert output[1, index["total_score"]] < output[0, index["total_score"]]
    assert output[1, index["terminal_capture_pressure"]] < output[
        0, index["terminal_capture_pressure"]
    ]
    assert output[0, index["support_pressure"]] == 0.0
    assert np.all(np.isfinite(output))


def test_rust_viability_poll_is_fixed_cyclic_atomic_and_allocation_free() -> None:
    bonesaw = pytest.importorskip("bonesaw")
    session = bonesaw.UpkieBalanceSession(str(MODEL))
    previous = np.zeros(3, np.float64)
    maximum = np.asarray([40.0, 40.0, 20.0])
    step = maximum.copy()
    proposal = np.empty(3, np.float64)
    diagnostics = np.empty(
        len(session.viability_poll_diagnostic_names), np.float64
    )
    expected = (
        [-40.0, 0.0, 0.0],
        [40.0, 0.0, 0.0],
        [0.0, -40.0, 0.0],
        [0.0, 40.0, 0.0],
        [0.0, 0.0, -20.0],
        [0.0, 0.0, 20.0],
    )
    for phase, wanted in enumerate(expected):
        _, allocation_calls, allocated_bytes = session.next_viability_poll(
            previous, maximum, step, proposal, diagnostics
        )
        assert allocation_calls == 0
        assert allocated_bytes == 0
        np.testing.assert_array_equal(proposal, np.asarray(wanted))
        assert diagnostics[2] == phase

    with pytest.raises(ValueError):
        session.next_viability_poll(
            np.asarray([np.nan, 0.0, 0.0]),
            maximum,
            step,
            proposal,
            diagnostics,
        )
    session.next_viability_poll(previous, maximum, step, proposal, diagnostics)
    assert diagnostics[2] == 0.0
