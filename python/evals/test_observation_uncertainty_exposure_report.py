from __future__ import annotations

import copy

import pytest

from observation_uncertainty_exposure_report import validate_audit


def row(exposure_ns: int, q: float, point: float, stopping: float) -> dict:
    return {
        "exposure_ns": exposure_ns,
        "inside_live_prediction_horizon": exposure_ns <= 5_000_000,
        "error_bound": {
            "joint_position_error_rad": q,
            "represented_point_position_error_m": point,
        },
        "joint_stopping_upper_erosion_rad_s2": stopping,
        "self_collision_raw_margin_m": 0.01,
        "self_collision_robust_margin_m": 0.01 - 2.0 * point,
        "self_collision_margin_erosion_m": 2.0 * point,
        "error_bound_allocation_calls": 0,
        "error_bound_allocated_bytes": 0,
        "solve": {
            "allocation_calls": 0,
            "allocated_bytes": 0,
            "bitwise_repeat": True,
        },
    }


def audit() -> dict:
    return {
        "status": "pass",
        "process_exit_code": 0,
        "execution": "fixed_state_without_policy_physics_or_integration",
        "nominal": {"collision_margin_m": 0.01},
        "sweep": [
            row(0, 0.0, 0.0, 0.0),
            row(2_500_000, 0.0002, 0.0001, 5.0),
            row(5_000_000, 0.0005, 0.0003, 11.0),
            row(7_500_000, 0.0008, 0.0004, 17.0),
        ],
    }


def test_validate_audit_accepts_monotone_allocation_free_contract() -> None:
    validate_audit(audit())


def test_validate_audit_rejects_raw_geometry_drift() -> None:
    candidate = copy.deepcopy(audit())
    candidate["sweep"][2]["self_collision_raw_margin_m"] = 0.009
    with pytest.raises(AssertionError):
        validate_audit(candidate)


def test_validate_audit_rejects_live_horizon_mislabel() -> None:
    candidate = copy.deepcopy(audit())
    candidate["sweep"][3]["inside_live_prediction_horizon"] = True
    with pytest.raises(AssertionError):
        validate_audit(candidate)
