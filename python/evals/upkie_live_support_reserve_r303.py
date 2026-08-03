#!/usr/bin/env python3
"""R303 causal wheel-load reserve witness.

This is deliberately an authority-boundary evaluation, not an executed policy.
It identifies the first measured bilateral wheel-load reserve crossing before
the MuJoCo contact mask changes.  A future Rust action can consume this exact
witness; until that action is independently admitted, this module withholds
all actuator authority.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from typing import Any

import numpy as np

import upkie_live_dynamic_contact_transition_r300 as r300


REVISION = "upkie-live-support-reserve-r303"
DEFAULT_MINIMUM_LOAD_FRACTION = 0.45


def _finite_force_pair(state: dict[str, Any]) -> np.ndarray | None:
    values = np.asarray(state["observed_wheel_normal_force_n"], np.float64)
    if values.shape != (2,) or not np.all(np.isfinite(values)):
        return None
    return np.maximum(values, 0.0)


def reserve_fraction(state: dict[str, Any]) -> float:
    """Return the weaker measured wheel's fraction of total normal load."""
    forces = _finite_force_pair(state)
    if forces is None:
        return math.nan
    total = float(np.sum(forces))
    return float(np.min(forces) / total) if total > 1.0e-9 else math.nan


def reserve_gate(
    state: dict[str, Any],
    previous_state: dict[str, Any] | None,
    *,
    minimum_load_fraction: float = DEFAULT_MINIMUM_LOAD_FRACTION,
) -> bool:
    """Bounded, fail-closed pre-loss witness; never emits a command.

    The trend clause prevents a one-sample static imbalance from arming the
    future action.  Invalid, non-bilateral, or zero-load observations are
    rejected rather than guessed.
    """
    if not math.isfinite(minimum_load_fraction) or not (
        0.0 < minimum_load_fraction < 0.5
    ):
        raise ValueError("minimum_load_fraction must be finite and in (0, 0.5)")
    if state["observed"] != [1, 1] or previous_state is None:
        return False
    current = reserve_fraction(state)
    previous = reserve_fraction(previous_state)
    return (
        math.isfinite(current)
        and math.isfinite(previous)
        and current <= minimum_load_fraction
        and current < previous
    )


def first_physics_loss_tick(states: list[dict[str, Any]]) -> int | None:
    return next(
        (
            int(state["index"])
            for state in states
            if state["physics_contact_active"] != [1, 1]
        ),
        None,
    )


def first_observed_loss_tick(states: list[dict[str, Any]]) -> int | None:
    return next(
        (int(state["index"]) for state in states if state["observed"] != [1, 1]),
        None,
    )


def first_reserve_trigger_tick(
    states: list[dict[str, Any]],
    *,
    minimum_load_fraction: float = DEFAULT_MINIMUM_LOAD_FRACTION,
    before_tick: int | None = None,
) -> int | None:
    for index, state in enumerate(states):
        if before_tick is not None and int(state["index"]) >= before_tick:
            break
        previous = states[index - 1] if index else None
        if reserve_gate(
            state,
            previous,
            minimum_load_fraction=minimum_load_fraction,
        ):
            return int(state["index"])
    return None


def evaluate(
    control: dict[str, Any],
    disturbed: dict[str, Any],
    *,
    minimum_load_fraction: float = DEFAULT_MINIMUM_LOAD_FRACTION,
) -> dict[str, Any]:
    control_states = control["states"]
    disturbed_states = disturbed["states"]
    physics_loss = first_physics_loss_tick(disturbed_states)
    trigger = first_reserve_trigger_tick(
        disturbed_states,
        minimum_load_fraction=minimum_load_fraction,
    )
    observed_loss = first_observed_loss_tick(disturbed_states)
    control_trigger = first_reserve_trigger_tick(
        control_states,
        minimum_load_fraction=minimum_load_fraction,
        # Compare the nominal trace over the same upright pre-loss horizon;
        # once a control trace is allowed to fall, body contacts are no longer
        # a useful false-positive test for the wheel-reserve witness.
        before_tick=physics_loss,
    )
    return {
        "revision": REVISION,
        "minimum_load_fraction": minimum_load_fraction,
        "control_trigger_tick": control_trigger,
        "disturbed_trigger_tick": trigger,
        "disturbed_physics_loss_tick": physics_loss,
        "disturbed_observed_loss_tick": observed_loss,
        "trigger_load_n": (
            None
            if trigger is None
            else [
                float(value)
                for value in disturbed_states[trigger][
                    "observed_wheel_normal_force_n"
                ]
            ]
        ),
        "trigger_load_fraction": (
            None if trigger is None else reserve_fraction(disturbed_states[trigger])
        ),
        "pre_loss_lead_ticks": (
            None if trigger is None or physics_loss is None else physics_loss - trigger
        ),
        # This artifact is intentionally telemetry-only.  A future R303 Rust
        # action must report its own admission/selection witness separately.
        "actuator_authority_emitted": False,
        "gates": {
            "control_has_no_false_trigger": control_trigger is None,
            "finite_force_trace": all(
                _finite_force_pair(state) is not None
                for state in (*control_states, *disturbed_states)
            ),
            "trigger_precedes_physics_loss": (
                trigger is not None
                and physics_loss is not None
                and trigger < physics_loss
            ),
            "trigger_does_not_claim_observed_loss": (
                trigger is None
                or observed_loss is None
                or trigger < observed_loss
            ),
        },
    }


def guard_probe_summary(case: dict[str, Any]) -> dict[str, Any]:
    """Summarize the separate default-off load-guard experiment."""
    summary = r300.summarize(case)
    return {
        "evaluation_override": True,
        "promoted": False,
        "ticks": int(summary["ticks"]),
        "terminal_tick": summary["terminal_tick"],
        "first_physics_loss_tick": first_physics_loss_tick(case["states"]),
        "first_observed_loss_tick": first_observed_loss_tick(case["states"]),
        "controller_step_us": summary["controller_step_us"],
        "max_iterations_ticks": summary["max_iterations_ticks"],
        "allocation_calls": int(summary["allocation_calls"]),
        "allocated_bytes": int(summary["allocated_bytes"]),
        "strict_recovery_tick": summary["supported_upright_recovery_tick"],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--maximum-ticks", type=int, default=80)
    parser.add_argument(
        "--minimum-load-fraction",
        type=float,
        default=DEFAULT_MINIMUM_LOAD_FRACTION,
    )
    args = parser.parse_args()
    model = pathlib.Path(args.model)
    disturbed = r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=args.maximum_ticks,
    )
    control = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=len(disturbed["states"]),
    )
    guard = r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=args.maximum_ticks,
        controller_options={"support_load_guard_enabled": True},
    )
    metrics = evaluate(
        control,
        disturbed,
        minimum_load_fraction=args.minimum_load_fraction,
    )
    metrics["guard_probe"] = guard_probe_summary(guard)
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0 if all(metrics["gates"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
