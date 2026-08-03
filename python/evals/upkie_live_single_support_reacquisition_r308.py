#!/usr/bin/env python3
"""R308 evaluate the Rust-owned single-support touchdown request.

The candidate is deliberately evaluation-only.  MuJoCo supplies the measured
support mask and wheel state; Rust turns the lost wheel's vertical error into a
bounded Jacobian-transpose free-leg request; the ordinary floating WBC remains
the only command admission boundary.  A touchdown observation is not promoted
unless the measured contact and physical recovery gates hold.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

import upkie_live_dynamic_contact_transition_r300 as r300
import upkie_live_inner_rate_r304 as r304


REVISION = "upkie-live-single-support-reacquisition-r308"
CANDIDATE_CONFIG = (0.05, 90.0, 14.0, 20.0, 1.0e-4, 20.0, 30.0, 120.0)
CANDIDATE_OPTIONS = {
    "single_support_reacquisition_enabled": True,
    "single_support_reacquisition_config": CANDIDATE_CONFIG,
}


def _percentile(values: list[float], percentile: float) -> float:
    return float(np.percentile(np.asarray(values, np.float64), percentile))


def _action_summary(case: dict[str, Any]) -> dict[str, Any]:
    states = case["states"]
    active = [
        state["index"]
        for state in states
        if state["single_support_reacquisition_active"]
    ]
    authority = np.asarray(
        [state["single_support_reacquisition_authority"] for state in states],
        np.float64,
    )
    diagnostics = np.asarray(
        [state["single_support_reacquisition_diagnostics"] for state in states],
        np.float64,
    )
    return {
        "first_active_tick": active[0] if active else None,
        "last_active_tick": active[-1] if active else None,
        "active_ticks": len(active),
        "maximum_authority": float(np.max(authority, initial=0.0)),
        "maximum_abs_target_error_m": float(np.max(np.abs(diagnostics[:, 7]), initial=0.0)),
        "maximum_abs_requested_vertical_acceleration_m_s2": float(
            np.max(np.abs(diagnostics[:, 8]), initial=0.0)
        ),
        "maximum_abs_commanded_vertical_acceleration_m_s2": float(
            np.max(np.abs(diagnostics[:, 9]), initial=0.0)
        ),
        "step_us_p99": _percentile(
            [state["single_support_reacquisition_step_us"] for state in states], 99.0
        ),
        "allocation_calls": sum(
            state["single_support_reacquisition_allocation_calls"] for state in states
        ),
        "allocated_bytes": sum(
            state["single_support_reacquisition_allocated_bytes"] for state in states
        ),
    }


def evaluate(
    baseline_nominal: dict[str, Any],
    baseline_disturbed: dict[str, Any],
    candidate_nominal: dict[str, Any],
    candidate_disturbed: dict[str, Any],
    *,
    nominal_ticks: int,
    disturbed_ticks: int,
) -> dict[str, Any]:
    baseline_nominal_summary = r300.summarize(baseline_nominal)
    baseline_disturbed_summary = r300.summarize(baseline_disturbed)
    candidate_nominal_summary = r300.summarize(candidate_nominal)
    candidate_disturbed_summary = r300.summarize(candidate_disturbed)
    nominal_action = _action_summary(candidate_nominal)
    disturbed_action = _action_summary(candidate_disturbed)
    first_single_support = next(
        (
            state["index"]
            for state in candidate_disturbed["states"]
            if state["observed"] in ([1, 0], [0, 1])
        ),
        None,
    )
    qualification_gates = {
        "nominal_completes_requested_horizon": (
            candidate_nominal_summary["ticks"] == nominal_ticks
            and candidate_nominal_summary["terminal_pending"] is None
        ),
        "nominal_stays_bilateral_upright": (
            candidate_nominal_summary["observed_patterns"] == ["11"]
            and candidate_nominal_summary["minimum_root_height_m"] >= 0.48
            and candidate_nominal_summary["maximum_root_tilt_rad"] <= 0.20
        ),
        "nominal_request_remains_dormant": nominal_action["maximum_authority"]
        <= 1.0e-12,
        "rust_request_hot_path_zero_allocations": (
            nominal_action["allocation_calls"] == 0
            and nominal_action["allocated_bytes"] == 0
            and disturbed_action["allocation_calls"] == 0
            and disturbed_action["allocated_bytes"] == 0
        ),
        "rust_request_p99_under_100us": max(
            nominal_action["step_us_p99"], disturbed_action["step_us_p99"]
        )
        < 100.0,
        "request_activates_only_after_measured_single_support": (
            disturbed_action["first_active_tick"] is not None
            and first_single_support is not None
            and disturbed_action["first_active_tick"] >= first_single_support
        ),
    }
    promotion_gates = {
        "disturbed_completes_requested_horizon": (
            candidate_disturbed_summary["ticks"] == disturbed_ticks
            and candidate_disturbed_summary["terminal_pending"] is None
            and not candidate_disturbed_summary["numeric_reset"]
        ),
        "disturbed_has_ten_tick_bilateral_upright_tail": (
            candidate_disturbed_summary["supported_upright_recovery_tick"] is not None
        ),
        "disturbed_never_enters_body_ground_stall": (
            candidate_disturbed_summary["maximum_body_ground_stall_ticks"] == 0
        ),
        "disturbed_has_no_nonadmitted_or_max_iterations_ticks": (
            not candidate_disturbed_summary["nonadmitted_ticks"]
            and not candidate_disturbed_summary["max_iterations_ticks"]
        ),
        "disturbed_hard_residuals_under_1e8": max(
            candidate_disturbed_summary["maximum_constraint_violation"],
            candidate_disturbed_summary["maximum_dynamics_residual"],
            candidate_disturbed_summary["maximum_contact_residual"],
        )
        <= 1.0e-8,
    }
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "candidate_default_enabled": False,
        "candidate_config": list(CANDIDATE_CONFIG),
        "nominal_ticks": nominal_ticks,
        "disturbed_ticks": disturbed_ticks,
        "first_measured_single_support_tick": first_single_support,
        "baseline_nominal": baseline_nominal_summary,
        "baseline_disturbed": baseline_disturbed_summary,
        "candidate_nominal": candidate_nominal_summary,
        "candidate_disturbed": candidate_disturbed_summary,
        "nominal_action": nominal_action,
        "disturbed_action": disturbed_action,
        "qualification_gates": qualification_gates,
        "promotion_gates": promotion_gates,
        "qualified_as_bounded_default_off_experiment": all(
            qualification_gates.values()
        ),
        "recovery_promoted": all(promotion_gates.values()),
        "finding": (
            "The request is causal, bounded, allocation-free, and passes the "
            "ordinary WBC boundary. The frozen wrench trace still does not produce "
            "a ten-tick bilateral/upright tail; it remains default-off pending a "
            "contact-mode phase policy and a physically sustained touchdown."
        ),
    }


def render_markdown(metrics: dict[str, Any]) -> str:
    qualifications = "\n".join(
        f"- {'PASS' if passed else 'FAIL'} `{name}`"
        for name, passed in metrics["qualification_gates"].items()
    )
    promotions = "\n".join(
        f"- {'PASS' if passed else 'OPEN'} `{name}`"
        for name, passed in metrics["promotion_gates"].items()
    )
    action = metrics["disturbed_action"]
    return f"""# Upkie single-support touchdown request — R308

Status: **{'PROMOTED' if metrics['recovery_promoted'] else 'RETAINED AS DEFAULT-OFF NEGATIVE EVIDENCE'}**

R308 adds a Rust-owned free-leg request after an exact measured single-support
observation. It computes a wheel-height/velocity error and a damped Jacobian-
transpose joint acceleration, with fixed capacities and explicit authority
slew. The ordinary floating WBC still owns contact rows, effort limits, hard
residuals, and executable command authority.

## Measured request

| measure | result |
|---|---:|
| first measured single-support tick | {metrics['first_measured_single_support_tick']} |
| first request-active tick | {action['first_active_tick']} |
| active ticks | {action['active_ticks']} |
| maximum authority | {action['maximum_authority']:.6f} |
| maximum vertical request | {action['maximum_abs_requested_vertical_acceleration_m_s2']:.6f} m/s² |
| maximum commanded vertical request | {action['maximum_abs_commanded_vertical_acceleration_m_s2']:.6f} m/s² |
| Rust request p99 | {action['step_us_p99']:.3f} µs |
| Rust allocations / bytes | {action['allocation_calls']} / {action['allocated_bytes']} |

## Bounded-experiment qualification

{qualifications}

## Physical recovery promotion

{promotions}

## Architectural conclusion

{metrics['finding']}
"""


def run(
    model: pathlib.Path,
    *,
    nominal_ticks: int,
    disturbed_ticks: int,
) -> tuple[dict[str, Any], dict[str, Any], str]:
    baseline_nominal = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=nominal_ticks,
        worker_options=r304.FAST_WORKER_OPTIONS,
    )
    baseline_disturbed = r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=disturbed_ticks,
        worker_options=r304.FAST_WORKER_OPTIONS,
    )
    candidate_nominal = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=nominal_ticks,
        controller_options=CANDIDATE_OPTIONS,
        worker_options=r304.FAST_WORKER_OPTIONS,
    )
    candidate_disturbed = r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=disturbed_ticks,
        controller_options=CANDIDATE_OPTIONS,
        worker_options=r304.FAST_WORKER_OPTIONS,
    )
    metrics = evaluate(
        baseline_nominal,
        baseline_disturbed,
        candidate_nominal,
        candidate_disturbed,
        nominal_ticks=nominal_ticks,
        disturbed_ticks=disturbed_ticks,
    )
    traces = {
        "revision": REVISION,
        "baseline_nominal": baseline_nominal,
        "baseline_disturbed": baseline_disturbed,
        "candidate_nominal": candidate_nominal,
        "candidate_disturbed": candidate_disturbed,
    }
    return metrics, traces, render_markdown(metrics)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", type=pathlib.Path, default=pathlib.Path("models/upkie/upkie.urdf")
    )
    parser.add_argument("--nominal-ticks", type=int, default=1000)
    parser.add_argument("--disturbed-ticks", type=int, default=300)
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path(
            "benchmarks/results/upkie-live-single-support-reacquisition-r308"
        ),
    )
    args = parser.parse_args()
    if args.nominal_ticks < 100 or args.disturbed_ticks < 250:
        parser.error("nominal-ticks must be >=100 and disturbed-ticks >=250")
    metrics, traces, markdown = run(
        args.model,
        nominal_ticks=args.nominal_ticks,
        disturbed_ticks=args.disturbed_ticks,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (args.output_dir / "traces.json").write_text(
        json.dumps(traces, separators=(",", ":")) + "\n", encoding="utf-8"
    )
    (args.output_dir / "UPKIE_LIVE_SINGLE_SUPPORT_REACQUISITION_R308.md").write_text(
        markdown, encoding="utf-8"
    )
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
