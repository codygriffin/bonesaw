#!/usr/bin/env python3
"""R307 qualify a Rust-owned measured wheel-load reserve action.

Python owns immutable cases, aggregation, and report rendering. The persistent
Rust balance session owns filtering, load trend, support geometry, DCM action,
slew, and authority. The candidate remains default-off unless every sustained
plant-consequence gate passes.
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


REVISION = "upkie-live-load-reserve-action-r307"
DIAGNOSTIC_INDEX = {
    "evidence_available": 0,
    "active": 1,
    "weaker_support_index": 2,
    "total_normal_force_n": 3,
    "support_0_load_fraction": 4,
    "support_1_load_fraction": 5,
    "raw_load_balance": 6,
    "filtered_load_balance": 7,
    "weaker_load_fraction": 8,
    "weaker_load_fraction_rate_per_s": 9,
    "predicted_weaker_load_fraction": 10,
    "support_center_m": 11,
    "half_support_track_m": 12,
    "measured_cop_m": 13,
    "lateral_dcm_m": 14,
    "baseline_zmp_m": 15,
    "restoring_zmp_m": 16,
    "commanded_zmp_m": 17,
    "requested_lateral_acceleration_m_s2": 18,
    "commanded_lateral_acceleration_m_s2": 19,
    "target_bank_angle_rad": 20,
    "commanded_bank_angle_rad": 21,
    "commanded_roll_acceleration_rad_s2": 22,
    "activation_pressure": 23,
    "authority": 24,
    "zmp_was_saturated": 25,
    "acceleration_was_saturated": 26,
    "bank_was_saturated": 27,
    "heading_world_rad": 28,
}

# Conservative attack is the best bounded CPU-only profile from the R307
# search. It is evidence, not a production default.
CANDIDATE_CONFIG = (
    0.47,
    0.38,
    0.005,
    0.030,
    0.040,
    0.012,
    1.50,
    5.00,
    0.020,
    10.0,
    12.0,
    8.0,
    80.0,
    np.deg2rad(25.0),
    3.0,
    80.0,
    14.0,
    80.0,
)
CANDIDATE_OPTIONS = {
    "support_load_reserve_action_enabled": True,
    "support_load_reserve_config": CANDIDATE_CONFIG,
}


def _percentile(values: list[float], percentile: float) -> float:
    return float(np.percentile(np.asarray(values, np.float64), percentile))


def _action_summary(case: dict[str, Any]) -> dict[str, Any]:
    states = case["states"]
    diagnostics = np.asarray(
        [state["support_load_reserve_diagnostics"] for state in states],
        np.float64,
    )
    active_ticks = [
        state["index"] for state in states if state["support_load_reserve_active"]
    ]
    return {
        "first_active_tick": active_ticks[0] if active_ticks else None,
        "last_active_tick": active_ticks[-1] if active_ticks else None,
        "maximum_authority": float(
            np.max(diagnostics[:, DIAGNOSTIC_INDEX["authority"]], initial=0.0)
        ),
        "minimum_filtered_weaker_load_fraction": float(
            np.min(
                diagnostics[:, DIAGNOSTIC_INDEX["weaker_load_fraction"]],
                initial=0.5,
            )
        ),
        "maximum_abs_lateral_dcm_m": float(
            np.max(
                np.abs(diagnostics[:, DIAGNOSTIC_INDEX["lateral_dcm_m"]]),
                initial=0.0,
            )
        ),
        "maximum_abs_commanded_lateral_acceleration_m_s2": float(
            np.max(
                np.abs(
                    diagnostics[
                        :,
                        DIAGNOSTIC_INDEX[
                            "commanded_lateral_acceleration_m_s2"
                        ],
                    ]
                ),
                initial=0.0,
            )
        ),
        "step_us_p99": _percentile(
            [state["support_load_reserve_step_us"] for state in states], 99.0
        ),
        "allocation_calls": sum(
            state["support_load_reserve_allocation_calls"] for state in states
        ),
        "allocated_bytes": sum(
            state["support_load_reserve_allocated_bytes"] for state in states
        ),
    }


def _stable_tail(states: list[dict[str, Any]], tail_ticks: int) -> bool:
    if len(states) < tail_ticks:
        return False
    return all(
        state["observed"] == [1, 1]
        and state["root_height_m"] >= 0.48
        and state["root_tilt_rad"] <= 0.20
        and state["automatic_reset_pending"] is None
        for state in states[-tail_ticks:]
    )


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
    stable_tail_ticks = min(250, disturbed_ticks)
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
        "nominal_action_remains_dormant": nominal_action["maximum_authority"]
        <= 1.0e-12,
        "rust_action_hot_path_zero_allocations": (
            nominal_action["allocation_calls"] == 0
            and nominal_action["allocated_bytes"] == 0
            and disturbed_action["allocation_calls"] == 0
            and disturbed_action["allocated_bytes"] == 0
        ),
        "rust_action_p99_under_100us": max(
            nominal_action["step_us_p99"], disturbed_action["step_us_p99"]
        )
        < 100.0,
        "disturbance_activates_before_first_support_loss": (
            disturbed_action["first_active_tick"] is not None
            and candidate_disturbed_summary["first_non_double_tick"] is not None
            and disturbed_action["first_active_tick"]
            < candidate_disturbed_summary["first_non_double_tick"]
        ),
    }
    promotion_gates = {
        "disturbed_completes_requested_horizon": (
            candidate_disturbed_summary["ticks"] == disturbed_ticks
            and candidate_disturbed_summary["terminal_pending"] is None
            and not candidate_disturbed_summary["numeric_reset"]
        ),
        "disturbed_has_five_second_bilateral_upright_tail": _stable_tail(
            candidate_disturbed["states"], stable_tail_ticks
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
        "stable_tail_ticks": stable_tail_ticks,
        "stable_tail_seconds": stable_tail_ticks * 0.020,
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
            "Measured load reserve is a useful causal trigger, but this root-task-only "
            "action does not achieve sustained wheel-supported recovery. The next "
            "controller chunk is contact-mode-aware reacquisition, not more timeout."
        ),
    }


def render_markdown(metrics: dict[str, Any]) -> str:
    baseline = metrics["baseline_disturbed"]
    candidate = metrics["candidate_disturbed"]
    qualifications = "\n".join(
        f"- {'PASS' if passed else 'FAIL'} `{name}`"
        for name, passed in metrics["qualification_gates"].items()
    )
    promotions = "\n".join(
        f"- {'PASS' if passed else 'OPEN'} `{name}`"
        for name, passed in metrics["promotion_gates"].items()
    )
    return f"""# Upkie measured load-reserve action — R307

Status: **{'PROMOTED' if metrics['recovery_promoted'] else 'RETAINED AS DEFAULT-OFF NEGATIVE EVIDENCE'}**

The Rust action is causal, bounded, allocation-free, and nominally dormant. It
activates at stream tick {metrics['disturbed_action']['first_active_tick']}. The
baseline first loses bilateral support at tick {baseline['first_non_double_tick']}
and reaches its fall boundary at tick {baseline['terminal_tick']}; the candidate
first loses bilateral support at tick {candidate['first_non_double_tick']} and
reaches its fall boundary at tick {candidate['terminal_tick']}.

That delay is not recovery. Promotion requires a {metrics['stable_tail_seconds']:.1f} s
bilateral upright tail, no body-ground stall, no non-admitted solve, and no late
fall. The candidate fails those consequence gates and remains default-off.

## Measured action

| measure | result |
|---|---:|
| maximum authority | {metrics['disturbed_action']['maximum_authority']:.6f} |
| minimum filtered weaker-wheel fraction | {metrics['disturbed_action']['minimum_filtered_weaker_load_fraction']:.6f} |
| maximum absolute lateral DCM | {metrics['disturbed_action']['maximum_abs_lateral_dcm_m']:.6f} m |
| maximum commanded lateral acceleration | {metrics['disturbed_action']['maximum_abs_commanded_lateral_acceleration_m_s2']:.6f} m/s² |
| Rust action p99 | {metrics['disturbed_action']['step_us_p99']:.3f} µs |
| Rust allocations / bytes | {metrics['disturbed_action']['allocation_calls']} / {metrics['disturbed_action']['allocated_bytes']} |

## Bounded-experiment qualification

{qualifications}

## Recovery promotion

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
        "--model",
        type=pathlib.Path,
        default=pathlib.Path("models/upkie/upkie.urdf"),
    )
    parser.add_argument("--nominal-ticks", type=int, default=1000)
    parser.add_argument("--disturbed-ticks", type=int, default=300)
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path(
            "benchmarks/results/upkie-live-load-reserve-action-r307"
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
    (args.output_dir / "UPKIE_LIVE_LOAD_RESERVE_ACTION_R307.md").write_text(
        markdown, encoding="utf-8"
    )
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0 if metrics["qualified_as_bounded_default_off_experiment"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
