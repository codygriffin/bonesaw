#!/usr/bin/env python3
"""R309 live 250/50 measured-landing phase and contact-mode evaluation.

This fixture keeps the public-rate MuJoCo worker (250 Hz integration, 50 Hz
Rust WBC) and compares an explicitly enabled R309 phase-aware landing
candidate with the same receiver/controller profile with the candidate off.
The candidate is deliberately evaluation-only and default-off.  A positive
result requires causal measured activation, bounded phase/mode transitions,
finite allocation-free Rust hot-path output, and a nominal run that remains
quiet.  A fall in the disturbed trace is retained as explicit negative
recovery evidence instead of being hidden by the worker's automatic reset.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

import upkie_live_dynamic_contact_transition_r300 as r300


REVISION = "upkie-live-measured-landing-r309"
CONTROL_HZ = 50
PHYSICS_HZ = 250
PHYSICS_SUBSTEPS_PER_CONTROL = 5
CONTROL_DT = 1.0 / CONTROL_HZ
DEFAULT_NOMINAL_TICKS = 1000
DEFAULT_DISTURBED_TICKS = 300

# R308 request parameters followed by the R309 precontact horizon.
CANDIDATE_CONFIG = (0.05, 90.0, 14.0, 20.0, 1.0e-4, 20.0, 30.0, 120.0, 0.20)
BASE_CONTROLLER_OPTIONS: dict[str, Any] = {
    # Prime only the measured receiver.  This keeps the balanced nominal
    # public-rate fixture from being rejected during the first cold-start
    # solve; it does not author contact authority.
    "contact_observation_prestart_samples": 4,
}
CANDIDATE_CONTROLLER_OPTIONS: dict[str, Any] = {
    **BASE_CONTROLLER_OPTIONS,
    "measured_landing_enabled": True,
    "measured_landing_config": CANDIDATE_CONFIG,
}
PUBLIC_WORKER_OPTIONS = {"balanced_nominal_joint_target": True}

DIAGNOSTIC_INDEX = {
    "physics_observation_exact": 0,
    "physics_support_mask": 1,
    "observed_support_mask": 2,
    "target_support_mask": 3,
    "precontact_active": 4,
    "touchdown_normal_active": 5,
    "locked_count": 6,
    "left_phase": 7,
    "right_phase": 8,
    "left_contact_mode": 9,
    "right_contact_mode": 10,
    "reacquisition_status": 11,
    "reacquisition_qualified": 12,
    "reacquisition_pending_samples": 13,
    "reacquisition_total_load_n": 14,
    "reacquisition_minimum_active_load_n": 15,
    "reacquisition_weakest_load_fraction": 16,
    "reacquisition_flags": 17,
    "request_active": 18,
    "request_authority": 19,
    "request_target_error_m": 20,
    "request_commanded_vertical_acceleration_m_s2": 21,
    "precontact_acceleration_norm_m_s2": 22,
    "precontact_acceleration_was_limited": 23,
    "transition_count": 24,
}


def _mask(values: list[int] | tuple[int, int]) -> int:
    """Encode left/right support booleans as the Rust bit mask."""

    return int(values[0]) | (int(values[1]) << 1)


def _diagnostic(state: dict[str, Any], name: str) -> float:
    return float(state["measured_landing_diagnostics"][DIAGNOSTIC_INDEX[name]])


def _first_tick(states: list[dict[str, Any]], predicate: Any) -> int | None:
    return next((int(state["index"]) for state in states if predicate(state)), None)


def _max_abs_diagnostic(states: list[dict[str, Any]], name: str) -> float:
    return float(
        max(
            (abs(_diagnostic(state, name)) for state in states),
            default=0.0,
        )
    )


def _action_summary(case: dict[str, Any]) -> dict[str, Any]:
    states = case["states"]
    authority = np.asarray(
        [_diagnostic(state, "request_authority") for state in states], np.float64
    )
    precontact = [
        state["index"] for state in states if state["measured_landing_precontact_active"]
    ]
    touchdown = [
        state["index"]
        for state in states
        if state["measured_landing_touchdown_normal_active"]
    ]
    request = [
        state["index"]
        for state in states
        if _diagnostic(state, "request_active") > 0.5
    ]
    qualification = [
        state["index"]
        for state in states
        if state["measured_landing_reacquisition_qualified"]
    ]
    return {
        "first_precontact_tick": precontact[0] if precontact else None,
        "first_touchdown_normal_tick": touchdown[0] if touchdown else None,
        "first_request_active_tick": request[0] if request else None,
        "first_reacquisition_qualified_tick": qualification[0]
        if qualification
        else None,
        "last_request_active_tick": request[-1] if request else None,
        "precontact_ticks": len(precontact),
        "touchdown_normal_ticks": len(touchdown),
        "request_active_ticks": len(request),
        "reacquisition_qualified_ticks": len(qualification),
        "maximum_authority": float(np.max(authority, initial=0.0)),
        "maximum_abs_target_error_m": _max_abs_diagnostic(
            states, "request_target_error_m"
        ),
        "maximum_abs_commanded_acceleration_m_s2": _max_abs_diagnostic(
            states, "request_commanded_vertical_acceleration_m_s2"
        ),
        "maximum_precontact_acceleration_norm_m_s2": _max_abs_diagnostic(
            states, "precontact_acceleration_norm_m_s2"
        ),
        "step_us_p99": float(
            np.percentile(
                np.asarray(
                    [state["measured_landing_step_us"] for state in states],
                    np.float64,
                ),
                99.0,
            )
        ),
        "step_us_maximum": float(
            max((state["measured_landing_step_us"] for state in states), default=0.0)
        ),
        "allocation_calls": int(
            sum(state["measured_landing_allocation_calls"] for state in states)
        ),
        "allocated_bytes": int(
            sum(state["measured_landing_allocated_bytes"] for state in states)
        ),
        "mode_patterns": sorted(
            {
                "".join(str(mode) for mode in state["measured_landing_contact_modes"])
                for state in states
            }
        ),
    }


def _mode_firewall(states: list[dict[str, Any]]) -> bool:
    """A rolling-wheel mode may only survive on an observed support leg.

    R309 starts in NormalPoint until both the transition phase and the
    force-backed reacquisition observer qualify.  A lost leg must therefore
    never retain mode 3 merely because it was rolling before the loss.
    """

    for state in states:
        modes = state["measured_landing_contact_modes"]
        observed = state["observed"]
        if len(modes) != 2 or len(observed) != 2:
            return False
        if any(int(mode) not in (0, 2, 3) for mode in modes):
            return False
        if any(int(mode) == 3 and int(active) == 0 for mode, active in zip(modes, observed)):
            return False
        diagnostics = state["measured_landing_diagnostics"]
        if len(diagnostics) != len(DIAGNOSTIC_INDEX) or not np.all(
            np.isfinite(np.asarray(diagnostics, np.float64))
        ):
            return False
        # The streamed mode fields are copied from the same Rust diagnostic
        # vector; disagreeing values indicate a publication race.
        if int(round(diagnostics[9])) != int(modes[0]) or int(round(diagnostics[10])) != int(
            modes[1]
        ):
            return False
    return True


def _causal_measured_window(states: list[dict[str, Any]]) -> bool:
    """Check both the five-frame worker window and the R309 exact snapshot."""

    if not r300._causal_window_contract(states):
        return False
    for state in states:
        diagnostics = state["measured_landing_diagnostics"]
        if not diagnostics or _diagnostic(state, "physics_observation_exact") < 0.5:
            return False
        if int(round(_diagnostic(state, "physics_support_mask"))) != _mask(
            state["physics_contact_active"]
        ):
            return False
        if int(round(_diagnostic(state, "observed_support_mask"))) != _mask(
            state["observed"]
        ):
            return False
        if int(round(_diagnostic(state, "target_support_mask"))) != 3:
            return False
    return True


def _finite_outputs(states: list[dict[str, Any]]) -> bool:
    scalar_fields = (
        "controller_step_us",
        "worker_step_us",
        "measured_landing_step_us",
        "measured_landing_allocation_calls",
        "measured_landing_allocated_bytes",
        "root_height_m",
        "root_tilt_rad",
        "wbc_maximum_constraint_violation",
        "wbc_dynamics_residual",
        "wbc_contact_residual",
        "physics_wheel_normal_force_n",
        "observed_wheel_normal_force_n",
    )
    for state in states:
        for field in scalar_fields:
            values = state[field] if isinstance(state[field], list) else [state[field]]
            if not np.all(np.isfinite(np.asarray(values, np.float64))):
                return False
        if not np.all(
            np.isfinite(
                np.asarray(state["measured_landing_diagnostics"], np.float64)
            )
        ):
            return False
        if not np.all(
            np.isfinite(np.asarray(state["measured_landing_contact_modes"], np.float64))
        ):
            return False
    return True


def _negative_disturbed_recovery(summary: dict[str, Any]) -> bool:
    """Record the expected current negative result without reset masking."""

    return (
        summary["terminal_pending"] == "fall"
        and not summary["numeric_reset"]
        and summary["supported_upright_recovery_tick"] is None
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
    disturbed_states = candidate_disturbed["states"]
    first_single_support = _first_tick(
        disturbed_states, lambda state: state["observed"] in ([1, 0], [0, 1])
    )
    first_physics_single_support = _first_tick(
        disturbed_states,
        lambda state: state["physics_contact_active"] in ([1, 0], [0, 1]),
    )
    first_precontact = disturbed_action["first_precontact_tick"]
    first_request = disturbed_action["first_request_active_tick"]
    qualification_gates = {
        "live_rate_split_is_250_50": candidate_disturbed["hello"]
        == {
            "physics_hz": PHYSICS_HZ,
            "control_hz": CONTROL_HZ,
            "physics_substeps_per_control": PHYSICS_SUBSTEPS_PER_CONTROL,
        },
        "nominal_completes_requested_horizon": (
            candidate_nominal_summary["ticks"] == nominal_ticks
            and candidate_nominal_summary["terminal_pending"] is None
            and not candidate_nominal_summary["numeric_reset"]
        ),
        "nominal_stays_bilateral_upright": (
            candidate_nominal_summary["observed_patterns"] == ["11"]
            and candidate_nominal_summary["minimum_root_height_m"] >= 0.48
            and candidate_nominal_summary["maximum_root_tilt_rad"] <= 0.20
        ),
        "nominal_measured_landing_remains_dormant": (
            nominal_action["maximum_authority"] <= 1.0e-12
            and nominal_action["first_precontact_tick"] is None
            and nominal_action["first_request_active_tick"] is None
            and nominal_action["mode_patterns"] == ["33"]
        ),
        "candidate_default_is_off": not any(
            bool(state["measured_landing_enabled"])
            for state in baseline_nominal["states"] + baseline_disturbed["states"]
        ),
        "measured_phase_activates_after_physics_loss": (
            first_physics_single_support is not None
            and first_single_support is not None
            and first_precontact is not None
            and first_request is not None
            and first_precontact >= first_physics_single_support
            and first_request >= first_precontact
        ),
        "mode_firewall_never_promotes_unobserved_leg": _mode_firewall(
            candidate_nominal["states"] + candidate_disturbed["states"]
        ),
        "causal_physics_window_and_exact_snapshot": _causal_measured_window(
            candidate_disturbed["states"]
        ),
        "finite_measured_landing_outputs": _finite_outputs(
            candidate_nominal["states"] + candidate_disturbed["states"]
        ),
        "rust_measured_landing_hot_path_zero_allocations": (
            nominal_action["allocation_calls"] == 0
            and nominal_action["allocated_bytes"] == 0
            and disturbed_action["allocation_calls"] == 0
            and disturbed_action["allocated_bytes"] == 0
        ),
        "measured_landing_p99_under_100us": max(
            nominal_action["step_us_p99"], disturbed_action["step_us_p99"]
        )
        < 100.0,
        "wbc_hard_rows_subset_measured_contact": candidate_disturbed_summary[
            "hard_subset_raw"
        ],
        "wbc_max_iterations_are_nonadmitted": all(
            state["wbc_status"] != "MaxIterations" or not state["wbc_admitted"]
            for state in candidate_disturbed["states"]
        ),
        "all_wbc_outputs_finite": _finite_outputs(candidate_disturbed["states"]),
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
    negative_evidence = {
        "candidate_disturbed_negative_recovery_is_explicit": _negative_disturbed_recovery(
            candidate_disturbed_summary
        ),
        "baseline_and_candidate_both_expose_same_boundary_class": (
            baseline_disturbed_summary["terminal_pending"]
            == candidate_disturbed_summary["terminal_pending"]
        ),
    }
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "candidate_default_enabled": False,
        "candidate_config": list(CANDIDATE_CONFIG),
        "public_worker_options": PUBLIC_WORKER_OPTIONS,
        "receiver_options": BASE_CONTROLLER_OPTIONS,
        "nominal_ticks": nominal_ticks,
        "disturbed_ticks": disturbed_ticks,
        "first_physics_single_support_tick": first_physics_single_support,
        "first_measured_single_support_tick": first_single_support,
        "baseline_nominal": baseline_nominal_summary,
        "baseline_disturbed": baseline_disturbed_summary,
        "candidate_nominal": candidate_nominal_summary,
        "candidate_disturbed": candidate_disturbed_summary,
        "nominal_action": nominal_action,
        "disturbed_action": disturbed_action,
        "qualification_gates": qualification_gates,
        "promotion_gates": promotion_gates,
        "negative_evidence": negative_evidence,
        "qualified_as_bounded_default_off_experiment": all(qualification_gates.values()),
        "recovery_promoted": all(promotion_gates.values()),
        "finding": (
            "R309 is a causal, phase-aware, force-backed landing boundary. "
            "The nominal candidate remains dormant and allocation-free; the "
            "current 8 N disturbed trace is retained as negative recovery "
            "evidence when it falls, rather than allowing automatic reset to "
            "turn an incomplete recovery into a pass."
        ),
    }


def render_markdown(metrics: dict[str, Any]) -> str:
    qualification_lines = "\n".join(
        f"- {'PASS' if passed else 'FAIL'} `{name}`"
        for name, passed in metrics["qualification_gates"].items()
    )
    promotion_lines = "\n".join(
        f"- {'PASS' if passed else 'OPEN'} `{name}`"
        for name, passed in metrics["promotion_gates"].items()
    )
    negative_lines = "\n".join(
        f"- {'PASS' if passed else 'FAIL'} `{name}`"
        for name, passed in metrics["negative_evidence"].items()
    )
    nominal = metrics["candidate_nominal"]
    disturbed = metrics["candidate_disturbed"]
    action = metrics["disturbed_action"]
    return f"""# Upkie measured landing phase boundary — R309

Status: **{'QUALIFIED DEFAULT-OFF EXPERIMENT' if metrics['qualified_as_bounded_default_off_experiment'] else 'FIXTURE FAILED'}**; recovery **{'PROMOTED' if metrics['recovery_promoted'] else 'RETAINED AS NEGATIVE EVIDENCE'}**.

R309 runs the Rust-owned phase-aware measured-landing candidate at the public
250 Hz MuJoCo / 50 Hz WBC rate.  The receiver is primed from four measured
prestart samples only to avoid cold-start rejection; no contact authority is
authored by that option.  The candidate uses the frozen configuration
`{list(CANDIDATE_CONFIG)}` and remains default-off.

## Candidate outcome

| measure | nominal | disturbed |
|---|---:|---:|
| ticks / terminal boundary | {nominal['ticks']} / {nominal['terminal_pending'] or '—'} | {disturbed['ticks']} / {disturbed['terminal_pending'] or '—'} |
| observed contact patterns | {', '.join(nominal['observed_patterns'])} | {', '.join(disturbed['observed_patterns'])} |
| first precontact / request tick | — | {action['first_precontact_tick']} / {action['first_request_active_tick']} |
| first force-backed qualification tick | — | {action['first_reacquisition_qualified_tick']} |
| contact-mode patterns | {', '.join(metrics['nominal_action']['mode_patterns'])} | {', '.join(action['mode_patterns'])} |
| maximum request authority | {metrics['nominal_action']['maximum_authority']:.6f} | {action['maximum_authority']:.6f} |
| max precontact acceleration norm | {metrics['nominal_action']['maximum_precontact_acceleration_norm_m_s2']:.6f} | {action['maximum_precontact_acceleration_norm_m_s2']:.6f} m/s² |
| measured-landing p99 / max µs | {metrics['nominal_action']['step_us_p99']:.3f} / {metrics['nominal_action']['step_us_maximum']:.3f} | {action['step_us_p99']:.3f} / {action['step_us_maximum']:.3f} |
| measured-landing allocations / bytes | {metrics['nominal_action']['allocation_calls']} / {metrics['nominal_action']['allocated_bytes']} | {action['allocation_calls']} / {action['allocated_bytes']} |
| min root height / max tilt | {nominal['minimum_root_height_m']:.5f} m / {nominal['maximum_root_tilt_rad']:.5f} rad | {disturbed['minimum_root_height_m']:.5f} m / {disturbed['maximum_root_tilt_rad']:.5f} rad |

## Bounded-experiment qualification

{qualification_lines}

## Recovery promotion

{promotion_lines}

## Explicit negative evidence

{negative_lines}

## Architectural conclusion

{metrics['finding']}
"""


def run(
    model: pathlib.Path,
    *,
    nominal_ticks: int = DEFAULT_NOMINAL_TICKS,
    disturbed_ticks: int = DEFAULT_DISTURBED_TICKS,
) -> tuple[dict[str, Any], dict[str, Any], str]:
    baseline_nominal = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=nominal_ticks,
        controller_options=BASE_CONTROLLER_OPTIONS,
        worker_options=PUBLIC_WORKER_OPTIONS,
    )
    baseline_disturbed = r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=disturbed_ticks,
        controller_options=BASE_CONTROLLER_OPTIONS,
        worker_options=PUBLIC_WORKER_OPTIONS,
    )
    candidate_nominal = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=nominal_ticks,
        controller_options=CANDIDATE_CONTROLLER_OPTIONS,
        worker_options=PUBLIC_WORKER_OPTIONS,
    )
    candidate_disturbed = r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=disturbed_ticks,
        controller_options=CANDIDATE_CONTROLLER_OPTIONS,
        worker_options=PUBLIC_WORKER_OPTIONS,
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
    parser.add_argument("--nominal-ticks", type=int, default=DEFAULT_NOMINAL_TICKS)
    parser.add_argument("--disturbed-ticks", type=int, default=DEFAULT_DISTURBED_TICKS)
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("benchmarks/results/upkie-live-measured-landing-r309"),
    )
    args = parser.parse_args()
    if args.nominal_ticks < 100 or args.disturbed_ticks < 100:
        parser.error("nominal-ticks and disturbed-ticks must both be >=100")
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
    (args.output_dir / "UPKIE_LIVE_MEASURED_LANDING_R309.md").write_text(
        markdown, encoding="utf-8"
    )
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0 if metrics["qualified_as_bounded_default_off_experiment"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
