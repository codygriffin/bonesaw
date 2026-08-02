#!/usr/bin/env python3
"""Leave-one-case-out audit of terminal-candidate acceleration realization."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_program_robustness_ab import cases, execute
from upkie_inexact_hold_forecast_selector_ab import DROP5, DROP5_MATCHED, DROP10
from upkie_mujoco_plant_report import CONTROL_DT, JOINT_ORDER


REVISION = "upkie-terminal-realization-calibration-r192"
ACTION_NAMES = ("withhold", "retained", "support_free")
COMPONENT_NAMES = ("roll_qdd", "pitch_qdd", *[f"{name}_qdd" for name in JOINT_ORDER])


def profiles() -> dict[str, dict[str, Any]]:
    common = {"inexact_hold_ticks": 1, "inexact_terminal_chooser": True}
    return {
        "drop5_default": {"profile": DROP5, **common},
        "drop10_default": {"profile": DROP10, **common},
        "drop5_zero_margin": {
            "profile": DROP5,
            **common,
            "inexact_terminal_minimum_component_improvement": 0.0,
        },
        "drop10_zero_margin": {
            "profile": DROP10,
            **common,
            "inexact_terminal_minimum_component_improvement": 0.0,
        },
        "drop5_matched": {"profile": DROP5_MATCHED, **common},
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_TERMINAL_REALIZATION_CALIBRATION_R192.html"
    )
    parser.add_argument("--cases", help="comma-separated debug-only case subset")
    return parser.parse_args()


def samples_from_trace(
    case_name: str, profile_name: str, trace: dict[str, Any]
) -> list[dict[str, Any]]:
    queried = np.asarray(
        trace["inexact_observation_terminal_selector_queried"], np.uint8
    ) != 0
    actions = np.asarray(
        trace["inexact_observation_terminal_selector_action"], np.uint8
    )
    candidate_root = np.asarray(
        trace["inexact_observation_terminal_selector_root_acceleration"], np.float64
    )
    candidate_joint = np.asarray(
        trace["inexact_observation_terminal_selector_joint_acceleration"], np.float64
    )
    root_twist = np.asarray(trace["root_twist"], np.float64)
    post_root_twist = np.asarray(trace["post_root_twist"], np.float64)
    joint_velocity = np.asarray(trace["v"], np.float64)
    post_joint_velocity = np.asarray(trace["post_v"], np.float64)
    rows: list[dict[str, Any]] = []
    for tick in np.flatnonzero(queried):
        action = int(actions[tick])
        predicted = np.concatenate(
            (candidate_root[tick, action], candidate_joint[tick, action])
        )
        realized = np.concatenate(
            (
                (post_root_twist[tick, :2] - root_twist[tick, :2]) / CONTROL_DT,
                (post_joint_velocity[tick] - joint_velocity[tick]) / CONTROL_DT,
            )
        )
        error = realized - predicted
        rows.append(
            {
                "case": case_name,
                "profile": profile_name,
                "tick": int(tick),
                "action": action,
                "predicted": predicted,
                "realized": realized,
                "error": error,
            }
        )
    return rows


def action_summary(samples: list[dict[str, Any]], action: int) -> dict[str, Any]:
    selected = [sample for sample in samples if sample["action"] == action]
    if not selected:
        return {"sample_count": 0, "case_count": 0}
    errors = np.stack([sample["error"] for sample in selected])
    absolute = np.abs(errors)
    return {
        "sample_count": len(selected),
        "case_count": len({sample["case"] for sample in selected}),
        "case_names": sorted({sample["case"] for sample in selected}),
        "absolute_error_by_component": {
            name: distribution(absolute[:, index])
            for index, name in enumerate(COMPONENT_NAMES)
        },
        "absolute_error_linf": distribution(np.max(absolute, axis=1)),
        "signed_error_mean": {
            name: float(np.mean(errors[:, index]))
            for index, name in enumerate(COMPONENT_NAMES)
        },
    }


def leave_one_case_out(
    samples: list[dict[str, Any]], case_names: tuple[str, ...], reserve: float
) -> dict[str, Any]:
    folds: list[dict[str, Any]] = []
    all_covered: list[np.ndarray] = []
    all_ratios: list[np.ndarray] = []
    for held_case in case_names:
        for action, action_name in enumerate(ACTION_NAMES):
            training = [
                sample
                for sample in samples
                if sample["case"] != held_case and sample["action"] == action
            ]
            held = [
                sample
                for sample in samples
                if sample["case"] == held_case and sample["action"] == action
            ]
            if not training or not held:
                continue
            bound = reserve * np.max(
                np.abs(np.stack([sample["error"] for sample in training])), axis=0
            )
            held_error = np.abs(np.stack([sample["error"] for sample in held]))
            covered = held_error <= bound[None, :]
            ratio = np.divide(
                held_error,
                bound[None, :],
                out=np.full_like(held_error, math.inf),
                where=bound[None, :] > 0.0,
            )
            ratio[(held_error == 0.0) & (bound[None, :] == 0.0)] = 0.0
            all_covered.append(covered)
            all_ratios.append(ratio)
            folds.append(
                {
                    "held_case": held_case,
                    "action": action_name,
                    "training_samples": len(training),
                    "held_samples": len(held),
                    "all_components_covered": int(np.sum(np.all(covered, axis=1))),
                    "component_values_covered": int(np.sum(covered)),
                    "component_values": int(covered.size),
                    "maximum_exceedance_ratio": float(np.max(ratio)),
                    "bound": {
                        name: float(bound[index])
                        for index, name in enumerate(COMPONENT_NAMES)
                    },
                }
            )
    if not all_covered:
        return {
            "reserve": reserve,
            "fold_count": 0,
            "sample_count": 0,
            "all_component_sample_coverage": 0.0,
            "component_value_coverage": 0.0,
            "maximum_exceedance_ratio": math.inf,
            "folds": [],
        }
    coverage = np.concatenate(all_covered, axis=0)
    ratios = np.concatenate(all_ratios, axis=0)
    return {
        "reserve": reserve,
        "fold_count": len(folds),
        "sample_count": int(coverage.shape[0]),
        "all_component_sample_coverage": float(np.mean(np.all(coverage, axis=1))),
        "component_value_coverage": float(np.mean(coverage)),
        "maximum_exceedance_ratio": float(np.max(ratios)),
        "folds": folds,
    }


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    selected_cases = cases()
    if args.cases:
        requested = set(args.cases.split(","))
        selected_cases = tuple(case for case in selected_cases if case.name in requested)
        missing = requested - {case.name for case in selected_cases}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")
    model = pathlib.Path(args.model).resolve()
    samples: list[dict[str, Any]] = []
    configured = profiles()
    timing: list[dict[str, Any]] = []
    for case_index, case in enumerate(selected_cases, 1):
        for profile_name, config in configured.items():
            kwargs = {key: value for key, value in config.items() if key != "profile"}
            run = execute(model, case, args.duration, config["profile"], **kwargs)
            samples.extend(samples_from_trace(case.name, profile_name, run["trace"]))
            timing.append(run["metrics"])
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            f"{sum(sample['case'] == case.name for sample in samples)} samples",
            flush=True,
        )
    finite = all(
        np.all(np.isfinite(sample[field]))
        for sample in samples
        for field in ("predicted", "realized", "error")
    )
    summaries = {
        name: action_summary(samples, action)
        for action, name in enumerate(ACTION_NAMES)
    }
    case_names = tuple(case.name for case in selected_cases)
    strict = leave_one_case_out(samples, case_names, 1.0)
    reserve = leave_one_case_out(samples, case_names, 1.05)
    gates = {
        "full_case_matrix": len(selected_cases) == len(cases()),
        "finite": finite,
        "all_actions_observed": all(
            summaries[name]["sample_count"] > 0 for name in ACTION_NAMES
        ),
        "every_action_spans_multiple_cases": all(
            summaries[name]["case_count"] >= 2 for name in ACTION_NAMES
        ),
        "leave_one_case_out_exercised": strict["fold_count"] > 0,
        "strict_loco_all_component_coverage": strict[
            "all_component_sample_coverage"
        ]
        == 1.0,
        "five_percent_reserve_loco_all_component_coverage": reserve[
            "all_component_sample_coverage"
        ]
        == 1.0,
        "zero_rust_allocation": all(item["allocation_free"] for item in timing),
        "zero_python_gc": all(item["python_gc_collections"] == 0 for item in timing),
    }
    calibration_admitted = all(gates.values())
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "case_count": len(selected_cases),
        "profile_count": len(configured),
        "sample_count": len(samples),
        "component_names": COMPONENT_NAMES,
        "action_summaries": summaries,
        "strict_leave_one_case_out": strict,
        "five_percent_reserve_leave_one_case_out": reserve,
        "gates": gates,
        "calibration_admitted": calibration_admitted,
        "controller_promoted": False,
        "scope": {
            "realized_acceleration": "finite difference of measured world root angular and joint velocities over the complete 5 ms control interval",
            "predicted_acceleration": "the exact selected candidate qdd supplied to the r191 terminal scorer",
            "split": "each bound is the componentwise training maximum over all other named plant cases; the held-out case never contributes to its bound",
            "not_claimed": "continuous-time error, unselected-candidate counterfactual response, impact injury, new morphology, hardware, or controller authority",
        },
    }
    table_rows = [
        [
            name,
            summary["sample_count"],
            summary["case_count"],
            f"{summary['absolute_error_linf']['p50']:.3f}" if summary["sample_count"] else "—",
            f"{summary['absolute_error_linf']['p99']:.3f}" if summary["sample_count"] else "—",
            f"{summary['absolute_error_linf']['maximum']:.3f}" if summary["sample_count"] else "—",
        ]
        for name, summary in summaries.items()
    ]
    report = "\n".join(
        [
            "# Bonesaw terminal candidate realization calibration · r192",
            "",
            f"> Cross-case calibration **{'PASS' if calibration_admitted else 'REJECTED'}** · controller promotion **NO**.",
            "",
            "## Result",
            "",
            f"The audit measures **{len(samples):,}** selected terminal actions across **{len(selected_cases)} cases × {len(configured)} profiles**. It compares each r191 candidate's exact roll/pitch and six-joint acceleration against the finite difference of measured MuJoCo velocities over the next complete 5 ms control interval.",
            f"Strict leave-one-case-out componentwise-max bounds cover **{100.0 * strict['all_component_sample_coverage']:.3f}%** of complete samples and **{100.0 * strict['component_value_coverage']:.3f}%** of scalar component values; the worst exceedance is **{strict['maximum_exceedance_ratio']:.3f}×**. A 5% reserve covers **{100.0 * reserve['all_component_sample_coverage']:.3f}%** of complete samples with a **{reserve['maximum_exceedance_ratio']:.3f}×** worst exceedance.",
            "",
            *markdown_table(
                ["selected action", "samples", "cases", "|error|∞ p50", "p99", "max"],
                table_rows,
            ),
            "",
            "## Admission gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Interpretation",
            "",
            "- This is a measured realization audit, not a new selector. A failed holdout bound cannot authorize a candidate and cannot be hidden inside the r191 aggregate score.",
            "- The realized value includes actuator/contact response and any declared external wrench during that 5 ms interval. It therefore tests the exact place where a WBC qdd prediction becomes a plant claim.",
            "- Only the selected candidate has a physical counterfactual. Unselected candidate accelerations remain model predictions and are not treated as realized evidence.",
            "- Python owns the corpus, split, and statistics. Rust remains the source of candidate qdd, typed selection, and allocation evidence.",
            "",
            "## Scope",
            "",
            "Upkie MuJoCo plant · measured world root angular velocity + six joint velocities · 5 ms interval finite difference · leave-one-named-case-out maximum bounds · no policy · no learned model · no hardware or injury claim.",
        ]
    )
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-terminal-realization-calibration-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    report_path = output / "UPKIE_TERMINAL_REALIZATION_CALIBRATION.md"
    report_path.write_text(report + "\n")
    pathlib.Path(args.web_report).write_text(
        render_report_html(report, title="Bonesaw r192 realization")
    )
    print(f"report: {report_path}")
    print(f"web: {args.web_report}")
    return 0 if finite else 1


if __name__ == "__main__":
    raise SystemExit(main())
