#!/usr/bin/env python3
"""R196 factor terminal-model residuals against measured contact impulse."""

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
from upkie_disturbance_envelope import semantic_trace_equal
from upkie_inexact_hold_forecast_selector_ab import DROP5, DROP5_MATCHED, DROP10
from upkie_inexact_support_hypothesis_envelope_ab import (
    PRESSURE_NAMES,
    model_limits,
    support_hypothesis_attempt_contract,
)
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-contact-impulse-residual-audit-r197"
DURATION_S = 6.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CONTACT_IMPULSE_RESIDUAL_AUDIT_R197.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def profiles() -> dict[str, dict[str, Any]]:
    common = {
        "inexact_hold_ticks": 1,
        "inexact_terminal_chooser": True,
        "inexact_terminal_support_hypothesis_envelope": True,
        "record_physical_contact_impulses": True,
    }
    return {
        "drop5": {"profile": DROP5, **common},
        "drop10": {"profile": DROP10, **common},
        "drop5_matched": {"profile": DROP5_MATCHED, **common},
    }


def safe_distribution(values: np.ndarray) -> dict[str, float]:
    if len(values):
        return distribution(values)
    return {name: math.nan for name in distribution(np.asarray([0.0]))}


def correlation(left: np.ndarray, right: np.ndarray) -> float:
    if len(left) < 2 or float(np.std(left)) == 0.0 or float(np.std(right)) == 0.0:
        return 0.0
    return float(np.corrcoef(left, right)[0, 1])


def source_samples(
    trace: dict[str, Any],
    balance: Any,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
) -> dict[str, np.ndarray]:
    lower, upper, velocity_limit, _effort_limit = limits
    queried = np.asarray(trace["inexact_observation_terminal_selector_queried"]) != 0
    ticks = np.flatnonzero(queried)
    actions = np.asarray(
        trace["inexact_observation_terminal_selector_action"], np.uint8
    )[ticks]
    physical_contact = np.asarray(trace["physical_contact_active"], np.uint8)[ticks]
    support_masks = physical_contact[:, 0] + 2 * physical_contact[:, 1]
    hypothesis_acceleration = np.asarray(
        trace["inexact_observation_terminal_hypothesis_acceleration"], np.float64
    )
    hypothesis_available = np.asarray(
        trace["inexact_observation_terminal_hypothesis_available"], np.uint8
    )
    envelopes = np.asarray(
        trace["inexact_observation_terminal_hypothesis_envelopes"], np.float64
    )
    effort = np.asarray(
        trace["inexact_observation_terminal_hypothesis_effort_utilization"],
        np.float64,
    )
    root_twist = np.asarray(trace["root_twist"], np.float64)
    post_root_twist = np.asarray(trace["post_root_twist"], np.float64)
    joint_velocity = np.asarray(trace["v"], np.float64)
    post_joint_velocity = np.asarray(trace["post_v"], np.float64)
    wheel_impulse = np.asarray(
        trace["physical_wheel_contact_impulse_ns"], np.float64
    )[ticks]
    constraint_impulse = np.asarray(
        trace["physical_constraint_generalized_impulse_ns"], np.float64
    )[ticks]
    external_impulse = (
        np.linalg.norm(np.asarray(trace["external_force_world"])[ticks], axis=1)
        * CONTROL_DT
    )
    names = tuple(balance.terminal_impact_diagnostic_names)
    name_index = {name: index for index, name in enumerate(names)}
    pressure_indices = np.asarray(
        [name_index[name] for name in PRESSURE_NAMES], np.int64
    )
    count = len(ticks)
    physical_error = np.empty(count, np.float64)
    best_error = np.empty(count, np.float64)
    best_support = np.empty(count, np.uint8)
    qdd_envelope_exceedance = np.empty(count, np.float64)
    pressure_envelope_exceedance = np.empty(count, np.float64)
    score_allocations = np.empty(count, np.uint64)
    score_bytes = np.empty(count, np.uint64)
    diagnostics = np.empty((3, len(names)), np.float64)
    selection = np.empty(6, np.float64)
    candidate_root = np.empty((3, 2), np.float64)
    candidate_joint = np.empty((3, 6), np.float64)
    candidate_available = np.empty(3, np.uint8)
    for cursor, (tick, action, physical_mask) in enumerate(
        zip(ticks, actions, support_masks, strict=True)
    ):
        action = int(action)
        physical_mask = int(physical_mask)
        actual = np.concatenate(
            (
                (post_root_twist[tick, :2] - root_twist[tick, :2]) / CONTROL_DT,
                (post_joint_velocity[tick] - joint_velocity[tick]) / CONTROL_DT,
            )
        )
        predicted = np.concatenate(
            (
                hypothesis_acceleration[tick, :, action, :2],
                hypothesis_acceleration[tick, :, action, 6:],
            ),
            axis=1,
        )
        errors = np.linalg.norm(predicted - actual, axis=1)
        best_support[cursor] = int(np.argmin(errors))
        best_error[cursor] = float(np.min(errors))
        physical_error[cursor] = float(errors[physical_mask])
        lower_qdd = np.min(predicted, axis=0)
        upper_qdd = np.max(predicted, axis=0)
        qdd_envelope_exceedance[cursor] = float(
            np.max(np.maximum(np.maximum(actual - upper_qdd, lower_qdd - actual), 0.0))
        )
        candidate_root[:] = hypothesis_acceleration[tick, physical_mask, :, :2]
        candidate_joint[:] = hypothesis_acceleration[tick, physical_mask, :, 6:]
        candidate_available[:] = hypothesis_available[tick, physical_mask]
        candidate_root[action] = actual[:2]
        candidate_joint[action] = actual[2:]
        timing = balance.score_terminal_impact_candidates(
            np.asarray(trace["inexact_observation_terminal_selector_state"])[tick],
            np.asarray(trace["q"])[tick],
            joint_velocity[tick],
            lower,
            upper,
            velocity_limit,
            candidate_available,
            candidate_root,
            candidate_joint,
            effort[tick],
            0,
            0.0,
            0.01,
            diagnostics,
            selection,
        )
        score_allocations[cursor] = int(timing[1])
        score_bytes[cursor] = int(timing[2])
        realized_pressure = diagnostics[action, pressure_indices]
        envelope_pressure = envelopes[tick, action, pressure_indices]
        pressure_envelope_exceedance[cursor] = float(
            np.max(np.maximum(realized_pressure - envelope_pressure, 0.0))
        )
    return {
        "action": actions,
        "support_mask": support_masks,
        "best_support_mask": best_support,
        "physical_support_error_norm": physical_error,
        "best_support_error_norm": best_error,
        "qdd_envelope_exceedance": qdd_envelope_exceedance,
        "pressure_envelope_exceedance": pressure_envelope_exceedance,
        "normal_impulse_ns": np.sum(wheel_impulse[:, :, 0], axis=1),
        "tangential_impulse_ns": np.sum(wheel_impulse[:, :, 1], axis=1),
        "constraint_impulse_norm": np.linalg.norm(constraint_impulse, axis=1),
        "external_impulse_ns": external_impulse,
        "score_allocation_calls": score_allocations,
        "score_allocated_bytes": score_bytes,
    }


def summarize(samples: dict[str, np.ndarray]) -> dict[str, Any]:
    count = len(samples["action"])
    physical_is_best = (
        samples["physical_support_error_norm"]
        <= samples["best_support_error_norm"] + 1.0e-12
    )
    qdd_covered = samples["qdd_envelope_exceedance"] <= 1.0e-12
    pressure_covered = samples["pressure_envelope_exceedance"] <= 1.0e-12
    return {
        "sample_count": count,
        "action_counts": {
            str(index): int(np.sum(samples["action"] == index))
            for index in range(3)
        },
        "physical_support_counts": {
            str(index): int(np.sum(samples["support_mask"] == index))
            for index in range(4)
        },
        "best_support_counts": {
            str(index): int(np.sum(samples["best_support_mask"] == index))
            for index in range(4)
        },
        "physical_support_is_best_fraction": (
            float(np.mean(physical_is_best)) if count else 0.0
        ),
        "qdd_componentwise_coverage": float(np.mean(qdd_covered)) if count else 0.0,
        "pressure_componentwise_coverage": (
            float(np.mean(pressure_covered)) if count else 0.0
        ),
        "physical_support_error_norm": safe_distribution(
            samples["physical_support_error_norm"]
        ),
        "best_support_error_norm": safe_distribution(
            samples["best_support_error_norm"]
        ),
        "qdd_envelope_exceedance": safe_distribution(
            samples["qdd_envelope_exceedance"]
        ),
        "pressure_envelope_exceedance": safe_distribution(
            samples["pressure_envelope_exceedance"]
        ),
        "normal_impulse_ns": safe_distribution(samples["normal_impulse_ns"]),
        "tangential_impulse_ns": safe_distribution(
            samples["tangential_impulse_ns"]
        ),
        "constraint_impulse_norm": safe_distribution(
            samples["constraint_impulse_norm"]
        ),
        "correlations": {
            "normal_impulse_vs_best_qdd_error": correlation(
                samples["normal_impulse_ns"], samples["best_support_error_norm"]
            ),
            "tangential_impulse_vs_best_qdd_error": correlation(
                samples["tangential_impulse_ns"],
                samples["best_support_error_norm"],
            ),
            "constraint_impulse_vs_best_qdd_error": correlation(
                samples["constraint_impulse_norm"],
                samples["best_support_error_norm"],
            ),
            "constraint_impulse_vs_pressure_exceedance": correlation(
                samples["constraint_impulse_norm"],
                samples["pressure_envelope_exceedance"],
            ),
        },
        "zero_rust_rescore_allocation": bool(
            np.all(samples["score_allocation_calls"] == 0)
            and np.all(samples["score_allocated_bytes"] == 0)
        ),
    }


def concatenate_samples(items: list[dict[str, np.ndarray]]) -> dict[str, np.ndarray]:
    return {name: np.concatenate([item[name] for item in items]) for name in items[0]}


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
    limits = model_limits(model)
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model))
    rows: dict[str, Any] = {}
    raw: list[dict[str, np.ndarray]] = []
    for case_index, case in enumerate(selected_cases, 1):
        rows[case.name] = {}
        for name, config in profiles().items():
            kwargs = {key: value for key, value in config.items() if key != "profile"}
            run = execute(model, case, args.duration, config["profile"], **kwargs)
            replay = execute(model, case, args.duration, config["profile"], **kwargs)
            samples = source_samples(run["trace"], balance, limits)
            raw.append(samples)
            rows[case.name][name] = {
                "metrics": run["metrics"],
                "attempt_contract": support_hypothesis_attempt_contract(run["trace"]),
                "replay_exact": semantic_trace_equal(run["trace"], replay["trace"]),
                "source_audit": summarize(samples),
            }
        print(f"[{case_index:02d}/{len(selected_cases)}] {case.name}", flush=True)
    aggregate = summarize(concatenate_samples(raw))
    arms = [arm for case_rows in rows.values() for arm in case_rows.values()]
    gates = {
        "all_replays_exact": all(arm["replay_exact"] for arm in arms),
        "attempted_every_unavailable_tick": all(
            arm["attempt_contract"]["attempted_every_unavailable_tick"]
            for arm in arms
        ),
        "finite_without_numeric_fault": all(
            arm["metrics"]["finite"] and not arm["metrics"]["numeric_fault"]
            for arm in arms
        ),
        "zero_rust_allocation_and_python_gc": all(
            arm["metrics"]["allocation_free"]
            and arm["metrics"]["python_gc_collections"] == 0
            and arm["source_audit"]["zero_rust_rescore_allocation"]
            for arm in arms
        ),
        "physical_impulse_witness_is_nonzero": (
            aggregate["normal_impulse_ns"]["maximum"] > 0.0
            and aggregate["constraint_impulse_norm"]["maximum"] > 0.0
        ),
    }
    audit_passed = all(gates.values())
    detail = []
    for case_name, case_rows in rows.items():
        for profile_name, arm in case_rows.items():
            audit = arm["source_audit"]
            detail.append(
                [
                    case_name,
                    profile_name,
                    audit["sample_count"],
                    f"{audit['physical_support_is_best_fraction'] * 100.0:.1f}%",
                    f"{audit['qdd_componentwise_coverage'] * 100.0:.1f}%",
                    f"{audit['pressure_componentwise_coverage'] * 100.0:.1f}%",
                    f"{audit['best_support_error_norm']['maximum']:.3f}",
                    f"{audit['constraint_impulse_norm']['maximum']:.3f}",
                ]
            )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "audit_passed": audit_passed,
        "authority_admitted": False,
        "gates": gates,
        "aggregate": aggregate,
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw contact-impulse residual source audit · r197",
            "",
            f"> Instrumentation **{'PASS' if audit_passed else 'FAIL'}** · authority **NOT ADMITTED**.",
            "",
            "## Boundary",
            "",
            "- Python records MuJoCo wheel normal/tangential impulse and generalized constraint impulse over each complete 5 ms plant interval. These witnesses are offline only and never enter the Rust controller, candidate selection, or replay state.",
            "- The audit compares realized roll/pitch plus six-joint acceleration with (a) the hypothesis matching measured physical support, (b) the best of all four discrete support hypotheses, and (c) the componentwise min/max qdd envelope before terminal scoring.",
            "- A correlation is descriptive source localization, not a calibrated bound, causal proof, conformal certificate, or hardware claim.",
            "- R196 already rejects smooth current-state residual conditioning under leave-one-case-out evaluation. R197 asks which completed-interval physical witness co-varies with that rejected tail; it does not refit the same model under another name.",
            "",
            "## Aggregate result",
            "",
            f"- Samples: **{aggregate['sample_count']}**. Measured physical support is the minimum-error discrete hypothesis on **{aggregate['physical_support_is_best_fraction'] * 100.0:.1f}%**.",
            f"- Four-support qdd envelope coverage: **{aggregate['qdd_componentwise_coverage'] * 100.0:.1f}%**; post-score pressure coverage: **{aggregate['pressure_componentwise_coverage'] * 100.0:.1f}%**.",
            f"- Best-support qdd error p95/max: **{aggregate['best_support_error_norm']['p95']:.3f}/{aggregate['best_support_error_norm']['maximum']:.3f}**. Qdd envelope exceedance p95/max: **{aggregate['qdd_envelope_exceedance']['p95']:.3f}/{aggregate['qdd_envelope_exceedance']['maximum']:.3f}**.",
            f"- Constraint-impulse correlation with best-support qdd error / pressure exceedance: **{aggregate['correlations']['constraint_impulse_vs_best_qdd_error']:+.3f}/{aggregate['correlations']['constraint_impulse_vs_pressure_exceedance']:+.3f}**.",
            f"- Normal/tangential wheel-impulse correlation with best-support qdd error: **{aggregate['correlations']['normal_impulse_vs_best_qdd_error']:+.3f}/{aggregate['correlations']['tangential_impulse_vs_best_qdd_error']:+.3f}**. The latter is the strongest measured source association, but it uses the completed interval and is therefore not a causal online feature.",
            "",
            *markdown_table(
                [
                    "case",
                    "profile",
                    "n",
                    "physical is best",
                    "qdd covered",
                    "pressure covered",
                    "best qdd err max",
                    "constraint impulse max",
                ],
                detail,
            ),
            "",
            "## Decision",
            "",
            "The four support masks are retained as visible model hypotheses, but the best mask is still tested against a nonzero model-to-plant residual. R197 does not install an empirical margin or alter authority. The next admissible model must condition and validate a causal pre-step witness on fresh held-out cases, or replace acceleration prediction with a bounded momentum/contact-impulse transition.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-contact-impulse-residual-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_CONTACT_IMPULSE_RESIDUAL_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(json.dumps({"audit_passed": audit_passed, "aggregate": aggregate}))
    return 0 if audit_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
