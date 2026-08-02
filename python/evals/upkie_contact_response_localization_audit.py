#!/usr/bin/env python3
"""R205 localization of point-response versus root-momentum transition error."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_impulse_residual_audit import profiles
from upkie_contact_program_robustness_ab import execute
from upkie_contact_transition_acceleration_interval_audit import (
    FRESH_HOLDOUT_NAME,
    selected_cases,
)
from upkie_contact_transition_response_audit import score_profile
from upkie_directional_contact_transition_audit import (
    STRUCTURED_ACCELERATION_RESERVE,
)
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-contact-response-localization-audit-r205"
DURATION_S = 6.0
VARIANTS = (
    "support_acceleration_only",
    "prospective_point_exact_impulse",
    "oracle_contact_centroid_exact_impulse",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CONTACT_RESPONSE_LOCALIZATION_AUDIT_R205.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    parser.add_argument("--plant-profiles", help="comma-separated smoke subset")
    return parser.parse_args()


def impulse_weighted_centroid(
    prospective_point_world_m: np.ndarray,
    contact_position_m_ns: np.ndarray,
    normal_impulse_ns: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    centroid = prospective_point_world_m.copy()
    available = normal_impulse_ns > 1.0e-12
    np.divide(
        contact_position_m_ns,
        normal_impulse_ns[:, None],
        out=centroid,
        where=available[:, None],
    )
    return centroid, available


def project_impulse_response(
    response: np.ndarray, impulse_world_ns: np.ndarray
) -> np.ndarray:
    return np.sum(response * impulse_world_ns[None, :, :], axis=(1, 2))


def candidate_interval(
    candidates: np.ndarray,
    impulse_delta_velocity: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    candidate_delta = candidates * CONTROL_DT + impulse_delta_velocity
    reserve = STRUCTURED_ACCELERATION_RESERVE * CONTROL_DT
    return (
        np.min(candidate_delta, axis=0) - reserve,
        np.max(candidate_delta, axis=0) + reserve,
    )


def record_sample(
    *,
    case: Any,
    plant_profile_name: str,
    tick: int,
    action: int,
    candidate_count: int,
    actual: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    model_timing_ns: int,
    model_allocation_calls: int,
    model_allocated_bytes: int,
    point_error_m: np.ndarray,
    contact_centroid_available: np.ndarray,
) -> dict[str, Any]:
    exceedance = np.maximum(np.maximum(lower - actual, actual - upper), 0.0)
    component_covered = exceedance <= 1.0e-12
    width = upper - lower
    midpoint = 0.5 * (upper + lower)
    utilization = np.divide(
        2.0 * np.abs(actual - midpoint),
        width,
        out=np.full_like(width, math.inf),
        where=width > 0.0,
    )
    return {
        "case": case.name,
        "fresh_holdout": case.name == FRESH_HOLDOUT_NAME,
        "plant_profile": plant_profile_name,
        "tick": int(tick),
        "selected_action": action,
        "candidate_count": candidate_count,
        "covered": bool(np.all(component_covered)),
        "component_covered": component_covered.tolist(),
        "component_exceedance": exceedance.tolist(),
        "interval_width": width.tolist(),
        "interval_utilization": utilization.tolist(),
        "actual_delta": actual.tolist(),
        "lower": lower.tolist(),
        "upper": upper.tolist(),
        # Exact impulse is supplied rather than bounded in this localization
        # ablation, so the only meaningful impulse-coverage statement is true.
        "impulse_covered": True,
        "model_timing_ns": model_timing_ns,
        "model_allocation_calls": model_allocation_calls,
        "model_allocated_bytes": model_allocated_bytes,
        "bound_timing_ns": 0,
        "bound_allocation_calls": 0,
        "bound_allocated_bytes": 0,
        "point_error_m": point_error_m.tolist(),
        "contact_centroid_available": contact_centroid_available.tolist(),
    }


def score_samples(samples: list[dict[str, Any]]) -> dict[str, Any]:
    result = score_profile(samples)
    point_error = np.asarray([sample["point_error_m"] for sample in samples])
    available = np.asarray(
        [sample["contact_centroid_available"] for sample in samples], np.bool_
    )
    norms = np.linalg.norm(point_error, axis=2)
    result["contact_centroid_available_fraction"] = float(np.mean(available))
    result["prospective_to_centroid_error_m"] = distribution(
        norms[available] if np.any(available) else np.asarray([0.0])
    )
    return result


def split_score(samples: list[dict[str, Any]]) -> dict[str, Any]:
    retained = [sample for sample in samples if not sample["fresh_holdout"]]
    fresh = [sample for sample in samples if sample["fresh_holdout"]]
    return {
        "all": score_samples(samples),
        "retained": score_samples(retained) if retained else None,
        "fresh_holdout": score_samples(fresh) if fresh else None,
    }


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    selected = selected_cases()
    if args.cases:
        requested = set(args.cases.split(","))
        selected = tuple(case for case in selected if case.name in requested)
        missing = requested - {case.name for case in selected}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")
    configured_profiles = profiles()
    if args.plant_profiles:
        requested = set(args.plant_profiles.split(","))
        configured_profiles = {
            name: config
            for name, config in configured_profiles.items()
            if name in requested
        }
        missing = requested - set(configured_profiles)
        if missing:
            raise SystemExit(f"unknown plant profiles: {sorted(missing)}")

    model = pathlib.Path(args.model).resolve()
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model))
    samples: dict[str, list[dict[str, Any]]] = {name: [] for name in VARIANTS}
    run_metrics: dict[str, Any] = {}
    bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)
    prospective_response = np.empty((12, 2, 3), np.float64)
    centroid_response = np.empty((12, 2, 3), np.float64)
    effective_mass = np.empty((2, 3), np.float64)

    for case_index, case in enumerate(selected, 1):
        run_metrics[case.name] = {}
        case_samples = 0
        for plant_profile_name, config in configured_profiles.items():
            kwargs = {name: value for name, value in config.items() if name != "profile"}
            kwargs["record_physical_contact_impulses"] = True
            kwargs["record_physical_prospective_contact_state"] = True
            run = execute(model, case, args.duration, config["profile"], **kwargs)
            trace = run["trace"]
            ticks = np.flatnonzero(
                np.asarray(trace["inexact_observation_terminal_selector_queried"], np.bool_)
            )
            case_samples += len(ticks)
            actions = np.asarray(
                trace["inexact_observation_terminal_selector_action"], np.uint8
            )
            hypothesis_acceleration = np.asarray(
                trace["inexact_observation_terminal_hypothesis_acceleration"], np.float64
            )
            hypothesis_available = np.asarray(
                trace["inexact_observation_terminal_hypothesis_available"], np.bool_
            )
            root_position = np.asarray(trace["root_position"], np.float64)
            root_quaternion = np.asarray(trace["root_quaternion_wxyz"], np.float64)
            q = np.asarray(trace["q"], np.float64)
            pre_velocity = np.concatenate(
                (np.asarray(trace["root_twist"]), np.asarray(trace["v"])), axis=1
            )
            post_velocity = np.concatenate(
                (np.asarray(trace["post_root_twist"]), np.asarray(trace["post_v"])), axis=1
            )
            actual_delta = post_velocity - pre_velocity
            prospective_point = np.asarray(
                trace["physical_wheel_prospective_contact_point_world_m"], np.float64
            )
            impulse_world = np.asarray(
                trace["physical_wheel_contact_impulse_world_ns"], np.float64
            )
            normal_impulse = np.asarray(
                trace["physical_wheel_contact_impulse_ns"], np.float64
            )[:, :, 0]
            position_m_ns = np.asarray(
                trace["physical_wheel_contact_position_m_ns"], np.float64
            )
            for tick in ticks:
                action = int(actions[tick])
                candidates = hypothesis_acceleration[
                    tick, hypothesis_available[tick, :, action], action
                ]
                if len(candidates) == 0:
                    raise RuntimeError("selected terminal action has no support hypothesis")
                centroid, centroid_available = impulse_weighted_centroid(
                    prospective_point[tick], position_m_ns[tick], normal_impulse[tick]
                )
                point_error = centroid - prospective_point[tick]
                prospective_timing = balance.model_contact_impulse_velocity_response(
                    root_position[tick],
                    root_quaternion[tick],
                    q[tick],
                    prospective_point[tick],
                    bases,
                    prospective_response,
                    effective_mass,
                )
                centroid_timing = balance.model_contact_impulse_velocity_response(
                    root_position[tick],
                    root_quaternion[tick],
                    q[tick],
                    centroid,
                    bases,
                    centroid_response,
                    effective_mass,
                )
                impulse_delta = {
                    "support_acceleration_only": np.zeros(12, np.float64),
                    "prospective_point_exact_impulse": project_impulse_response(
                        prospective_response, impulse_world[tick]
                    ),
                    "oracle_contact_centroid_exact_impulse": project_impulse_response(
                        centroid_response, impulse_world[tick]
                    ),
                }
                for name in VARIANTS:
                    lower, upper = candidate_interval(candidates, impulse_delta[name])
                    timing = (
                        0
                        if name == "support_acceleration_only"
                        else int(
                            prospective_timing[0]
                            if name == "prospective_point_exact_impulse"
                            else centroid_timing[0]
                        )
                    )
                    allocation_calls = (
                        0
                        if name == "support_acceleration_only"
                        else int(
                            prospective_timing[1]
                            if name == "prospective_point_exact_impulse"
                            else centroid_timing[1]
                        )
                    )
                    allocated_bytes = (
                        0
                        if name == "support_acceleration_only"
                        else int(
                            prospective_timing[2]
                            if name == "prospective_point_exact_impulse"
                            else centroid_timing[2]
                        )
                    )
                    samples[name].append(
                        record_sample(
                            case=case,
                            plant_profile_name=plant_profile_name,
                            tick=int(tick),
                            action=action,
                            candidate_count=len(candidates),
                            actual=actual_delta[tick],
                            lower=lower,
                            upper=upper,
                            model_timing_ns=timing,
                            model_allocation_calls=allocation_calls,
                            model_allocated_bytes=allocated_bytes,
                            point_error_m=point_error,
                            contact_centroid_available=centroid_available,
                        )
                    )
            run_metrics[case.name][plant_profile_name] = run["metrics"]
        print(f"[{case_index:02d}/{len(selected)}] {case.name}: samples={case_samples}", flush=True)

    results = {name: split_score(rows) for name, rows in samples.items()}
    rows = [
        [
            name,
            result["all"]["sample_count"],
            f"{result['retained']['sample_coverage'] * 100.0:.3f}%"
            if result["retained"]
            else "—",
            f"{result['fresh_holdout']['sample_coverage'] * 100.0:.3f}%"
            if result["fresh_holdout"]
            else "—",
            f"{result['all']['maximum_component_exceedance_per_s']:.6f}",
            f"{result['all']['root_angular_width_rad_s']['p95']:.3f}",
            f"{result['all']['root_linear_width_m_s']['p95']:.3f}",
            f"{result['all']['joint_width_rad_s']['p95']:.3f}",
            f"{result['all']['model_response_timing_ns']['p99'] / 1_000.0:.3f}",
        ]
        for name, result in results.items()
    ]
    all_runs_finite = all(
        row["finite"] and not row["numeric_fault"]
        for case_rows in run_metrics.values()
        for row in case_rows.values()
    )
    zero_plant_allocation = all(
        row["allocation_free"] and row["python_gc_collections"] == 0
        for case_rows in run_metrics.values()
        for row in case_rows.values()
    )
    mechanism_passed = all_runs_finite and zero_plant_allocation and all(
        result["all"]["zero_rust_allocation"] for result in results.values()
    )
    prospective = results["prospective_point_exact_impulse"]
    centroid = results["oracle_contact_centroid_exact_impulse"]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "variants": VARIANTS,
        "structured_acceleration_reserve": STRUCTURED_ACCELERATION_RESERVE.tolist(),
        "mechanism_passed": mechanism_passed,
        "prospective_strict_retained_coverage": prospective["retained"]["sample_coverage"] == 1.0,
        "oracle_centroid_strict_retained_coverage": centroid["retained"]["sample_coverage"] == 1.0,
        "authority_admitted": False,
        "all_runs_finite": all_runs_finite,
        "zero_plant_allocation_and_python_gc": zero_plant_allocation,
        "results": results,
        "run_metrics": run_metrics,
    }
    report = "\n".join(
        [
            "# Bonesaw contact-response localization audit · r205",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · prospective-point strict retained coverage **{'PASS' if metrics['prospective_strict_retained_coverage'] else 'FAIL'}** · oracle-centroid strict retained coverage **{'PASS' if metrics['oracle_centroid_strict_retained_coverage'] else 'FAIL'}** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- This is a label-only decomposition, not a reachable tube. Every row receives the completed signed world impulse; the oracle row additionally receives its impulse-weighted contact centroid. Neither field is available to online authority.",
            "- All variants retain the same four support-hypothesis accelerations and R203 structured acceleration reserve. The comparison isolates whether R204's remaining velocity misses arise from omitting impulse, evaluating `M⁻¹Jᵀ` at the causal prospective point, or residual generalized momentum beyond both.",
            "- The contact centroid is weighted by normal impulse across all MuJoCo substeps in the 5 ms interval. A no-contact wheel falls back to its prospective point and contributes zero impulse.",
            "",
            "## Exact-impulse response localization",
            "",
            *markdown_table(
                [
                    "variant",
                    "n",
                    "retained",
                    "fresh",
                    "max miss /s",
                    "root ω p95",
                    "root v p95",
                    "joint p95",
                    "response p99 µs",
                ],
                rows,
            ),
            "",
            f"Impulse-weighted centroid availability is **{centroid['all']['contact_centroid_available_fraction'] * 100.0:.1f}%** of wheel/sample pairs; prospective-to-centroid error is **{centroid['all']['prospective_to_centroid_error_m']['p95'] * 1_000.0:.3f} mm p95 / {centroid['all']['prospective_to_centroid_error_m']['maximum'] * 1_000.0:.3f} mm max** when available.",
            "",
            f"Exact impulse at the prospective point improves retained coverage from **{results['support_acceleration_only']['retained']['sample_coverage'] * 100.0:.3f}%** to **{prospective['retained']['sample_coverage'] * 100.0:.3f}%**. Replacing that point with the completed normal-impulse centroid lowers coverage to **{centroid['retained']['sample_coverage'] * 100.0:.3f}%** and raises maximum miss from **{prospective['all']['maximum_component_exceedance_per_s']:.6f}/s** to **{centroid['all']['maximum_component_exceedance_per_s']:.6f}/s**.",
            "",
            "## Decision",
            "",
            "The oracle normal-impulse centroid does not close the misses and is worse than the causal prospective point. A single relocated force point therefore cannot represent the distributed wheel contact. The next contact layer must retain spatial impulse moment/contact distribution, while root/generalized-momentum residual remains a separate calibrated layer. A tighter force-only polytope cannot establish authority by itself, and label-only fields cannot enter command authority.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-contact-response-localization-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_CONTACT_RESPONSE_LOCALIZATION_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "support_only_retained_coverage": results["support_acceleration_only"]["retained"]["sample_coverage"],
                "prospective_retained_coverage": prospective["retained"]["sample_coverage"],
                "centroid_retained_coverage": centroid["retained"]["sample_coverage"],
                "centroid_point_error_p95_mm": centroid["all"]["prospective_to_centroid_error_m"]["p95"] * 1_000.0,
            }
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
