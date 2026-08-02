#!/usr/bin/env python3
"""R202 model-owned contact-response and velocity-jump construction audit."""

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
from upkie_contact_program_robustness_ab import cases, execute
from upkie_contact_transition_interval_audit import urdf_total_mass_kg
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-contact-transition-response-audit-r202"
DURATION_S = 6.0
SQRT_TWO = math.sqrt(2.0)
COORDINATE_NAMES = (
    "root_wx",
    "root_wy",
    "root_wz",
    "root_vx",
    "root_vy",
    "root_vz",
    "left_hip",
    "left_knee",
    "left_wheel",
    "right_hip",
    "right_knee",
    "right_wheel",
)
BOUND_PROFILES = {
    "model_exact_100mps2": {
        "normal_acceleration_upper_m_s2": 100.0,
        "effective_mass_scale": 1.0,
        "total_mass_floor_scale": 0.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
    "model_2x_100mps2": {
        "normal_acceleration_upper_m_s2": 100.0,
        "effective_mass_scale": 2.0,
        "total_mass_floor_scale": 0.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
    "total_mass_floor_100mps2": {
        "normal_acceleration_upper_m_s2": 100.0,
        "effective_mass_scale": 1.0,
        "total_mass_floor_scale": 1.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
    "total_mass_floor_250mps2": {
        "normal_acceleration_upper_m_s2": 250.0,
        "effective_mass_scale": 1.0,
        "total_mass_floor_scale": 1.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
}
PRIMARY_PROFILE = "total_mass_floor_100mps2"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CONTACT_TRANSITION_RESPONSE_AUDIT_R202.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    parser.add_argument("--plant-profiles", help="comma-separated smoke subset")
    return parser.parse_args()


def build_contact_witnesses(
    prospective_velocity_m_s: np.ndarray,
    directional_effective_mass_kg: np.ndarray,
    total_mass_kg: float,
    friction_coefficient: float,
    profile: dict[str, float],
) -> np.ndarray:
    contacts = prospective_velocity_m_s.shape[0]
    witness = np.empty((contacts, 4), np.float64)
    closing_reserve = profile["normal_acceleration_upper_m_s2"] * CONTROL_DT
    witness[:, 0] = np.maximum(-prospective_velocity_m_s[:, 2], 0.0) + closing_reserve
    model_mass = (
        directional_effective_mass_kg[:, 2] * profile["effective_mass_scale"]
    )
    mass_floor = total_mass_kg * profile["total_mass_floor_scale"]
    witness[:, 1] = np.maximum(model_mass, mass_floor)
    witness[:, 2] = total_mass_kg * 9.81 * profile["normal_load_scale"]
    witness[:, 3] = friction_coefficient
    return witness


def aggregate_candidate_bounds(
    balance: Any,
    witnesses: np.ndarray,
    response: np.ndarray,
    candidates: np.ndarray,
    profile: dict[str, float],
    impulse_upper: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, int, int, int]:
    aggregate_lower = np.full(lower.shape, np.inf, np.float64)
    aggregate_upper = np.full(upper.shape, -np.inf, np.float64)
    elapsed_ns = 0
    allocation_calls = 0
    allocated_bytes = 0
    for candidate in candidates:
        timing = balance.bound_contact_transition_velocity_jump(
            np.asarray([0.0, CONTROL_DT]),
            profile["restitution_upper"],
            witnesses,
            candidate,
            response,
            impulse_upper,
            lower,
            upper,
        )
        np.minimum(aggregate_lower, lower, out=aggregate_lower)
        np.maximum(aggregate_upper, upper, out=aggregate_upper)
        elapsed_ns += int(timing[0])
        allocation_calls += int(timing[1])
        allocated_bytes += int(timing[2])
    return (
        aggregate_lower,
        aggregate_upper,
        elapsed_ns,
        allocation_calls,
        allocated_bytes,
    )


def score_profile(samples: list[dict[str, Any]]) -> dict[str, Any]:
    covered = np.asarray([sample["covered"] for sample in samples], np.bool_)
    component_covered = np.asarray(
        [sample["component_covered"] for sample in samples], np.bool_
    )
    exceedance = np.asarray(
        [sample["component_exceedance"] for sample in samples], np.float64
    )
    width = np.asarray([sample["interval_width"] for sample in samples], np.float64)
    utilization = np.asarray(
        [sample["interval_utilization"] for sample in samples], np.float64
    )
    impulse_covered = np.asarray(
        [sample["impulse_covered"] for sample in samples], np.bool_
    )
    model_timing_ns = np.asarray(
        [sample["model_timing_ns"] for sample in samples], np.float64
    )
    bound_timing_ns = np.asarray(
        [sample["bound_timing_ns"] for sample in samples], np.float64
    )
    complete_coverage = np.all(component_covered, axis=1)
    uncovered_samples = [
        {
            "case": sample["case"],
            "plant_profile": sample["plant_profile"],
            "tick": sample["tick"],
            "selected_action": sample["selected_action"],
            "coordinates": [
                name
                for name, component_ok in zip(
                    COORDINATE_NAMES, component_covered[index], strict=True
                )
                if not component_ok
            ],
            "maximum_component_exceedance_per_s": float(np.max(exceedance[index])),
        }
        for index, sample in enumerate(samples)
        if not complete_coverage[index]
    ]
    worst = max(
        samples,
        key=lambda sample: (
            max(sample["component_exceedance"]),
            max(sample["interval_width"]),
        ),
    )
    return {
        "sample_count": len(samples),
        "covered_samples": int(np.sum(covered)),
        "sample_coverage": float(np.mean(covered)),
        "component_coverage": float(np.mean(component_covered)),
        "impulse_sample_coverage": float(np.mean(impulse_covered)),
        "maximum_component_exceedance_per_s": float(np.max(exceedance)),
        "uncovered_samples": uncovered_samples,
        "interval_width_per_s": distribution(width.reshape(-1)),
        "interval_width_norm_per_s": distribution(np.linalg.norm(width, axis=1)),
        "maximum_coordinate_width_per_s": distribution(np.max(width, axis=1)),
        "root_angular_width_rad_s": distribution(width[:, :3].reshape(-1)),
        "root_linear_width_m_s": distribution(width[:, 3:6].reshape(-1)),
        "joint_width_rad_s": distribution(width[:, 6:].reshape(-1)),
        "covered_interval_utilization": distribution(
            utilization[component_covered]
            if np.any(component_covered)
            else np.asarray([math.inf])
        ),
        "model_response_timing_ns": distribution(model_timing_ns),
        "bound_timing_ns": distribution(bound_timing_ns),
        "zero_rust_allocation": all(
            sample["model_allocation_calls"] == 0
            and sample["model_allocated_bytes"] == 0
            and sample["bound_allocation_calls"] == 0
            and sample["bound_allocated_bytes"] == 0
            for sample in samples
        ),
        "worst_sample": worst,
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
    configured_profiles = profiles()
    if args.plant_profiles:
        requested_profiles = set(args.plant_profiles.split(","))
        configured_profiles = {
            name: config
            for name, config in configured_profiles.items()
            if name in requested_profiles
        }
        missing_profiles = requested_profiles - set(configured_profiles)
        if missing_profiles:
            raise SystemExit(f"unknown plant profiles: {sorted(missing_profiles)}")

    model = pathlib.Path(args.model).resolve()
    total_mass_kg = urdf_total_mass_kg(model)
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model))
    samples_by_profile: dict[str, list[dict[str, Any]]] = {
        name: [] for name in BOUND_PROFILES
    }
    run_metrics: dict[str, Any] = {}
    contact_bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)
    response = np.empty((12, 2, 3), np.float64)
    effective_mass = np.empty((2, 3), np.float64)
    impulse_upper = np.empty((2, 3), np.float64)
    lower = np.empty(12, np.float64)
    upper = np.empty(12, np.float64)

    for case_index, case in enumerate(selected_cases, 1):
        run_metrics[case.name] = {}
        case_samples = 0
        for plant_profile_name, config in configured_profiles.items():
            kwargs = {name: value for name, value in config.items() if name != "profile"}
            kwargs["record_physical_contact_impulses"] = True
            kwargs["record_physical_prospective_contact_state"] = True
            run = execute(model, case, args.duration, config["profile"], **kwargs)
            trace = run["trace"]
            queried = np.asarray(
                trace["inexact_observation_terminal_selector_queried"], np.bool_
            )
            ticks = np.flatnonzero(queried)
            case_samples += len(ticks)
            actions = np.asarray(
                trace["inexact_observation_terminal_selector_action"], np.uint8
            )
            hypothesis_acceleration = np.asarray(
                trace["inexact_observation_terminal_hypothesis_acceleration"],
                np.float64,
            )
            hypothesis_available = np.asarray(
                trace["inexact_observation_terminal_hypothesis_available"], np.bool_
            )
            root_position = np.asarray(trace["root_position"], np.float64)
            root_quaternion = np.asarray(trace["root_quaternion_wxyz"], np.float64)
            q = np.asarray(trace["q"], np.float64)
            pre_velocity = np.concatenate(
                (
                    np.asarray(trace["root_twist"], np.float64),
                    np.asarray(trace["v"], np.float64),
                ),
                axis=1,
            )
            post_velocity = np.concatenate(
                (
                    np.asarray(trace["post_root_twist"], np.float64),
                    np.asarray(trace["post_v"], np.float64),
                ),
                axis=1,
            )
            actual_delta = post_velocity - pre_velocity
            points = np.asarray(
                trace["physical_wheel_prospective_contact_point_world_m"],
                np.float64,
            )
            prospective_velocity = np.asarray(
                trace["physical_wheel_prospective_contact_velocity_m_s"],
                np.float64,
            )
            actual_impulse = np.asarray(
                trace["physical_wheel_contact_impulse_ns"], np.float64
            )
            for tick in ticks:
                action = int(actions[tick])
                available = hypothesis_available[tick, :, action]
                candidates = hypothesis_acceleration[tick, available, action]
                if len(candidates) == 0:
                    raise RuntimeError(
                        f"selected terminal action has no support hypothesis at tick {tick}"
                    )
                model_timing = balance.model_contact_impulse_velocity_response(
                    root_position[tick],
                    root_quaternion[tick],
                    q[tick],
                    points[tick],
                    contact_bases,
                    response,
                    effective_mass,
                )
                for bound_name, bound_profile in BOUND_PROFILES.items():
                    witnesses = build_contact_witnesses(
                        prospective_velocity[tick],
                        effective_mass,
                        total_mass_kg,
                        case.friction,
                        bound_profile,
                    )
                    (
                        aggregate_lower,
                        aggregate_upper,
                        bound_timing_ns,
                        bound_allocation_calls,
                        bound_allocated_bytes,
                    ) = aggregate_candidate_bounds(
                        balance,
                        witnesses,
                        response,
                        candidates,
                        bound_profile,
                        impulse_upper,
                        lower,
                        upper,
                    )
                    actual = actual_delta[tick]
                    component_exceedance = np.maximum(
                        np.maximum(aggregate_lower - actual, actual - aggregate_upper),
                        0.0,
                    )
                    component_covered = component_exceedance <= 1.0e-12
                    width = aggregate_upper - aggregate_lower
                    midpoint = 0.5 * (aggregate_upper + aggregate_lower)
                    utilization = np.divide(
                        2.0 * np.abs(actual - midpoint),
                        width,
                        out=np.full_like(width, math.inf),
                        where=width > 0.0,
                    )
                    normal_upper = impulse_upper[:, 2]
                    tangent_upper = SQRT_TWO * impulse_upper[:, 0]
                    impulse_covered = bool(
                        np.all(actual_impulse[tick, :, 0] <= normal_upper + 1.0e-12)
                        and np.all(
                            actual_impulse[tick, :, 1] <= tangent_upper + 1.0e-12
                        )
                    )
                    samples_by_profile[bound_name].append(
                        {
                            "case": case.name,
                            "plant_profile": plant_profile_name,
                            "tick": int(tick),
                            "selected_action": action,
                            "candidate_count": int(len(candidates)),
                            "covered": bool(np.all(component_covered)),
                            "component_covered": component_covered.tolist(),
                            "component_exceedance": component_exceedance.tolist(),
                            "interval_width": width.tolist(),
                            "interval_utilization": utilization.tolist(),
                            "actual_delta": actual.tolist(),
                            "lower": aggregate_lower.tolist(),
                            "upper": aggregate_upper.tolist(),
                            "impulse_covered": impulse_covered,
                            "normal_effective_mass_kg": effective_mass[:, 2].tolist(),
                            "model_timing_ns": int(model_timing[0]),
                            "model_allocation_calls": int(model_timing[1]),
                            "model_allocated_bytes": int(model_timing[2]),
                            "bound_timing_ns": bound_timing_ns,
                            "bound_allocation_calls": bound_allocation_calls,
                            "bound_allocated_bytes": bound_allocated_bytes,
                        }
                    )
            run_metrics[case.name][plant_profile_name] = run["metrics"]
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: samples={case_samples}",
            flush=True,
        )

    results = {name: score_profile(samples) for name, samples in samples_by_profile.items()}
    primary = results[PRIMARY_PROFILE]
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
    mechanism_passed = (
        all_runs_finite
        and zero_plant_allocation
        and all(result["zero_rust_allocation"] for result in results.values())
    )
    rows = [
        [
            name,
            result["sample_count"],
            f"{result['sample_coverage'] * 100.0:.3f}%",
            f"{result['component_coverage'] * 100.0:.3f}%",
            f"{result['impulse_sample_coverage'] * 100.0:.3f}%",
            f"{result['maximum_component_exceedance_per_s']:.3f}",
            f"{result['root_angular_width_rad_s']['p95']:.3f}",
            f"{result['root_linear_width_m_s']['p95']:.3f}",
            f"{result['joint_width_rad_s']['p95']:.3f}",
            f"{result['model_response_timing_ns']['p99'] / 1_000.0:.3f}",
            f"{result['bound_timing_ns']['p99'] / 1_000.0:.3f}",
        ]
        for name, result in results.items()
    ]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "total_mass_kg": total_mass_kg,
        "control_interval_s": CONTROL_DT,
        "primary_profile": PRIMARY_PROFILE,
        "bound_profiles": BOUND_PROFILES,
        "mechanism_passed": mechanism_passed,
        "primary_strict_velocity_jump_coverage": primary["sample_coverage"] == 1.0,
        "useful_width_admitted": False,
        "fresh_holdout_passed": False,
        "authority_admitted": False,
        "all_runs_finite": all_runs_finite,
        "zero_plant_allocation_and_python_gc": zero_plant_allocation,
        "results": results,
        "run_metrics": run_metrics,
    }
    worst = primary["worst_sample"]
    report = "\n".join(
        [
            "# Bonesaw model-owned contact response audit · r202",
            "",
            f"> Rust mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · primary full-sample velocity-jump coverage **{primary['sample_coverage'] * 100.0:.3f}%** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- Rust now owns FK, the floating mass matrix, one in-place Cholesky factorization, each prospective wheel-point Jacobian, all `M⁻¹Jᵀ` solves, and directional effective mass. Python supplies state and collision-query points, then owns corpus construction and scoring.",
            "- The generalized tangent is `[root angular; root linear; six joints]`. For the selected terminal action, the interval is the componentwise union of every available left/right support-hypothesis acceleration plus the physical contact-impulse box. Completed-step MuJoCo velocity and impulse are labels only.",
            "- Width is reported beside coverage: a trivially huge interval is not silently promoted. Model-derived normal effective mass is evaluated directly; the declared total-mass floor is a conservative construction parameter, not a learned residual.",
            "",
            "## Named-case construction result",
            "",
            *markdown_table(
                [
                    "bound profile",
                    "n",
                    "sample coverage",
                    "component coverage",
                    "impulse coverage",
                    "max miss /s",
                    "root ω width p95",
                    "root v width p95",
                    "joint width p95",
                    "model p99 µs",
                    "bound p99 µs",
                ],
                rows,
            ),
            "",
            f"Primary worst sample: **{worst['case']}/{worst['plant_profile']}/tick {worst['tick']}**, action **{worst['selected_action']}**, support candidates **{worst['candidate_count']}**, maximum component miss **{max(worst['component_exceedance']):.6f}/s**.",
            "",
            "Primary uncovered samples: "
            + "; ".join(
                f"**{sample['case']}/{sample['plant_profile']}/tick {sample['tick']}** "
                f"({', '.join(sample['coordinates'])}, "
                f"{sample['maximum_component_exceedance_per_s']:.6f}/s)"
                for sample in primary["uncovered_samples"]
            )
            + ".",
            "",
            "## Decision",
            "",
            "This revision can admit the model-owned response mechanism, but not command authority. Promotion still requires zero-miss coverage with useful width on a fresh morphology/friction/contact-timing holdout, explicit response uncertainty or a proven morphology envelope, plant-consequence non-regression, and the independent 5 ms timing gate.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-contact-transition-response-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_CONTACT_TRANSITION_RESPONSE_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "primary_sample_coverage": primary["sample_coverage"],
                "primary_component_coverage": primary["component_coverage"],
                "primary_impulse_coverage": primary["impulse_sample_coverage"],
                "primary_maximum_component_exceedance_per_s": primary[
                    "maximum_component_exceedance_per_s"
                ],
            }
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
