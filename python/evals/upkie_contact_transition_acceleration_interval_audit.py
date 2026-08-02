#!/usr/bin/env python3
"""R203 continuous-acceleration interval audit around the R202 contact tube."""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_contact_impulse_residual_audit import profiles
from upkie_contact_program_robustness_ab import cases, execute
from upkie_contact_transition_interval_audit import urdf_total_mass_kg
from upkie_contact_transition_response_audit import (
    build_contact_witnesses,
    score_profile,
)


REVISION = "upkie-contact-transition-acceleration-interval-audit-r203"
DURATION_S = 6.0
FRESH_HOLDOUT_NAME = "forward_4n_friction_0p02_fresh"
IMPULSE_PROFILE = {
    "normal_acceleration_upper_m_s2": 100.0,
    "effective_mass_scale": 1.0,
    "total_mass_floor_scale": 0.0,
    "normal_load_scale": 2.0,
    "restitution_upper": 1.0,
}
ACCELERATION_PROFILES = {
    "zero_reserve": {
        "root_angular_rad_s2": 0.0,
        "root_linear_m_s2": 0.0,
        "joint_rad_s2": 0.0,
    },
    "root_linear_2mps2": {
        "root_angular_rad_s2": 0.0,
        "root_linear_m_s2": 2.0,
        "joint_rad_s2": 0.0,
    },
    # Frozen from the R202 retained-corpus maximum root-linear miss divided by
    # 5 ms (3.33 m/s²), rounded up before the fresh μ=0.02 row is evaluated.
    "root_linear_5mps2": {
        "root_angular_rad_s2": 0.0,
        "root_linear_m_s2": 5.0,
        "joint_rad_s2": 0.0,
    },
    "root_linear_10mps2": {
        "root_angular_rad_s2": 0.0,
        "root_linear_m_s2": 10.0,
        "joint_rad_s2": 0.0,
    },
    "structured_5_5_50": {
        "root_angular_rad_s2": 5.0,
        "root_linear_m_s2": 5.0,
        "joint_rad_s2": 50.0,
    },
}
PRIMARY_PROFILE = "root_linear_5mps2"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CONTACT_TRANSITION_ACCELERATION_INTERVAL_AUDIT_R203.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    parser.add_argument("--plant-profiles", help="comma-separated smoke subset")
    return parser.parse_args()


def acceleration_reserve(config: dict[str, float]) -> np.ndarray:
    return np.asarray(
        [config["root_angular_rad_s2"]] * 3
        + [config["root_linear_m_s2"]] * 3
        + [config["joint_rad_s2"]] * 6,
        np.float64,
    )


def aggregate_candidate_bounds(
    balance: Any,
    witnesses: np.ndarray,
    response: np.ndarray,
    candidates: np.ndarray,
    reserve: np.ndarray,
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
        timing = balance.bound_contact_transition_velocity_jump_with_acceleration_interval(
            np.asarray([0.0, 0.005], np.float64),
            IMPULSE_PROFILE["restitution_upper"],
            witnesses,
            candidate - reserve,
            candidate + reserve,
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


def selected_cases() -> tuple[Any, ...]:
    retained = cases()
    source = {case.name: case for case in retained}
    fresh = dataclasses.replace(
        source["forward_4n_friction_0p03"],
        name=FRESH_HOLDOUT_NAME,
        friction=0.02,
    )
    return (*retained, fresh)


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
        name: [] for name in ACCELERATION_PROFILES
    }
    run_metrics: dict[str, Any] = {}
    bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)
    response = np.empty((12, 2, 3), np.float64)
    effective_mass = np.empty((2, 3), np.float64)
    impulse_upper = np.empty((2, 3), np.float64)
    lower = np.empty(12, np.float64)
    upper = np.empty(12, np.float64)
    reserves = {
        name: acceleration_reserve(config)
        for name, config in ACCELERATION_PROFILES.items()
    }
    replay_rows: list[dict[str, Any]] = []

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
                np.asarray(
                    trace["inexact_observation_terminal_selector_queried"], np.bool_
                )
            )
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
                    bases,
                    response,
                    effective_mass,
                )
                witnesses = build_contact_witnesses(
                    prospective_velocity[tick],
                    effective_mass,
                    total_mass_kg,
                    case.friction,
                    IMPULSE_PROFILE,
                )
                replay_candidate = np.full((4, 12), np.nan, np.float64)
                replay_candidate[: len(candidates)] = candidates
                replay_rows.append(
                    {
                        "case": case.name,
                        "plant_profile": plant_profile_name,
                        "tick": int(tick),
                        "fresh_holdout": case.name == FRESH_HOLDOUT_NAME,
                        "friction": case.friction,
                        "candidate_count": len(candidates),
                        "candidates": replay_candidate,
                        "response": response.copy(),
                        "effective_mass": effective_mass.copy(),
                        "prospective_velocity": prospective_velocity[tick].copy(),
                        "actual_impulse": actual_impulse[tick].copy(),
                        "actual_delta": actual_delta[tick].copy(),
                    }
                )
                for profile_name, reserve in reserves.items():
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
                        reserve,
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
                    tangent_upper = math.sqrt(2.0) * impulse_upper[:, 0]
                    impulse_covered = bool(
                        np.all(actual_impulse[tick, :, 0] <= normal_upper + 1.0e-12)
                        and np.all(
                            actual_impulse[tick, :, 1] <= tangent_upper + 1.0e-12
                        )
                    )
                    samples_by_profile[profile_name].append(
                        {
                            "case": case.name,
                            "fresh_holdout": case.name == FRESH_HOLDOUT_NAME,
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
            f"[{case_index:02d}/{len(selected)}] {case.name}: samples={case_samples}",
            flush=True,
        )

    results: dict[str, Any] = {}
    for name, samples in samples_by_profile.items():
        retained = [sample for sample in samples if not sample["fresh_holdout"]]
        holdout = [sample for sample in samples if sample["fresh_holdout"]]
        results[name] = {
            "all": score_profile(samples),
            "retained": score_profile(retained) if retained else None,
            "fresh_holdout": score_profile(holdout) if holdout else None,
        }
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
    mechanism_passed = all_runs_finite and zero_plant_allocation and all(
        row["all"]["zero_rust_allocation"] for row in results.values()
    )
    rows = []
    for name, result in results.items():
        overall = result["all"]
        retained = result["retained"]
        holdout = result["fresh_holdout"]
        rows.append(
            [
                name,
                overall["sample_count"],
                f"{retained['sample_coverage'] * 100.0:.3f}%" if retained else "—",
                f"{holdout['sample_coverage'] * 100.0:.3f}%" if holdout else "—",
                f"{overall['component_coverage'] * 100.0:.3f}%",
                f"{overall['maximum_component_exceedance_per_s']:.5f}",
                f"{overall['root_angular_width_rad_s']['p95']:.3f}",
                f"{overall['root_linear_width_m_s']['p95']:.3f}",
                f"{overall['joint_width_rad_s']['p95']:.3f}",
                f"{overall['bound_timing_ns']['p99'] / 1_000.0:.3f}",
            ]
        )
    fresh_passed = bool(
        primary["fresh_holdout"]
        and primary["fresh_holdout"]["sample_coverage"] == 1.0
    )
    strict_retained = bool(
        primary["retained"] and primary["retained"]["sample_coverage"] == 1.0
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "total_mass_kg": total_mass_kg,
        "impulse_profile": IMPULSE_PROFILE,
        "acceleration_profiles": ACCELERATION_PROFILES,
        "primary_profile": PRIMARY_PROFILE,
        "fresh_holdout_name": FRESH_HOLDOUT_NAME,
        "primary_frozen_before_fresh_holdout": True,
        "mechanism_passed": mechanism_passed,
        "primary_strict_retained_coverage": strict_retained,
        "primary_fresh_holdout_coverage": fresh_passed,
        "authority_admitted": False,
        "all_runs_finite": all_runs_finite,
        "zero_plant_allocation_and_python_gc": zero_plant_allocation,
        "results": results,
        "run_metrics": run_metrics,
    }
    worst = primary["all"]["worst_sample"]
    report = "\n".join(
        [
            "# Bonesaw continuous-acceleration interval audit · r203",
            "",
            f"> Rust mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · retained strict coverage **{'PASS' if strict_retained else 'FAIL'}** · fresh μ=0.02 holdout **{'PASS' if fresh_passed else 'FAIL'}** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- R203 holds R202's tighter model-exact contact-impulse profile and model-owned `M⁻¹Jᵀ` fixed. It adds a distinct componentwise continuous generalized-acceleration interval around every support-hypothesis candidate, then considers all acceleration/time endpoint products before contact impulse is projected.",
            "- The primary ±5 m/s² root-linear reserve was frozen from the R202 retained maximum miss (0.01665 m/s over 5 ms, rounded above 3.33 m/s²) before introducing the new μ=0.02 row. It adds exactly 0.05 m/s root-linear width over a complete 5 ms interval and adds no angular/joint width.",
            "- Coverage and width stay separate. The fresh friction row tests construction transfer only; it is still the same Upkie morphology, MuJoCo contact law, controller, and observation-loss family.",
            "",
            "## Reserve sweep",
            "",
            *markdown_table(
                [
                    "acceleration profile",
                    "n",
                    "retained samples",
                    "fresh samples",
                    "component coverage",
                    "max miss /s",
                    "root ω width p95",
                    "root v width p95",
                    "joint width p95",
                    "bound p99 µs",
                ],
                rows,
            ),
            "",
            f"Primary worst sample: **{worst['case']}/{worst['plant_profile']}/tick {worst['tick']}**, maximum miss **{max(worst['component_exceedance']):.6f}/s**.",
            "",
            "## Decision",
            "",
            "A passing reserve admits only the acceleration-interval mechanism. Authority remains off because the independent friction box still produces very large wheel/joint widths, the fresh row is not a morphology/contact-law holdout, external-wrench provenance is not yet typed into the online command, and terminal consequence plus the 5 ms end-to-end gate remain separate.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        destination / "upkie-contact-transition-replay.npz",
        case=np.asarray([row["case"] for row in replay_rows]),
        plant_profile=np.asarray([row["plant_profile"] for row in replay_rows]),
        tick=np.asarray([row["tick"] for row in replay_rows], np.int64),
        fresh_holdout=np.asarray(
            [row["fresh_holdout"] for row in replay_rows], np.bool_
        ),
        friction=np.asarray([row["friction"] for row in replay_rows], np.float64),
        candidate_count=np.asarray(
            [row["candidate_count"] for row in replay_rows], np.uint8
        ),
        candidates=np.asarray([row["candidates"] for row in replay_rows]),
        response=np.asarray([row["response"] for row in replay_rows]),
        effective_mass=np.asarray([row["effective_mass"] for row in replay_rows]),
        prospective_velocity=np.asarray(
            [row["prospective_velocity"] for row in replay_rows]
        ),
        actual_impulse=np.asarray([row["actual_impulse"] for row in replay_rows]),
        actual_delta=np.asarray([row["actual_delta"] for row in replay_rows]),
    )
    (destination / "upkie-contact-transition-acceleration-interval-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_CONTACT_TRANSITION_ACCELERATION_INTERVAL_AUDIT.md").write_text(
        report
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "primary_retained_coverage": (
                    primary["retained"]["sample_coverage"]
                    if primary["retained"]
                    else None
                ),
                "primary_fresh_holdout_coverage": (
                    primary["fresh_holdout"]["sample_coverage"]
                    if primary["fresh_holdout"]
                    else None
                ),
                "primary_all_component_coverage": primary["all"][
                    "component_coverage"
                ],
                "primary_maximum_component_exceedance_per_s": primary["all"][
                    "maximum_component_exceedance_per_s"
                ],
            }
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
