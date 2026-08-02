#!/usr/bin/env python3
"""R204 audit of passive directional contact-impulse bounds and useful width."""

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
from upkie_contact_transition_interval_audit import urdf_total_mass_kg
from upkie_contact_transition_response_audit import (
    build_contact_witnesses,
    score_profile,
)
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-directional-contact-transition-audit-r204"
DURATION_S = 6.0
STRUCTURED_ACCELERATION_RESERVE = np.asarray(
    [5.0] * 3 + [5.0] * 3 + [50.0] * 6, np.float64
)
BASELINE_IMPULSE_PROFILE = {
    "normal_acceleration_upper_m_s2": 100.0,
    "effective_mass_scale": 1.0,
    "total_mass_floor_scale": 0.0,
    "normal_load_scale": 2.0,
    "restitution_upper": 1.0,
}
DIRECTIONAL_PROFILES = {
    "exact_slip_zero_tangent_load": {
        "tangential_acceleration_upper_m_s2": 0.0,
        "normal_acceleration_upper_m_s2": 100.0,
        "effective_mass_scale": 1.0,
        "tangential_load_upper_n": 0.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
    "passive_100mps2_8n": {
        "tangential_acceleration_upper_m_s2": 100.0,
        "normal_acceleration_upper_m_s2": 100.0,
        "effective_mass_scale": 1.0,
        "tangential_load_upper_n": 8.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
    "passive_250mps2_8n": {
        "tangential_acceleration_upper_m_s2": 250.0,
        "normal_acceleration_upper_m_s2": 100.0,
        "effective_mass_scale": 1.0,
        "tangential_load_upper_n": 8.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
    "passive_100mps2_16n": {
        "tangential_acceleration_upper_m_s2": 100.0,
        "normal_acceleration_upper_m_s2": 100.0,
        "effective_mass_scale": 1.0,
        "tangential_load_upper_n": 16.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
}
PRIMARY_PROFILE = "passive_100mps2_8n"
BASELINE_NAME = "coulomb_box_r203_structured"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_DIRECTIONAL_CONTACT_TRANSITION_AUDIT_R204.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    parser.add_argument("--plant-profiles", help="comma-separated smoke subset")
    return parser.parse_args()


def build_directional_witnesses(
    prospective_velocity_m_s: np.ndarray,
    directional_effective_mass_kg: np.ndarray,
    total_mass_kg: float,
    friction_coefficient: float,
    config: dict[str, float],
) -> np.ndarray:
    witnesses = np.empty((prospective_velocity_m_s.shape[0], 10), np.float64)
    speed_reserve = np.asarray(
        [
            config["tangential_acceleration_upper_m_s2"] * CONTROL_DT,
            config["tangential_acceleration_upper_m_s2"] * CONTROL_DT,
            config["normal_acceleration_upper_m_s2"] * CONTROL_DT,
        ],
        np.float64,
    )
    witnesses[:, :3] = np.abs(prospective_velocity_m_s) + speed_reserve
    witnesses[:, 3:6] = (
        directional_effective_mass_kg * config["effective_mass_scale"]
    )
    witnesses[:, 6] = config["tangential_load_upper_n"]
    witnesses[:, 7] = config["tangential_load_upper_n"]
    witnesses[:, 8] = total_mass_kg * 9.81 * config["normal_load_scale"]
    witnesses[:, 9] = friction_coefficient
    return witnesses


def aggregate_bounds(
    balance: Any,
    method: str,
    restitution_upper: float,
    witnesses: np.ndarray,
    response: np.ndarray,
    candidates: np.ndarray,
    impulse_upper: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, int, int]:
    aggregate_lower = np.full(lower.shape, np.inf, np.float64)
    aggregate_upper = np.full(upper.shape, -np.inf, np.float64)
    elapsed_ns = 0
    allocation_calls = 0
    allocated_bytes = 0
    for candidate in candidates:
        timing = getattr(balance, method)(
            np.asarray([0.0, CONTROL_DT], np.float64),
            restitution_upper,
            witnesses,
            candidate - STRUCTURED_ACCELERATION_RESERVE,
            candidate + STRUCTURED_ACCELERATION_RESERVE,
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
        impulse_upper.copy(),
        elapsed_ns,
        allocation_calls,
        allocated_bytes,
    )


def sample_record(
    *,
    case: Any,
    plant_profile_name: str,
    tick: int,
    action: int,
    candidate_count: int,
    actual_delta: np.ndarray,
    actual_impulse_world: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    impulse_upper: np.ndarray,
    effective_mass: np.ndarray,
    model_timing: tuple[int, int, int],
    bound_timing: tuple[int, int, int],
) -> dict[str, Any]:
    exceedance = np.maximum(np.maximum(lower - actual_delta, actual_delta - upper), 0.0)
    component_covered = exceedance <= 1.0e-12
    width = upper - lower
    midpoint = 0.5 * (upper + lower)
    utilization = np.divide(
        2.0 * np.abs(actual_delta - midpoint),
        width,
        out=np.full_like(width, math.inf),
        where=width > 0.0,
    )
    component_impulse_covered = (
        np.abs(actual_impulse_world) <= impulse_upper + 1.0e-12
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
        "actual_delta": actual_delta.tolist(),
        "lower": lower.tolist(),
        "upper": upper.tolist(),
        "impulse_covered": bool(np.all(component_impulse_covered)),
        "impulse_component_covered": component_impulse_covered.tolist(),
        "actual_impulse_world_ns": actual_impulse_world.tolist(),
        "impulse_upper_ns": impulse_upper.tolist(),
        "normal_effective_mass_kg": effective_mass[:, 2].tolist(),
        "model_timing_ns": int(model_timing[0]),
        "model_allocation_calls": int(model_timing[1]),
        "model_allocated_bytes": int(model_timing[2]),
        "bound_timing_ns": int(bound_timing[0]),
        "bound_allocation_calls": int(bound_timing[1]),
        "bound_allocated_bytes": int(bound_timing[2]),
    }


def score_samples(samples: list[dict[str, Any]]) -> dict[str, Any]:
    result = score_profile(samples)
    actual = np.abs(
        np.asarray([sample["actual_impulse_world_ns"] for sample in samples], np.float64)
    )
    upper = np.asarray(
        [sample["impulse_upper_ns"] for sample in samples], np.float64
    )
    utilization = np.divide(
        actual,
        upper,
        out=np.zeros_like(actual),
        where=upper > 0.0,
    )
    component_covered = actual <= upper + 1.0e-12
    result["impulse_component_coverage"] = float(np.mean(component_covered))
    result["maximum_impulse_exceedance_ns"] = float(
        np.max(np.maximum(actual - upper, 0.0))
    )
    result["maximum_impulse_utilization"] = distribution(
        np.max(utilization, axis=(1, 2))
    )
    result["impulse_utilization_by_world_axis"] = {
        axis: distribution(utilization[:, :, index].reshape(-1))
        for index, axis in enumerate(("x", "y", "z"))
    }
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
    total_mass_kg = urdf_total_mass_kg(model)
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model))
    samples: dict[str, list[dict[str, Any]]] = {
        BASELINE_NAME: [],
        **{name: [] for name in DIRECTIONAL_PROFILES},
    }
    run_metrics: dict[str, Any] = {}
    bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)
    response = np.empty((12, 2, 3), np.float64)
    effective_mass = np.empty((2, 3), np.float64)
    impulse_upper = np.empty((2, 3), np.float64)
    lower = np.empty(12, np.float64)
    upper = np.empty(12, np.float64)

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
            points = np.asarray(
                trace["physical_wheel_prospective_contact_point_world_m"], np.float64
            )
            prospective_velocity = np.asarray(
                trace["physical_wheel_prospective_contact_velocity_m_s"], np.float64
            )
            actual_impulse_world = np.asarray(
                trace["physical_wheel_contact_impulse_world_ns"], np.float64
            )
            for tick in ticks:
                action = int(actions[tick])
                available = hypothesis_available[tick, :, action]
                candidates = hypothesis_acceleration[tick, available, action]
                if len(candidates) == 0:
                    raise RuntimeError("selected terminal action has no support hypothesis")
                model_timing = balance.model_contact_impulse_velocity_response(
                    root_position[tick],
                    root_quaternion[tick],
                    q[tick],
                    points[tick],
                    bases,
                    response,
                    effective_mass,
                )
                baseline_witnesses = build_contact_witnesses(
                    prospective_velocity[tick],
                    effective_mass,
                    total_mass_kg,
                    case.friction,
                    BASELINE_IMPULSE_PROFILE,
                )
                baseline = aggregate_bounds(
                    balance,
                    "bound_contact_transition_velocity_jump_with_acceleration_interval",
                    BASELINE_IMPULSE_PROFILE["restitution_upper"],
                    baseline_witnesses,
                    response,
                    candidates,
                    impulse_upper,
                    lower,
                    upper,
                )
                samples[BASELINE_NAME].append(
                    sample_record(
                        case=case,
                        plant_profile_name=plant_profile_name,
                        tick=int(tick),
                        action=action,
                        candidate_count=len(candidates),
                        actual_delta=actual_delta[tick],
                        actual_impulse_world=actual_impulse_world[tick],
                        lower=baseline[0],
                        upper=baseline[1],
                        impulse_upper=baseline[2],
                        effective_mass=effective_mass,
                        model_timing=model_timing,
                        bound_timing=baseline[3:],
                    )
                )
                for profile_name, profile in DIRECTIONAL_PROFILES.items():
                    witnesses = build_directional_witnesses(
                        prospective_velocity[tick],
                        effective_mass,
                        total_mass_kg,
                        case.friction,
                        profile,
                    )
                    directional = aggregate_bounds(
                        balance,
                        "bound_directional_contact_transition_velocity_jump",
                        profile["restitution_upper"],
                        witnesses,
                        response,
                        candidates,
                        impulse_upper,
                        lower,
                        upper,
                    )
                    samples[profile_name].append(
                        sample_record(
                            case=case,
                            plant_profile_name=plant_profile_name,
                            tick=int(tick),
                            action=action,
                            candidate_count=len(candidates),
                            actual_delta=actual_delta[tick],
                            actual_impulse_world=actual_impulse_world[tick],
                            lower=directional[0],
                            upper=directional[1],
                            impulse_upper=directional[2],
                            effective_mass=effective_mass,
                            model_timing=model_timing,
                            bound_timing=directional[3:],
                        )
                    )
            run_metrics[case.name][plant_profile_name] = run["metrics"]
        print(f"[{case_index:02d}/{len(selected)}] {case.name}: samples={case_samples}", flush=True)

    results = {name: split_score(rows) for name, rows in samples.items()}
    baseline = results[BASELINE_NAME]["all"]
    primary = results[PRIMARY_PROFILE]
    rows = []
    for name, result in results.items():
        overall = result["all"]
        retained = result["retained"]
        fresh = result["fresh_holdout"]
        rows.append(
            [
                name,
                overall["sample_count"],
                f"{retained['sample_coverage'] * 100.0:.3f}%" if retained else "—",
                f"{fresh['sample_coverage'] * 100.0:.3f}%" if fresh else "—",
                f"{overall['impulse_sample_coverage'] * 100.0:.3f}%",
                (
                    f"{overall['maximum_impulse_utilization']['p95'] * 100.0:.1f}%"
                    if overall["impulse_sample_coverage"] == 1.0
                    else "— (fails)"
                ),
                f"{overall['root_angular_width_rad_s']['p95']:.3f}",
                f"{overall['root_linear_width_m_s']['p95']:.3f}",
                f"{overall['joint_width_rad_s']['p95']:.3f}",
                f"{overall['joint_width_rad_s']['p95'] / baseline['joint_width_rad_s']['p95']:.3f}×",
                f"{overall['bound_timing_ns']['p99'] / 1_000.0:.3f}",
            ]
        )
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
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "total_mass_kg": total_mass_kg,
        "baseline_name": BASELINE_NAME,
        "baseline_impulse_profile": BASELINE_IMPULSE_PROFILE,
        "directional_profiles": DIRECTIONAL_PROFILES,
        "structured_acceleration_reserve": STRUCTURED_ACCELERATION_RESERVE.tolist(),
        "primary_profile": PRIMARY_PROFILE,
        "mechanism_passed": mechanism_passed,
        "primary_strict_retained_coverage": bool(
            primary["retained"] and primary["retained"]["sample_coverage"] == 1.0
        ),
        "primary_strict_fresh_coverage": bool(
            primary["fresh_holdout"]
            and primary["fresh_holdout"]["sample_coverage"] == 1.0
        ),
        "primary_impulse_coverage": primary["all"]["impulse_sample_coverage"],
        "useful_width_admitted": False,
        "authority_admitted": False,
        "all_runs_finite": all_runs_finite,
        "zero_plant_allocation_and_python_gc": zero_plant_allocation,
        "results": results,
        "run_metrics": run_metrics,
    }
    report = "\n".join(
        [
            "# Bonesaw directional contact-transition audit · r204",
            "",
            f"> Rust mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · primary retained/fresh strict coverage **{'PASS' if metrics['primary_strict_retained_coverage'] and metrics['primary_strict_fresh_coverage'] else 'FAIL'}** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- R204 keeps R203's structured acceleration reserve and model-exact normal impulse. Each tangential axis is bounded by the smaller of Coulomb capacity and a passive slip-arrest witness `m_eff v_slip + F_t Δt`.",
            "- The world-frame impulse label is the signed sum across the complete 5 ms plant interval. It is used only for scoring. Exact prospective velocity and MuJoCo contact impulse remain simulator-oracle fields without online sensor authority.",
            "- This is a directional outer box, not a coupled multi-contact Delassus polytope. Coverage and width are reported separately; width reduction alone cannot admit authority.",
            "",
            "## Directional reserve sweep",
            "",
            *markdown_table(
                [
                    "profile",
                    "n",
                    "retained velocity",
                    "fresh velocity",
                    "impulse coverage",
                    "impulse util p95",
                    "root ω p95",
                    "root v p95",
                    "joint p95",
                    "joint/base",
                    "bound p99 µs",
                ],
                rows,
            ),
            "",
            f"Primary signed world-impulse utilization is **{primary['all']['maximum_impulse_utilization']['p95'] * 100.0:.1f}% p95 / {primary['all']['maximum_impulse_utilization']['maximum'] * 100.0:.1f}% max** with **{primary['all']['impulse_component_coverage'] * 100.0:.3f}%** component coverage and **{primary['all']['maximum_impulse_exceedance_ns']:.6f} N·s** maximum exceedance.",
            "",
            "Primary uncovered retained samples: "
            + "; ".join(
                f"**{sample['case']}/{sample['plant_profile']}/tick {sample['tick']}** "
                f"({', '.join(sample['coordinates'])}, "
                f"{sample['maximum_component_exceedance_per_s']:.6f}/s)"
                for sample in primary["retained"]["uncovered_samples"]
            )
            + ".",
            "",
            "## Decision",
            "",
            "The directional mechanism may be retained only if it covers the world-impulse and generalized-velocity labels with materially smaller widths. It cannot establish online authority until the witnesses have typed sensor/load provenance, another morphology/contact-law holdout passes, terminal consequence is non-regressing, and the independent 5 ms gate passes.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-directional-contact-transition-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_DIRECTIONAL_CONTACT_TRANSITION_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "primary_retained_velocity_coverage": (
                    primary["retained"]["sample_coverage"] if primary["retained"] else None
                ),
                "primary_fresh_velocity_coverage": (
                    primary["fresh_holdout"]["sample_coverage"]
                    if primary["fresh_holdout"]
                    else None
                ),
                "primary_impulse_coverage": primary["all"]["impulse_sample_coverage"],
                "primary_joint_width_ratio": (
                    primary["all"]["joint_width_rad_s"]["p95"]
                    / baseline["joint_width_rad_s"]["p95"]
                ),
            }
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
