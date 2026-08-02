#!/usr/bin/env python3
"""R201 plant audit of the Rust contact-transition impulse interval."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_impulse_residual_audit import profiles
from upkie_contact_program_robustness_ab import cases, execute
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-contact-transition-interval-audit-r201"
DURATION_S = 6.0
SQRT_TWO = math.sqrt(2.0)
BOUND_PROFILES = {
    "no_closing_reserve": {
        "normal_acceleration_upper_m_s2": 0.0,
        "effective_mass_scale": 1.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
    "passive_50mps2": {
        "normal_acceleration_upper_m_s2": 50.0,
        "effective_mass_scale": 1.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
    "passive_100mps2": {
        "normal_acceleration_upper_m_s2": 100.0,
        "effective_mass_scale": 1.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
    "passive_250mps2": {
        "normal_acceleration_upper_m_s2": 250.0,
        "effective_mass_scale": 1.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
    "half_mass_100mps2": {
        "normal_acceleration_upper_m_s2": 100.0,
        "effective_mass_scale": 0.5,
        "normal_load_scale": 2.0,
        "restitution_upper": 1.0,
    },
    "inelastic_100mps2": {
        "normal_acceleration_upper_m_s2": 100.0,
        "effective_mass_scale": 1.0,
        "normal_load_scale": 2.0,
        "restitution_upper": 0.0,
    },
}
PRIMARY_PROFILE = "passive_100mps2"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CONTACT_TRANSITION_INTERVAL_AUDIT_R201.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def urdf_total_mass_kg(model: pathlib.Path) -> float:
    root = ET.parse(model).getroot()
    masses = [
        float(mass.get("value", "nan"))
        for mass in root.findall("./link/inertial/mass")
    ]
    if not masses or any(not math.isfinite(value) or value <= 0.0 for value in masses):
        raise ValueError("URDF must provide finite positive inertial mass for every rigid link")
    return float(sum(masses))


def build_contact_witnesses(
    prospective_velocity_m_s: np.ndarray,
    total_mass_kg: float,
    friction_coefficient: float,
    profile: dict[str, float],
) -> np.ndarray:
    contacts = prospective_velocity_m_s.shape[0]
    witness = np.empty((contacts, 4), np.float64)
    closing_reserve = (
        profile["normal_acceleration_upper_m_s2"] * CONTROL_DT
    )
    witness[:, 0] = np.maximum(-prospective_velocity_m_s[:, 2], 0.0) + closing_reserve
    witness[:, 1] = total_mass_kg * profile["effective_mass_scale"]
    witness[:, 2] = total_mass_kg * 9.81 * profile["normal_load_scale"]
    witness[:, 3] = friction_coefficient
    return witness


def score_profile(samples: list[dict[str, Any]]) -> dict[str, Any]:
    covered = np.asarray([sample["covered"] for sample in samples], np.bool_)
    component_covered = np.asarray(
        [sample["component_covered"] for sample in samples], np.bool_
    )
    normal_exceedance = np.asarray(
        [sample["normal_exceedance"] for sample in samples], np.float64
    )
    tangential_exceedance = np.asarray(
        [sample["tangential_exceedance"] for sample in samples], np.float64
    )
    normal_upper = np.asarray(
        [sample["maximum_normal_upper"] for sample in samples], np.float64
    )
    utilization = np.asarray(
        [sample["maximum_impulse_utilization"] for sample in samples], np.float64
    )
    timing_ns = np.asarray([sample["timing_ns"] for sample in samples], np.float64)
    return {
        "sample_count": len(samples),
        "covered_samples": int(np.sum(covered)),
        "sample_coverage": float(np.mean(covered)),
        "component_coverage": float(np.mean(component_covered)),
        "maximum_normal_exceedance_ns": float(np.max(normal_exceedance)),
        "maximum_tangential_exceedance_ns": float(np.max(tangential_exceedance)),
        "maximum_normal_impulse_upper_ns": distribution(normal_upper),
        "maximum_impulse_utilization": distribution(utilization),
        "rust_timing_ns": distribution(timing_ns),
        "zero_rust_allocation": all(
            sample["allocation_calls"] == 0 and sample["allocated_bytes"] == 0
            for sample in samples
        ),
        "worst_sample": max(
            samples,
            key=lambda sample: (
                max(sample["normal_exceedance"], sample["tangential_exceedance"]),
                sample["maximum_impulse_utilization"],
            ),
        ),
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
    total_mass_kg = urdf_total_mass_kg(model)
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model))
    samples_by_profile: dict[str, list[dict[str, Any]]] = {
        name: [] for name in BOUND_PROFILES
    }
    run_metrics: dict[str, Any] = {}
    zero_acceleration = np.zeros(1, np.float64)
    zero_response = np.zeros((1, 2, 3), np.float64)
    impulse_upper = np.empty((2, 3), np.float64)
    delta_lower = np.empty(1, np.float64)
    delta_upper = np.empty(1, np.float64)
    for case_index, case in enumerate(selected_cases, 1):
        run_metrics[case.name] = {}
        case_sample_count = 0
        for profile_name, config in profiles().items():
            kwargs = {name: value for name, value in config.items() if name != "profile"}
            kwargs["record_physical_contact_impulses"] = True
            kwargs["record_physical_prospective_contact_state"] = True
            run = execute(
                model,
                case,
                args.duration,
                config["profile"],
                **kwargs,
            )
            trace = run["trace"]
            queried = np.asarray(
                trace["inexact_observation_terminal_selector_queried"], np.bool_
            )
            ticks = np.flatnonzero(queried)
            case_sample_count += len(ticks)
            actual_impulse = np.asarray(
                trace["physical_wheel_contact_impulse_ns"], np.float64
            )
            prospective_distance = np.asarray(
                trace["physical_wheel_prospective_contact_distance_m"], np.float64
            )
            prospective_velocity = np.asarray(
                trace["physical_wheel_prospective_contact_velocity_m_s"], np.float64
            )
            for tick in ticks:
                for bound_name, bound_profile in BOUND_PROFILES.items():
                    witnesses = build_contact_witnesses(
                        prospective_velocity[tick],
                        total_mass_kg,
                        case.friction,
                        bound_profile,
                    )
                    timing = balance.bound_contact_transition_velocity_jump(
                        np.asarray([0.0, CONTROL_DT]),
                        bound_profile["restitution_upper"],
                        witnesses,
                        zero_acceleration,
                        zero_response,
                        impulse_upper,
                        delta_lower,
                        delta_upper,
                    )
                    normal = actual_impulse[tick, :, 0]
                    tangent = actual_impulse[tick, :, 1]
                    normal_upper = impulse_upper[:, 2]
                    tangential_magnitude_upper = SQRT_TWO * impulse_upper[:, 0]
                    normal_exceedance = np.maximum(normal - normal_upper, 0.0)
                    tangential_exceedance = np.maximum(
                        tangent - tangential_magnitude_upper, 0.0
                    )
                    component_covered = np.concatenate(
                        (normal_exceedance <= 1.0e-12, tangential_exceedance <= 1.0e-12)
                    )
                    ratios = np.concatenate(
                        (
                            np.divide(
                                normal,
                                normal_upper,
                                out=np.zeros_like(normal),
                                where=normal_upper > 0.0,
                            ),
                            np.divide(
                                tangent,
                                tangential_magnitude_upper,
                                out=np.zeros_like(tangent),
                                where=tangential_magnitude_upper > 0.0,
                            ),
                        )
                    )
                    samples_by_profile[bound_name].append(
                        {
                            "case": case.name,
                            "plant_profile": profile_name,
                            "tick": int(tick),
                            "covered": bool(np.all(component_covered)),
                            "component_covered": component_covered.tolist(),
                            "normal_exceedance": float(np.max(normal_exceedance)),
                            "tangential_exceedance": float(
                                np.max(tangential_exceedance)
                            ),
                            "maximum_normal_upper": float(np.max(normal_upper)),
                            "maximum_impulse_utilization": float(np.max(ratios)),
                            "minimum_prospective_distance_m": float(
                                np.min(prospective_distance[tick])
                            ),
                            "maximum_closing_speed_m_s": float(
                                np.max(np.maximum(-prospective_velocity[tick, :, 2], 0.0))
                            ),
                            "timing_ns": int(timing[0]),
                            "allocation_calls": int(timing[1]),
                            "allocated_bytes": int(timing[2]),
                        }
                    )
            run_metrics[case.name][profile_name] = run["metrics"]
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: samples={case_sample_count}",
            flush=True,
        )
    results = {
        name: score_profile(samples) for name, samples in samples_by_profile.items()
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
        result["zero_rust_allocation"] for result in results.values()
    )
    rows = [
        [
            name,
            result["sample_count"],
            f"{result['sample_coverage'] * 100.0:.3f}%",
            f"{result['component_coverage'] * 100.0:.3f}%",
            f"{result['maximum_normal_exceedance_ns']:.6f}",
            f"{result['maximum_tangential_exceedance_ns']:.6f}",
            f"{result['maximum_normal_impulse_upper_ns']['p95']:.3f}",
            f"{result['maximum_impulse_utilization']['p95']:.3f}",
            f"{result['rust_timing_ns']['p99'] / 1_000.0:.3f}",
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
        "primary_strict_impulse_coverage": primary["sample_coverage"] == 1.0,
        "authority_admitted": False,
        "all_runs_finite": all_runs_finite,
        "zero_plant_allocation_and_python_gc": zero_plant_allocation,
        "results": results,
        "run_metrics": run_metrics,
    }
    worst = primary["worst_sample"]
    report = "\n".join(
        [
            "# Bonesaw contact-transition interval audit · r201",
            "",
            f"> Rust mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · primary impulse coverage **{primary['sample_coverage'] * 100.0:.3f}%** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- Rust bounds each prospective wheel contact with `J_n ≤ m_eff,max(1+e_max)v_close,max + F_n,max Δt_max`, tangential axes with a friction-cone outer box, and projects the impulse through caller-owned `M⁻¹Jᵀ` into a generalized velocity-jump interval. The hot path is fixed-size and caller-buffered.",
            "- This plant audit evaluates the impulse part first. The prospective bottom-of-wheel distance and velocity are computed before the scored 5 ms interval. Completed-interval MuJoCo impulses are labels only and never enter the bound.",
            f"- Effective normal mass is bounded by the full free-floating URDF mass (**{total_mass_kg:.3f} kg**) per contact. The primary profile declares passive restitution `e≤1`, two-body-weight sustained load per contact, and 100 m/s² of unmodeled closing acceleration over 5 ms. It is fixed before case comparison, not learned from a held-out residual.",
            "- Tangential MuJoCo impulse is reported as a magnitude; it is checked against `√2 μJ_n,max`, the magnitude radius of Rust's independent two-axis outer box.",
            "",
            "## Named-case result",
            "",
            *markdown_table(
                [
                    "bound profile",
                    "n",
                    "sample coverage",
                    "component coverage",
                    "max normal miss N·s",
                    "max tangent miss N·s",
                    "normal bound p95 N·s",
                    "utilization p95",
                    "Rust p99 µs",
                ],
                rows,
            ),
            "",
            f"Primary worst sample: **{worst['case']}/{worst['plant_profile']}/tick {worst['tick']}**; normal/tangential exceedance **{worst['normal_exceedance']:.6f}/{worst['tangential_exceedance']:.6f} N·s**, prospective distance **{worst['minimum_prospective_distance_m']:.6f} m**, closing speed **{worst['maximum_closing_speed_m_s']:.3f} m/s**.",
            "",
            "## Decision",
            "",
            "Even 100% impulse coverage would admit only this physical outer-bound mechanism. Authority additionally requires a Rust-model-derived `M⁻¹Jᵀ`, componentwise realized velocity-jump coverage on fresh morphology/friction/contact-timing holdouts, useful interval width, plant consequence non-regression, and the independent 5 ms timing gate. Exact MuJoCo geometry and impulse remain evaluation oracles.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-contact-transition-interval-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_CONTACT_TRANSITION_INTERVAL_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "primary_sample_coverage": primary["sample_coverage"],
                "primary_component_coverage": primary["component_coverage"],
                "primary_maximum_normal_exceedance_ns": primary[
                    "maximum_normal_exceedance_ns"
                ],
                "primary_maximum_tangential_exceedance_ns": primary[
                    "maximum_tangential_exceedance_ns"
                ],
            }
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
