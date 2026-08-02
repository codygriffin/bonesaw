#!/usr/bin/env python3
"""R206 exact-label spatial contact-wrench and generalized-momentum audit."""

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
from upkie_contact_response_localization_audit import (
    candidate_interval,
    project_impulse_response,
    record_sample,
    split_score,
)
from upkie_contact_transition_acceleration_interval_audit import selected_cases
from upkie_directional_contact_transition_audit import STRUCTURED_ACCELERATION_RESERVE
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-spatial-contact-wrench-audit-r206"
DURATION_S = 6.0
VARIANTS = (
    "support_acceleration_only",
    "prospective_point_exact_force",
    "prospective_point_exact_spatial_wrench",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_SPATIAL_CONTACT_WRENCH_AUDIT_R206.html"
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    parser.add_argument("--plant-profiles", help="comma-separated smoke subset")
    return parser.parse_args()


def translate_moment_from_world_origin(
    moment_world_origin_nms: np.ndarray,
    reference_point_world_m: np.ndarray,
    impulse_world_ns: np.ndarray,
) -> np.ndarray:
    """Translate a spatial impulse moment from world origin to each reference."""
    return moment_world_origin_nms - np.cross(reference_point_world_m, impulse_world_ns)


def spatial_projection(response: np.ndarray, moment: np.ndarray, force: np.ndarray) -> np.ndarray:
    wrench = np.concatenate((moment, force), axis=1)
    return np.einsum("dcx,cx->d", response, wrench, optimize=False)


def residual_score(values: list[np.ndarray]) -> dict[str, Any]:
    residual = np.asarray(values, np.float64)
    norm = np.linalg.norm(residual, axis=1)
    return {
        "minimum_candidate_norm": distribution(norm),
        "root_angular_impulse_abs": distribution(np.abs(residual[:, :3]).reshape(-1)),
        "root_linear_impulse_abs": distribution(np.abs(residual[:, 3:6]).reshape(-1)),
        "joint_impulse_abs": distribution(np.abs(residual[:, 6:]).reshape(-1)),
    }


def reconstruction_score(values: list[np.ndarray]) -> dict[str, Any]:
    residual = np.asarray(values, np.float64)
    return {
        "norm": distribution(np.linalg.norm(residual, axis=1)),
        "root_angular_norm": distribution(np.linalg.norm(residual[:, :3], axis=1)),
        "root_linear_norm": distribution(np.linalg.norm(residual[:, 3:6], axis=1)),
        "joint_norm": distribution(np.linalg.norm(residual[:, 6:], axis=1)),
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
    momentum_residuals: dict[str, list[np.ndarray]] = {name: [] for name in VARIANTS}
    contact_reconstruction_residuals: dict[str, list[np.ndarray]] = {
        name: [] for name in VARIANTS
    }
    run_metrics: dict[str, Any] = {}
    bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)
    point_response = np.empty((12, 2, 3), np.float64)
    effective_mass = np.empty((2, 3), np.float64)
    spatial_response = np.empty((12, 2, 6), np.float64)
    spatial_delassus = np.empty((12, 12), np.float64)
    residual_scratch = np.empty((4, 12), np.float64)
    replay: dict[str, list[Any]] = {
        "case": [],
        "plant_profile": [],
        "tick": [],
        "fresh_holdout": [],
        "candidate_count": [],
        "candidates": [],
        "root_position": [],
        "root_quaternion_wxyz": [],
        "q": [],
        "prospective_point_world_m": [],
        "impulse_world_ns": [],
        "moment_world_origin_nms": [],
        "moment_about_prospective_nms": [],
        "physical_constraint_impulse_bonesaw_order": [],
        "reconstructed_spatial_generalized_impulse": [],
        "actual_delta": [],
        "spatial_response": [],
        "spatial_delassus": [],
    }
    spatial_moment_norms: list[float] = []
    spatial_timing: list[int] = []
    point_timing: list[int] = []
    momentum_timing: list[int] = []
    contact_projection_timing: list[int] = []
    minimum_spatial_eigenvalue: list[float] = []
    spatial_condition: list[float] = []
    zero_rust_allocation = True

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
            actions = np.asarray(trace["inexact_observation_terminal_selector_action"], np.uint8)
            acceleration = np.asarray(
                trace["inexact_observation_terminal_hypothesis_acceleration"], np.float64
            )
            available = np.asarray(
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
            moment_origin = np.asarray(
                trace["physical_wheel_contact_moment_world_origin_nms"], np.float64
            )
            physical_constraint_impulse = np.asarray(
                trace["physical_constraint_generalized_impulse_ns"], np.float64
            )
            contact_projection_input = np.empty((len(VARIANTS), 12), np.float64)
            contact_projection_output = np.empty_like(contact_projection_input)
            zero_delta = np.zeros(12, np.float64)
            physical_constraint_bonesaw = np.empty(12, np.float64)
            for tick in ticks:
                action = int(actions[tick])
                candidates = acceleration[tick, available[tick, :, action], action]
                if len(candidates) == 0:
                    raise RuntimeError("selected action has no support hypothesis")
                point_query = balance.model_contact_impulse_velocity_response(
                    root_position[tick],
                    root_quaternion[tick],
                    q[tick],
                    prospective_point[tick],
                    bases,
                    point_response,
                    effective_mass,
                )
                spatial_query = balance.model_contact_spatial_impulse_velocity_response(
                    root_position[tick],
                    root_quaternion[tick],
                    q[tick],
                    prospective_point[tick],
                    bases,
                    spatial_response,
                    spatial_delassus,
                )
                zero_rust_allocation &= point_query[1:] == (0, 0)
                zero_rust_allocation &= spatial_query[1:] == (0, 0)
                point_timing.append(int(point_query[0]))
                spatial_timing.append(int(spatial_query[0]))
                moment = translate_moment_from_world_origin(
                    moment_origin[tick], prospective_point[tick], impulse_world[tick]
                )
                spatial_moment_norms.extend(np.linalg.norm(moment, axis=1).tolist())
                eigenvalues = np.linalg.eigvalsh(spatial_delassus)
                minimum_spatial_eigenvalue.append(float(eigenvalues[0]))
                spatial_condition.append(float(np.linalg.cond(spatial_delassus)))
                impulse_delta = {
                    "support_acceleration_only": np.zeros(12, np.float64),
                    "prospective_point_exact_force": project_impulse_response(
                        point_response, impulse_world[tick]
                    ),
                    "prospective_point_exact_spatial_wrench": spatial_projection(
                        spatial_response, moment, impulse_world[tick]
                    ),
                }
                for variant_index, name in enumerate(VARIANTS):
                    np.negative(
                        impulse_delta[name], out=contact_projection_input[variant_index]
                    )
                projection_query = (
                    balance.model_generalized_momentum_impulse_residuals(
                        root_position[tick],
                        root_quaternion[tick],
                        q[tick],
                        zero_delta,
                        contact_projection_input,
                        contact_projection_output,
                    )
                )
                zero_rust_allocation &= projection_query[1:] == (0, 0)
                contact_projection_timing.append(int(projection_query[0]))
                # MuJoCo's free-joint covector is linear then angular;
                # Bonesaw's public floating tangent is angular then linear.
                physical_constraint_bonesaw[:3] = physical_constraint_impulse[
                    tick, 3:6
                ]
                physical_constraint_bonesaw[3:6] = physical_constraint_impulse[
                    tick, :3
                ]
                physical_constraint_bonesaw[6:] = physical_constraint_impulse[
                    tick, 6:
                ]
                for variant_index, name in enumerate(VARIANTS):
                    contact_reconstruction_residuals[name].append(
                        (
                            contact_projection_output[variant_index]
                            - physical_constraint_bonesaw
                        ).copy()
                    )
                    lower, upper = candidate_interval(candidates, impulse_delta[name])
                    predicted = candidates * CONTROL_DT + impulse_delta[name]
                    residual = residual_scratch[: len(candidates)]
                    residual_timing = balance.model_generalized_momentum_impulse_residuals(
                        root_position[tick],
                        root_quaternion[tick],
                        q[tick],
                        actual_delta[tick],
                        predicted,
                        residual,
                    )
                    zero_rust_allocation &= residual_timing[1:] == (0, 0)
                    momentum_timing.append(int(residual_timing[0]))
                    chosen = int(np.argmin(np.linalg.norm(residual, axis=1)))
                    momentum_residuals[name].append(residual[chosen].copy())
                    query_timing = 0
                    if name == "prospective_point_exact_force":
                        query_timing = int(point_query[0])
                    elif name == "prospective_point_exact_spatial_wrench":
                        query_timing = int(spatial_query[0])
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
                            model_timing_ns=query_timing,
                            model_allocation_calls=0,
                            model_allocated_bytes=0,
                            point_error_m=np.zeros((2, 3), np.float64),
                            contact_centroid_available=np.ones(2, np.bool_),
                        )
                    )
                padded = np.full((4, 12), np.nan, np.float64)
                padded[: len(candidates)] = candidates
                for key, value in (
                    ("case", case.name),
                    ("plant_profile", plant_profile_name),
                    ("tick", int(tick)),
                    ("fresh_holdout", case.name.endswith("fresh")),
                    ("candidate_count", len(candidates)),
                    ("candidates", padded),
                    ("root_position", root_position[tick].copy()),
                    ("root_quaternion_wxyz", root_quaternion[tick].copy()),
                    ("q", q[tick].copy()),
                    ("prospective_point_world_m", prospective_point[tick].copy()),
                    ("impulse_world_ns", impulse_world[tick].copy()),
                    ("moment_world_origin_nms", moment_origin[tick].copy()),
                    ("moment_about_prospective_nms", moment.copy()),
                    (
                        "physical_constraint_impulse_bonesaw_order",
                        physical_constraint_bonesaw.copy(),
                    ),
                    (
                        "reconstructed_spatial_generalized_impulse",
                        contact_projection_output[
                            VARIANTS.index(
                                "prospective_point_exact_spatial_wrench"
                            )
                        ].copy(),
                    ),
                    ("actual_delta", actual_delta[tick].copy()),
                    ("spatial_response", spatial_response.copy()),
                    ("spatial_delassus", spatial_delassus.copy()),
                ):
                    replay[key].append(value)
            run_metrics[case.name][plant_profile_name] = run["metrics"]
        print(f"[{case_index:02d}/{len(selected)}] {case.name}: samples={case_samples}", flush=True)

    results = {name: split_score(rows) for name, rows in samples.items()}
    momentum = {name: residual_score(rows) for name, rows in momentum_residuals.items()}
    contact_reconstruction = {
        name: reconstruction_score(rows)
        for name, rows in contact_reconstruction_residuals.items()
    }
    table_rows = []
    for name in VARIANTS:
        result = results[name]
        fresh_coverage = (
            f"{result['fresh_holdout']['sample_coverage'] * 100.0:.3f}%"
            if result["fresh_holdout"] is not None
            else "—"
        )
        table_rows.append(
            [
                name,
                result["all"]["sample_count"],
                f"{result['retained']['sample_coverage'] * 100.0:.3f}%",
                fresh_coverage,
                f"{result['all']['maximum_component_exceedance_per_s']:.3f}",
                f"{result['all']['root_angular_width_rad_s']['p95']:.3f}",
                f"{result['all']['root_linear_width_m_s']['p95']:.3f}",
                f"{result['all']['joint_width_rad_s']['p95']:.3f}",
                f"{momentum[name]['minimum_candidate_norm']['p95']:.4f}",
                f"{result['all']['model_response_timing_ns']['p99'] / 1_000.0:.3f}",
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
    mechanism_passed = all_runs_finite and zero_plant_allocation and zero_rust_allocation
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "authority_admitted": False,
        "zero_rust_allocation": zero_rust_allocation,
        "zero_plant_allocation_and_python_gc": zero_plant_allocation,
        "all_runs_finite": all_runs_finite,
        "results": results,
        "generalized_momentum_residual": momentum,
        "contact_generalized_impulse_reconstruction_residual": (
            contact_reconstruction
        ),
        "contact_moment_about_prospective_nms": distribution(spatial_moment_norms),
        "point_response_timing_ns": distribution(point_timing),
        "spatial_response_timing_ns": distribution(spatial_timing),
        "momentum_residual_timing_ns": distribution(momentum_timing),
        "contact_projection_timing_ns": distribution(contact_projection_timing),
        "spatial_delassus_minimum_eigenvalue": distribution(minimum_spatial_eigenvalue),
        "spatial_delassus_condition": distribution(spatial_condition),
        "run_metrics": run_metrics,
    }
    force = results["prospective_point_exact_force"]
    spatial = results["prospective_point_exact_spatial_wrench"]
    spatial_fresh_coverage = (
        spatial["fresh_holdout"]["sample_coverage"]
        if spatial["fresh_holdout"] is not None
        else None
    )
    force_fresh_coverage = (
        force["fresh_holdout"]["sample_coverage"]
        if force["fresh_holdout"] is not None
        else None
    )
    report = "\n".join(
        [
            "# Bonesaw spatial contact-wrench audit · r206",
            "",
            f"> Spatial response mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · strict retained/fresh coverage **{'PASS' if spatial['retained']['sample_coverage'] == 1.0 and spatial_fresh_coverage == 1.0 else 'FAIL'}** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- The plant label integrates each wheel's complete signed force impulse and spatial moment about world origin, including contact free torque, across every 1 kHz substep. The evaluator translates that moment exactly to the causal prospective point.",
            "- Rust emits the twelve-axis spatial response and complete Delassus operator in `[moment XYZ; force XYZ]` order. A separate Rust query maps velocity error to generalized-momentum impulse `M(q) Δv`; neither completed label is online authority.",
            "- As a conservation cross-check, Rust maps each reconstructed contact velocity jump back through `M(q_pre)` and compares it with MuJoCo's independently recorded generalized constraint impulse, reordered to Bonesaw's `[root angular; root linear; joints]` tangent.",
            "- All variants retain identical support hypotheses and the unchanged R203 structured acceleration reserve. This isolates whether preserving distributed-contact moment explains the R205 force-point residual.",
            "",
            "## Exact-label decomposition",
            "",
            *markdown_table(
                [
                    "variant",
                    "n",
                    "retained coverage",
                    "fresh coverage",
                    "max miss /s",
                    "root ω p95",
                    "root v p95",
                    "joint p95",
                    "momentum residual p95",
                    "response p99 µs",
                ],
                table_rows,
            ),
            "",
            f"Exact spatial moment changes retained coverage **{force['retained']['sample_coverage'] * 100.0:.3f}%→{spatial['retained']['sample_coverage'] * 100.0:.3f}%** and fresh coverage **{('—' if force_fresh_coverage is None else f'{force_fresh_coverage * 100.0:.3f}%')}→{('—' if spatial_fresh_coverage is None else f'{spatial_fresh_coverage * 100.0:.3f}%')}**. Contact-moment magnitude is **{metrics['contact_moment_about_prospective_nms']['p95']:.6f} N·m·s p95 / {metrics['contact_moment_about_prospective_nms']['maximum']:.6f} max**.",
            "",
            "## Contact-impulse conservation cross-check",
            "",
            *markdown_table(
                ["variant", "norm p50", "norm p95", "norm p99", "norm max"],
                [
                    [
                        name,
                        f"{contact_reconstruction[name]['norm']['p50']:.6g}",
                        f"{contact_reconstruction[name]['norm']['p95']:.6g}",
                        f"{contact_reconstruction[name]['norm']['p99']:.6g}",
                        f"{contact_reconstruction[name]['norm']['maximum']:.6g}",
                    ]
                    for name in VARIANTS
                ],
            ),
            "",
            f"The independent `MΔv` projection costs **{metrics['contact_projection_timing_ns']['p99'] / 1_000.0:.3f} µs p99** with zero Rust allocation. Residual here measures pre-state Jacobian/mass reconstruction against substep-integrated plant constraint impulse; it is not the broader observed-motion residual below.",
            "",
            f"Spatial response p99 is **{metrics['spatial_response_timing_ns']['p99'] / 1_000.0:.3f} µs** and generalized-momentum residual p99 is **{metrics['momentum_residual_timing_ns']['p99'] / 1_000.0:.3f} µs**, both zero-allocation. The raw mixed-unit spatial Delassus condition is **{metrics['spatial_delassus_condition']['p95']:.3g} p95**; it is scale-dependent diagnostic evidence, not a coordinate-invariant physical condition number.",
            "",
            "## Decision",
            "",
            "The spatial response and momentum-residual mechanisms may be retained, but exact completed wrench is a simulator label and this decomposition is not a reachable tube. Authority still requires causal bounded spatial-wrench witnesses, strict second-morphology/contact-law holdout, conservative residual calibration, consequence non-regression, deadline evidence and hardware realization.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-spatial-contact-wrench-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_SPATIAL_CONTACT_WRENCH_AUDIT.md").write_text(report)
    np.savez_compressed(
        destination / "upkie-spatial-contact-wrench-replay.npz",
        **{name: np.asarray(values) for name, values in replay.items()},
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "force_retained_coverage": force["retained"]["sample_coverage"],
                "spatial_retained_coverage": spatial["retained"]["sample_coverage"],
                "spatial_fresh_coverage": spatial_fresh_coverage,
                "spatial_response_p99_us": metrics["spatial_response_timing_ns"]["p99"] / 1_000.0,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
