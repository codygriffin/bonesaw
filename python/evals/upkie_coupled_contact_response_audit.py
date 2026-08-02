#!/usr/bin/env python3
"""R205 coupled Delassus contact-response audit and frozen replay export."""

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
from upkie_directional_contact_transition_audit import (
    DIRECTIONAL_PROFILES,
    PRIMARY_PROFILE,
    STRUCTURED_ACCELERATION_RESERVE,
    aggregate_bounds,
    build_directional_witnesses,
)
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-coupled-contact-response-audit-r207"
DURATION_S = 6.0
SWEEP_COUNTS = (1, 2, 4, 8, 16)
PRIMARY_SWEEPS = 16
GROUPS = {
    "root_angular_rad_s": slice(0, 3),
    "root_linear_m_s": slice(3, 6),
    "joint_rad_s": slice(6, 12),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_COUPLED_CONTACT_RESPONSE_AUDIT_R207.html"
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    parser.add_argument("--plant-profiles", help="comma-separated smoke subset")
    return parser.parse_args()


def score_envelope(actual: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> dict[str, Any]:
    exceedance = np.maximum(np.maximum(lower - actual, actual - upper), 0.0)
    covered = exceedance <= 1.0e-12
    width = upper - lower
    result: dict[str, Any] = {
        "sample_count": int(len(actual)),
        "sample_coverage": float(np.mean(np.all(covered, axis=1))),
        "component_coverage": float(np.mean(covered)),
        "maximum_exceedance": float(np.max(exceedance)),
        "exceedance": distribution(exceedance.reshape(-1)),
    }
    for name, section in GROUPS.items():
        result[f"{name}_width"] = distribution(width[:, section].reshape(-1))
        result[f"{name}_exceedance"] = distribution(exceedance[:, section].reshape(-1))
    return result


def fit_loco_residual(
    cases: np.ndarray,
    fresh: np.ndarray,
    actual: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    expanded_lower = lower.copy()
    expanded_upper = upper.copy()
    retained = ~fresh
    for case in np.unique(cases):
        test = cases == case
        train = retained if case == FRESH_HOLDOUT_NAME else retained & ~test
        if not np.any(train):
            continue
        lower_residual = np.minimum(actual[train] - lower[train], 0.0).min(axis=0)
        upper_residual = np.maximum(actual[train] - upper[train], 0.0).max(axis=0)
        expanded_lower[test] += lower_residual
        expanded_upper[test] += upper_residual
    return expanded_lower, expanded_upper


def impulse_score(actual: np.ndarray, predicted: np.ndarray) -> dict[str, Any]:
    error = predicted - actual
    return {
        "rmse_ns": float(np.sqrt(np.mean(np.square(error)))),
        "maximum_abs_error_ns": float(np.max(np.abs(error))),
        "absolute_error_ns": distribution(np.abs(error).reshape(-1)),
        "normal_absolute_error_ns": distribution(np.abs(error[:, :, 2]).reshape(-1)),
        "tangent_absolute_error_ns": distribution(np.abs(error[:, :, :2]).reshape(-1)),
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
    bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)
    response = np.empty((12, 2, 3), np.float64)
    effective_mass = np.empty((2, 3), np.float64)
    delassus = np.empty((6, 6), np.float64)
    impulse_upper = np.empty((2, 3), np.float64)
    directional_lower = np.empty(12, np.float64)
    directional_upper = np.empty(12, np.float64)
    coupled_impulse = np.empty((2, 3), np.float64)
    contact_velocity_after = np.empty((2, 3), np.float64)

    replay: dict[str, list[Any]] = {
        "case": [],
        "plant_profile": [],
        "tick": [],
        "fresh_holdout": [],
        "friction": [],
        "candidate_count": [],
        "candidates": [],
        "response": [],
        "effective_mass": [],
        "delassus": [],
        "prospective_velocity": [],
        "actual_impulse": [],
        "actual_delta": [],
        "impulse_upper": [],
        "directional_lower": [],
        "directional_upper": [],
        "model_timing_ns": [],
    }
    sweep_rows: dict[int, dict[str, list[np.ndarray] | list[int]]] = {
        sweeps: {"impulse": [], "lower": [], "upper": [], "after": [], "timing": []}
        for sweeps in SWEEP_COUNTS
    }
    run_metrics: dict[str, Any] = {}
    zero_rust_allocation = True

    for case_index, case in enumerate(selected, 1):
        run_metrics[case.name] = {}
        case_sample_count = 0
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
            case_sample_count += len(ticks)
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
            actual_impulse = np.asarray(
                trace["physical_wheel_contact_impulse_world_ns"], np.float64
            )

            for tick in ticks:
                action = int(actions[tick])
                available = hypothesis_available[tick, :, action]
                candidates = hypothesis_acceleration[tick, available, action]
                if len(candidates) == 0:
                    raise RuntimeError("selected action has no support hypothesis")
                model_timing = balance.model_contact_impulse_velocity_response_with_delassus(
                    root_position[tick],
                    root_quaternion[tick],
                    q[tick],
                    points[tick],
                    bases,
                    response,
                    effective_mass,
                    delassus,
                )
                zero_rust_allocation &= model_timing[1:] == (0, 0)
                profile = DIRECTIONAL_PROFILES[PRIMARY_PROFILE]
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
                    directional_lower,
                    directional_upper,
                )
                zero_rust_allocation &= directional[4:] == (0, 0)

                padded_candidates = np.full((4, 12), np.nan, np.float64)
                padded_candidates[: len(candidates)] = candidates
                replay["case"].append(case.name)
                replay["plant_profile"].append(plant_profile_name)
                replay["tick"].append(int(tick))
                replay["fresh_holdout"].append(case.name == FRESH_HOLDOUT_NAME)
                replay["friction"].append(case.friction)
                replay["candidate_count"].append(len(candidates))
                replay["candidates"].append(padded_candidates)
                replay["response"].append(response.copy())
                replay["effective_mass"].append(effective_mass.copy())
                replay["delassus"].append(delassus.copy())
                replay["prospective_velocity"].append(prospective_velocity[tick].copy())
                replay["actual_impulse"].append(actual_impulse[tick].copy())
                replay["actual_delta"].append(actual_delta[tick].copy())
                replay["impulse_upper"].append(directional[2])
                replay["directional_lower"].append(directional[0])
                replay["directional_upper"].append(directional[1])
                replay["model_timing_ns"].append(int(model_timing[0]))

                for sweeps in SWEEP_COUNTS:
                    timing = balance.solve_coupled_contact_impulse(
                        prospective_velocity[tick],
                        delassus,
                        directional[2],
                        np.full(2, case.friction, np.float64),
                        0.0,
                        0.0,
                        sweeps,
                        coupled_impulse,
                        contact_velocity_after,
                    )
                    zero_rust_allocation &= timing[1:] == (0, 0)
                    impulse_delta = np.einsum(
                        "dca,ca->d", response, coupled_impulse, optimize=False
                    )
                    centers = candidates * CONTROL_DT + impulse_delta
                    lower = centers.min(axis=0) - STRUCTURED_ACCELERATION_RESERVE * CONTROL_DT
                    upper = centers.max(axis=0) + STRUCTURED_ACCELERATION_RESERVE * CONTROL_DT
                    row = sweep_rows[sweeps]
                    row["impulse"].append(coupled_impulse.copy())
                    row["lower"].append(lower)
                    row["upper"].append(upper)
                    row["after"].append(contact_velocity_after.copy())
                    row["timing"].append(int(timing[0]))
            run_metrics[case.name][plant_profile_name] = run["metrics"]
        print(
            f"[{case_index:02d}/{len(selected)}] {case.name}: samples={case_sample_count}",
            flush=True,
        )

    arrays = {name: np.asarray(values) for name, values in replay.items()}
    actual_delta = arrays["actual_delta"]
    actual_impulse = arrays["actual_impulse"]
    cases = arrays["case"]
    fresh = arrays["fresh_holdout"]
    scores: dict[str, Any] = {}
    table_rows = []
    for sweeps in SWEEP_COUNTS:
        row = {name: np.asarray(values) for name, values in sweep_rows[sweeps].items()}
        raw = score_envelope(actual_delta, row["lower"], row["upper"])
        loco_lower, loco_upper = fit_loco_residual(
            cases, fresh, actual_delta, row["lower"], row["upper"]
        )
        loco = score_envelope(actual_delta, loco_lower, loco_upper)
        impulse = impulse_score(actual_impulse, row["impulse"])
        post_speed = np.linalg.norm(row["after"], axis=2)
        score = {
            "raw_envelope": raw,
            "loco_empirical_residual_envelope": loco,
            "impulse": impulse,
            "post_contact_speed_m_s": distribution(post_speed.reshape(-1)),
            "timing_ns": distribution(row["timing"].reshape(-1)),
        }
        scores[str(sweeps)] = score
        table_rows.append(
            [
                sweeps,
                f"{raw['sample_coverage'] * 100.0:.3f}%",
                f"{loco['sample_coverage'] * 100.0:.3f}%",
                f"{impulse['rmse_ns']:.5f}",
                f"{raw['maximum_exceedance']:.5f}",
                f"{loco['root_angular_rad_s_width']['p95']:.3f}",
                f"{loco['root_linear_m_s_width']['p95']:.3f}",
                f"{loco['joint_rad_s_width']['p95']:.3f}",
                f"{score['post_contact_speed_m_s']['p95']:.4f}",
                f"{score['timing_ns']['p99'] / 1_000.0:.3f}",
            ]
        )

    directional_score = score_envelope(
        actual_delta, arrays["directional_lower"], arrays["directional_upper"]
    )
    delassus = arrays["delassus"]
    cross = delassus[:, :3, 3:]
    diagonal_norm = np.hypot(
        np.linalg.norm(delassus[:, :3, :3], axis=(1, 2)),
        np.linalg.norm(delassus[:, 3:, 3:], axis=(1, 2)),
    )
    coupling_ratio = np.linalg.norm(cross, axis=(1, 2)) / np.maximum(
        diagonal_norm, 1.0e-18
    )
    minimum_eigenvalue = np.linalg.eigvalsh(delassus)[:, 0]
    condition = np.linalg.cond(delassus)
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
    mechanism_passed = bool(
        all_runs_finite
        and zero_plant_allocation
        and zero_rust_allocation
        and np.all(np.isfinite(delassus))
        and np.all(minimum_eigenvalue > 0.0)
    )
    primary = scores[str(PRIMARY_SWEEPS)]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "sample_count": int(len(actual_delta)),
        "sweep_counts": list(SWEEP_COUNTS),
        "primary_sweeps": PRIMARY_SWEEPS,
        "mechanism_passed": mechanism_passed,
        "authority_admitted": False,
        "zero_rust_allocation": zero_rust_allocation,
        "zero_plant_allocation_and_python_gc": zero_plant_allocation,
        "all_runs_finite": all_runs_finite,
        "directional_baseline": directional_score,
        "scores": scores,
        "delassus_cross_contact_frobenius_ratio": distribution(coupling_ratio),
        "delassus_minimum_eigenvalue": distribution(minimum_eigenvalue),
        "delassus_condition": distribution(condition),
        "model_timing_ns": distribution(arrays["model_timing_ns"]),
        "run_metrics": run_metrics,
    }
    report = "\n".join(
        [
            "# Bonesaw coupled contact-response audit · r207",
            "",
            f"> Coupled Rust mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · strict raw coverage **{'PASS' if primary['raw_envelope']['sample_coverage'] == 1.0 else 'FAIL'}** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- Rust now emits the complete symmetric two-wheel Delassus operator `W = J M⁻¹ Jᵀ`, not only six independent effective masses. A deterministic forward/reverse projected solve retains wheel-to-wheel coupling, nonnegative bounded normal impulse, directional passive caps, restitution, and a circular Coulomb section.",
            "- The coupled output is a point prediction and convergence witness, not an outer bound. The leave-one-named-case-out residual below is label-calibrated diagnostic evidence and is never fed into command authority.",
            "- MuJoCo is used once to freeze labels and all causal/model inputs. The saved NPZ supports later policy/physics-free sweep analysis through the Rust kernel.",
            "",
            "## Fixed-work sweep",
            "",
            *markdown_table(
                [
                    "sweeps",
                    "raw coverage",
                    "LOCO residual coverage",
                    "impulse RMSE N·s",
                    "raw max miss",
                    "LOCO root ω p95",
                    "LOCO root v p95",
                    "LOCO joint p95",
                    "post-contact speed p95",
                    "solve p99 µs",
                ],
                table_rows,
            ),
            "",
            f"The R204 directional outer tube covers **{directional_score['sample_coverage'] * 100.0:.3f}%** of these complete samples at p95 width **{directional_score['root_angular_rad_s_width']['p95']:.3f} / {directional_score['root_linear_m_s_width']['p95']:.3f} / {directional_score['joint_rad_s_width']['p95']:.3f}**. It remains the conservative comparison, not the coupled point solve.",
            "",
            f"Cross-wheel Delassus coupling ratio is **{metrics['delassus_cross_contact_frobenius_ratio']['p50']:.4f} p50 / {metrics['delassus_cross_contact_frobenius_ratio']['p95']:.4f} p95**. Minimum eigenvalue is **{metrics['delassus_minimum_eigenvalue']['minimum']:.6g}** and condition number is **{metrics['delassus_condition']['p95']:.3f} p95**.",
            "",
            "## Decision",
            "",
            "The full coupled response and fixed-work solve are admitted as model machinery only. Promotion requires a conservative coupled reachable set or independently calibrated root-momentum residual, strict fresh/second-morphology holdout, non-regressing terminal consequence, typed online contact/load witnesses, and the separate deadline/hardware gates.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-coupled-contact-response-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_COUPLED_CONTACT_RESPONSE_AUDIT.md").write_text(report)
    np.savez_compressed(
        destination / "upkie-coupled-contact-response-replay.npz",
        **arrays,
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "sample_count": len(actual_delta),
                "raw_coverage": primary["raw_envelope"]["sample_coverage"],
                "loco_coverage": primary["loco_empirical_residual_envelope"][
                    "sample_coverage"
                ],
                "impulse_rmse_ns": primary["impulse"]["rmse_ns"],
                "solve_p99_us": primary["timing_ns"]["p99"] / 1_000.0,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
