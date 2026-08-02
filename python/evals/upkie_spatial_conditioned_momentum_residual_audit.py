#!/usr/bin/env python3
"""R210 spatial-wrench-conditioned generalized-momentum residual audit."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_transition_acceleration_interval_audit import FRESH_HOLDOUT_NAME
from upkie_contact_transition_response_audit import score_profile
from upkie_directional_contact_transition_audit import STRUCTURED_ACCELERATION_RESERVE
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-spatial-conditioned-momentum-residual-audit-r210"
VARIANTS = (
    "spatial_wrench_without_momentum_box",
    "loco_coordinate_momentum_box",
    "loco_group_symmetric_momentum_box",
)
R204_P95_WIDTH_CEILING = {
    "root_angular_rad_s": 9.687,
    "root_linear_m_s": 0.991,
    "joint_rad_s": 58.020,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument(
        "--replay",
        default=(
            "benchmarks/results/upkie-spatial-contact-wrench-audit-r206/"
            "upkie-spatial-contact-wrench-replay.npz"
        ),
    )
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_SPATIAL_CONDITIONED_MOMENTUM_RESIDUAL_AUDIT_R210.html",
    )
    return parser.parse_args()


def spatial_delta_velocity(
    response: np.ndarray,
    moment_nms: np.ndarray,
    impulse_ns: np.ndarray,
) -> np.ndarray:
    wrench = np.concatenate((moment_nms, impulse_ns), axis=1)
    return np.einsum("dcx,cx->d", response, wrench, optimize=False)


def calibrate_loco_boxes(
    cases: np.ndarray,
    selected_residual: np.ndarray,
) -> dict[str, dict[str, tuple[np.ndarray, np.ndarray]]]:
    """Fit boxes without ever reading the named evaluation fold."""
    boxes: dict[str, dict[str, tuple[np.ndarray, np.ndarray]]] = {}
    for fold in np.unique(cases):
        training = selected_residual[cases != fold]
        if len(training) == 0:
            raise ValueError("LOCO calibration requires at least two named cases")
        coordinate_lower = np.minimum(np.min(training, axis=0), 0.0)
        coordinate_upper = np.maximum(np.max(training, axis=0), 0.0)
        group_radius = np.asarray(
            [
                np.max(np.abs(training[:, :3])),
                np.max(np.abs(training[:, 3:6])),
                np.max(np.abs(training[:, 6:])),
            ],
            np.float64,
        )
        grouped_upper = np.concatenate(
            (
                np.full(3, group_radius[0]),
                np.full(3, group_radius[1]),
                np.full(selected_residual.shape[1] - 6, group_radius[2]),
            )
        )
        boxes[str(fold)] = {
            "coordinate": (coordinate_lower, coordinate_upper),
            "group_symmetric": (-grouped_upper, grouped_upper),
        }
    return boxes


def record_sample(
    *,
    case: str,
    plant_profile: str,
    tick: int,
    candidate_count: int,
    actual: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    timing: tuple[int, int, int],
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
        "case": case,
        "fresh_holdout": case == FRESH_HOLDOUT_NAME,
        "plant_profile": plant_profile,
        "tick": tick,
        "selected_action": 0,
        "candidate_count": candidate_count,
        "covered": bool(np.all(component_covered)),
        "component_covered": component_covered.tolist(),
        "component_exceedance": exceedance.tolist(),
        "interval_width": width.tolist(),
        "interval_utilization": utilization.tolist(),
        "actual_delta": actual.tolist(),
        "lower": lower.tolist(),
        "upper": upper.tolist(),
        "impulse_covered": True,
        "model_timing_ns": int(timing[0]),
        "model_allocation_calls": int(timing[1]),
        "model_allocated_bytes": int(timing[2]),
        "bound_timing_ns": 0,
        "bound_allocation_calls": 0,
        "bound_allocated_bytes": 0,
    }


def split_score(samples: list[dict[str, Any]]) -> dict[str, Any]:
    retained = [sample for sample in samples if not sample["fresh_holdout"]]
    fresh = [sample for sample in samples if sample["fresh_holdout"]]
    return {
        "all": score_profile(samples),
        "retained": score_profile(retained) if retained else None,
        "fresh_holdout": score_profile(fresh) if fresh else None,
    }


def main() -> int:
    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    replay_path = pathlib.Path(args.replay).resolve()
    if not replay_path.is_file():
        raise SystemExit(f"missing R206 replay: {replay_path}")
    replay = np.load(replay_path, allow_pickle=False)
    required = {
        "case",
        "plant_profile",
        "tick",
        "candidate_count",
        "candidates",
        "root_position",
        "root_quaternion_wxyz",
        "q",
        "impulse_world_ns",
        "moment_about_prospective_nms",
        "actual_delta",
        "spatial_response",
    }
    missing = required - set(replay.files)
    if missing:
        raise SystemExit(f"R206 replay is missing fields: {sorted(missing)}")

    cases = np.asarray(replay["case"])
    plant_profiles = np.asarray(replay["plant_profile"])
    ticks = np.asarray(replay["tick"], np.int64)
    candidate_count = np.asarray(replay["candidate_count"], np.int64)
    candidates = np.asarray(replay["candidates"], np.float64)
    root_position = np.asarray(replay["root_position"], np.float64)
    root_quaternion = np.asarray(replay["root_quaternion_wxyz"], np.float64)
    q = np.asarray(replay["q"], np.float64)
    impulse = np.asarray(replay["impulse_world_ns"], np.float64)
    moment = np.asarray(replay["moment_about_prospective_nms"], np.float64)
    actual = np.asarray(replay["actual_delta"], np.float64)
    response = np.asarray(replay["spatial_response"], np.float64)
    sample_count = len(cases)
    if sample_count == 0 or any(
        len(array) != sample_count
        for array in (
            plant_profiles,
            ticks,
            candidate_count,
            candidates,
            root_position,
            root_quaternion,
            q,
            impulse,
            moment,
            actual,
            response,
        )
    ):
        raise SystemExit("R206 replay arrays have inconsistent sample dimensions")

    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model))
    maximum_candidates = candidates.shape[1]
    residual_scratch = np.empty((maximum_candidates, 12), np.float64)
    selected_residual = np.empty((sample_count, 12), np.float64)
    spatial_delta = np.empty((sample_count, 12), np.float64)
    selected_candidate_index = np.empty(sample_count, np.uint8)
    residual_timing: list[int] = []
    zero_rust_allocation = True
    for index in range(sample_count):
        count = int(candidate_count[index])
        if count <= 0 or count > maximum_candidates:
            raise SystemExit(f"invalid candidate count at replay row {index}: {count}")
        spatial_delta[index] = spatial_delta_velocity(
            response[index], moment[index], impulse[index]
        )
        predicted = candidates[index, :count] * CONTROL_DT + spatial_delta[index]
        timing = balance.model_generalized_momentum_impulse_residuals(
            root_position[index],
            root_quaternion[index],
            q[index],
            actual[index],
            predicted,
            residual_scratch[:count],
        )
        zero_rust_allocation &= timing[1:] == (0, 0)
        residual_timing.append(int(timing[0]))
        norm_squared = np.einsum(
            "ij,ij->i", residual_scratch[:count], residual_scratch[:count]
        )
        selected = int(np.argmin(norm_squared))
        selected_candidate_index[index] = selected
        selected_residual[index] = residual_scratch[selected]

    boxes = calibrate_loco_boxes(cases, selected_residual)
    samples: dict[str, list[dict[str, Any]]] = {name: [] for name in VARIANTS}
    velocity_lower = np.empty(12, np.float64)
    velocity_upper = np.empty(12, np.float64)
    reserve = STRUCTURED_ACCELERATION_RESERVE * CONTROL_DT
    projection_timing: dict[str, list[int]] = {name: [] for name in VARIANTS}
    for index in range(sample_count):
        count = int(candidate_count[index])
        base = candidates[index, :count] * CONTROL_DT + spatial_delta[index]
        base_lower = np.min(base, axis=0) - reserve
        base_upper = np.max(base, axis=0) + reserve
        fold_boxes = boxes[str(cases[index])]
        for name in VARIANTS:
            timing = (0, 0, 0)
            if name == "spatial_wrench_without_momentum_box":
                lower = base_lower
                upper = base_upper
            else:
                scheme = (
                    "coordinate"
                    if name == "loco_coordinate_momentum_box"
                    else "group_symmetric"
                )
                momentum_lower, momentum_upper = fold_boxes[scheme]
                timing = balance.model_generalized_velocity_interval_from_momentum_box(
                    root_position[index],
                    root_quaternion[index],
                    q[index],
                    momentum_lower,
                    momentum_upper,
                    velocity_lower,
                    velocity_upper,
                )
                zero_rust_allocation &= timing[1:] == (0, 0)
                lower = base_lower + velocity_lower
                upper = base_upper + velocity_upper
            projection_timing[name].append(int(timing[0]))
            samples[name].append(
                record_sample(
                    case=str(cases[index]),
                    plant_profile=str(plant_profiles[index]),
                    tick=int(ticks[index]),
                    candidate_count=count,
                    actual=actual[index],
                    lower=lower,
                    upper=upper,
                    timing=timing,
                )
            )

    results = {name: split_score(rows) for name, rows in samples.items()}
    box_metrics: dict[str, Any] = {}
    for fold, schemes in boxes.items():
        box_metrics[fold] = {}
        for scheme, (lower, upper) in schemes.items():
            box_metrics[fold][scheme] = {
                "lower": lower.tolist(),
                "upper": upper.tolist(),
                "width": (upper - lower).tolist(),
                "maximum_abs_root_angular_nms": float(
                    np.max(np.maximum(np.abs(lower[:3]), np.abs(upper[:3])))
                ),
                "maximum_abs_root_linear_ns": float(
                    np.max(np.maximum(np.abs(lower[3:6]), np.abs(upper[3:6])))
                ),
                "maximum_abs_joint_nms": float(
                    np.max(np.maximum(np.abs(lower[6:]), np.abs(upper[6:])))
                ),
            }
    table_rows = [
        [
            name,
            result["all"]["sample_count"],
            f"{result['retained']['sample_coverage'] * 100.0:.3f}%",
            f"{result['fresh_holdout']['sample_coverage'] * 100.0:.3f}%",
            f"{result['all']['maximum_component_exceedance_per_s']:.6f}",
            f"{result['all']['root_angular_width_rad_s']['p95']:.3f}",
            f"{result['all']['root_linear_width_m_s']['p95']:.3f}",
            f"{result['all']['joint_width_rad_s']['p95']:.3f}",
            f"{result['all']['model_response_timing_ns']['p99'] / 1_000.0:.3f}",
        ]
        for name, result in results.items()
    ]
    coordinate = results["loco_coordinate_momentum_box"]
    grouped = results["loco_group_symmetric_momentum_box"]
    def preserves_r204_width(result: dict[str, Any]) -> bool:
        scored = result["all"]
        return (
            scored["root_angular_width_rad_s"]["p95"]
            <= R204_P95_WIDTH_CEILING["root_angular_rad_s"]
            and scored["root_linear_width_m_s"]["p95"]
            <= R204_P95_WIDTH_CEILING["root_linear_m_s"]
            and scored["joint_width_rad_s"]["p95"]
            <= R204_P95_WIDTH_CEILING["joint_rad_s"]
        )

    coordinate_strict = (
        coordinate["retained"]["sample_coverage"] == 1.0
        and coordinate["fresh_holdout"]["sample_coverage"] == 1.0
    )
    grouped_strict = (
        grouped["retained"]["sample_coverage"] == 1.0
        and grouped["fresh_holdout"]["sample_coverage"] == 1.0
    )
    coordinate_width_gate = preserves_r204_width(coordinate)
    grouped_width_gate = preserves_r204_width(grouped)
    mechanism_passed = zero_rust_allocation and all(
        np.all(np.isfinite(array))
        for array in (selected_residual, spatial_delta, velocity_lower, velocity_upper)
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "source_replay": str(replay_path),
        "sample_count": sample_count,
        "named_case_count": len(np.unique(cases)),
        "zero_physics_steps": True,
        "zero_policy_or_controller_steps": True,
        "mechanism_passed": mechanism_passed,
        "authority_admitted": False,
        "zero_rust_allocation": zero_rust_allocation,
        "calibration_uses_evaluation_fold": False,
        "selected_residual_candidate_is_label_ranked": True,
        "r204_p95_width_ceiling": R204_P95_WIDTH_CEILING,
        "coordinate_strict_coverage": coordinate_strict,
        "coordinate_preserves_r204_width": coordinate_width_gate,
        "grouped_strict_coverage": grouped_strict,
        "grouped_preserves_r204_width": grouped_width_gate,
        "profile_promoted": False,
        "results": results,
        "loco_momentum_boxes": box_metrics,
        "selected_momentum_residual_norm": distribution(
            np.linalg.norm(selected_residual, axis=1)
        ),
        "residual_query_timing_ns": distribution(residual_timing),
        "projection_timing_ns": {
            name: distribution(values) for name, values in projection_timing.items()
        },
    }
    report = "\n".join(
        [
            "# Bonesaw spatial-conditioned momentum-residual audit · r210",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · coordinate LOCO strict coverage **{'PASS' if coordinate_strict else 'FAIL'}** / R204 width **{'PASS' if coordinate_width_gate else 'FAIL'}** · grouped LOCO strict coverage **{'PASS' if grouped_strict else 'FAIL'}** / R204 width **{'PASS' if grouped_width_gate else 'FAIL'}** · profile **NOT PROMOTED** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- This audit replays immutable R206 arrays: **zero MuJoCo steps and zero policy/controller steps**. Unlike R209's unconditioned tube, R210 first subtracts the exact completed spatial-wrench response, then asks whether the remaining momentum residual can be both strict and useful. Rust computes the residuals and maps held-out boxes through the full model inverse mass.",
            "- For each calibration sample, the lowest-Euclidean-norm residual among the already declared support hypotheses is retained. That candidate ranking uses completed labels and is disclosed; it is not an online selector.",
            "- Every named case is evaluated with a box calibrated from all other named cases. The held-out fold is never read during calibration. The coordinate box retains signed per-coordinate extrema and zero; the grouped box uses one symmetric radius for root moment, root linear impulse and joint impulse respectively.",
            "- The base response uses the exact completed R206 spatial wrench, so this is an optimistic conditional decomposition. Even strict residual coverage would validate only the residual mechanism—not causal contact authority. R203's independent structured acceleration reserve remains unchanged.",
            "",
            "## Leave-one-named-case-out result",
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
                    "projection p99 µs",
                ],
                table_rows,
            ),
            "",
            f"Usefulness is conjunctive: strict retained/fresh coverage plus p95 width no worse than the R204 directional diagnostic (**{R204_P95_WIDTH_CEILING['root_angular_rad_s']:.3f} rad/s root angular, {R204_P95_WIDTH_CEILING['root_linear_m_s']:.3f} m/s root linear, {R204_P95_WIDTH_CEILING['joint_rad_s']:.3f} rad/s joints**). The coordinate box fails strict coverage and both root-width ceilings; the grouped box closes coverage only by failing every width ceiling.",
            "",
            f"The label-ranked momentum residual norm is **{metrics['selected_momentum_residual_norm']['p95']:.6f} p95 / {metrics['selected_momentum_residual_norm']['maximum']:.6f} max**. Rust residual-query p99 is **{metrics['residual_query_timing_ns']['p99'] / 1_000.0:.3f} µs**; coordinate/group box projection p99 is **{metrics['projection_timing_ns']['loco_coordinate_momentum_box']['p99'] / 1_000.0:.3f}/{metrics['projection_timing_ns']['loco_group_symmetric_momentum_box']['p99'] / 1_000.0:.3f} µs**, all zero-allocation.",
            "",
            "## Decision",
            "",
            "Even after granting an oracle-quality completed spatial wrench, neither LOCO residual profile is useful: one is narrow enough only in joints and still misses; the other obtains strict coverage by becoming substantially wider than R204. No profile is promoted. The next calibration must condition on causal contact phase/load/slip without reading the evaluation label, freeze before a genuinely new morphology/contact law, and pair with a causal spatial-wrench set. Any exact wrench or label-ranked candidate remains evaluation-only.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-spatial-conditioned-momentum-residual-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_SPATIAL_CONDITIONED_MOMENTUM_RESIDUAL_AUDIT.md").write_text(
        report
    )
    np.savez_compressed(
        destination / "upkie-spatial-conditioned-momentum-residual-replay.npz",
        case=cases,
        plant_profile=plant_profiles,
        tick=ticks,
        selected_candidate_index=selected_candidate_index,
        selected_momentum_residual=selected_residual,
        spatial_delta_velocity=spatial_delta,
        actual_delta=actual,
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "coordinate_retained_coverage": coordinate["retained"][
                    "sample_coverage"
                ],
                "coordinate_fresh_coverage": coordinate["fresh_holdout"][
                    "sample_coverage"
                ],
                "grouped_retained_coverage": grouped["retained"]["sample_coverage"],
                "grouped_fresh_coverage": grouped["fresh_holdout"][
                    "sample_coverage"
                ],
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
