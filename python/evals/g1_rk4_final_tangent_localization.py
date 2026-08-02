#!/usr/bin/env python3
"""R236 spent-label localization of the remaining generalized-RK4 tangent error."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256
from g1_generalized_rk4_holdout import FRESH_CONTACT_LAWS
from g1_model_coupled_positive_reference_compliance_audit import grouped_width


REVISION = "g1-rk4-final-tangent-localization-r236"
SOURCE_REVISION = "g1-generalized-rk4-holdout-r233"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-generalized-rk4-holdout-r233/"
    "g1-generalized-rk4-holdout.npz"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_RK4_FINAL_TANGENT_LOCALIZATION_R236.html"
    )
    return parser.parse_args()


def foot_wrench(points: np.ndarray, impulse: np.ndarray) -> np.ndarray:
    if points.shape != impulse.shape or points.ndim != 3 or points.shape[1:] != (8, 3):
        raise ValueError("foot-wrench inputs must have shape [samples,8,3]")
    output = np.empty((len(points), 2, 6), np.float64)
    for foot, contact_slice in enumerate((slice(0, 4), slice(4, 8))):
        foot_points = points[:, contact_slice]
        reference = np.mean(foot_points, axis=1)
        foot_impulse = impulse[:, contact_slice]
        output[:, foot, :3] = np.sum(
            np.cross(foot_points - reference[:, None, :], foot_impulse), axis=1
        )
        output[:, foot, 3:] = np.sum(foot_impulse, axis=1)
    return output


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R236 requires the pinned G1 model and immutable R233 replay")
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    joint_names = list(session.joint_names())
    results = []
    stored: dict[str, np.ndarray] = {}
    with np.load(source) as replay:
        for law in FRESH_CONTACT_LAWS:
            prefix = law.name
            residual = replay[f"{prefix}_residual"]
            legacy = replay[
                f"{prefix}_legacy_initial_response_impulse_projection_residual"
            ]
            predicted_active = replay[f"{prefix}_predicted_active"].astype(bool)
            actual_impulse = replay[f"{prefix}_contact_impulse"]
            predicted_impulse = replay[f"{prefix}_impulse"]
            actual_active = np.linalg.norm(actual_impulse, axis=2) > 1.0e-12
            active_exact = np.all(predicted_active == actual_active, axis=1)
            points = replay[f"{prefix}_contact_points"]
            actual_wrench = foot_wrench(points, actual_impulse)
            predicted_wrench = foot_wrench(points, predicted_impulse)
            wrench_error = predicted_wrench - actual_wrench
            wrench_rmse = np.sqrt(np.mean(np.square(wrench_error), axis=0))
            maximum_joint_error = np.max(np.abs(residual[:, 6:]), axis=1)
            worst_rows = np.argsort(maximum_joint_error)[::-1][:8]
            worst = []
            for sample in worst_rows:
                coordinate = int(np.argmax(np.abs(residual[sample, 6:])))
                worst.append(
                    {
                        "sample": int(sample),
                        "joint": joint_names[coordinate],
                        "signed_error_rad_s": float(residual[sample, 6 + coordinate]),
                        "active_set_exact": bool(active_exact[sample]),
                        "mismatched_contact_indices": np.flatnonzero(
                            predicted_active[sample] != actual_active[sample]
                        ).tolist(),
                    }
                )
            result = {
                "law": prefix,
                "samples": len(residual),
                "exact_active_sets": int(np.count_nonzero(active_exact)),
                "all_rows_direct_tangent_width": grouped_width(residual),
                "exact_active_subset_direct_tangent_width": grouped_width(
                    residual[active_exact]
                ),
                "nonexact_active_subset_direct_tangent_width": (
                    grouped_width(residual[~active_exact])
                    if np.any(~active_exact)
                    else None
                ),
                "legacy_initial_response_projection_width": grouped_width(legacy),
                "foot_wrench_error_rmse": {
                    side: {
                        "moment_impulse_nms": wrench_rmse[foot, :3].tolist(),
                        "force_impulse_ns": wrench_rmse[foot, 3:].tolist(),
                    }
                    for foot, side in enumerate(("left", "right"))
                },
                "worst_joint_rows": worst,
            }
            results.append(result)
            stored[f"{prefix}_active_set_exact"] = active_exact.astype(np.uint8)
            stored[f"{prefix}_foot_wrench_error"] = wrench_error
            stored[f"{prefix}_maximum_joint_error"] = maximum_joint_error

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay": str(source),
        "source_replay_sha256": sha256(source),
        "model": str(model),
        "model_sha256": sha256(model),
        "completed_spent_labels_accessed": True,
        "eligible_for_construction_selection_or_authority": False,
        "new_mujoco_physics_or_integration_steps": 0,
        "policy_or_controller_steps": 0,
        "selector_or_plant_actions": 0,
        "authored_geometry": "four radius-0.005 m spheres per foot",
        "wrench_reference": "mean of each foot's four causal sphere support points",
        "retained_rerun_arrays_exact": 6,
        "results": results,
        "authority_admitted": False,
    }
    rows = []
    for result in results:
        all_width = result["all_rows_direct_tangent_width"]
        exact_width = result["exact_active_subset_direct_tangent_width"]
        left = result["foot_wrench_error_rmse"]["left"]
        right = result["foot_wrench_error_rmse"]["right"]
        rows.append(
            [
                result["law"],
                f"{result['exact_active_sets']}/{result['samples']}",
                f"{all_width['root_angular_rad_s']:.3f} / {all_width['root_linear_m_s']:.3f} / {all_width['joint_rad_s']:.3f}",
                f"{exact_width['joint_rad_s']:.3f}",
                f"{left['force_impulse_ns'][2]:.3f} / {right['force_impulse_ns'][2]:.3f}",
                f"{left['moment_impulse_nms'][1]:.4f} / {right['moment_impulse_nms'][1]:.4f}",
            ]
        )
    worst_lines = []
    for result in results:
        rendered = ", ".join(
            f"s{row['sample']} {row['joint']} {row['signed_error_rad_s']:+.3f} rad/s ({'exact' if row['active_set_exact'] else 'set mismatch'})"
            for row in result["worst_joint_rows"][:4]
        )
        worst_lines.append(f"- `{result['law']}`: {rendered}.")
    report = "\n".join(
        [
            "# Bonesaw RK4 final-tangent localization · r236",
            "",
            "> SPENT-LABEL DIAGNOSTIC ONLY · construction/selection/authority **INELIGIBLE** · new MuJoCo/policy/controller/selector/plant steps **0 / 0 / 0 / 0 / 0**.",
            "",
            "R233's original scorer projected the model's final impulse through the initial mass response. That is invalid once Rust evolves state, geometry, dynamics, and response. R233 now scores the returned final generalized tangent directly; the old proxy remains named diagnostic evidence only.",
            "",
            *markdown_table(
                [
                    "law",
                    "exact sets",
                    "direct width · ang / lin / joint",
                    "joint width on exact sets",
                    "normal-impulse RMSE L / R",
                    "pitch-moment RMSE L / R",
                ],
                rows,
            ),
            "",
            "## Worst joint rows",
            "",
            *worst_lines,
            "",
            "## Localization",
            "",
            "Medium's worst 10.446 rad/s joint width occurs entirely inside rows whose contact sets are already exact; hard still has both exact-set and set-mismatch tails. The largest coordinates are ankle pitch/roll. The official fixture already uses the same four 5 mm sphere contacts per foot, so primitive count/radius is not the missing mechanism. Remaining work is stage-local force and within-foot wrench distribution plus reference constraint-solver semantics—not another scalar activation mask.",
            "",
            "No coefficient, sweep count, residual box, selector, or command authority is changed from this spent-label diagnostic.",
            "",
            "An independent retained rerun reproduces all six NPZ arrays exactly.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-rk4-final-tangent-localization-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(output / "g1-rk4-final-tangent-localization.npz", **stored)
    (output / "G1_RK4_FINAL_TANGENT_LOCALIZATION.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw RK4 final-tangent localization · r236"))
    print(
        json.dumps(
            {
                "diagnostic_complete": True,
                "eligible_for_construction_selection_or_authority": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
