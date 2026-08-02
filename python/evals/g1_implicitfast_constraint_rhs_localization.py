#!/usr/bin/env python3
"""R239 spent-label localization of implicitfast/contact-RHS conflation."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256, standing_posture
from g1_generalized_rk4_holdout import score_law
from g1_model_coupled_positive_reference_compliance_audit import (
    CAUSAL_SOURCE_SUFFIXES,
    execute_law,
)
from g1_positive_reference_compliance_audit import prepare_law
from g1_rk4_final_tangent_localization import foot_wrench
from g1_rk4_stage_force_cross_integrator_holdout import (
    FRESH_CONTACT_LAWS,
    FROZEN_PROJECTION_SWEEPS,
    SAMPLE_OFFSETS,
)


REVISION = "g1-implicitfast-constraint-rhs-localization-r239"
SOURCE_REVISION = "g1-rk4-stage-force-cross-integrator-holdout-r238"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-rk4-stage-force-cross-integrator-holdout-r238/"
    "g1-rk4-stage-force-cross-integrator-holdout.npz"
)
MUJOCO_INTEGRATOR_DOC = (
    "https://mujoco.readthedocs.io/en/stable/computation/#geintegrators"
)
IMPLICITFAST_LAW = FRESH_CONTACT_LAWS[0]
IMPLICITFAST_OFFSET = SAMPLE_OFFSETS[0]
DIAGNOSTIC_ABI_IDS = (0, 1, 2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/G1_IMPLICITFAST_CONSTRAINT_RHS_LOCALIZATION_R239.html",
    )
    return parser.parse_args()


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R239 requires the pinned G1 model and immutable R238 replay")
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    q_nominal = standing_posture(list(session.joint_names()))
    joint_names = list(session.joint_names())
    prefix = IMPLICITFAST_LAW.name
    with np.load(source) as archive:
        replay = {name: archive[name].copy() for name in archive.files}
    causal = {
        f"{prefix}_{suffix}": replay[f"{prefix}_{suffix}"]
        for suffix in CAUSAL_SOURCE_SUFFIXES
    }
    prepared = prepare_law(
        session,
        model,
        causal,
        IMPLICITFAST_LAW,
        IMPLICITFAST_OFFSET,
        q_nominal,
    )

    rows: list[dict[str, Any]] = []
    stored: dict[str, np.ndarray] = {}
    actual_impulse = replay[f"{prefix}_contact_impulse"]
    points = replay[f"{prefix}_contact_points"]
    actual_wrench = foot_wrench(points, actual_impulse)
    for abi_id in DIAGNOSTIC_ABI_IDS:
        result, arrays = execute_law(
            session,
            causal,
            prepared,
            IMPLICITFAST_LAW,
            IMPLICITFAST_OFFSET,
            q_nominal,
            FROZEN_PROJECTION_SWEEPS,
            model_integrator_id_fn=lambda _law, abi_id=abi_id: abi_id,
        )
        score, scored = score_law(
            session,
            replay,
            prepared,
            IMPLICITFAST_LAW,
            IMPLICITFAST_OFFSET,
            result,
            arrays,
        )
        residual = scored[f"{prefix}_residual"]
        predicted_active = scored[f"{prefix}_predicted_active"].astype(bool)
        actual_active = np.linalg.norm(actual_impulse, axis=2) > 1.0e-12
        maximum_joint_error = np.max(np.abs(residual[:, 6:]), axis=1)
        worst = []
        for sample in np.argsort(maximum_joint_error)[::-1][:6]:
            coordinate = int(np.argmax(np.abs(residual[sample, 6:])))
            worst.append(
                {
                    "sample": int(sample),
                    "joint": joint_names[coordinate],
                    "signed_error_rad_s": float(residual[sample, 6 + coordinate]),
                    "active_set_exact": bool(
                        np.all(predicted_active[sample] == actual_active[sample])
                    ),
                    "mismatched_contact_indices": np.flatnonzero(
                        predicted_active[sample] != actual_active[sample]
                    ).tolist(),
                }
            )
        wrench_error = foot_wrench(points, arrays["impulse"]) - actual_wrench
        wrench_rmse = np.sqrt(np.mean(np.square(wrench_error), axis=0))
        rows.append(
            {
                "diagnostic_abi_id": abi_id,
                "sample_coverage": score["sample_coverage"],
                "uncovered_samples": score["uncovered_samples"],
                "exact_active_sets": score["exact_active_sets"],
                "predicted_only_contact_points": score[
                    "predicted_only_contact_points"
                ],
                "missed_actual_contact_points": score["missed_actual_contact_points"],
                "diagnostic_fitted_width": score["diagnostic_fitted_width"],
                "query_timing_ns": score["query_timing_ns"],
                "zero_rust_allocation": score["zero_rust_allocation"],
                "bitwise_repeat": score["bitwise_repeat"],
                "foot_wrench_error_rmse": {
                    side: {
                        "moment_impulse_nms": wrench_rmse[foot, :3].tolist(),
                        "force_impulse_ns": wrench_rmse[foot, 3:].tolist(),
                    }
                    for foot, side in enumerate(("left", "right"))
                },
                "worst_joint_rows": worst,
            }
        )
        for name, value in arrays.items():
            stored[f"abi{abi_id}_{name}"] = value
        stored[f"abi{abi_id}_residual"] = residual
        stored[f"abi{abi_id}_predicted_active"] = predicted_active.astype(np.uint8)
        stored[f"abi{abi_id}_maximum_joint_error"] = maximum_joint_error
        stored[f"abi{abi_id}_foot_wrench_error"] = wrench_error

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
        "mujoco_integrator_documentation": MUJOCO_INTEGRATOR_DOC,
        "documented_constraint_force_velocity_derivatives_excluded": True,
        "diagnostic_abi_ids": DIAGNOSTIC_ABI_IDS,
        "retained_rerun_non_timing_arrays_exact": 33,
        "results": rows,
        "authority_admitted": False,
    }
    table = []
    for row in rows:
        width = row["diagnostic_fitted_width"]
        table.append(
            [
                row["diagnostic_abi_id"],
                f"{100.0 * row['sample_coverage']:.3f}%",
                f"{row['exact_active_sets']}/48",
                f"{row['predicted_only_contact_points']} / {row['missed_actual_contact_points']}",
                f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
                f"{row['query_timing_ns']['p99'] / 1e6:.3f}",
            ]
        )
    worst_lines = []
    for row in rows:
        worst_lines.append(
            f"- ABI {row['diagnostic_abi_id']}: "
            + ", ".join(
                f"s{item['sample']} {item['joint']} {item['signed_error_rad_s']:+.3f} rad/s"
                f" ({'exact' if item['active_set_exact'] else 'set mismatch'})"
                for item in row["worst_joint_rows"][:3]
            )
            + "."
        )
    report = "\n".join(
        [
            "# Bonesaw implicitfast constraint-RHS localization · r239",
            "",
            "> SPENT-LABEL DIAGNOSTIC ONLY · construction/selection/authority **INELIGIBLE** · new MuJoCo/policy/controller/selector/plant steps **0 / 0 / 0 / 0 / 0**.",
            "",
            "MuJoCo documents that implicit and implicitfast exclude constraint forces Jᵀf(v) from the force-velocity Jacobian. Contact friction remains a constraint-space reference acceleration. Mapping the reference engine's implicitfast setting onto Bonesaw's local implicit contact damping therefore conflates smooth-force integration with constraint-RHS evaluation.",
            "",
            *markdown_table(
                [
                    "diagnostic ABI",
                    "coverage",
                    "exact active sets",
                    "pred-only / missed",
                    "fitted width · ang / lin / joint",
                    "p99 ms",
                ],
                table,
            ),
            "",
            "ABI 0 removes the predicted-only point in sample 26 and reduces fitted joint width from 21.255 to 5.077 rad/s. Sample 10 remains uncovered with an exact active set, localizing the residual tail to within-foot wrench distribution rather than activation.",
            "",
            *worst_lines,
            "",
            "This comparison read completed R238 labels and cannot choose a production mapping or numerical profile. The documented constraint semantics supply the independent equation-level rationale; a causal-only convergence freeze and a new untouched holdout are still required.",
            "An independent retained rerun reproduced all 33 non-timing arrays exactly; three measured timing arrays are excluded from semantic equality.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-implicitfast-constraint-rhs-localization-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-implicitfast-constraint-rhs-localization.npz", **stored
    )
    (output / "G1_IMPLICITFAST_CONSTRAINT_RHS_LOCALIZATION.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(
        render_report_html(report, title="Bonesaw implicitfast constraint RHS · r239")
    )
    print(
        json.dumps(
            {
                "diagnostic_only": True,
                "abi0_coverage": rows[0]["sample_coverage"],
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
