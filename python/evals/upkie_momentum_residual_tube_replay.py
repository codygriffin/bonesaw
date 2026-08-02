#!/usr/bin/env python3
"""R209 physics-free generalized-momentum residual tube replay."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_coupled_contact_regularization_replay import split_score
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-momentum-residual-tube-replay-r209"
FIT_MODES = ("nearest_candidate", "all_candidates")
SCALES = (1.0, 1.25)


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
        "--web-report", default="web/UPKIE_MOMENTUM_RESIDUAL_TUBE_REPLAY_R209.html"
    )
    return parser.parse_args()


def loco_momentum_bounds(
    cases: np.ndarray,
    fresh: np.ndarray,
    candidate_count: np.ndarray,
    residuals: np.ndarray,
    mode: str,
    scale: float,
) -> tuple[np.ndarray, np.ndarray]:
    if mode not in FIT_MODES or scale < 1.0 or not np.isfinite(scale):
        raise ValueError("invalid residual fit profile")
    sample_count, _, dof = residuals.shape
    lower = np.empty((sample_count, dof), np.float64)
    upper = np.empty((sample_count, dof), np.float64)
    retained = ~fresh
    residual_norm = np.linalg.norm(residuals, axis=2)
    residual_norm[~np.isfinite(residual_norm)] = np.inf
    nearest = np.argmin(residual_norm, axis=1)
    for case in np.unique(cases):
        test = cases == case
        train = retained if np.all(fresh[test]) else retained & ~test
        if not np.any(train):
            raise ValueError(f"no training rows for held-out case {case}")
        if mode == "nearest_candidate":
            values = residuals[train, nearest[train]]
        else:
            values = np.concatenate(
                [residuals[index, : int(candidate_count[index])] for index in np.flatnonzero(train)],
                axis=0,
            )
        lower[test] = np.minimum(np.min(values, axis=0), 0.0) * scale
        upper[test] = np.maximum(np.max(values, axis=0), 0.0) * scale
    return lower, upper


def main() -> int:
    args = parse_args()
    replay_path = pathlib.Path(args.replay).resolve()
    replay = np.load(replay_path, allow_pickle=False)
    required = {
        "case",
        "fresh_holdout",
        "candidate_count",
        "candidates",
        "root_position",
        "root_quaternion_wxyz",
        "q",
        "actual_delta",
    }
    missing = required - set(replay.files)
    if missing:
        raise SystemExit(f"replay missing arrays: {sorted(missing)}")
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(pathlib.Path(args.model).resolve()))
    cases = replay["case"]
    fresh = replay["fresh_holdout"]
    candidate_count = replay["candidate_count"]
    candidates = replay["candidates"]
    root_position = replay["root_position"]
    root_quaternion = replay["root_quaternion_wxyz"]
    q = replay["q"]
    actual = replay["actual_delta"]
    sample_count = len(cases)
    residuals = np.full((sample_count, 4, 12), np.nan, np.float64)
    residual_timing = np.empty(sample_count, np.int64)
    zero_rust_allocation = True
    for sample in range(sample_count):
        count = int(candidate_count[sample])
        predicted = candidates[sample, :count] * CONTROL_DT
        timing = balance.model_generalized_momentum_impulse_residuals(
            root_position[sample],
            root_quaternion[sample],
            q[sample],
            actual[sample],
            predicted,
            residuals[sample, :count],
        )
        residual_timing[sample] = timing[0]
        zero_rust_allocation &= timing[1:] == (0, 0)

    velocity_lower = np.empty(12, np.float64)
    velocity_upper = np.empty(12, np.float64)
    results: dict[str, Any] = {}
    rows: list[list[Any]] = []
    for mode in FIT_MODES:
        for scale in SCALES:
            momentum_lower, momentum_upper = loco_momentum_bounds(
                cases, fresh, candidate_count, residuals, mode, scale
            )
            lower = np.empty((sample_count, 12), np.float64)
            upper = np.empty((sample_count, 12), np.float64)
            projection_timing = np.empty(sample_count, np.int64)
            for sample in range(sample_count):
                timing = balance.model_generalized_velocity_interval_from_momentum_box(
                    root_position[sample],
                    root_quaternion[sample],
                    q[sample],
                    momentum_lower[sample],
                    momentum_upper[sample],
                    velocity_lower,
                    velocity_upper,
                )
                projection_timing[sample] = timing[0]
                zero_rust_allocation &= timing[1:] == (0, 0)
                count = int(candidate_count[sample])
                centers = candidates[sample, :count] * CONTROL_DT
                lower[sample] = np.min(centers, axis=0) + velocity_lower
                upper[sample] = np.max(centers, axis=0) + velocity_upper
            score = split_score(fresh, actual, lower, upper)
            key = f"{mode}_scale_{scale:.2f}"
            results[key] = {
                "mode": mode,
                "scale": scale,
                "score": score,
                "momentum_lower_abs": distribution(np.abs(momentum_lower).reshape(-1)),
                "momentum_upper_abs": distribution(np.abs(momentum_upper).reshape(-1)),
                "projection_timing_ns": distribution(projection_timing),
                "zero_rust_allocation": zero_rust_allocation,
            }
            rows.append(
                [
                    mode,
                    f"{scale:.2f}×",
                    f"{score['retained']['sample_coverage'] * 100.0:.3f}%",
                    f"{score['fresh']['sample_coverage'] * 100.0:.3f}%",
                    f"{score['all']['component_coverage'] * 100.0:.3f}%",
                    f"{score['all']['maximum_exceedance']:.5f}",
                    f"{score['all']['root_angular_rad_s_width']['p95']:.3f}",
                    f"{score['all']['root_linear_m_s_width']['p95']:.3f}",
                    f"{score['all']['joint_rad_s_width']['p95']:.3f}",
                    f"{results[key]['projection_timing_ns']['p99'] / 1_000.0:.3f}",
                ]
            )

    best_coverage = max(
        results,
        key=lambda key: results[key]["score"]["all"]["sample_coverage"],
    )
    narrowest_joint = min(
        results,
        key=lambda key: results[key]["score"]["all"]["joint_rad_s_width"]["p95"],
    )
    mechanism_passed = zero_rust_allocation and all(
        np.all(np.isfinite(residuals[index, : int(candidate_count[index])]))
        for index in range(sample_count)
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(pathlib.Path(args.model).resolve()),
        "replay": str(replay_path),
        "sample_count": sample_count,
        "retained_sample_count": int(np.sum(~fresh)),
        "fresh_sample_count": int(np.sum(fresh)),
        "fit_modes": list(FIT_MODES),
        "scales": list(SCALES),
        "physics_steps": 0,
        "policy_steps": 0,
        "mechanism_passed": mechanism_passed,
        "zero_rust_allocation": zero_rust_allocation,
        "authority_admitted": False,
        "residual_timing_ns": distribution(residual_timing),
        "best_coverage_profile": best_coverage,
        "narrowest_joint_profile": narrowest_joint,
        "results": results,
    }
    report = "\n".join(
        [
            "# Bonesaw generalized-momentum residual tube replay · r209",
            "",
            f"> Rust covector/tangent mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · physics steps **0** · policy steps **0** · strict retained/fresh coverage **{'PASS' if results[best_coverage]['score']['retained']['sample_coverage'] == 1.0 and results[best_coverage]['score']['fresh']['sample_coverage'] == 1.0 else 'FAIL'}** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- The immutable R206 replay supplies state, support-hypothesis accelerations and completed velocity labels. This run performs no MuJoCo integration and no policy/controller step.",
            "- Rust first computes generalized-momentum residual covectors `M(q)(Δv_observed−Δv_predicted)`. Python fits signed leave-one-named-case-out boxes; Rust maps each box back through the exact full inverse mass to a velocity interval.",
            "- Nearest-candidate fitting is an optimistic support-oracle diagnostic. All-candidate fitting encloses every declared support hypothesis. Both are empirical construction bounds, not calibrated online authority.",
            "",
            "## Momentum-space LOCO sweep",
            "",
            *markdown_table(
                [
                    "fit mode",
                    "scale",
                    "retained coverage",
                    "fresh coverage",
                    "component coverage",
                    "max miss",
                    "root ω p95",
                    "root v p95",
                    "joint p95",
                    "projection p99 µs",
                ],
                rows,
            ),
            "",
            f"Highest complete-sample coverage is **{best_coverage}** at **{results[best_coverage]['score']['all']['sample_coverage'] * 100.0:.3f}%**. Narrowest joint tube is **{narrowest_joint}** at **{results[narrowest_joint]['score']['all']['joint_rad_s_width']['p95']:.3f} rad/s p95**.",
            "",
            f"Rust residual p99 is **{metrics['residual_timing_ns']['p99'] / 1_000.0:.3f} µs**; inverse-mass box projection remains allocation-free. R204's causal directional comparison is 98.120% retained / 100% fresh at 9.687 / 0.991 / 58.020 p95 width.",
            "",
            "## Decision",
            "",
            "The covector→tangent mechanism is admitted as deterministic model machinery. These empirical boxes are rejected for command authority unless strict retained/fresh and second-morphology/contact-law holdouts close at useful width, with causal feature provenance, consequence non-regression, deadline evidence and hardware calibration kept separate.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-momentum-residual-tube-replay-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_MOMENTUM_RESIDUAL_TUBE_REPLAY.md").write_text(report)
    np.savez_compressed(
        destination / "upkie-momentum-residual-targets.npz",
        case=cases,
        fresh_holdout=fresh,
        candidate_count=candidate_count,
        residuals=residuals,
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "best_coverage_profile": best_coverage,
                "best_coverage": results[best_coverage]["score"]["all"][
                    "sample_coverage"
                ],
                "narrowest_joint_profile": narrowest_joint,
                "physics_steps": 0,
                "policy_steps": 0,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
