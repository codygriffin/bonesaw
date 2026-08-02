#!/usr/bin/env python3
"""R208 policy/physics-free diagonal-compliance sweep over the frozen R207 replay."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_coupled_contact_response_audit import (
    fit_loco_residual,
    impulse_score,
    score_envelope,
)
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-coupled-contact-regularization-replay-r208"
REGULARIZATION_RATIOS = (0.0, 1.0e-6, 1.0e-4, 1.0e-2, 1.0e-1, 1.0, 10.0)
SWEEP_COUNTS = (1, 2, 4, 8, 16)
ACCELERATION_RESERVE = np.asarray([5.0] * 6 + [50.0] * 6, np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument(
        "--replay",
        default=(
            "benchmarks/results/upkie-coupled-contact-response-audit-r207/"
            "upkie-coupled-contact-response-replay.npz"
        ),
    )
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_COUPLED_CONTACT_REGULARIZATION_REPLAY_R208.html",
    )
    return parser.parse_args()


def split_score(
    fresh: np.ndarray, actual: np.ndarray, lower: np.ndarray, upper: np.ndarray
) -> dict[str, Any]:
    return {
        "all": score_envelope(actual, lower, upper),
        "retained": score_envelope(actual[~fresh], lower[~fresh], upper[~fresh]),
        "fresh": score_envelope(actual[fresh], lower[fresh], upper[fresh]),
    }


def main() -> int:
    args = parse_args()
    replay_path = pathlib.Path(args.replay).resolve()
    replay = np.load(replay_path, allow_pickle=False)
    required = {
        "case",
        "fresh_holdout",
        "friction",
        "candidate_count",
        "candidates",
        "response",
        "delassus",
        "prospective_velocity",
        "actual_impulse",
        "actual_delta",
        "impulse_upper",
    }
    missing = required - set(replay.files)
    if missing:
        raise SystemExit(f"replay missing arrays: {sorted(missing)}")

    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(pathlib.Path(args.model).resolve()))
    cases = replay["case"]
    fresh = replay["fresh_holdout"]
    friction = replay["friction"]
    candidate_count = replay["candidate_count"]
    candidates = replay["candidates"]
    response = replay["response"]
    delassus = replay["delassus"]
    velocity = replay["prospective_velocity"]
    actual_impulse = replay["actual_impulse"]
    actual_delta = replay["actual_delta"]
    impulse_upper = replay["impulse_upper"]
    sample_count = len(cases)
    if sample_count == 0 or int(np.sum(fresh)) == 0:
        raise SystemExit("replay must contain retained and fresh rows")

    impulse = np.empty((2, 3), np.float64)
    regularized_after = np.empty((2, 3), np.float64)
    results: dict[str, Any] = {}
    table_rows: list[list[Any]] = []
    zero_rust_allocation = True
    for ratio in REGULARIZATION_RATIOS:
        for sweeps in SWEEP_COUNTS:
            predicted_impulse = np.empty((sample_count, 2, 3), np.float64)
            physical_after = np.empty((sample_count, 2, 3), np.float64)
            lower = np.empty((sample_count, 12), np.float64)
            upper = np.empty((sample_count, 12), np.float64)
            timings = np.empty(sample_count, np.int64)
            for sample in range(sample_count):
                timing = balance.solve_coupled_contact_impulse(
                    velocity[sample],
                    delassus[sample],
                    impulse_upper[sample],
                    np.full(2, friction[sample], np.float64),
                    0.0,
                    ratio,
                    sweeps,
                    impulse,
                    regularized_after,
                )
                zero_rust_allocation &= timing[1:] == (0, 0)
                timings[sample] = timing[0]
                predicted_impulse[sample] = impulse
                flat_impulse = impulse.reshape(-1)
                physical_after[sample] = (
                    velocity[sample].reshape(-1)
                    + delassus[sample] @ flat_impulse
                ).reshape(2, 3)
                impulse_delta = np.einsum(
                    "dca,ca->d", response[sample], impulse, optimize=False
                )
                count = int(candidate_count[sample])
                centers = candidates[sample, :count] * CONTROL_DT + impulse_delta
                lower[sample] = centers.min(axis=0) - ACCELERATION_RESERVE * CONTROL_DT
                upper[sample] = centers.max(axis=0) + ACCELERATION_RESERVE * CONTROL_DT

            loco_lower, loco_upper = fit_loco_residual(
                cases, fresh, actual_delta, lower, upper
            )
            raw = split_score(fresh, actual_delta, lower, upper)
            loco = split_score(fresh, actual_delta, loco_lower, loco_upper)
            impulse_metrics = impulse_score(actual_impulse, predicted_impulse)
            physical_speed = np.linalg.norm(physical_after, axis=2)
            key = f"ratio_{ratio:.0e}_sweeps_{sweeps}"
            results[key] = {
                "regularization_ratio": ratio,
                "sweeps": sweeps,
                "raw_envelope": raw,
                "loco_empirical_residual_envelope": loco,
                "impulse": impulse_metrics,
                "physical_post_contact_speed_m_s": distribution(
                    physical_speed.reshape(-1)
                ),
                "timing_ns": distribution(timings),
                "zero_rust_allocation": zero_rust_allocation,
            }
            table_rows.append(
                [
                    f"{ratio:.0e}",
                    sweeps,
                    f"{raw['retained']['sample_coverage'] * 100.0:.3f}%",
                    f"{raw['fresh']['sample_coverage'] * 100.0:.3f}%",
                    f"{loco['retained']['sample_coverage'] * 100.0:.3f}%",
                    f"{loco['fresh']['sample_coverage'] * 100.0:.3f}%",
                    f"{impulse_metrics['rmse_ns']:.5f}",
                    f"{raw['all']['maximum_exceedance']:.3f}",
                    f"{loco['all']['root_angular_rad_s_width']['p95']:.3f}",
                    f"{loco['all']['root_linear_m_s_width']['p95']:.3f}",
                    f"{loco['all']['joint_rad_s_width']['p95']:.3f}",
                    f"{results[key]['physical_post_contact_speed_m_s']['p95']:.3f}",
                    f"{results[key]['timing_ns']['p99'] / 1_000.0:.3f}",
                ]
            )

    best_impulse = min(results, key=lambda key: results[key]["impulse"]["rmse_ns"])
    best_max_miss = min(
        results,
        key=lambda key: results[key]["raw_envelope"]["all"]["maximum_exceedance"],
    )
    best_loco_coverage = max(
        results,
        key=lambda key: results[key]["loco_empirical_residual_envelope"]["all"][
            "sample_coverage"
        ],
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(pathlib.Path(args.model).resolve()),
        "replay": str(replay_path),
        "sample_count": sample_count,
        "retained_sample_count": int(np.sum(~fresh)),
        "fresh_sample_count": int(np.sum(fresh)),
        "regularization_ratios": list(REGULARIZATION_RATIOS),
        "sweep_counts": list(SWEEP_COUNTS),
        "zero_rust_allocation": zero_rust_allocation,
        "physics_steps": 0,
        "policy_steps": 0,
        "best_impulse_rmse_profile": best_impulse,
        "best_max_miss_profile": best_max_miss,
        "best_loco_coverage_profile": best_loco_coverage,
        "authority_admitted": False,
        "results": results,
    }
    report = "\n".join(
        [
            "# Bonesaw coupled-contact regularization replay · r208",
            "",
            f"> Frozen replay **PASS** · Rust allocation **{'PASS' if zero_rust_allocation else 'FAIL'}** · physics steps **0** · policy steps **0** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- This run loads the immutable R207 arrays and invokes only the Rust coupled-contact kernel. It performs no MuJoCo integration and no policy/controller step.",
            "- The regularization ratio adds explicit diagonal contact compliance `W_ii · ratio` inside the projected solve. Returned physical post-contact velocity is independently recomputed with the unmodified Delassus matrix.",
            "- The full ratio/sweep grid was declared as a construction sweep. Label-ranked profiles are observations, not automatic selectors or held-out authority.",
            "",
            "## Policy/physics-free sweep",
            "",
            *markdown_table(
                [
                    "diag ratio",
                    "sweeps",
                    "raw retained",
                    "raw fresh",
                    "LOCO retained",
                    "LOCO fresh",
                    "impulse RMSE N·s",
                    "raw max miss",
                    "LOCO root ω p95",
                    "LOCO root v p95",
                    "LOCO joint p95",
                    "physical post speed p95",
                    "solve p99 µs",
                ],
                table_rows,
            ),
            "",
            f"Lowest impulse RMSE: **{best_impulse}** at **{results[best_impulse]['impulse']['rmse_ns']:.5f} N·s**. Lowest raw maximum miss: **{best_max_miss}** at **{results[best_max_miss]['raw_envelope']['all']['maximum_exceedance']:.3f}**. Highest LOCO complete-sample coverage: **{best_loco_coverage}** at **{results[best_loco_coverage]['loco_empirical_residual_envelope']['all']['sample_coverage'] * 100.0:.3f}%**.",
            "",
            "## Decision",
            "",
            "Diagonal compliance is retained only if it improves conditioning/error without hiding physical post-contact residual or inflating the calibrated tube. This construction grid cannot establish calibration, strict second-morphology coverage, spatial-wrench completeness, consequence non-regression, deadline evidence, or hardware authority.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-coupled-contact-regularization-replay-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_COUPLED_CONTACT_REGULARIZATION_REPLAY.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "sample_count": sample_count,
                "physics_steps": 0,
                "policy_steps": 0,
                "best_impulse_rmse_profile": best_impulse,
                "best_max_miss_profile": best_max_miss,
                "best_loco_coverage_profile": best_loco_coverage,
                "zero_rust_allocation": zero_rust_allocation,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if zero_rust_allocation else 1


if __name__ == "__main__":
    raise SystemExit(main())
