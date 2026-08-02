#!/usr/bin/env python3
"""R212 zero-plant replay of causal grouped acceleration reachable sets.

The directional contact tube and candidate centers are immutable R207 inputs.
Only a predeclared, componentwise continuous-acceleration reserve changes.  The
actual post-transition velocity is read strictly after each Rust bound query
and is used for scoring only.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_transition_interval_audit import urdf_total_mass_kg
from upkie_directional_contact_transition_audit import (
    DIRECTIONAL_PROFILES,
    PRIMARY_PROFILE as DIRECTIONAL_PROFILE,
    build_directional_witnesses,
)
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-grouped-acceleration-reachable-set-replay-r212"
SOURCE_REVISION = "upkie-coupled-contact-response-audit-r207"
SOURCE_DEFAULT = pathlib.Path(
    "benchmarks/results/upkie-coupled-contact-response-audit-r207/"
    "upkie-coupled-contact-response-replay.npz"
)
PROFILES = {
    "r204_structured_5_5_50": (5.0, 5.0, 50.0),
    "root_angular_50": (50.0, 5.0, 50.0),
    # Frozen for the next morphology/contact-law audit. These are round values
    # already represented independently in R203, not a label-minimal fit.
    "frozen_grouped_50_10_50": (50.0, 10.0, 50.0),
    "quarter_hard_limit_50_25_50": (50.0, 25.0, 50.0),
    "hard_limit_sensitivity_200_25_200": (200.0, 25.0, 200.0),
}
PRIMARY_PROFILE = "frozen_grouped_50_10_50"
GROUPS = {
    "root_angular_rad_s": slice(0, 3),
    "root_linear_m_s": slice(3, 6),
    "joint_rad_s": slice(6, None),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--source", default=str(SOURCE_DEFAULT))
    parser.add_argument(
        "--output", default=f"benchmarks/results/{REVISION}"
    )
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_GROUPED_ACCELERATION_REACHABLE_SET_REPLAY_R212.html",
    )
    return parser.parse_args()


def reserve_vector(profile: tuple[float, float, float], dof: int) -> np.ndarray:
    if dof < 6:
        raise ValueError("floating generalized tangent must contain at least 6 rows")
    root_angular, root_linear, joint = profile
    values = np.asarray(
        [root_angular] * 3 + [root_linear] * 3 + [joint] * (dof - 6),
        np.float64,
    )
    if not np.all(np.isfinite(values)) or np.any(values < 0.0):
        raise ValueError("acceleration reserve must be finite and nonnegative")
    return values


def score_envelope(
    actual: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    selected: np.ndarray | None = None,
) -> dict[str, Any]:
    if selected is None:
        selected = np.ones(len(actual), np.bool_)
    actual = actual[selected]
    lower = lower[selected]
    upper = upper[selected]
    exceedance = np.maximum(np.maximum(lower - actual, actual - upper), 0.0)
    component_covered = exceedance <= 1.0e-12
    sample_covered = np.all(component_covered, axis=1)
    width = upper - lower
    result: dict[str, Any] = {
        "sample_count": int(len(actual)),
        "sample_coverage": float(np.mean(sample_covered)),
        "covered_sample_count": int(np.sum(sample_covered)),
        "component_coverage": float(np.mean(component_covered)),
        "maximum_exceedance": float(np.max(exceedance)),
        "exceedance": distribution(exceedance.reshape(-1)),
    }
    for name, section in GROUPS.items():
        result[f"{name}_width"] = distribution(width[:, section].reshape(-1))
        result[f"{name}_exceedance"] = distribution(
            exceedance[:, section].reshape(-1)
        )
    misses = np.argwhere(exceedance > 1.0e-12)
    result["misses"] = [
        {
            "selected_row": int(row),
            "coordinate": int(coordinate),
            "exceedance": float(exceedance[row, coordinate]),
        }
        for row, coordinate in misses
    ]
    return result


def replay_profile(
    session: Any,
    replay: Any,
    witnesses: list[np.ndarray],
    reserve: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    samples, dof = replay["actual_delta"].shape
    lower_all = np.empty((samples, dof), np.float64)
    upper_all = np.empty((samples, dof), np.float64)
    lower = np.empty(dof, np.float64)
    upper = np.empty(dof, np.float64)
    impulse_upper = np.empty((2, 3), np.float64)
    elapsed: list[int] = []
    allocation_calls = 0
    allocated_bytes = 0
    transition = np.asarray([0.0, CONTROL_DT], np.float64)
    directional = DIRECTIONAL_PROFILES[DIRECTIONAL_PROFILE]
    for row in range(samples):
        aggregate_lower = np.full(dof, np.inf, np.float64)
        aggregate_upper = np.full(dof, -np.inf, np.float64)
        for candidate in replay["candidates"][row, : int(replay["candidate_count"][row])]:
            timing = session.bound_directional_contact_transition_velocity_jump(
                transition,
                directional["restitution_upper"],
                witnesses[row],
                candidate - reserve,
                candidate + reserve,
                replay["response"][row],
                impulse_upper,
                lower,
                upper,
            )
            np.minimum(aggregate_lower, lower, out=aggregate_lower)
            np.maximum(aggregate_upper, upper, out=aggregate_upper)
            elapsed.append(int(timing[0]))
            allocation_calls += int(timing[1])
            allocated_bytes += int(timing[2])
        lower_all[row] = aggregate_lower
        upper_all[row] = aggregate_upper
    elapsed_array = np.asarray(elapsed, np.float64)
    timing = {
        "query_count": int(len(elapsed)),
        "p50_ns": float(np.percentile(elapsed_array, 50)),
        "p95_ns": float(np.percentile(elapsed_array, 95)),
        "p99_ns": float(np.percentile(elapsed_array, 99)),
        "maximum_ns": int(np.max(elapsed_array)),
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
    }
    return lower_all, upper_all, timing


def main() -> int:
    args = parse_args()
    source = pathlib.Path(args.source).resolve()
    model = pathlib.Path(args.model).resolve()
    if not source.is_file():
        raise SystemExit(f"missing immutable replay: {source}")
    replay = np.load(source)
    actual = np.asarray(replay["actual_delta"], np.float64)
    fresh = np.asarray(replay["fresh_holdout"], np.bool_)
    dof = actual.shape[1]
    total_mass_kg = urdf_total_mass_kg(model)

    import bonesaw

    session = bonesaw.UpkieBalanceSession(str(model))
    directional = DIRECTIONAL_PROFILES[DIRECTIONAL_PROFILE]
    witnesses = [
        build_directional_witnesses(
            replay["prospective_velocity"][row],
            replay["effective_mass"][row],
            total_mass_kg,
            float(replay["friction"][row]),
            directional,
        )
        for row in range(len(actual))
    ]

    results: dict[str, Any] = {}
    envelopes: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for name, profile in PROFILES.items():
        reserve = reserve_vector(profile, dof)
        lower, upper, timing = replay_profile(
            session, replay, witnesses, reserve
        )
        envelopes[name] = (lower, upper)
        results[name] = {
            "reserve": {
                "root_angular_rad_s2": profile[0],
                "root_linear_m_s2": profile[1],
                "joint_rad_s2": profile[2],
            },
            "all": score_envelope(actual, lower, upper),
            "retained": score_envelope(actual, lower, upper, ~fresh),
            "fresh_holdout": score_envelope(actual, lower, upper, fresh),
            "timing": timing,
        }

    baseline_lower = np.asarray(replay["directional_lower"], np.float64)
    baseline_upper = np.asarray(replay["directional_upper"], np.float64)
    replay_match = bool(
        np.array_equal(envelopes["r204_structured_5_5_50"][0], baseline_lower)
        and np.array_equal(envelopes["r204_structured_5_5_50"][1], baseline_upper)
    )
    replay_max_abs_error = float(
        max(
            np.max(
                np.abs(
                    envelopes["r204_structured_5_5_50"][0] - baseline_lower
                )
            ),
            np.max(
                np.abs(
                    envelopes["r204_structured_5_5_50"][1] - baseline_upper
                )
            ),
        )
    )
    primary = results[PRIMARY_PROFILE]
    strict = bool(
        primary["retained"]["sample_coverage"] == 1.0
        and primary["fresh_holdout"]["sample_coverage"] == 1.0
    )
    zero_allocation = all(
        result["timing"]["allocation_calls"] == 0
        and result["timing"]["allocated_bytes"] == 0
        for result in results.values()
    )
    mechanism_passed = replay_match and zero_allocation
    rows = []
    for name, result in results.items():
        rows.append(
            [
                name,
                f"{result['reserve']['root_angular_rad_s2']:.0f}/"
                f"{result['reserve']['root_linear_m_s2']:.0f}/"
                f"{result['reserve']['joint_rad_s2']:.0f}",
                f"{result['retained']['sample_coverage'] * 100.0:.3f}%",
                f"{result['fresh_holdout']['sample_coverage'] * 100.0:.3f}%",
                f"{result['all']['maximum_exceedance']:.6f}",
                f"{result['all']['root_angular_rad_s_width']['p95']:.3f}",
                f"{result['all']['root_linear_m_s_width']['p95']:.3f}",
                f"{result['all']['joint_rad_s_width']['p95']:.3f}",
                f"{result['timing']['p99_ns'] / 1_000.0:.3f}",
            ]
        )

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source": str(source),
        "model": str(model),
        "sample_count": int(len(actual)),
        "retained_sample_count": int(np.sum(~fresh)),
        "fresh_sample_count": int(np.sum(fresh)),
        "physics_steps": 0,
        "policy_steps": 0,
        "controller_steps": 0,
        "primary_profile": PRIMARY_PROFILE,
        "directional_profile": DIRECTIONAL_PROFILE,
        "baseline_bitwise_replay_match": replay_match,
        "baseline_max_abs_replay_error": replay_max_abs_error,
        "zero_rust_allocation": zero_allocation,
        "mechanism_passed": mechanism_passed,
        "primary_strict_coverage": strict,
        "construction_frozen_for_next_morphology": True,
        "authority_admitted": False,
        "results": results,
    }
    report = "\n".join(
        [
            "# Bonesaw grouped acceleration reachable-set replay · r212",
            "",
            f"> Rust replay **{'PASS' if mechanism_passed else 'FAIL'}** · frozen grouped profile strict retained/fresh coverage **{'PASS' if strict else 'FAIL'}** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            f"R212 executes **0 physics, 0 policy, and 0 controller steps**. It replays {len(actual)} immutable R207 contact transitions and reruns only the allocation-free Rust directional bound. The completed velocity label is scoring-only.",
            "",
            "The reserve is a continuous 5 ms generalized-acceleration reachable set around each recorded causal terminal hypothesis. It is not an impact impulse, measured external wrench, learned residual, candidate selector, or completed-contact oracle.",
            "",
            "The primary **50/10/50** profile combines round group magnitudes already represented independently in R203. It was selected with knowledge of Upkie calibration results, so this run is construction evidence—not a held-out authority certificate. It is now frozen before the next morphology/contact-law evaluation.",
            "",
            "## Replay sweep",
            "",
            *markdown_table(
                [
                    "profile",
                    "reserve ω/v/q̈",
                    "retained",
                    "fresh",
                    "max miss /s",
                    "root ω p95",
                    "root v p95",
                    "joint p95",
                    "Rust p99 µs",
                ],
                rows,
            ),
            "",
            f"The original R204/R207 envelope replays **{'bitwise-identically' if replay_match else 'with a mismatch'}** (maximum absolute error `{replay_max_abs_error:.3e}`). All timed Rust calls made **0 allocations**.",
            "",
            "## Decision",
            "",
            "The frozen grouped set closes the current Upkie corpus without the extreme root/joint widths of the momentum residual boxes. It remains diagnostic until the exact same 50/10/50 construction passes a genuinely new morphology and contact law, retains useful normalized width, and demonstrates non-regressing plant consequence under the independent 5 ms deadline gate.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-grouped-acceleration-reachable-set-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_GROUPED_ACCELERATION_REACHABLE_SET_REPLAY.md").write_text(
        report
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "primary_strict_coverage": strict,
                "primary_retained_coverage": primary["retained"]["sample_coverage"],
                "primary_fresh_coverage": primary["fresh_holdout"]["sample_coverage"],
                "primary_width_p95": [
                    primary["all"]["root_angular_rad_s_width"]["p95"],
                    primary["all"]["root_linear_m_s_width"]["p95"],
                    primary["all"]["joint_rad_s_width"]["p95"],
                ],
            }
        )
    )
    return 0 if mechanism_passed and strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
