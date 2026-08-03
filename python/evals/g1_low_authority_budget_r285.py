#!/usr/bin/env python3
"""Audit explicit Preference/Style anytime budgets without policy or physics."""

from __future__ import annotations

import argparse
import json
import pathlib
from typing import Any

import numpy as np

from cpu_reference_report import render_report_html
from g1_cumulative_solve_diagnostics_r282 import (
    R281_NON_TIMING_ARRAY_COUNT,
    R281_NON_TIMING_DIGEST,
    non_timing_digest,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
REVISION = "g1-low-authority-budget-r285"
RESULT_DIR = ROOT / "benchmarks/results" / REVISION
RESULT = RESULT_DIR / f"{REVISION}-metrics.json"
REPORT = RESULT_DIR / "G1_LOW_AUTHORITY_BUDGET_R285.md"
WEB_REPORT = ROOT / "web/G1_LOW_AUTHORITY_BUDGET_R285.html"
NEW_DIAGNOSTICS = {
    "low_authority_budget_exhausted_level",
    "cumulative_low_authority_budget_exhausted_mask",
}


def _load(path: pathlib.Path) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    return (
        json.loads((path / "floating-walk-metrics.json").read_text()),
        dict(np.load(path / "floating-walk-raw.npz", allow_pickle=False)),
    )


def _profile(
    name: str,
    document: dict[str, Any],
    raw: dict[str, np.ndarray],
    baseline: dict[str, np.ndarray],
) -> dict[str, Any]:
    metrics = document["metrics"]
    established = {key: value for key, value in raw.items() if key not in NEW_DIAGNOSTICS}
    count, digest = non_timing_digest(established)
    exhaustion_ticks = np.flatnonzero(raw["cumulative_low_authority_budget_exhausted_mask"])
    changed_names: list[str] = []
    first_divergence: int | None = None
    for key in sorted(set(baseline) & set(raw)):
        if key == "step_ns" or key in NEW_DIAGNOSTICS:
            continue
        left, right = baseline[key], raw[key]
        if np.array_equal(left, right, equal_nan=True):
            continue
        changed_names.append(key)
        if left.ndim == 0 or left.shape[0] != metrics["ticks"]:
            continue
        equal = (left == right) | (np.isnan(left) & np.isnan(right))
        if left.ndim > 1:
            equal = np.all(equal, axis=tuple(range(1, left.ndim)))
        indices = np.flatnonzero(~equal)
        if indices.size:
            tick = int(indices[0])
            first_divergence = tick if first_divergence is None else min(first_divergence, tick)
    return {
        "name": name,
        "established_non_timing_arrays": count,
        "established_non_timing_sha256": digest,
        "established_reference_exact": (
            count == R281_NON_TIMING_ARRAY_COUNT and digest == R281_NON_TIMING_DIGEST
        ),
        "p50_us": float(metrics["latency_us"]["p50"]),
        "p99_us": float(metrics["latency_us"]["p99"]),
        "maximum_us": float(metrics["latency_us"]["max"]),
        "over_5ms": int(metrics["deadline_misses"]["5ms"]),
        "root_rms_m": float(metrics["root_tracking_rms_m"]),
        "center_of_mass_rms_m": float(metrics["center_of_mass_tracking_rms_m"]),
        "stance_foot_rms_m": float(metrics["stance_foot_tracking_rms_m"]),
        "maximum_dynamics_residual": float(metrics["maximum_dynamics_residual"]),
        "maximum_contact_residual": float(
            metrics["maximum_contact_acceleration_residual"]
        ),
        "failed_or_infeasible_ticks": int(
            metrics["status_counts"]["failed"]
            + metrics["status_counts"]["primal_infeasible"]
        ),
        "normal_contact_contingency_ticks": int(
            metrics["status_counts"]["normal_contact_contingency"]
        ),
        "budget_exhaustion_ticks": [int(tick) for tick in exhaustion_ticks],
        "budget_exhaustion_count": int(exhaustion_ticks.size),
        "final_exhaustion_levels": {
            key: int(value)
            for key, value in metrics["solver_work"][
                "low_authority_budget_exhausted_by_level"
            ].items()
        },
        "cumulative_exhaustion_levels": {
            key: int(value)
            for key, value in metrics["cumulative_solver_work"][
                "low_authority_budget_exhausted_by_level"
            ].items()
        },
        "changed_common_arrays": len(changed_names),
        "first_semantic_divergence_tick": first_divergence,
    }


def build_result(paths: dict[str, pathlib.Path]) -> dict[str, Any]:
    loaded = {name: _load(path) for name, path in paths.items()}
    baseline_document, baseline_raw = loaded["unbounded"]
    profiles = {
        name: _profile(name, document, raw, baseline_raw)
        for name, (document, raw) in loaded.items()
    }
    baseline = profiles["unbounded"]
    style2 = profiles["style2"]
    return {
        "schema": "bonesaw.g1-low-authority-budget-r285.v1",
        "revision": REVISION,
        "execution": {
            "policy_steps": 0,
            "physics_steps": 0,
            "ticks": int(baseline_document["metrics"]["ticks"]),
            "dt_seconds": float(baseline_document["metrics"]["dt_seconds"]),
            "timing_cpu": 4,
            "maximum_feasibility_iterations": 7,
            "maximum_feasibility_projection_sweeps": 8,
        },
        "contract": {
            "unbounded_default_exact": baseline["established_reference_exact"],
            "unbounded_exhaustion_count": baseline["budget_exhaustion_count"],
            "hard_layers_budgeted": False,
            "preference_and_style_independently_budgeted": True,
            "exhaustion_is_typed_per_final_attempt_and_cumulatively": True,
        },
        "profiles": profiles,
        "style2_tradeoff": {
            "p99_change_percent": float(
                100.0 * (style2["p99_us"] / baseline["p99_us"] - 1.0)
            ),
            "deadline_miss_change_percent": float(
                100.0 * (style2["over_5ms"] / baseline["over_5ms"] - 1.0)
            ),
            "root_rms_change_percent": float(
                100.0 * (style2["root_rms_m"] / baseline["root_rms_m"] - 1.0)
            ),
            "two_exhaustions_cause_closed_loop_divergence": (
                style2["budget_exhaustion_count"] == 2
                and style2["first_semantic_divergence_tick"]
                == style2["budget_exhaustion_ticks"][0]
            ),
        },
        "verdict": {
            "mechanism_retained_default_off": True,
            "style1_profile_promoted": False,
            "style2_profile_promoted": False,
            "p99_gate_demonstrated_under_degradation": style2["p99_us"] < 5_000.0,
            "tracking_gate_passed": False,
            "authority_admitted": False,
        },
    }


def validate_result(result: dict[str, Any]) -> None:
    if result["revision"] != REVISION:
        raise ValueError("unexpected R285 revision")
    if result["execution"]["policy_steps"] or result["execution"]["physics_steps"]:
        raise ValueError("R285 must remain policy/physics free")
    if not result["contract"]["unbounded_default_exact"]:
        raise ValueError("the dormant R285 mechanism changed the established replay")
    if result["contract"]["unbounded_exhaustion_count"] != 0:
        raise ValueError("the unbounded default reported a false exhaustion")
    style2 = result["profiles"]["style2"]
    if style2["budget_exhaustion_ticks"] != [155, 158]:
        raise ValueError("unexpected Style-2 exhaustion witness")
    if style2["failed_or_infeasible_ticks"] != 0:
        raise ValueError("Style-2 must retain hard-feasible execution")
    if style2["maximum_dynamics_residual"] > 1e-8 or style2["maximum_contact_residual"] > 1e-8:
        raise ValueError("Style-2 violated a hard residual")
    verdict = result["verdict"]
    if verdict["style1_profile_promoted"] or verdict["style2_profile_promoted"]:
        raise ValueError("R285 degraded profiles must remain rejected")
    if verdict["tracking_gate_passed"] or verdict["authority_admitted"]:
        raise ValueError("R285 cannot admit tracking or authority")


def build_report(result: dict[str, Any]) -> str:
    rows = []
    for name in ("unbounded", "style1", "style2", "pref20_style2"):
        row = result["profiles"][name]
        rows.append(
            f"| {name.replace('_', '+')} | {row['budget_exhaustion_count']} | "
            f"{row['p50_us']:.1f} | {row['p99_us']:.1f} | {row['over_5ms']} | "
            f"{row['root_rms_m']:.3f} | {row['center_of_mass_rms_m']:.3f} | "
            f"{row['stance_foot_rms_m']:.3f} | {row['normal_contact_contingency_ticks']} |"
        )
    style2 = result["profiles"]["style2"]
    tradeoff = result["style2_tradeoff"]
    return "\n".join(
        [
            "# G1 low-authority anytime budget · R285",
            "",
            "> Mechanism **RETAINED DEFAULT-OFF** · hard feasibility **PASS** · degraded profiles **REJECTED** · authority **CLOSED**.",
            "",
            "R285 evaluates the suggested ‘fewer QP iterations, continue operating’ boundary at the only soft layers allowed to degrade: Preference and terminal Style. Invariant, Viability, Intent, equality, bounds, and named hard rows are never capped. Exhaustion returns the current feasible solution, preserves every completed higher optimum, skips lower authority, and emits both a final-attempt level and a cumulative retry-safe mask.",
            "",
            "## Policy-free / physics-free replay",
            "",
            "| profile | exhausted ticks | p50 µs | p99 µs | >5 ms | root RMS m | CoM RMS m | stance-foot RMS m | normal contingencies |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
            *rows,
            "",
            f"Style-2 exhausts on only ticks `{style2['budget_exhaustion_ticks']}`. It lowers p99 by {-tradeoff['p99_change_percent']:.2f}% and 5 ms misses by {-tradeoff['deadline_miss_change_percent']:.2f}%, but the first omitted terminal refinement is also the first semantic divergence. Root RMS then increases by {tradeoff['root_rms_change_percent']:.2f}% and normal-contact contingencies rise to {style2['normal_contact_contingency_ticks']}. This is a closed-loop sensitivity result, not a hard-feasibility failure: dynamics/contact residuals stay below 1e-8 and failed/infeasible ticks remain zero.",
            "",
            "## Decision",
            "",
            "Retain the independently configurable and fully typed mechanism as a default-off degraded-mode primitive. Reject every measured finite profile for walking promotion. A static per-level call count is too blunt: two early Style truncations alter the later contact path. The next continuous design must carry explicit low-authority progress or use a supervisor with a tracking-error budget; it cannot infer safety from a local hard-feasible solve alone.",
            "",
            "The unbounded default reproduces the established 89-array digest exactly and reports zero false exhaustion. No policy, physics, actuator command, contact authority, or hardware authority is introduced.",
            "",
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--unbounded", type=pathlib.Path, required=True)
    parser.add_argument("--style1", type=pathlib.Path, required=True)
    parser.add_argument("--style2", type=pathlib.Path, required=True)
    parser.add_argument("--pref20-style2", type=pathlib.Path, required=True)
    args = parser.parse_args()
    result = build_result(
        {
            "unbounded": args.unbounded,
            "style1": args.style1,
            "style2": args.style2,
            "pref20_style2": args.pref20_style2,
        }
    )
    validate_result(result)
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    RESULT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    report = build_report(result)
    REPORT.write_text(report)
    WEB_REPORT.write_text(render_report_html(report, title="G1 low-authority budget · R285"))
    print(f"validated {RESULT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
