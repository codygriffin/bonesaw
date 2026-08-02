#!/usr/bin/env python3
"""Admit force-point / rank-minimal kinematic declarations for finite support."""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_exact_dynamic_batch_report import (
    SOLVED_STATUSES,
    distribution,
    dynamic_buffers,
    markdown_table,
    render_report_html,
    run_dynamic,
    status_counts,
)
from cuda_point_query_mirror_report import pin_point_products
from finite_support_patch_report import (
    MODEL,
    PATCH_MARGIN_M,
    PATCH_STABLE_ID,
    failure_and_isolation,
    independent_oracle,
    invariance_case,
    make_case,
    zero_load_case,
)


# Disabled=0, LockedPoint=1, NormalPoint=2, RollingPoint=3. The four sole
# force points retain 4×3 force variables while emitting 3+1+0+2 = 6 rows.
RANK_MINIMAL_MODES = [1, 2, 0, 3, 0]
ALL_LOCKED_MODES = [1, 1, 1, 1, 1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--timing-repeats", type=int, default=60)
    parser.add_argument("--output", default="benchmarks/results/rank-minimal-support-r85")
    parser.add_argument("--web-report", default="web/RANK_MINIMAL_SUPPORT_R85.html")
    return parser.parse_args()


def selected_axes(mode: int) -> tuple[int, ...]:
    return {0: (), 1: (0, 1, 2), 2: (2,), 3: (1, 2)}[mode]


def rank_oracle_case(samples: int) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session, case = make_case(samples, contact_kinematic_modes=RANK_MINIMAL_MODES)
    result = run_dynamic(session, case)
    oracle = independent_oracle(session, case, result)
    point_position, point_jacobian = pin_point_products(
        MODEL,
        list(session.joint_names),
        list(session.point_frame_names),
        np.asarray(session.point_offsets, np.float64),
        case["q"],
        case["roots"],
    )
    ranks: list[int] = []
    minimum_singular_values: list[float] = []
    full_sole_residual = 0.0
    for agent in range(samples):
        rows = []
        for contact, mode in enumerate(RANK_MINIMAL_MODES[:4]):
            for axis in selected_axes(mode):
                rows.append(point_jacobian[agent, 1 + contact, axis])
        matrix = np.vstack(rows)
        singular = np.linalg.svd(matrix, compute_uv=False)
        ranks.append(int(np.sum(singular > 1e-8)))
        minimum_singular_values.append(float(np.min(singular)))
        implied = np.einsum("pcg,g->pc", point_jacobian[agent, 1:5], result["qdd"][agent])
        full_sole_residual = max(full_sole_residual, float(np.max(np.abs(implied))))
    margins = oracle.pop("oracle_margin_m")
    cop = oracle.pop("oracle_cop_xy_m")
    rank_fingerprint = json.loads(session.backend_fingerprint_json)["kernel_hash"]
    all_locked, _ = make_case(samples, contact_kinematic_modes=ALL_LOCKED_MODES)
    all_locked_fingerprint = json.loads(all_locked.backend_fingerprint_json)["kernel_hash"]
    passed = bool(
        np.all(np.isin(result["status"], SOLVED_STATUSES))
        and set(ranks) == {6}
        and min(minimum_singular_values) > 1e-3
        and full_sole_residual <= 3e-8
        and np.min(margins) >= PATCH_MARGIN_M - 2e-8
        and oracle["reported_margin_max_abs_delta_m"] <= 3e-8
        and oracle["pinocchio_dynamics_linf"] <= 3e-7
        and oracle["pinocchio_contact_acceleration_linf"] <= 3e-8
        and rank_fingerprint != all_locked_fingerprint
        and result["allocation_calls"][0] == 0
        and result["allocated_bytes"][0] == 0
    )
    metrics = {
        "samples": samples,
        "contact_modes": RANK_MINIMAL_MODES,
        "kernel_abi_version": int(session.kernel_abi_version),
        "sole_force_points": 4,
        "sole_force_variables": 12,
        "sole_kinematic_rows": 6,
        "rank_values": sorted(set(ranks)),
        "minimum_singular_value": min(minimum_singular_values),
        "maximum_implied_all_point_acceleration_linf": full_sole_residual,
        "minimum_oracle_support_margin_m": float(np.min(margins)),
        **oracle,
        "descriptor_fingerprint_changes_with_modes": rank_fingerprint != all_locked_fingerprint,
        "status_counts": status_counts(result["status"]),
        "allocation_calls": int(result["allocation_calls"][0]),
        "allocated_bytes": int(result["allocated_bytes"][0]),
        "pass": passed,
    }
    raw = {
        "oracle_cop_xy_m": cop,
        "oracle_support_margin_m": margins,
        "qdd": result["qdd"].copy(),
        "effort": result["effort"].copy(),
        "contact_force_basis": result["contact_force_basis"].copy(),
    }
    return metrics, raw


def force_preference_conflict() -> dict[str, Any]:
    session, case = make_case(2, contact_kinematic_modes=RANK_MINIMAL_MODES)
    case["task_active"].fill(0)
    total_weight = 40.2 * 9.81
    case["nominal_normal_force"][:] = np.asarray([total_weight, 0.0, 0.0, 0.0, 0.0])
    case["support_patch_enabled"][:, 0] = [0, 1]
    result = run_dynamic(session, case)
    oracle = independent_oracle(session, case, result)
    margins = oracle["oracle_margin_m"]
    normal = result["contact_force_basis"][:, :4, 2]
    nominal = case["nominal_normal_force"][:, :4]
    style_residual = np.linalg.norm(normal - nominal, axis=1)
    passed = bool(
        np.all(np.isin(result["status"], SOLVED_STATUSES))
        and margins[0] < PATCH_MARGIN_M - 1e-4
        and abs(margins[1] - PATCH_MARGIN_M) <= 2e-8
        and style_residual[1] > style_residual[0]
        and result["limiting_support_patch"][1] == PATCH_STABLE_ID
    )
    return {
        "without_patch_oracle_margin_m": float(margins[0]),
        "with_patch_oracle_margin_m": float(margins[1]),
        "requested_margin_m": PATCH_MARGIN_M,
        "without_patch_force_style_l2_n": float(style_residual[0]),
        "with_patch_force_style_l2_n": float(style_residual[1]),
        "unconstrained_normal_force_n": normal[0].tolist(),
        "constrained_normal_force_n": normal[1].tolist(),
        "pass": passed,
    }


def timing_profile(agents: int, repeats: int, modes: list[int]) -> dict[str, Any]:
    session, case = make_case(agents, contact_kinematic_modes=modes)
    buffers = dynamic_buffers(session)
    for _ in range(4):
        run_dynamic(session, case, buffers)
    solve = np.empty(repeats, np.float64)
    pipeline = np.empty(repeats, np.float64)
    calls = allocated = 0
    stages = ("joint_envelope", "fk", "jacobian", "dynamics", "point", "emission", "solve")
    for index in range(repeats):
        result = run_dynamic(session, case, buffers)
        solve[index] = result["solve_execute_ns"][0]
        pipeline[index] = sum(float(result[f"{stage}_execute_ns"][0]) for stage in stages)
        calls += int(result["allocation_calls"][0])
        allocated += int(result["allocated_bytes"][0])
    return {
        "batch_size": agents,
        "solve_execute_ns": distribution(solve),
        "full_pipeline_ns": distribution(pipeline),
        "allocation_calls": calls,
        "allocated_bytes": allocated,
    }


def timing_comparison(repeats: int) -> list[dict[str, Any]]:
    rows = []
    for agents in (1, 8, 32):
        locked = timing_profile(agents, repeats, ALL_LOCKED_MODES)
        minimal = timing_profile(agents, repeats, RANK_MINIMAL_MODES)
        rows.append(
            {
                "batch_size": agents,
                "repeats_per_profile": repeats,
                "all_locked": locked,
                "rank_minimal": minimal,
                "solve_p50_delta_percent": 100.0
                * (minimal["solve_execute_ns"]["p50"] / locked["solve_execute_ns"]["p50"] - 1.0),
                "solve_p99_delta_percent": 100.0
                * (minimal["solve_execute_ns"]["p99"] / locked["solve_execute_ns"]["p99"] - 1.0),
            }
        )
    return rows


def report_markdown(metrics: dict[str, Any]) -> str:
    oracle = metrics["rank_oracle"]
    conflict = metrics["force_preference_conflict"]
    authority = [
        ["Invariant", "six rank-minimal sole rows", "rank 6; all 12 material-point acceleration components implied"],
        ["Invariant", "floating dynamics", "independent Pinocchio hard residual"],
        ["Viability", "finite patch 8401 at 20 mm", "independent weighted-CoP margin + limiting stable ID"],
        ["Viability", "joint stopping envelope", "separate qdd interval, recovery, and limiting joint"],
        ["Intent", "point attractor", "physical-unit residual; cannot perturb support/dynamics"],
        ["Preference/Style", "normal-force distribution", "yields from corner nominal when CoP margin binds"],
        ["Contact resource", "four force points / twelve force variables", "unilateral and friction margins remain per point"],
        ["Actuator resource", "effort headroom", "separate from support geometry and task residual"],
        ["Solver budget", "status, clipped steps, hard violation", "bounded work remains distinct from physical headroom"],
        ["Backend", "CpuExactF64", "zero-allocation semantic reference; CUDA unavailable"],
    ]
    timing = []
    for row in metrics["timing_comparison"]:
        locked = row["all_locked"]["solve_execute_ns"]
        minimal = row["rank_minimal"]["solve_execute_ns"]
        timing.append(
            [
                row["batch_size"],
                f'{locked["p50"] / 1e3:.3f}',
                f'{minimal["p50"] / 1e3:.3f}',
                f'{row["solve_p50_delta_percent"]:.2f}%',
                f'{locked["p99"] / 1e3:.3f}',
                f'{minimal["p99"] / 1e3:.3f}',
                row["rank_minimal"]["allocation_calls"],
            ]
        )
    return "\n".join(
        [
            "# Bonesaw rank-minimal support declaration · r85",
            "",
            "## Outcome",
            "",
            f'**Admission: {"PASS" if metrics["admission"] else "FAIL"}.** Kernel ABI 2 separates force-point existence from kinematic row mode. Four sole points retain twelve independently bounded force variables and the exact finite CoP cone, while Locked + Normal + Disabled + Rolling modes emit six full-rank rigid-foot equations. Rust owns topology, emission, solve, and memory; Python owns independent oracles and reporting. No policy or physics rollout is used.',
            "",
            "## Example authority stack",
            "",
            *markdown_table(["layer", "example authority", "separate witness"], authority),
            "",
            "## Independent rank + dynamics + support oracle",
            "",
            *markdown_table(
                ["states", "rows", "rank", "σmin", "implied all-point L∞", "Pin dynamics L∞", "CoP min m", "pass"],
                [[oracle["samples"], oracle["sole_kinematic_rows"], oracle["rank_values"], f'{oracle["minimum_singular_value"]:.3e}', f'{oracle["maximum_implied_all_point_acceleration_linf"]:.3e}', f'{oracle["pinocchio_dynamics_linf"]:.3e}', f'{oracle["minimum_oracle_support_margin_m"]:.6f}', oracle["pass"]]],
            ),
            "",
            "Pinocchio independently stacks the six selected rows and then checks all twelve sole-point acceleration components. NumPy independently rebuilds the hull and force-weighted CoP. Contact mode is descriptor-fingerprinted, so changing row semantics cannot reuse a stale kernel identity.",
            "",
            "## Support versus Style force preference",
            "",
            *markdown_table(["condition", "CoP margin m", "force-style L2 N"], [
                ["patch disabled", f'{conflict["without_patch_oracle_margin_m"]:.6f}', f'{conflict["without_patch_force_style_l2_n"]:.3f}'],
                ["20 mm patch enabled", f'{conflict["with_patch_oracle_margin_m"]:.6f}', f'{conflict["with_patch_force_style_l2_n"]:.3f}'],
            ]),
            "",
            "A corner-loaded nominal force distribution is legal without the patch but violates the requested reserve. Enabling patch 8401 moves CoP to exactly 20 mm and sacrifices only the lower-authority force-distribution objective.",
            "",
            "## All-locked versus rank-minimal timing (untrimmed)",
            "",
            *markdown_table(["agents", "locked p50 us", "minimal p50 us", "p50 Δ", "locked p99 us", "minimal p99 us", "alloc calls"], timing),
            "",
            "Profiles run sequentially on the same host and retain every sample; this is a directional implementation audit, not a pinned alternating hardware-counter promotion claim.",
            "",
            "## Failure isolation and determinism",
            "",
            *markdown_table(["check", "result"], [[key, value] for key, value in metrics["failure_and_isolation"].items()]),
            "",
            *markdown_table(["check", "result"], [[key, value] for key, value in metrics["invariance"].items()]),
            "",
            "## Deliberate scope boundary",
            "",
            "R85 exposes Disabled, LockedPoint, NormalPoint, and RollingPoint fixed modes. RollingWheel needs its coordinate and stabilization constants in a later descriptor revision. General automatic row-basis synthesis, contact switching, CoM-in-polygon tasks, integration, and CUDA remain unavailable. The all-locked and rank-minimal solvers may select different lower-layer joint/force optima even though both satisfy the same rigid-foot hard manifold; this report admits physical constraints and independent witnesses, not bitwise solution equivalence across a changed equality basis.",
            "",
        ]
    )


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    oracle, raw = rank_oracle_case(args.samples)
    conflict = force_preference_conflict()
    zero_load = zero_load_case(RANK_MINIMAL_MODES)
    failure = failure_and_isolation(RANK_MINIMAL_MODES)
    invariance = invariance_case(RANK_MINIMAL_MODES)
    timing = timing_comparison(args.timing_repeats)
    admission = bool(
        oracle["pass"]
        and conflict["pass"]
        and zero_load["pass"]
        and failure["pass"]
        and invariance["pass"]
        and all(
            row[profile]["allocation_calls"] == 0 and row[profile]["allocated_bytes"] == 0
            for row in timing
            for profile in ("all_locked", "rank_minimal")
        )
    )
    metrics = {
        "schema_version": 1,
        "revision": "rank-minimal-support-r85",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "processor": platform.processor()},
        "policy": None,
        "physics_rollout": None,
        "rank_oracle": oracle,
        "force_preference_conflict": conflict,
        "zero_load": zero_load,
        "failure_and_isolation": failure,
        "invariance": invariance,
        "timing_comparison": timing,
        "admission": admission,
    }
    np.savez_compressed(output / "rank-minimal-support-raw.npz", **raw)
    (output / "rank-minimal-support-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report_markdown(metrics)
    (output / "RANK_MINIMAL_SUPPORT_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(render_report_html(markdown, title="Bonesaw rank-minimal support · r85"))
    print(json.dumps(metrics, indent=2))
    if not admission:
        raise SystemExit("rank-minimal support admission failed")


if __name__ == "__main__":
    main()
