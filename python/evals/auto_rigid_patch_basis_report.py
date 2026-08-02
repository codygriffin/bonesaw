#!/usr/bin/env python3
"""Admit compiler-derived, descriptor-fingerprinted rigid-patch row bases."""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import CpuMirrorBatchSession
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
    CONTACT_IDS,
    CONTACT_SLOTS,
    MODEL,
    PATCH_MARGIN_M,
    PATCH_STABLE_ID,
    POINT_FRAMES,
    POINT_IDS,
    POINT_OFFSETS,
    independent_oracle,
    make_case,
)
from rank_minimal_support_report import ALL_LOCKED_MODES, RANK_MINIMAL_MODES, selected_axes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--timing-repeats", type=int, default=60)
    parser.add_argument("--output", default="benchmarks/results/auto-rigid-patch-basis-r87")
    parser.add_argument("--web-report", default="web/AUTO_RIGID_PATCH_BASIS_R87.html")
    return parser.parse_args()


def session_hash(session: CpuMirrorBatchSession) -> str:
    return str(json.loads(session.backend_fingerprint_json)["kernel_hash"])


def auto_case(samples: int) -> tuple[CpuMirrorBatchSession, dict[str, np.ndarray]]:
    return make_case(samples, auto_rigid_support_basis=True)


def independent_rank_and_manifold(
    session: CpuMirrorBatchSession,
    case: dict[str, np.ndarray],
    result: dict[str, np.ndarray],
) -> dict[str, Any]:
    _, jacobian = pin_point_products(
        MODEL,
        list(session.joint_names),
        list(session.point_frame_names),
        np.asarray(session.point_offsets, np.float64),
        case["q"],
        case["roots"],
    )
    modes = list(session.contact_kinematic_modes)
    ranks: list[int] = []
    minimum_singular_values: list[float] = []
    implied_all_point_linf = 0.0
    for agent in range(case["q"].shape[0]):
        rows = [
            jacobian[agent, 1 + contact, axis]
            for contact, mode in enumerate(modes[:4])
            for axis in selected_axes(mode)
        ]
        matrix = np.vstack(rows)
        singular = np.linalg.svd(matrix, compute_uv=False)
        ranks.append(int(np.sum(singular > 1e-8)))
        minimum_singular_values.append(float(np.min(singular)))
        implied = np.einsum("pcg,g->pc", jacobian[agent, 1:5], result["qdd"][agent])
        implied_all_point_linf = max(implied_all_point_linf, float(np.max(np.abs(implied))))
    return {
        "contact_modes": modes,
        "row_count": sum(len(selected_axes(mode)) for mode in modes[:4]),
        "rank_values": sorted(set(ranks)),
        "minimum_singular_value": min(minimum_singular_values),
        "maximum_implied_all_point_acceleration_linf": implied_all_point_linf,
    }


def oracle_case(samples: int) -> dict[str, Any]:
    session, case = auto_case(samples)
    result = run_dynamic(session, case)
    oracle = independent_oracle(session, case, result)
    rank = independent_rank_and_manifold(session, case, result)
    margins = oracle.pop("oracle_margin_m")
    oracle.pop("oracle_cop_xy_m")

    explicit_session, explicit_case = make_case(
        samples,
        contact_kinematic_modes=RANK_MINIMAL_MODES,
    )
    explicit_result = run_dynamic(explicit_session, explicit_case)
    explicit_rank = independent_rank_and_manifold(
        explicit_session,
        explicit_case,
        explicit_result,
    )
    all_locked, _ = make_case(samples, contact_kinematic_modes=ALL_LOCKED_MODES)
    repeated, _ = auto_case(samples)
    passed = bool(
        np.all(np.isin(result["status"], SOLVED_STATUSES))
        and rank["row_count"] == 6
        and rank["rank_values"] == [6]
        and rank["minimum_singular_value"] > 1e-3
        and rank["maximum_implied_all_point_acceleration_linf"] <= 3e-8
        and explicit_rank["rank_values"] == [6]
        and np.min(margins) >= PATCH_MARGIN_M - 2e-8
        and oracle["reported_margin_max_abs_delta_m"] <= 3e-8
        and oracle["pinocchio_dynamics_linf"] <= 3e-7
        and oracle["pinocchio_contact_acceleration_linf"] <= 3e-8
        and session_hash(session) == session_hash(repeated)
        and session_hash(session) != session_hash(all_locked)
        and result["allocation_calls"][0] == 0
        and result["allocated_bytes"][0] == 0
    )
    return {
        "samples": samples,
        "kernel_abi_version": int(session.kernel_abi_version),
        **rank,
        "explicit_r85_contact_modes": list(explicit_session.contact_kinematic_modes),
        "explicit_r85_minimum_singular_value": explicit_rank["minimum_singular_value"],
        "minimum_oracle_support_margin_m": float(np.min(margins)),
        **oracle,
        "repeat_descriptor_exact": session_hash(session) == session_hash(repeated),
        "descriptor_differs_from_all_locked": session_hash(session) != session_hash(all_locked),
        "allocation_calls": int(result["allocation_calls"][0]),
        "allocated_bytes": int(result["allocated_bytes"][0]),
        "status_counts": status_counts(result["status"]),
        "pass": passed,
    }


def invalid_geometry_case() -> dict[str, Any]:
    collinear = list(POINT_OFFSETS)
    for slot in range(1, 5):
        collinear[slot] = (-0.08 + 0.05 * (slot - 1), 0.0, -0.07)
    rejected = False
    message = ""
    try:
        CpuMirrorBatchSession(
            str(MODEL),
            1,
            point_frame_names=POINT_FRAMES,
            point_offsets=collinear,
            point_stable_ids=POINT_IDS,
            contact_query_slots=CONTACT_SLOTS,
            contact_stable_ids=CONTACT_IDS,
            support_patch_first_contact_slots=[0],
            support_patch_contact_counts=[4],
            support_patch_stable_ids=[PATCH_STABLE_ID],
            auto_rigid_support_basis=True,
        )
    except ValueError as error:
        rejected = "rank deficient" in str(error)
        message = str(error)
    return {"collinear_patch_rejected": rejected, "message": message, "pass": rejected}


def timing_profile(agents: int, repeats: int, profile: str) -> dict[str, Any]:
    if profile == "auto":
        session, case = auto_case(agents)
    elif profile == "explicit":
        session, case = make_case(agents, contact_kinematic_modes=RANK_MINIMAL_MODES)
    else:
        session, case = make_case(agents, contact_kinematic_modes=ALL_LOCKED_MODES)
    buffers = dynamic_buffers(session)
    for _ in range(4):
        run_dynamic(session, case, buffers)
    samples = np.empty(repeats, np.float64)
    calls = allocated = 0
    for index in range(repeats):
        result = run_dynamic(session, case, buffers)
        samples[index] = result["solve_execute_ns"][0]
        calls += int(result["allocation_calls"][0])
        allocated += int(result["allocated_bytes"][0])
    return {
        "solve_execute_ns": distribution(samples),
        "allocation_calls": calls,
        "allocated_bytes": allocated,
    }


def timing(repeats: int) -> list[dict[str, Any]]:
    rows = []
    for agents in (1, 8, 32):
        automatic = timing_profile(agents, repeats, "auto")
        explicit = timing_profile(agents, repeats, "explicit")
        locked = timing_profile(agents, repeats, "locked")
        rows.append({
            "batch_size": agents,
            "repeats_per_profile": repeats,
            "automatic": automatic,
            "explicit_r85": explicit,
            "all_locked": locked,
            "auto_vs_explicit_p50_percent": 100.0 * (
                automatic["solve_execute_ns"]["p50"]
                / explicit["solve_execute_ns"]["p50"] - 1.0
            ),
            "auto_vs_locked_p50_percent": 100.0 * (
                automatic["solve_execute_ns"]["p50"]
                / locked["solve_execute_ns"]["p50"] - 1.0
            ),
        })
    return rows


def report_markdown(metrics: dict[str, Any]) -> str:
    oracle = metrics["oracle"]
    timing_rows = [
        [
            row["batch_size"],
            f'{row["automatic"]["solve_execute_ns"]["p50"] / 1e3:.3f}',
            f'{row["explicit_r85"]["solve_execute_ns"]["p50"] / 1e3:.3f}',
            f'{row["all_locked"]["solve_execute_ns"]["p50"] / 1e3:.3f}',
            f'{row["auto_vs_explicit_p50_percent"]:+.2f}%',
            f'{row["auto_vs_locked_p50_percent"]:+.2f}%',
        ]
        for row in metrics["timing"]
    ]
    return f"""# Bonesaw automatic rigid-patch basis · r87

## Outcome

**Admission: {'PASS' if metrics['pass'] else 'FAIL'}.** Construction derives a deterministic six-row rigid-body basis from fixed same-frame contact geometry. The selected per-point modes enter the existing descriptor fingerprint; execution remains the zero-allocation R85 path. Python and Pinocchio independently rebuild rank, hard dynamics/contact, the implied full sole manifold, hull, weighted CoP, and support margin. No policy or physics rollout is used.

## Compiler-selected basis

{markdown_table(['signal', 'result'], [
    ['selected modes', str(oracle['contact_modes']) + ' · Disabled=0 Locked=1 Normal=2 Rolling=3'],
    ['rows / independent rank', f"{oracle['row_count']} / {oracle['rank_values']}"],
    ['minimum singular value', f"{oracle['minimum_singular_value']:.6f}"],
    ['implied all-point acceleration L∞', f"{oracle['maximum_implied_all_point_acceleration_linf']:.3e}"],
    ['explicit R85 modes / σmin', f"{oracle['explicit_r85_contact_modes']} / {oracle['explicit_r85_minimum_singular_value']:.6f}"],
    ['repeat descriptor / differs from all locked', f"{oracle['repeat_descriptor_exact']} / {oracle['descriptor_differs_from_all_locked']}"],
])}

The compiler enumerates only prefix-compatible Disabled/Normal/Rolling/Locked declarations totaling six scalar rows, rejects deficient candidates, and selects the candidate with the largest minimum singular value. Point coordinates are centered before selection, so a body-frame origin translation cannot change the choice.

## Independent physical oracle

{markdown_table(['signal', 'result'], [
    ['Pinocchio dynamics L∞', f"{oracle['pinocchio_dynamics_linf']:.3e}"],
    ['Pinocchio selected contact L∞', f"{oracle['pinocchio_contact_acceleration_linf']:.3e}"],
    ['independent minimum CoP margin', f"{oracle['minimum_oracle_support_margin_m']:.9f} m"],
    ['reported-vs-independent margin delta', f"{oracle['reported_margin_max_abs_delta_m']:.3e} m"],
    ['allocation calls / bytes', f"{oracle['allocation_calls']} / {oracle['allocated_bytes']}"],
])}

## Invalid geometry

{markdown_table(['case', 'result'], [
    ['four collinear same-frame points', metrics['invalid_geometry']['message']],
    ['typed rejection', str(metrics['invalid_geometry']['collinear_patch_rejected'])],
])}

## Solve timing (60 untrimmed calls/profile)

{markdown_table(['agents', 'auto p50 µs', 'explicit R85 p50 µs', 'all locked p50 µs', 'auto vs explicit', 'auto vs locked'], timing_rows)}

Construction-time enumeration is excluded from solve timing. Automatic and explicit bases can choose different legal lower-layer qdd/force optima, so this gate claims the same independently checked rigid-foot hard manifold—not bitwise cross-basis solutions.

## Deliberate boundary

R87 assumes one body frame and canonical frame X/Y/Z matching runtime tangent-X/tangent-Y/normal inputs. General authored contact frames, state-dependent switching, RollingWheel constants, CoM-in-polygon tasks, live finite-foot composition, and CUDA remain unavailable.
"""


def main() -> None:
    args = parse_args()
    oracle = oracle_case(args.samples)
    invalid = invalid_geometry_case()
    timing_rows = timing(args.timing_repeats)
    timing_pass = all(
        row[profile]["allocation_calls"] == 0
        and row[profile]["allocated_bytes"] == 0
        for row in timing_rows
        for profile in ("automatic", "explicit_r85", "all_locked")
    )
    metrics = {
        "schema": 1,
        "revision": "auto-rigid-patch-basis-r87",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "oracle": oracle,
        "invalid_geometry": invalid,
        "timing": timing_rows,
        "timing_allocation_pass": timing_pass,
        "pass": bool(oracle["pass"] and invalid["pass"] and timing_pass),
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "auto-rigid-patch-basis-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    markdown = report_markdown(metrics)
    (output / "AUTO_RIGID_PATCH_BASIS_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(
        render_report_html(markdown, title="Bonesaw automatic rigid-patch basis · r87")
    )
    print(json.dumps(metrics, indent=2))
    if not metrics["pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
