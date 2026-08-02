#!/usr/bin/env python3
"""Exercise live finite-foot and joint-stopping authority over WebSocket."""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np
from websocket import create_connection

from cpu_reference_report import (
    distribution,
    markdown_table as markdown_table_rows,
    render_report_html,
)
from live_authority_stream_report import receive_kind


def markdown_table(headers: list[str], rows: list[list[Any]]) -> str:
    return "\n".join(markdown_table_rows(headers, rows))


def safe_correlation(left: np.ndarray, right: np.ndarray) -> float | None:
    if np.std(left) == 0.0 or np.std(right) == 0.0:
        return None
    return float(np.corrcoef(left, right)[0, 1])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="ws://127.0.0.1:8797/ws")
    parser.add_argument("--samples", type=int, default=60)
    parser.add_argument("--lean-x", type=float, default=0.16)
    parser.add_argument("--output", default="benchmarks/results/live-flat-foot-authority-r90")
    parser.add_argument("--web-report", default="web/LIVE_FLAT_FOOT_AUTHORITY_R90.html")
    return parser.parse_args()


def frame_translation(message: dict[str, Any], frame_names: list[str], name: str) -> list[float]:
    frame_id = frame_names.index(name)
    frame = next(frame for frame in message["frames"] if frame["id"] == frame_id)
    return list(frame["translation"])


def run(url: str, samples: int, lean_x: float) -> dict[str, Any]:
    socket = create_connection(url, timeout=10)
    try:
        hello = receive_kind(socket, "hello", 1)
        contract = {
            signal["stable_id"]: signal["availability"]
            for signal in hello["authority_contract"]["signals"]
        }
        assert contract["finite_support"] == "measured"
        assert contract["joint_stopping"] == "measured"
        assert len(hello["support_patches"]) == 2
        assert all(len(patch["points"]) == 4 for patch in hello["support_patches"])
        assert {patch["stable_id"] for patch in hello["support_patches"]} == {8401, 8402}

        idle = receive_kind(socket, "state")
        assert idle["metrics"]["authority_profile"] == "kinematic_controller"
        squat_handle = hello["interaction_handles"][0]["frame"]
        squat_target = frame_translation(idle, hello["frame_names"], squat_handle)
        squat_target[2] -= 0.04
        socket.send(json.dumps({"type": "drag", "frame": squat_handle, "target": squat_target}))

        dynamic = None
        for _ in range(100):
            candidate = receive_kind(socket, "state")
            if candidate["metrics"]["authority_profile"] == "floating_dynamic_wbc":
                dynamic = candidate
                break
        assert dynamic is not None
        prelean_margin = float(dynamic["metrics"]["minimum_support_margin_m"])

        lean_target = frame_translation(dynamic, hello["frame_names"], "chest")
        lean_target[0] += lean_x
        socket.send(json.dumps({"type": "drag", "frame": "chest", "target": lean_target}))

        solve_us = np.empty(samples, np.float64)
        query_batch_us = np.empty(samples, np.float64)
        query_count = np.empty(samples, np.int64)
        support_margin = np.empty(samples, np.float64)
        stopping_margin = np.empty(samples, np.float64)
        effort = np.empty(samples, np.float64)
        hard_residual = np.empty(samples, np.float64)
        active_tasks = np.empty(samples, np.int64)
        task_pseudoinverse_calls = np.empty(samples, np.int64)
        task_jacobi_sweeps = np.empty(samples, np.int64)
        clipped_steps = np.empty(samples, np.int64)
        feasibility_projection_sweeps = np.empty(samples, np.int64)
        feasibility_polish_iterations = np.empty(samples, np.int64)
        limiting_patches: list[int] = []
        limiting_joints: list[int] = []
        statuses: dict[str, int] = {}
        for index in range(samples):
            state = receive_kind(socket, "state")
            metrics = state["metrics"]
            assert metrics["authority_profile"] == "floating_dynamic_wbc"
            solve_us[index] = metrics["solve_us"]
            query_batch_us[index] = metrics["query_batch_us"]
            query_count[index] = metrics["query_count"]
            support_margin[index] = metrics["minimum_support_margin_m"]
            stopping_margin[index] = metrics["minimum_joint_stopping_margin_rad_s2"]
            effort[index] = metrics["maximum_torque_utilization"]
            hard_residual[index] = max(
                metrics["dynamics_residual_linf"],
                metrics["contact_residual_linf"],
                metrics["maximum_constraint_violation"],
            )
            active_tasks[index] = sum(task["active"] for task in metrics["task_residuals"])
            task_pseudoinverse_calls[index] = metrics["task_pseudoinverse_calls"]
            task_jacobi_sweeps[index] = metrics["task_jacobi_sweeps"]
            clipped_steps[index] = metrics["clipped_steps"]
            feasibility_projection_sweeps[index] = metrics[
                "feasibility_projection_sweeps"
            ]
            feasibility_polish_iterations[index] = metrics[
                "feasibility_polish_iterations"
            ]
            limiting_patches.append(int(metrics["limiting_support_patch"]))
            limiting_joints.append(int(metrics["limiting_joint_stopping"]))
            statuses[state["status"]] = statuses.get(state["status"], 0) + 1
        socket.send(json.dumps({"type": "release"}))
        socket.send(json.dumps({"type": "reset"}))

        support_floor = min(patch["minimum_margin_m"] for patch in hello["support_patches"])
        passed = bool(
            np.all(np.isfinite(solve_us))
            and np.all(np.isfinite(query_batch_us))
            and np.all(query_count == 4)
            and np.min(support_margin) >= support_floor - 2e-8
            and np.min(stopping_margin) >= -2e-8
            and np.max(hard_residual) <= 1e-7
            and np.max(effort) <= 1.0 + 1e-8
            and set(limiting_patches) <= {8401, 8402}
            and np.min(active_tasks) >= 4
        )
        return {
            "pass": passed,
            "model": hello["model"],
            "contract_source": hello["authority_contract"]["source"],
            "samples": samples,
            "lean_x_m": lean_x,
            "support_patch_ids": [patch["stable_id"] for patch in hello["support_patches"]],
            "support_patch_point_counts": [len(patch["points"]) for patch in hello["support_patches"]],
            "requested_support_margin_m": support_floor,
            "prelean_support_margin_m": prelean_margin,
            "minimum_support_margin_m": float(np.min(support_margin)),
            "final_support_margin_m": float(support_margin[-1]),
            "support_margin_pressure_ticks": int(np.sum(support_margin < 0.03)),
            "limiting_patch_counts": {
                str(patch): limiting_patches.count(patch) for patch in sorted(set(limiting_patches))
            },
            "minimum_joint_stopping_margin_rad_s2": float(np.min(stopping_margin)),
            "limiting_joint_counts": {
                str(joint): limiting_joints.count(joint) for joint in sorted(set(limiting_joints))
            },
            "maximum_effort_utilization": float(np.max(effort)),
            "maximum_hard_residual": float(np.max(hard_residual)),
            "active_task_count_range": [int(np.min(active_tasks)), int(np.max(active_tasks))],
            "status_counts": statuses,
            "solve_us": distribution(solve_us),
            "query_batch_us": distribution(query_batch_us),
            "query_count_range": [int(np.min(query_count)), int(np.max(query_count))],
            "query_budget_overrun_count": int(np.sum(solve_us > 5_000.0)),
            "frame_batch_budget_overrun_count": int(np.sum(query_batch_us > 20_000.0)),
            "solver_work": {
                "task_pseudoinverse_calls": distribution(task_pseudoinverse_calls),
                "task_jacobi_sweeps": distribution(task_jacobi_sweeps),
                "clipped_steps": distribution(clipped_steps),
                "feasibility_projection_sweeps": distribution(
                    feasibility_projection_sweeps
                ),
                "feasibility_polish_iterations": distribution(
                    feasibility_polish_iterations
                ),
            },
            "solve_time_correlation": {
                "task_pseudoinverse_calls": safe_correlation(
                    solve_us, task_pseudoinverse_calls
                ),
                "task_jacobi_sweeps": safe_correlation(solve_us, task_jacobi_sweeps),
                "clipped_steps": safe_correlation(solve_us, clipped_steps),
                "feasibility_projection_sweeps": safe_correlation(
                    solve_us, feasibility_projection_sweeps
                ),
                "feasibility_polish_iterations": safe_correlation(
                    solve_us, feasibility_polish_iterations
                ),
            },
        }
    finally:
        socket.close()


def report_markdown(metrics: dict[str, Any]) -> str:
    timing = metrics["solve_us"]
    batch_timing = metrics["query_batch_us"]
    return f"""# Bonesaw live finite-foot authority · r90

## Outcome

**Admission: {'PASS' if metrics['pass'] else 'FAIL'}.** A separate flat-foot browser adapter uses the toy humanoid's authored mass, inertia, limits, and collision geometry. Rust construction emits eight force points, two exact 20 mm CoP patches, and two R87-derived six-row rigid-foot bases. Every state-local raw-WBC query composes the R83 position/velocity/braking envelope before solving. Python drives the same WebSocket commands as the browser and scores state frames; guided pose updates deliberately avoid a policy, external physics, or synthetic body-response claim.

## Live authority transition

{markdown_table(['signal', 'result'], [
    ['profile', metrics['contract_source']],
    ['support patches / force points', f"{metrics['support_patch_ids']} / {metrics['support_patch_point_counts']}"],
    ['commanded chest lean', f"{metrics['lean_x_m']:.3f} m"],
    ['support margin: before / minimum / final', f"{metrics['prelean_support_margin_m']*1000:.3f} / {metrics['minimum_support_margin_m']*1000:.3f} / {metrics['final_support_margin_m']*1000:.3f} mm"],
    ['ticks below 30 mm optional reserve', metrics['support_margin_pressure_ticks']],
    ['limiting patch counts', str(metrics['limiting_patch_counts'])],
    ['minimum solved joint-stopping reserve', f"{metrics['minimum_joint_stopping_margin_rad_s2']:.6f} rad/s²"],
    ['maximum effort utilization', f"{100*metrics['maximum_effort_utilization']:.3f}%"],
    ['maximum hard residual', f"{metrics['maximum_hard_residual']:.3e}"],
    ['active task count range', str(metrics['active_task_count_range'])],
    ['status counts', str(metrics['status_counts'])],
])}

Support and joint stopping remain separate Viability rows. Effort, hard residual, task residuals, solver work, and model availability remain independent; there is no aggregate health verdict.

## End-to-end query and batch timing

{markdown_table(['scope', 'count', 'p50 µs', 'p95 µs', 'p99 µs', 'max µs', 'budget overruns'], [
[
    'maximum single query per streamed frame',
    metrics['samples'],
    f"{timing['p50']:.3f}",
    f"{timing['p95']:.3f}",
    f"{timing['p99']:.3f}",
    f"{timing['maximum']:.3f}",
    f"{metrics['query_budget_overrun_count']} / 5 ms",
], [
    'sum of four 5 ms subqueries',
    metrics['samples'],
    f"{batch_timing['p50']:.3f}",
    f"{batch_timing['p95']:.3f}",
    f"{batch_timing['p99']:.3f}",
    f"{batch_timing['maximum']:.3f}",
    f"{metrics['frame_batch_budget_overrun_count']} / 20 ms",
]])}

`solve_us` now has one unambiguous contract: it is the maximum end-to-end single-query duration among the four 5 ms subqueries represented by a 50 Hz streamed frame. `query_batch_us` is their sum and is compared with 20 ms. Transport, JSON serialization, and browser rendering are outside both measurements. Physical admission does not erase either compute-budget witness.

## Solver-work attribution

{markdown_table(['work counter', 'p50', 'p99', 'max', 'correlation with solve time'], [
    [name, f"{values['p50']:.1f}", f"{values['p99']:.1f}", f"{values['maximum']:.1f}", 'constant' if metrics['solve_time_correlation'][name] is None else f"{metrics['solve_time_correlation'][name]:.3f}"]
    for name, values in metrics['solver_work'].items()
])}

These counters distinguish task-level dense pseudoinverse/Jacobi work from hard-feasibility projection and active-set polishing. They are diagnostic witnesses, not iteration budgets to tune until timing turns green.

## Deliberate boundary

The toy geometry is a CPU concept fixture, not hardware calibration. The guided editor queries instantaneous feasibility at each commanded pose and does not integrate Bonesaw acceleration as a physics surrogate. An unguided 200 Hz self-integration experiment drifted into a hard-row numerical failure under sustained lean; it is not promoted and remains a required closed-loop/physics eval. General contact switching, calibrated actuator realization, thermal state, a production non-wheeled model, and CUDA remain unavailable.
"""


def main() -> None:
    args = parse_args()
    result = run(args.url, args.samples, args.lean_x)
    metrics = {
        "schema": 1,
        "revision": "live-flat-foot-authority-r90",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        **result,
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "live-flat-foot-authority-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    markdown = report_markdown(metrics)
    (output / "LIVE_FLAT_FOOT_AUTHORITY_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(
        render_report_html(markdown, title="Bonesaw live finite-foot authority · r90")
    )
    print(json.dumps(metrics, indent=2))
    if not metrics["pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
