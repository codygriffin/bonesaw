#!/usr/bin/env python3
"""Audit the CPU floating world-SDF acceleration barrier without a policy or plant."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import WorldSdfBarrierSession
from cpu_reference_report import distribution, markdown_table, render_report_html


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/tight_avoidance_toy.urdf")
    parser.add_argument("--repeats", type=int, default=2000)
    parser.add_argument("--hard-margin", type=float, default=0.02)
    parser.add_argument("--influence-margin", type=float, default=0.10)
    parser.add_argument("--frequency-hz", type=float, default=2.0)
    parser.add_argument("--damping-ratio", type=float, default=1.0)
    parser.add_argument("--wall-y", type=float, default=0.55)
    parser.add_argument(
        "--output", default="benchmarks/results/world-sdf-barrier-r109"
    )
    parser.add_argument("--web-report", default="web/WORLD_SDF_BARRIER_R109.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def make_plane_field(wall_y: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    origin = np.array([-1.0, -1.0, -1.0], dtype=np.float64)
    spacing = np.array([0.1, 0.1, 0.1], dtype=np.float64)
    dimensions_xyz = np.array([21, 21, 21], dtype=np.int64)
    world_y = origin[1] + spacing[1] * np.arange(dimensions_xyz[1])
    values = np.empty(
        (dimensions_xyz[2], dimensions_xyz[1], dimensions_xyz[0]),
        dtype=np.float64,
    )
    values[:] = (wall_y - world_y)[None, :, None]
    return values, origin, spacing


def buffers(generalized_dof: int) -> dict[str, np.ndarray]:
    return {
        "qdd": np.zeros(generalized_dof, dtype=np.float64),
        "evidence": np.zeros(8, dtype=np.float64),
        "probe_ids": np.zeros(2, dtype=np.uint32),
        "body_ids": np.zeros(2, dtype=np.uint32),
        "counts": np.zeros(3, dtype=np.uint32),
        "source": np.zeros(2, dtype=np.uint8),
        "proxy_quality": np.zeros(2, dtype=np.uint8),
        "status": np.zeros(1, dtype=np.uint8),
    }


def query(
    session: WorldSdfBarrierSession,
    q: np.ndarray,
    v: np.ndarray,
    root_translation: np.ndarray,
    root_twist: np.ndarray,
    desired: np.ndarray,
    output: dict[str, np.ndarray],
) -> tuple[int, int, int]:
    return session.evaluate_into(
        q,
        v,
        root_translation,
        root_twist,
        desired,
        output["qdd"],
        output["evidence"],
        output["probe_ids"],
        output["body_ids"],
        output["counts"],
        output["source"],
        output["proxy_quality"],
        output["status"],
    )


def semantic_digest(output: dict[str, np.ndarray]) -> str:
    digest = hashlib.sha256()
    for name in sorted(output):
        digest.update(name.encode())
        digest.update(output[name].tobytes())
    return digest.hexdigest()


def session(
    args: argparse.Namespace,
    values: np.ndarray,
    origin: np.ndarray,
    spacing: np.ndarray,
    *,
    enabled: bool,
    outside_policy: int,
) -> WorldSdfBarrierSession:
    return WorldSdfBarrierSession(
        args.model,
        values,
        origin,
        spacing,
        outside_policy=outside_policy,
        enabled=enabled,
        hard_margin_m=args.hard_margin,
        influence_margin_m=args.influence_margin,
        natural_frequency_hz=args.frequency_hz,
        damping_ratio=args.damping_ratio,
        maximum_acceleration=200.0,
        maximum_torque=2000.0,
    )


def benchmark(
    active: WorldSdfBarrierSession,
    repeats: int,
    desired: np.ndarray,
) -> dict[str, Any]:
    output = buffers(active.dof + 6)
    q = np.array([0.12], dtype=np.float64)
    v = np.array([0.31], dtype=np.float64)
    root_translation = np.array([0.0, 0.01, 0.0], dtype=np.float64)
    root_twist = np.array([0.0, 0.0, 0.0, 0.0, 0.04, 0.0], dtype=np.float64)
    for _ in range(50):
        query(active, q, v, root_translation, root_twist, desired, output)
    elapsed_us = np.zeros(repeats, dtype=np.float64)
    allocation_calls = np.zeros(repeats, dtype=np.uint64)
    allocated_bytes = np.zeros(repeats, dtype=np.uint64)
    replay_digest = None
    exact_replay = True
    for repeat in range(repeats):
        elapsed_ns, calls, byte_count = query(
            active, q, v, root_translation, root_twist, desired, output
        )
        elapsed_us[repeat] = elapsed_ns * 1e-3
        allocation_calls[repeat] = calls
        allocated_bytes[repeat] = byte_count
        digest = semantic_digest(output)
        if replay_digest is None:
            replay_digest = digest
        else:
            exact_replay &= digest == replay_digest
    return {
        "timing_us": distribution(elapsed_us),
        "maximum_allocation_calls": int(np.max(allocation_calls)),
        "maximum_allocated_bytes": int(np.max(allocated_bytes)),
        "semantic_replay_exact": bool(exact_replay),
        "semantic_digest": replay_digest,
    }


def boundary_policy_audit(
    args: argparse.Namespace,
    values: np.ndarray,
    origin: np.ndarray,
    spacing: np.ndarray,
) -> dict[str, Any]:
    desired = np.zeros(7, dtype=np.float64)
    q = np.zeros(1, dtype=np.float64)
    v = np.zeros(1, dtype=np.float64)
    root_twist = np.zeros(6, dtype=np.float64)
    reject = session(
        args, values, origin, spacing, enabled=True, outside_policy=0
    )
    output = buffers(7)
    rejected = False
    try:
        query(
            reject,
            q,
            v,
            np.array([1.3, 0.0, 0.0], dtype=np.float64),
            root_twist,
            desired,
            output,
        )
    except ValueError as error:
        rejected = "known field" in str(error)

    occupied = session(
        args, values, origin, spacing, enabled=True, outside_policy=1
    )
    query(
        occupied,
        q,
        v,
        np.array([1.3, 0.0, 0.0], dtype=np.float64),
        root_twist,
        desired,
        output,
    )
    occupied_source = int(output["source"][0])
    occupied_distance = float(output["evidence"][0])
    occupied_active = int(output["counts"][2])
    occupied_status = int(output["status"][0])

    nonfinite_rejected = False
    invalid = values.copy()
    invalid[0, 0, 0] = np.nan
    try:
        session(args, invalid, origin, spacing, enabled=True, outside_policy=0)
    except ValueError as error:
        nonfinite_rejected = "grid" in str(error).lower()
    return {
        "reject_policy_withheld_outside_query": rejected,
        "occupied_boundary_source_code": occupied_source,
        "occupied_boundary_minimum_distance_m": occupied_distance,
        "occupied_boundary_active_probe_count": occupied_active,
        "occupied_boundary_status_code": occupied_status,
        "deep_outside_query_not_admitted": occupied_status not in {0, 1},
        "nonfinite_grid_rejected": nonfinite_rejected,
    }


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    values, origin, spacing = make_plane_field(args.wall_y)
    active = session(args, values, origin, spacing, enabled=True, outside_policy=0)
    disabled = session(args, values, origin, spacing, enabled=False, outside_policy=0)
    if active.dof != 1 or active.probe_count <= 1 or disabled.probe_count != 0:
        raise RuntimeError("r109 fixture or world-probe layout changed")

    generalized_dof = active.dof + 6
    coordinates = np.linspace(-0.08, 0.20, 321, dtype=np.float64)
    phase = np.linspace(0.0, 6.0 * np.pi, coordinates.size)
    root_y = 0.02 * np.sin(phase)
    root_y_velocity = 0.07 * np.cos(phase)
    joint_velocity = 0.35 * np.cos(0.7 * phase)
    desired = np.zeros(generalized_dof, dtype=np.float64)
    desired[6] = 60.0
    active_output = buffers(generalized_dof)
    disabled_output = buffers(generalized_dof)

    distance_error = np.zeros(coordinates.size, dtype=np.float64)
    velocity_error = np.zeros(coordinates.size, dtype=np.float64)
    gradient_error = np.zeros(coordinates.size, dtype=np.float64)
    required_error = np.zeros(coordinates.size, dtype=np.float64)
    active_count_error = np.zeros(coordinates.size, dtype=np.int64)
    active_residuals: list[float] = []
    active_achieved_minus_required: list[float] = []
    active_qdd_delta: list[float] = []
    status_codes: set[int] = set()
    closest_probe_ids: set[int] = set()
    closest_body_ids: set[int] = set()
    omega = 2.0 * np.pi * args.frequency_hz

    for index, coordinate in enumerate(coordinates):
        q = np.array([coordinate], dtype=np.float64)
        v = np.array([joint_velocity[index]], dtype=np.float64)
        root_translation = np.array([0.0, root_y[index], 0.0], dtype=np.float64)
        root_twist = np.array(
            [0.0, 0.0, 0.0, 0.0, root_y_velocity[index], 0.0],
            dtype=np.float64,
        )
        query(active, q, v, root_translation, root_twist, desired, active_output)
        query(disabled, q, v, root_translation, root_twist, desired, disabled_output)
        status_codes.add(int(active_output["status"][0]))
        closest_probe_ids.add(int(active_output["probe_ids"][0]))
        closest_body_ids.add(int(active_output["body_ids"][0]))
        expected_distance = (
            args.wall_y - (root_y[index] + 0.3 + coordinate) - 0.1
        )
        expected_margin = expected_distance - args.hard_margin
        expected_velocity = -(root_y_velocity[index] + joint_velocity[index])
        expected_active = int(expected_distance < args.influence_margin)
        distance_error[index] = abs(
            active_output["evidence"][0] - expected_distance
        )
        gradient_error[index] = abs(active_output["evidence"][2] - 1.0)
        active_count_error[index] = abs(
            int(active_output["counts"][2]) - expected_active
        )
        if expected_active:
            expected_required = (
                -2.0 * args.damping_ratio * omega * expected_velocity
                - omega * omega * expected_margin
            )
            velocity_error[index] = abs(
                active_output["evidence"][3] - expected_velocity
            )
            required_error[index] = abs(
                active_output["evidence"][5] - expected_required
            )
            active_residuals.append(float(active_output["evidence"][7]))
            active_achieved_minus_required.append(
                float(active_output["evidence"][6] - active_output["evidence"][5])
            )
            active_qdd_delta.append(
                float(active_output["qdd"][6] - disabled_output["qdd"][6])
            )
            if active_output["source"][1] != 0:
                raise RuntimeError("active plane barrier lost trilinear provenance")
        elif active_output["probe_ids"][1] != np.iinfo(np.uint32).max:
            raise RuntimeError("inactive world state reported a limiting active probe")

    residuals = np.asarray(active_residuals, dtype=np.float64)
    achieved = np.asarray(active_achieved_minus_required, dtype=np.float64)
    qdd_delta = np.asarray(active_qdd_delta, dtype=np.float64)
    benchmark_result = benchmark(active, args.repeats, desired)
    boundary = boundary_policy_audit(args, values, origin, spacing)
    timing = benchmark_result["timing_us"]
    passed = bool(
        status_codes.issubset({0, 1})
        and len(closest_probe_ids) == 1
        and len(closest_body_ids) == 1
        and np.max(distance_error) < 1e-12
        and np.max(velocity_error) < 1e-12
        and np.max(gradient_error) < 1e-12
        and np.max(required_error) < 1e-9
        and np.max(active_count_error) == 0
        and np.min(residuals) >= -1e-7
        and np.min(achieved) >= -1e-7
        and np.max(np.abs(qdd_delta)) > 1.0
        and boundary["reject_policy_withheld_outside_query"]
        and boundary["occupied_boundary_source_code"] == 1
        and boundary["occupied_boundary_minimum_distance_m"] < 0.0
        and boundary["occupied_boundary_active_probe_count"] > 0
        and boundary["deep_outside_query_not_admitted"]
        and boundary["nonfinite_grid_rejected"]
        and benchmark_result["semantic_replay_exact"]
        and benchmark_result["maximum_allocation_calls"] == 0
        and benchmark_result["maximum_allocated_bytes"] == 0
        and timing["p99"] < 5_000.0
        and timing["maximum"] < 50_000.0
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free floating WBC queries against an immutable dense world SDF",
        "model": args.model,
        "grid_dimensions_xyz": [values.shape[2], values.shape[1], values.shape[0]],
        "grid_spacing_m": spacing.tolist(),
        "wall_y_m": args.wall_y,
        "coordinate_samples": int(coordinates.size),
        "active_samples": int(residuals.size),
        "represented_probe_count": active.probe_count,
        "closest_probe_ids": sorted(closest_probe_ids),
        "closest_body_ids": sorted(closest_body_ids),
        "hard_margin_m": args.hard_margin,
        "influence_margin_m": args.influence_margin,
        "natural_frequency_hz": args.frequency_hz,
        "damping_ratio": args.damping_ratio,
        "maximum_distance_oracle_error_m": float(np.max(distance_error)),
        "maximum_relative_velocity_oracle_error_mps": float(np.max(velocity_error)),
        "maximum_gradient_norm_error": float(np.max(gradient_error)),
        "maximum_required_acceleration_oracle_error_mps2": float(
            np.max(required_error)
        ),
        "maximum_active_count_error": int(np.max(active_count_error)),
        "minimum_barrier_residual_mps2": float(np.min(residuals)),
        "minimum_achieved_minus_required_mps2": float(np.min(achieved)),
        "maximum_enabled_minus_disabled_joint_acceleration_mps2": float(
            np.max(np.abs(qdd_delta))
        ),
        "status_codes": sorted(status_codes),
        "boundary_policy": boundary,
        "benchmark": benchmark_result,
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    timing = audit["benchmark"]["timing_us"]
    boundary = audit["boundary_policy"]
    table = "\n".join(
        markdown_table(
            ["query", "p50 / p99 / max µs", "alloc calls / bytes", "exact replay"],
            [[
                "floating world-SDF barrier",
                f"{timing['p50']:.3f} / {timing['p99']:.3f} / {timing['maximum']:.3f}",
                f"{audit['benchmark']['maximum_allocation_calls']} / {audit['benchmark']['maximum_allocated_bytes']}",
                audit["benchmark"]["semantic_replay_exact"],
            ]],
        )
    )
    return f"""# Bonesaw CPU world-SDF collision authority · r109

## Outcome

**{audit['status'].upper()}.** R109 adds an immutable dense signed-distance grid to the CPU core and samples deterministic body-attached sphere probes against it inside the floating WBC. The independent NumPy plane oracle evaluates {audit['coordinate_samples']} caller-authored states without a policy, integration, contact response, or external physics; {audit['active_samples']} states activate the world barrier.

Distance, relative-velocity, gradient-norm, and required-acceleration oracle errors are `{audit['maximum_distance_oracle_error_m']:.3e}` m, `{audit['maximum_relative_velocity_oracle_error_mps']:.3e}` m/s, `{audit['maximum_gradient_norm_error']:.3e}`, and `{audit['maximum_required_acceleration_oracle_error_mps2']:.3e}` m/s². The minimum post-solve barrier residual is `{audit['minimum_barrier_residual_mps2']:.3e}` m/s².

{table}

The barrier changes joint acceleration by as much as **{audit['maximum_enabled_minus_disabled_joint_acceleration_mps2']:.3f} m/s²** relative to the world-disabled query. This is state-local authority, not simulated obstacle response.

## Unknown-space contract

Every grid node must be finite at construction. The Reject policy withheld the out-of-volume query: **{boundary['reject_policy_withheld_outside_query']}**. The OccupiedBoundary policy instead produced source code `{boundary['occupied_boundary_source_code']}`, minimum clearance `{boundary['occupied_boundary_minimum_distance_m']:.3f}` m, and {boundary['occupied_boundary_active_probe_count']} active probes. The deliberately deep outside fault returned bounded-solve status `{boundary['occupied_boundary_status_code']}` and was not admitted: **{boundary['deep_outside_query_not_admitted']}**. A non-finite grid was rejected: **{boundary['nonfinite_grid_rejected']}**. Unknown space never silently becomes free.

## Barrier semantics

Trilinear interpolation returns both the scalar SDF and its analytic, unnormalized gradient, so the emitted Jacobian is the derivative of the reported scalar rather than a cosmetically normalized normal. For `h = sdf(center) - sphere_radius - hard_margin`, Rust emits `J qdd + bias + 2 ζ ω hdot + ω² h >= 0`. `bias` includes the current gradient projection of rigid-point `Jdot·v`; SDF Hessian curvature and voxel-feature switching remain excluded and explicitly named.

## Authority boundary

World collision is separately typed from self-collision, support, joint stopping, effort, solver budget, sampled/adaptive commanded-segment admission, unknown-space policy, and realized plant response. Sphere covers are conservative robot probes, not mesh CCD. No aggregate health score is produced.

## Remaining work

The world barrier must next be streamed as its own live authority row and paired with commanded-segment world validation. Dynamic SDF updates, external obstacle slots, mesh validation, SDF Hessian/rate bounds, and calibrated plant response remain separate milestones.
"""


def main() -> None:
    args = parse_args()
    if (
        args.repeats <= 1
        or not np.isfinite(args.hard_margin)
        or not np.isfinite(args.influence_margin)
        or args.influence_margin < args.hard_margin
        or not np.isfinite(args.frequency_hz)
        or args.frequency_hz <= 0.0
        or not np.isfinite(args.damping_ratio)
        or args.damping_ratio < 0.0
        or not np.isfinite(args.wall_y)
    ):
        raise SystemExit("invalid benchmark count, wall, or barrier configuration")
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = evaluate(args)
    sources = [
        pathlib.Path("crates/bonesaw-core/src/world_collision.rs"),
        pathlib.Path("crates/bonesaw-core/src/dynamic_wbc.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path("python/bonesaw/__init__.py"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "world-sdf-barrier-r109",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [model, *sources]},
        "audit": audit,
    }
    (output / "world-sdf-barrier-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "WORLD_SDF_BARRIER_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw CPU world-SDF collision authority · r109")
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
