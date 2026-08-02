#!/usr/bin/env python3
"""Compare shared tight-primitive avoidance with the explicit sphere fallback."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import CollisionAvoidanceSession
from cpu_reference_report import distribution, markdown_table, render_report_html


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/tight_avoidance_toy.urdf")
    parser.add_argument("--repeats", type=int, default=1000)
    parser.add_argument("--hard-margin", type=float, default=0.02)
    parser.add_argument("--influence-margin", type=float, default=0.10)
    parser.add_argument(
        "--output", default="benchmarks/results/tight-primitive-avoidance-r107"
    )
    parser.add_argument(
        "--web-report", default="web/TIGHT_PRIMITIVE_AVOIDANCE_R107.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def buffers(pair_count: int, dof: int) -> dict[str, np.ndarray]:
    return {
        "pair_ids": np.zeros(pair_count, dtype=np.uint32),
        "quality": np.zeros(pair_count, dtype=np.uint8),
        "distance": np.zeros(pair_count, dtype=np.float64),
        "normal": np.zeros((pair_count, 3), dtype=np.float64),
        "point_a": np.zeros((pair_count, 3), dtype=np.float64),
        "point_b": np.zeros((pair_count, 3), dtype=np.float64),
        "jacobian": np.zeros((pair_count, dof), dtype=np.float64),
    }


def query(
    session: CollisionAvoidanceSession,
    tight: bool,
    q: np.ndarray,
    v: np.ndarray,
    output: dict[str, np.ndarray],
) -> tuple[int, int, int]:
    return session.evaluate_into(
        tight,
        q,
        v,
        output["pair_ids"],
        output["quality"],
        output["distance"],
        output["normal"],
        output["point_a"],
        output["point_b"],
        output["jacobian"],
    )


def semantic_digest(output: dict[str, np.ndarray]) -> str:
    digest = hashlib.sha256()
    for name in sorted(output):
        digest.update(name.encode())
        digest.update(output[name].tobytes())
    return digest.hexdigest()


def benchmark(
    session: CollisionAvoidanceSession,
    tight: bool,
    q: np.ndarray,
    v: np.ndarray,
    repeats: int,
) -> dict[str, Any]:
    pair_count = session.tight_pair_count if tight else session.sphere_cover_pair_count
    output = buffers(pair_count, session.dof)
    for _ in range(20):
        query(session, tight, q, v, output)
    elapsed_us = np.zeros(repeats, dtype=np.float64)
    allocation_calls = np.zeros(repeats, dtype=np.uint64)
    allocated_bytes = np.zeros(repeats, dtype=np.uint64)
    reference_digest = None
    exact_replay = True
    for repeat in range(repeats):
        elapsed_ns, calls, byte_count = query(session, tight, q, v, output)
        elapsed_us[repeat] = elapsed_ns * 1e-3
        allocation_calls[repeat] = calls
        allocated_bytes[repeat] = byte_count
        digest = semantic_digest(output)
        if reference_digest is None:
            reference_digest = digest
        else:
            exact_replay &= digest == reference_digest
    return {
        "pair_count": pair_count,
        "timing_us": distribution(elapsed_us),
        "maximum_allocation_calls": int(np.max(allocation_calls)),
        "maximum_allocated_bytes": int(np.max(allocated_bytes)),
        "semantic_replay_exact": bool(exact_replay),
        "semantic_digest": reference_digest,
    }


def evaluate(
    model: pathlib.Path,
    repeats: int,
    hard_margin: float,
    influence_margin: float,
) -> dict[str, Any]:
    session = CollisionAvoidanceSession(str(model))
    if session.dof != 1:
        raise RuntimeError("the fixed R107 fixture must have exactly one coordinate")
    tight_output = buffers(session.tight_pair_count, session.dof)
    cover_output = buffers(session.sphere_cover_pair_count, session.dof)
    q_values = np.linspace(-0.12, 0.20, 321, dtype=np.float64)
    tight_distance = np.zeros(q_values.size, dtype=np.float64)
    cover_distance = np.zeros(q_values.size, dtype=np.float64)
    tight_jacobian = np.zeros(q_values.size, dtype=np.float64)
    normal_error = np.zeros(q_values.size, dtype=np.float64)
    witness_error = np.zeros(q_values.size, dtype=np.float64)
    finite_difference_error = np.zeros(q_values.size, dtype=np.float64)
    velocity = np.array([0.37], dtype=np.float64)
    epsilon = 1e-7
    shifted_output = buffers(session.tight_pair_count, session.dof)
    for index, coordinate in enumerate(q_values):
        q = np.array([coordinate], dtype=np.float64)
        query(session, True, q, velocity, tight_output)
        query(session, False, q, velocity, cover_output)
        tight_distance[index] = tight_output["distance"][0]
        cover_distance[index] = float(np.min(cover_output["distance"]))
        tight_jacobian[index] = tight_output["jacobian"][0, 0]
        normal_error[index] = abs(np.linalg.norm(tight_output["normal"][0]) - 1.0)
        witness_error[index] = abs(
            np.linalg.norm(tight_output["point_b"][0] - tight_output["point_a"][0])
            - tight_distance[index]
        )
        query(
            session,
            True,
            np.array([coordinate + epsilon], dtype=np.float64),
            np.zeros(1, dtype=np.float64),
            shifted_output,
        )
        finite_difference = (
            shifted_output["distance"][0] - tight_distance[index]
        ) / epsilon
        finite_difference_error[index] = abs(
            finite_difference - tight_jacobian[index]
        )

    analytic_distance = 0.15 + q_values
    false_influence = np.logical_and(
        cover_distance < influence_margin, tight_distance >= influence_margin
    )
    false_collision = np.logical_and(
        cover_distance < hard_margin, tight_distance >= hard_margin
    )
    tight_benchmark = benchmark(
        session,
        True,
        np.array([0.0], dtype=np.float64),
        velocity,
        repeats,
    )
    cover_benchmark = benchmark(
        session,
        False,
        np.array([0.0], dtype=np.float64),
        velocity,
        repeats,
    )
    maximum_distance_error = float(np.max(np.abs(tight_distance - analytic_distance)))
    maximum_jacobian_error = float(np.max(finite_difference_error))
    passed = bool(
        session.tight_pair_count == 1
        and session.sphere_cover_pair_count == 3
        and maximum_distance_error < 1e-12
        and maximum_jacobian_error < 1e-8
        and float(np.max(normal_error)) < 1e-12
        and float(np.max(witness_error)) < 1e-12
        and np.count_nonzero(false_influence) > 0
        and np.count_nonzero(false_collision) > 0
        and tight_benchmark["semantic_replay_exact"]
        and cover_benchmark["semantic_replay_exact"]
        and tight_benchmark["maximum_allocation_calls"] == 0
        and tight_benchmark["maximum_allocated_bytes"] == 0
        and cover_benchmark["maximum_allocation_calls"] == 0
        and cover_benchmark["maximum_allocated_bytes"] == 0
        and tight_benchmark["timing_us"]["maximum"] < 5_000.0
        and cover_benchmark["timing_us"]["maximum"] < 5_000.0
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free shared tight-primitive distance/Jacobian query",
        "model": str(model),
        "hard_margin_m": hard_margin,
        "influence_margin_m": influence_margin,
        "coordinate_sweep": {
            "minimum": float(q_values[0]),
            "maximum": float(q_values[-1]),
            "samples": int(q_values.size),
        },
        "tight_pair_count": session.tight_pair_count,
        "sphere_cover_pair_count": session.sphere_cover_pair_count,
        "tight_minimum_distance_m": float(np.min(tight_distance)),
        "sphere_cover_minimum_distance_m": float(np.min(cover_distance)),
        "maximum_sphere_cover_overreach_m": float(
            np.max(tight_distance - cover_distance)
        ),
        "false_influence_samples": int(np.count_nonzero(false_influence)),
        "false_collision_samples": int(np.count_nonzero(false_collision)),
        "maximum_analytic_distance_error_m": maximum_distance_error,
        "maximum_finite_difference_jacobian_error": maximum_jacobian_error,
        "maximum_normal_norm_error": float(np.max(normal_error)),
        "maximum_surface_witness_error_m": float(np.max(witness_error)),
        "tight_quality_codes": sorted(int(value) for value in np.unique(tight_output["quality"])),
        "tight": tight_benchmark,
        "sphere_cover_fallback": cover_benchmark,
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    rows = []
    for name in ["tight", "sphere_cover_fallback"]:
        result = audit[name]
        timing = result["timing_us"]
        rows.append(
            [
                name,
                result["pair_count"],
                f"{timing['p50']:.3f} / {timing['p99']:.3f} / {timing['maximum']:.3f}",
                f"{result['maximum_allocation_calls']} / {result['maximum_allocated_bytes']}",
                result["semantic_replay_exact"],
            ]
        )
    comparison = "\n".join(
        markdown_table(
            ["representation", "pairs", "p50 / p99 / max µs", "alloc calls / bytes", "exact replay"],
            rows,
        )
    )
    return f"""# Bonesaw shared tight-primitive avoidance · r107

## Outcome

**{audit['status'].upper()}.** R107 derives the differential avoidance row from the same tight primitive pair and signed-distance routine used by command admission. The witness carries a deterministic normal, closest surface points, locally selected rigid-body feature points, stable pair ID, distance quality, relative normal velocity, and analytic Jacobian. The previous sphere-cover evaluator remains a separately named fallback API.

The fixed box–sphere fixture sweeps {audit['coordinate_sweep']['samples']} states without a policy or physics rollout. Tight geometry has {audit['tight_pair_count']} pair; the cover needs {audit['sphere_cover_pair_count']}. Maximum cover overreach is **{audit['maximum_sphere_cover_overreach_m'] * 1e3:.3f} mm**. It creates {audit['false_influence_samples']} false influence samples and {audit['false_collision_samples']} false hard-collision samples that the shared primitive witness does not report.

{comparison}

The analytic distance matches the fixture oracle within `{audit['maximum_analytic_distance_error_m']:.3e}` m. Its Jacobian matches a finite-difference distance derivative within `{audit['maximum_finite_difference_jacobian_error']:.3e}`; normal and surface-witness errors are `{audit['maximum_normal_norm_error']:.3e}` and `{audit['maximum_surface_witness_error_m']:.3e}` m. Both representations replay exactly and allocate zero calls/bytes inside the timed Rust query.

## Authority boundary

This removes representation disagreement between avoidance and command admission for supported spheres, capsules, cylinders-as-capsules, and boxes. It does not make the browser pose a realized plant, and it does not aggregate avoidance pressure with support, effort, thermal, solver, or command-selection authority. Box–box still reports the conservative separating-axis lower bound and labels that quality explicitly.

## Remaining work

The fixed-root kinematic controller now consumes shared primitive rows; the floating dynamic WBC still needs the corresponding acceleration-level barrier and typed active-pair telemetry. World geometry, authored exclusion matrices, mesh CCD, closest-feature continuous-rate bounds, contact response, and calibrated plant behavior remain separate milestones.
"""


def main() -> None:
    args = parse_args()
    if (
        args.repeats <= 1
        or not np.isfinite(args.hard_margin)
        or not np.isfinite(args.influence_margin)
        or args.hard_margin < 0.0
        or args.influence_margin < args.hard_margin
    ):
        raise SystemExit("invalid repeats or collision margins")
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = evaluate(model, args.repeats, args.hard_margin, args.influence_margin)
    sources = [
        pathlib.Path("crates/bonesaw-core/src/collision.rs"),
        pathlib.Path("crates/bonesaw-core/src/controller.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path("python/bonesaw/__init__.py"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "tight-primitive-avoidance-r107",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [model, *sources]},
        "audit": audit,
    }
    (output / "tight-primitive-avoidance-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "TIGHT_PRIMITIVE_AVOIDANCE_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw shared tight-primitive avoidance · r107")
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
