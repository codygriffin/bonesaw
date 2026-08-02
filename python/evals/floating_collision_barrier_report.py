#!/usr/bin/env python3
"""Audit the floating acceleration-level closest-feature collision barrier."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import FloatingCollisionBarrierSession
from cpu_reference_report import distribution, markdown_table, render_report_html


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/tight_avoidance_toy.urdf")
    parser.add_argument("--repeats", type=int, default=2000)
    parser.add_argument("--hard-margin", type=float, default=0.02)
    parser.add_argument("--influence-margin", type=float, default=0.10)
    parser.add_argument("--frequency-hz", type=float, default=2.0)
    parser.add_argument("--damping-ratio", type=float, default=1.0)
    parser.add_argument(
        "--output", default="benchmarks/results/floating-collision-barrier-r108"
    )
    parser.add_argument(
        "--web-report", default="web/FLOATING_COLLISION_BARRIER_R108.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def buffers(generalized_dof: int) -> dict[str, np.ndarray]:
    return {
        "qdd": np.zeros(generalized_dof, dtype=np.float64),
        "evidence": np.zeros(7, dtype=np.float64),
        "pair_ids": np.zeros(2, dtype=np.uint32),
        "counts": np.zeros(3, dtype=np.uint32),
        "quality": np.zeros(2, dtype=np.uint8),
        "status": np.zeros(1, dtype=np.uint8),
    }


def query(
    session: FloatingCollisionBarrierSession,
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
        output["pair_ids"],
        output["counts"],
        output["quality"],
        output["status"],
    )


def semantic_digest(output: dict[str, np.ndarray]) -> str:
    digest = hashlib.sha256()
    for name in sorted(output):
        digest.update(name.encode())
        digest.update(output[name].tobytes())
    return digest.hexdigest()


def benchmark(
    session: FloatingCollisionBarrierSession,
    repeats: int,
    q: np.ndarray,
    v: np.ndarray,
    desired: np.ndarray,
) -> dict[str, Any]:
    output = buffers(session.dof + 6)
    root_translation = np.array([0.31, -0.27, 0.44], dtype=np.float64)
    root_twist = np.array([0.0, 0.0, 0.0, 0.41, -0.22, 0.17], dtype=np.float64)
    for _ in range(50):
        query(session, q, v, root_translation, root_twist, desired, output)
    elapsed_us = np.zeros(repeats, dtype=np.float64)
    allocation_calls = np.zeros(repeats, dtype=np.uint64)
    allocated_bytes = np.zeros(repeats, dtype=np.uint64)
    replay_digest = None
    exact_replay = True
    for repeat in range(repeats):
        elapsed_ns, calls, byte_count = query(
            session, q, v, root_translation, root_twist, desired, output
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


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    enabled = FloatingCollisionBarrierSession(
        args.model,
        enabled=True,
        hard_margin_m=args.hard_margin,
        influence_margin_m=args.influence_margin,
        natural_frequency_hz=args.frequency_hz,
        damping_ratio=args.damping_ratio,
        maximum_acceleration=200.0,
        maximum_torque=2000.0,
    )
    disabled = FloatingCollisionBarrierSession(
        args.model,
        enabled=False,
        maximum_acceleration=200.0,
        maximum_torque=2000.0,
    )
    if enabled.dof != 1 or enabled.pair_count != 1 or disabled.pair_count != 0:
        raise RuntimeError("R108 fixture/session layout changed")

    generalized_dof = enabled.dof + 6
    q_values = np.linspace(-0.12, 0.20, 321, dtype=np.float64)
    v_values = -0.6 + 0.35 * np.sin(np.linspace(0.0, 4.0 * np.pi, q_values.size))
    desired = np.zeros(generalized_dof, dtype=np.float64)
    desired[6] = -50.0
    root_translation = np.array([0.31, -0.27, 0.44], dtype=np.float64)
    root_twist = np.array([0.0, 0.0, 0.0, 0.41, -0.22, 0.17], dtype=np.float64)
    enabled_output = buffers(generalized_dof)
    disabled_output = buffers(generalized_dof)

    distance_error = np.zeros(q_values.size, dtype=np.float64)
    velocity_error = np.zeros(q_values.size, dtype=np.float64)
    required_error = np.zeros(q_values.size, dtype=np.float64)
    active_count_error = np.zeros(q_values.size, dtype=np.int64)
    active_residuals: list[float] = []
    active_achieved_minus_required: list[float] = []
    active_qdd_delta: list[float] = []
    status_codes: set[int] = set()
    omega = 2.0 * np.pi * args.frequency_hz

    for index, (coordinate, velocity) in enumerate(zip(q_values, v_values)):
        q = np.array([coordinate], dtype=np.float64)
        v = np.array([velocity], dtype=np.float64)
        query(enabled, q, v, root_translation, root_twist, desired, enabled_output)
        query(disabled, q, v, root_translation, root_twist, desired, disabled_output)
        status_codes.add(int(enabled_output["status"][0]))
        expected_distance = 0.15 + coordinate
        expected_margin = expected_distance - args.hard_margin
        expected_active = int(expected_distance < args.influence_margin)
        distance_error[index] = abs(enabled_output["evidence"][0] - expected_distance)
        active_count_error[index] = abs(
            int(enabled_output["counts"][2]) - expected_active
        )
        if expected_active:
            expected_required = (
                -2.0 * args.damping_ratio * omega * velocity
                - omega * omega * expected_margin
            )
            velocity_error[index] = abs(enabled_output["evidence"][2] - velocity)
            required_error[index] = abs(
                enabled_output["evidence"][4] - expected_required
            )
            active_residuals.append(float(enabled_output["evidence"][6]))
            active_achieved_minus_required.append(
                float(enabled_output["evidence"][5] - enabled_output["evidence"][4])
            )
            active_qdd_delta.append(
                float(enabled_output["qdd"][6] - disabled_output["qdd"][6])
            )
            if enabled_output["pair_ids"][1] != 0:
                raise RuntimeError("active barrier lost stable pair provenance")
        elif enabled_output["pair_ids"][1] != np.iinfo(np.uint32).max:
            raise RuntimeError("inactive state reported an active limiting pair")

    active_residuals_array = np.asarray(active_residuals, dtype=np.float64)
    active_achieved_array = np.asarray(
        active_achieved_minus_required, dtype=np.float64
    )
    active_qdd_delta_array = np.asarray(active_qdd_delta, dtype=np.float64)
    benchmark_result = benchmark(
        enabled,
        args.repeats,
        np.array([-0.12], dtype=np.float64),
        np.array([-0.6], dtype=np.float64),
        desired,
    )
    timing = benchmark_result["timing_us"]
    passed = bool(
        status_codes.issubset({0, 1})
        and np.max(distance_error) < 1e-12
        and np.max(active_count_error) == 0
        and np.max(velocity_error) < 1e-12
        and np.max(required_error) < 1e-10
        and np.min(active_residuals_array) >= -1e-7
        and np.min(active_achieved_array) >= -1e-7
        and np.max(active_qdd_delta_array) > 1.0
        and benchmark_result["semantic_replay_exact"]
        and benchmark_result["maximum_allocation_calls"] == 0
        and benchmark_result["maximum_allocated_bytes"] == 0
        and timing["p99"] < 5_000.0
        and timing["maximum"] < 50_000.0
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free floating WBC acceleration-barrier queries",
        "model": args.model,
        "coordinate_samples": int(q_values.size),
        "active_samples": int(active_residuals_array.size),
        "represented_pair_count": enabled.pair_count,
        "hard_margin_m": args.hard_margin,
        "influence_margin_m": args.influence_margin,
        "natural_frequency_hz": args.frequency_hz,
        "damping_ratio": args.damping_ratio,
        "maximum_distance_oracle_error_m": float(np.max(distance_error)),
        "maximum_relative_velocity_oracle_error_mps": float(np.max(velocity_error)),
        "maximum_required_acceleration_oracle_error_mps2": float(
            np.max(required_error)
        ),
        "maximum_active_count_error": int(np.max(active_count_error)),
        "minimum_barrier_residual_mps2": float(np.min(active_residuals_array)),
        "maximum_barrier_residual_mps2": float(np.max(active_residuals_array)),
        "minimum_achieved_minus_required_mps2": float(np.min(active_achieved_array)),
        "maximum_enabled_minus_disabled_joint_acceleration_mps2": float(
            np.max(active_qdd_delta_array)
        ),
        "status_codes": sorted(status_codes),
        "benchmark": benchmark_result,
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    timing = audit["benchmark"]["timing_us"]
    table = "\n".join(
        markdown_table(
            ["query", "p50 / p99 / max µs", "alloc calls / bytes", "exact replay"],
            [[
                "floating tight barrier",
                f"{timing['p50']:.3f} / {timing['p99']:.3f} / {timing['maximum']:.3f}",
                f"{audit['benchmark']['maximum_allocation_calls']} / {audit['benchmark']['maximum_allocated_bytes']}",
                audit["benchmark"]["semantic_replay_exact"],
            ]],
        )
    )
    return f"""# Bonesaw floating collision viability barrier · r108

## Outcome

**{audit['status'].upper()}.** R108 lifts the R107 closest-feature witness into the floating CPU WBC as a relative-degree-two hard inequality. The query retains closest-pair and limiting-active-pair provenance, distance quality, relative normal velocity, rigid-feature bias acceleration, required/achieved normal acceleration, and post-solve barrier residual.

The fixed box–sphere oracle evaluates {audit['coordinate_samples']} independent observed states without a policy, integration, contact response, or external physics. {audit['active_samples']} states activate the barrier. Distance, relative velocity, and required-acceleration oracle errors are `{audit['maximum_distance_oracle_error_m']:.3e}` m, `{audit['maximum_relative_velocity_oracle_error_mps']:.3e}` m/s, and `{audit['maximum_required_acceleration_oracle_error_mps2']:.3e}` m/s². The minimum post-solve barrier residual is `{audit['minimum_barrier_residual_mps2']:.3e}` m/s².

{table}

The enabled barrier changes the inward joint acceleration by as much as **{audit['maximum_enabled_minus_disabled_joint_acceleration_mps2']:.3f} m/s²** relative to the collision-disabled WBC query. This is an authority constraint, not a simulated body response.

## Barrier semantics

For `h = distance - hard_margin`, Rust emits `J qdd + bias + 2 ζ ω hdot + ω² h >= 0`. `bias` is the projected rigid-feature `Jdot·v` term. Closest-feature switching and normal-direction curvature are excluded, so this is explicitly a local acceleration barrier rather than exact continuous collision detection. The separately retained R106 sampled/adaptive command gate still owns between-tick trajectory admission.

## Authority boundary

Collision viability remains a separate row from dynamics/contact invariants, finite-support reserve, joint stopping, actuator effort, thermal state, solver budget, trajectory clearance, and command selection. Unsupported authored geometry is counted in the typed evidence instead of being treated as clear. No aggregate health score is produced.

## Remaining work

The r108 WebSocket contract and browser authority stack now consume this typed evidence as an independent Viability row. General authored exclusions, world collision, mesh CCD, closest-feature continuous rate bounds, contact response, and calibrated plant behavior remain separate milestones.
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
    ):
        raise SystemExit("invalid benchmark count or barrier configuration")
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = evaluate(args)
    sources = [
        pathlib.Path("crates/bonesaw-core/src/collision.rs"),
        pathlib.Path("crates/bonesaw-core/src/dynamic_wbc.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path("python/bonesaw/__init__.py"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "floating-collision-barrier-r108",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [model, *sources]},
        "audit": audit,
    }
    (output / "floating-collision-barrier-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "FLOATING_COLLISION_BARRIER_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw floating collision viability barrier · r108")
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
