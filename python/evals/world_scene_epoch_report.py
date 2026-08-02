#!/usr/bin/env python3
"""Policy-/physics-free audit of versioned world-scene snapshot authority."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import DynamicAdvanceSession
from cpu_reference_report import distribution, markdown_table, render_report_html


HORIZON_NS = 20_000_000
SCENE_INVALID = 1 << 17
PRIMARY_WORLD_UNKNOWN = 1 << 15
CONTINGENCY_WORLD_UNKNOWN = 1 << 16
VALIDITY_NAMES = {
    0: "valid",
    1: "epoch_mismatch",
    2: "source_from_future",
    3: "not_yet_valid",
    4: "expired_at_tick",
    5: "horizon_expired",
    6: "too_old",
    7: "invalid_stamp",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/collision_sweep_toy.urdf")
    parser.add_argument("--repeats", type=int, default=2000)
    parser.add_argument("--output", default="benchmarks/results/world-scene-epoch-r113")
    parser.add_argument("--web-report", default="web/WORLD_SCENE_EPOCH_R113.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def plane_field() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    origin = np.array([-0.5, -1.0, -1.0], dtype=np.float64)
    spacing = np.array([0.1, 0.1, 0.1], dtype=np.float64)
    world_x = origin[0] + spacing[0] * np.arange(21)
    values = np.empty((21, 21, 21), dtype=np.float64)
    values[:] = (world_x - 0.2)[None, None, :]
    return values, origin, spacing


def session(
    args: argparse.Namespace,
    values: np.ndarray,
    origin: np.ndarray,
    spacing: np.ndarray,
    *,
    scene_epoch: int,
    source_time_ns: int,
    valid_from_ns: int,
    valid_until_ns: int,
    expected_epoch: int,
    maximum_age_ns: int | None,
    require_horizon: bool = True,
) -> DynamicAdvanceSession:
    return DynamicAdvanceSession(
        args.model,
        [],
        maximum_generalized_acceleration=100.0,
        maximum_generalized_effort=1000.0,
        world_sdf_values_zyx=values,
        world_grid_origin_world=origin,
        world_grid_spacing=spacing,
        world_probe_body_names=["slider"],
        world_collision_clearance=0.02,
        certify_world_collision_between_samples=True,
        world_collision_max_subdivision_depth=3,
        world_scene_epoch=scene_epoch,
        world_scene_source_time_ns=source_time_ns,
        world_scene_valid_from_ns=valid_from_ns,
        world_scene_valid_until_ns=valid_until_ns,
        expected_world_scene_epoch=expected_epoch,
        maximum_world_scene_age_ns=maximum_age_ns,
        require_world_scene_horizon_validity=require_horizon,
    )


def buffers(runtime: DynamicAdvanceSession) -> dict[str, np.ndarray]:
    return {
        "qdd": np.zeros((1, runtime.generalized_dof), dtype=np.float64),
        "splice_position": np.zeros((1, runtime.actuator_count), dtype=np.float64),
        "splice_velocity": np.zeros((1, runtime.actuator_count), dtype=np.float64),
        "splice_acceleration": np.zeros((1, runtime.actuator_count), dtype=np.float64),
        "metrics": np.zeros((1, 9), dtype=np.float64),
        "ids": np.zeros((1, 29), dtype=np.uint64),
        "work": np.zeros((1, 8), dtype=np.uint64),
        "root": np.zeros((1, 48), dtype=np.float64),
        "timing": np.zeros((1, 3), dtype=np.uint64),
    }


def run(runtime: DynamicAdvanceSession, out: dict[str, np.ndarray]) -> None:
    q = np.array([[-0.3]], dtype=np.float64)
    v = np.zeros((1, 1), dtype=np.float64)
    runtime.reset_command_state(q[0], v[0])
    runtime.run_world_trace(
        0,
        q,
        v,
        np.zeros((1, 6), dtype=np.float64),
        np.zeros((1, runtime.generalized_dof), dtype=np.float64),
        out["qdd"],
        out["splice_position"],
        out["splice_velocity"],
        out["splice_acceleration"],
        out["metrics"],
        out["ids"],
        out["work"],
        out["root"],
        out["timing"],
    )


def signed(value: np.uint64) -> int:
    return int(np.asarray(value, dtype=np.uint64).view(np.int64))


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    values, origin, spacing = plane_field()
    cases = [
        ("valid", 7, 0, 0, 100_000_000, 7, 5_000_000, 0),
        ("epoch_mismatch", 7, 0, 0, 100_000_000, 8, 5_000_000, 1),
        ("source_from_future", 7, 1_000_000, 0, 100_000_000, 7, None, 2),
        ("not_yet_valid", 7, -1_000_000, 1_000_000, 100_000_000, 7, None, 3),
        ("expired_at_tick", 7, -10_000_000, -10_000_000, -1, 7, None, 4),
        ("horizon_expired", 7, 0, 0, 10_000_000, 7, None, 5),
        ("too_old", 7, -10_000_000, -20_000_000, 100_000_000, 7, 5_000_000, 6),
    ]
    rows: list[dict[str, Any]] = []
    max_allocations = 0
    max_allocated_bytes = 0
    for (
        name,
        scene_epoch,
        source_time_ns,
        valid_from_ns,
        valid_until_ns,
        expected_epoch,
        maximum_age_ns,
        expected_validity,
    ) in cases:
        runtime = session(
            args,
            values,
            origin,
            spacing,
            scene_epoch=scene_epoch,
            source_time_ns=source_time_ns,
            valid_from_ns=valid_from_ns,
            valid_until_ns=valid_until_ns,
            expected_epoch=expected_epoch,
            maximum_age_ns=maximum_age_ns,
        )
        out = buffers(runtime)
        run(runtime, out)
        ids = out["ids"][0]
        flags = int(ids[14])
        validity = int(ids[23])
        selection = int(ids[16])
        max_allocations = max(max_allocations, int(out["timing"][0, 1]))
        max_allocated_bytes = max(max_allocated_bytes, int(out["timing"][0, 2]))
        rows.append(
            {
                "name": name,
                "validity": VALIDITY_NAMES[validity],
                "expected_validity": VALIDITY_NAMES[expected_validity],
                "scene_epoch": int(ids[22]),
                "source_time_ns": signed(ids[24]),
                "valid_from_ns": signed(ids[25]),
                "valid_until_ns": signed(ids[26]),
                "tick_time_ns": signed(ids[27]),
                "horizon_end_ns": signed(ids[28]),
                "selection": selection,
                "primary_valid": bool(ids[17]),
                "contingency_valid": bool(ids[18]),
                "flags": flags,
                "scene_invalid_flag": bool(flags & SCENE_INVALID),
                "primary_unknown_flag": bool(flags & PRIMARY_WORLD_UNKNOWN),
                "contingency_unknown_flag": bool(flags & CONTINGENCY_WORLD_UNKNOWN),
            }
        )

    malformed_rejected = False
    try:
        session(
            args,
            values,
            origin,
            spacing,
            scene_epoch=7,
            source_time_ns=0,
            valid_from_ns=2,
            valid_until_ns=1,
            expected_epoch=7,
            maximum_age_ns=None,
        )
    except ValueError:
        malformed_rejected = True

    benchmark_runtime = session(
        args,
        values,
        origin,
        spacing,
        scene_epoch=7,
        source_time_ns=0,
        valid_from_ns=0,
        valid_until_ns=100_000_000,
        expected_epoch=7,
        maximum_age_ns=5_000_000,
    )
    benchmark_out = buffers(benchmark_runtime)
    elapsed_us = np.empty(args.repeats, dtype=np.float64)
    allocation_calls = np.empty(args.repeats, dtype=np.uint64)
    allocated_bytes = np.empty(args.repeats, dtype=np.uint64)
    digests: set[str] = set()
    for repeat in range(args.repeats):
        run(benchmark_runtime, benchmark_out)
        elapsed_us[repeat] = benchmark_out["timing"][0, 0] / 1000.0
        allocation_calls[repeat] = benchmark_out["timing"][0, 1]
        allocated_bytes[repeat] = benchmark_out["timing"][0, 2]
        digest = hashlib.sha256()
        for name in ("metrics", "ids", "work", "root"):
            digest.update(benchmark_out[name].tobytes())
        digests.add(digest.hexdigest())

    passed = bool(
        malformed_rejected
        and all(row["validity"] == row["expected_validity"] for row in rows)
        and rows[0]["selection"] == 0
        and rows[0]["primary_valid"]
        and not rows[0]["scene_invalid_flag"]
        and all(row["selection"] == 2 for row in rows[1:])
        and all(not row["primary_valid"] for row in rows[1:])
        and all(not row["contingency_valid"] for row in rows[1:])
        and all(row["scene_invalid_flag"] for row in rows[1:])
        and all(row["primary_unknown_flag"] for row in rows[1:])
        and all(row["contingency_unknown_flag"] for row in rows[1:])
        and max_allocations == 0
        and max_allocated_bytes == 0
        and int(np.max(allocation_calls)) == 0
        and int(np.max(allocated_bytes)) == 0
        and len(digests) == 1
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free, integrator-free immutable scene-snapshot admission",
        "cases": rows,
        "malformed_stamp_rejected": malformed_rejected,
        "maximum_case_allocation_calls": max_allocations,
        "maximum_case_allocated_bytes": max_allocated_bytes,
        "benchmark": {
            "repeats": args.repeats,
            "timing_us": distribution(elapsed_us),
            "maximum_allocation_calls": int(np.max(allocation_calls)),
            "maximum_allocated_bytes": int(np.max(allocated_bytes)),
            "semantic_replay_exact": len(digests) == 1,
            "semantic_digest": next(iter(digests)),
        },
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    case_table = "\n".join(
        markdown_table(
            ["case", "typed validity", "selection", "scene/unknown flags"],
            [
                [
                    row["name"],
                    row["validity"],
                    ("Primary", "Contingency", "Rejected")[row["selection"]],
                    f"{row['scene_invalid_flag']} / {row['primary_unknown_flag']} / {row['contingency_unknown_flag']}",
                ]
                for row in audit["cases"]
            ],
        )
    )
    timing = audit["benchmark"]["timing_us"]
    return f"""# Bonesaw versioned world-scene snapshot authority · r113

## Outcome

**{audit['status'].upper()}.** One immutable SDF snapshot now carries an independent scene epoch, source timestamp, and closed validity interval in smooth `control_world`. The command transaction checks the expected epoch, source causality, current validity, maximum age, and full 20 ms horizon coverage before either Primary or brake can gain authority. There is no policy, physics engine, state integration, map-frame jump, or plant rollout.

{case_table}

The valid snapshot selects Primary. Epoch mismatch, future source time, not-yet-valid data, expiry at the tick, expiry inside the command horizon, and excessive age all reject both plans with a dedicated scene-invalid bit plus independent primary/brake world-unknown bits. A malformed validity interval is rejected at construction.

## Timing and allocation

The complete valid WBC + actuator/root prediction + robust two-plan world transaction ran {audit['benchmark']['repeats']} times at `{timing['p50']:.3f}/{timing['p99']:.3f}/{timing['maximum']:.3f}` µs p50/p99/max. Semantic replay was exact: **{audit['benchmark']['semantic_replay_exact']}**. Maximum timed allocation calls/bytes were `{audit['benchmark']['maximum_allocation_calls']}/{audit['benchmark']['maximum_allocated_bytes']}`.

## Authority boundary

The scene epoch is not the MotionProgram epoch and is not `map` or `odom`. Global localization corrections remain external-frame observations; they cannot jump the smooth WBC root. R113 versions the immutable field consumed by local and segment queries and proves freshness fail-closed. It does not yet hot-swap field storage inside one controller call, interpolate scene epochs, or sweep moving-obstacle primitives. Those require an explicit fixed-capacity scene-view input and velocity-aware obstacle evidence.
"""


def main() -> None:
    args = parse_args()
    if args.repeats <= 1:
        raise SystemExit("repeats must be greater than one")
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    model = pathlib.Path(args.model)
    audit = evaluate(args)
    sources = [
        pathlib.Path("crates/bonesaw-core/src/world_collision.rs"),
        pathlib.Path("crates/bonesaw-core/src/dynamic_controller.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "world-scene-epoch-r113",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [model, *sources]},
        "audit": audit,
    }
    (output / "world-scene-epoch-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "WORLD_SCENE_EPOCH_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(
            report, title="Bonesaw versioned world-scene snapshot authority · r113"
        )
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
