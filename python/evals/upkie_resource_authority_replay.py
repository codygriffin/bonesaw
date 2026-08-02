#!/usr/bin/env python3
"""Policy-/physics-free Upkie resource-authority replay.

Rust consumes immutable per-state acceleration and actuator-effort availability
without allocating in the query loop. Python authors the sensitivity corpus and
independently reconstructs hard equations and scaled resource margins.
"""

from __future__ import annotations

import argparse
import gc
import json
import pathlib
import platform
import time
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_transition_replay import (
    independent_mode_oracle,
    make_transition_corpus,
    slice_corpus,
)
from upkie_state_local_wbc_report import (
    SOLVED,
    allocate_outputs,
    new_session,
    rolling_descriptors,
    rss_bytes,
    run_case,
    semantic_equal,
    sha256,
    status_counts,
    urdf_effort_limits,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=10_000)
    parser.add_argument("--seed", type=lambda value: int(value, 0), default=0xB0E5A8)
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument(
        "--output", default="benchmarks/results/upkie-resource-authority-r125"
    )
    parser.add_argument(
        "--web-report", default="web/UPKIE_RESOURCE_AUTHORITY_R125.html"
    )
    return parser.parse_args()


def availability(samples: int, generalized: int, dof: int) -> tuple[np.ndarray, np.ndarray]:
    phase = np.arange(samples, dtype=np.float64)
    acceleration = np.empty((samples, generalized), np.float64)
    effort = np.empty((samples, dof), np.float64)
    for coordinate in range(generalized):
        wave = 0.5 + 0.5 * np.sin(phase * (0.0019 + 0.00011 * coordinate) + coordinate)
        acceleration[:, coordinate] = 0.22 + 0.78 * wave
    for coordinate in range(dof):
        wave = 0.5 + 0.5 * np.cos(phase * (0.0023 + 0.00017 * coordinate) + 0.7 * coordinate)
        effort[:, coordinate] = 0.04 + 0.96 * wave
    return np.ascontiguousarray(acceleration), np.ascontiguousarray(effort)


def run_resource_case(
    model_path: pathlib.Path,
    corpus: dict[str, np.ndarray],
    frame_ids: np.ndarray,
    coordinates: np.ndarray,
    coefficients: np.ndarray,
    *,
    acceleration_scales: np.ndarray | None,
    effort_scales: np.ndarray | None,
    outputs: dict[str, np.ndarray] | None = None,
) -> dict[str, np.ndarray]:
    return run_case(
        new_session(model_path),
        corpus,
        frame_ids,
        None,
        coordinates,
        coefficients,
        outputs,
        contact_mode_trace=corpus["contact_mode_trace"],
        target_weights=np.zeros(2, np.float64),
        generalized_acceleration_limit_scales=acceleration_scales,
        actuator_effort_limit_scales=effort_scales,
    )


def desired_acceleration(corpus: dict[str, np.ndarray]) -> np.ndarray:
    return np.column_stack(
        (
            np.zeros((len(corpus["q"]), 3)),
            corpus["root_accelerations"],
            corpus["joint_accelerations"],
        )
    )


def case_metrics(
    name: str,
    corpus: dict[str, np.ndarray],
    out: dict[str, np.ndarray],
    acceleration_scales: np.ndarray,
    effort_scales: np.ndarray,
    effort_limits: np.ndarray,
    oracle: dict[str, Any],
) -> dict[str, Any]:
    acceleration_margin = 250.0 * acceleration_scales - np.abs(out["generalized_acceleration"])
    effort_margin = effort_limits[None, :] * effort_scales - np.abs(out["actuator_torque"])
    error = out["generalized_acceleration"] - desired_acceleration(corpus)
    acceleration_contact = np.any(acceleration_margin <= 1e-7, axis=1)
    effort_contact = np.any(effort_margin <= 1e-7, axis=1)
    latency_us = out["step_ns"].astype(np.float64) / 1000.0
    return {
        "name": name,
        "status_counts": status_counts(out["status"]),
        "all_solved_or_slack": bool(np.all(np.isin(out["status"], SOLVED))),
        "all_finite": bool(
            np.all(np.isfinite(out["generalized_acceleration"]))
            and np.all(np.isfinite(out["actuator_torque"]))
            and np.all(np.isfinite(out["contact_force_basis"]))
        ),
        "tracking_rms": float(np.sqrt(np.mean(error * error))),
        "tracking_per_tick_rms": distribution(np.sqrt(np.mean(error * error, axis=1))),
        "minimum_acceleration_margin": float(np.min(acceleration_margin)),
        "minimum_effort_margin_nm": float(np.min(effort_margin)),
        "acceleration_bound_queries": int(np.sum(acceleration_contact)),
        "effort_bound_queries": int(np.sum(effort_contact)),
        "simultaneous_bound_queries": int(np.sum(acceleration_contact & effort_contact)),
        "minimum_acceleration_scale": float(np.min(acceleration_scales)),
        "minimum_effort_scale": float(np.min(effort_scales)),
        "maximum_reported_effort_utilization": float(np.nanmax(out["maximum_torque_utilization"])),
        "dynamics_linf": oracle["dynamics_linf"],
        "contact_linf": oracle["contact_linf"],
        "minimum_normal_force_n": oracle["minimum_normal_force_n"],
        "minimum_friction_margin_n": oracle["minimum_friction_margin_n"],
        "inactive_force_linf": oracle["inactive_force_linf"],
        "latency_us": distribution(latency_us),
        "allocation_calls": int(np.sum(out["allocation_calls"])),
        "allocated_bytes": int(np.sum(out["allocated_bytes"])),
    }


def atomic_scale_faults(
    model_path: pathlib.Path,
    corpus: dict[str, np.ndarray],
    frame_ids: np.ndarray,
    coordinates: np.ndarray,
    coefficients: np.ndarray,
) -> dict[str, Any]:
    sample = slice_corpus(corpus, slice(0, 64))
    generalized = sample["q"].shape[1] + 6
    dof = sample["q"].shape[1]
    good_a = np.ones((64, generalized), np.float64)
    good_e = np.ones((64, dof), np.float64)
    cases: list[tuple[str, np.ndarray, np.ndarray]] = []
    for name, value in (("negative_acceleration_scale", -0.1), ("above_one_acceleration_scale", 1.1), ("nonfinite_acceleration_scale", np.nan)):
        bad = good_a.copy()
        bad[31, 2] = value
        cases.append((name, bad, good_e))
    for name, value in (("negative_effort_scale", -0.1), ("above_one_effort_scale", 1.1), ("nonfinite_effort_scale", np.inf)):
        bad = good_e.copy()
        bad[31, 2] = value
        cases.append((name, good_a, bad))
    rows = []
    for name, accel, effort in cases:
        session = new_session(model_path)
        outputs = allocate_outputs(session, 64)
        for array in outputs.values():
            array.fill(77)
        rejected = False
        try:
            run_case(
                session,
                sample,
                frame_ids,
                None,
                coordinates,
                coefficients,
                outputs,
                contact_mode_trace=sample["contact_mode_trace"],
                target_weights=np.zeros(2, np.float64),
                generalized_acceleration_limit_scales=accel,
                actuator_effort_limit_scales=effort,
            )
        except ValueError:
            rejected = True
        rows.append(
            {
                "fault": name,
                "typed_value_error": rejected,
                "outputs_unchanged": all(np.all(array == 77) for array in outputs.values()),
            }
        )
    return {
        "rows": rows,
        "all_typed_and_atomic": all(
            row["typed_value_error"] and row["outputs_unchanged"] for row in rows
        ),
    }


def combined_invariance(
    model_path: pathlib.Path,
    corpus: dict[str, np.ndarray],
    frame_ids: np.ndarray,
    coordinates: np.ndarray,
    coefficients: np.ndarray,
    acceleration: np.ndarray,
    effort: np.ndarray,
    baseline: dict[str, np.ndarray],
) -> dict[str, Any]:
    reverse = np.arange(len(corpus["q"]) - 1, -1, -1)
    reversed_out = run_resource_case(
        model_path,
        slice_corpus(corpus, reverse),
        frame_ids,
        coordinates,
        coefficients,
        acceleration_scales=np.ascontiguousarray(acceleration[reverse]),
        effort_scales=np.ascontiguousarray(effort[reverse]),
    )
    inverse = np.argsort(reverse)
    reordered = {name: value[inverse] for name, value in reversed_out.items()}
    boundaries = np.linspace(0, len(corpus["q"]), 5, dtype=int)
    parts = []
    for start, stop in zip(boundaries[:-1], boundaries[1:], strict=True):
        parts.append(
            run_resource_case(
                model_path,
                slice_corpus(corpus, slice(start, stop)),
                frame_ids,
                coordinates,
                coefficients,
                acceleration_scales=np.ascontiguousarray(acceleration[start:stop]),
                effort_scales=np.ascontiguousarray(effort[start:stop]),
            )
        )
    chunked = {name: np.concatenate([part[name] for part in parts]) for name in baseline}
    allocation_calls = int(
        np.sum(reversed_out["allocation_calls"])
        + sum(np.sum(part["allocation_calls"]) for part in parts)
    )
    allocated_bytes = int(
        np.sum(reversed_out["allocated_bytes"])
        + sum(np.sum(part["allocated_bytes"]) for part in parts)
    )
    return {
        "reverse_order_exact": semantic_equal(baseline, reordered),
        "four_chunk_exact": semantic_equal(baseline, chunked),
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
    }


def report_markdown(metrics: dict[str, Any]) -> str:
    rows = []
    for case in metrics["cases"]:
        timing = case["latency_us"]
        rows.append(
            [
                case["name"],
                case["status_counts"],
                f'{case["minimum_acceleration_scale"]:.2f} / {case["minimum_effort_scale"]:.2f}',
                f'{case["tracking_rms"]:.3f}',
                f'{case["minimum_acceleration_margin"]:.2e} / {case["minimum_effort_margin_nm"]:.2e}',
                f'{case["acceleration_bound_queries"]} / {case["effort_bound_queries"]}',
                f'{case["dynamics_linf"]:.2e} / {case["contact_linf"]:.2e}',
                f'{timing["p50"]:.1f} / {timing["p99"]:.1f} / {timing["maximum"]:.1f}',
            ]
        )
    return "\n".join(
        [
            "# Bonesaw Upkie resource-authority replay · r125",
            "",
            "## Outcome",
            "",
            f'> **Admission: {"PASS" if metrics["admission"] else "FAIL"}.** '
            f'{metrics["samples"]:,} immutable Upkie states were queried in nominal, '
            "acceleration-derated, effort-derated, and combined resource regimes. No policy, "
            "contact estimator, state integration, simulator, or plant model participates.",
            "",
            "## Resource sensitivity",
            "",
            *markdown_table(
                ["case", "statuses", "min qdd / effort scale", "tracking RMS", "min qdd / effort margin", "qdd / effort bound queries", "dyn / contact L∞", "p50 / p99 / max µs"],
                rows,
            ),
            "",
            "Each scale is external state-local authority in `[0,1]`, not inferred temperature, "
            "reliability, policy, or body response. Rust applies it directly to the configured "
            "bounds before solving; task residual remains the continuous record of motion that "
            "could not be tracked.",
            "",
            "## Boundary checks",
            "",
            *markdown_table(
                ["check", "result"],
                [
                    ["nominal all-ones equals absent scales", metrics["nominal_transparency_exact"]],
                    ["combined repeat bitwise exact", metrics["combined_repeat_exact"]],
                    ["combined reverse-order exact", metrics["invariance"]["reverse_order_exact"]],
                    ["combined four-chunk exact", metrics["invariance"]["four_chunk_exact"]],
                    ["inputs immutable", metrics["inputs_immutable"]],
                    ["all hot-loop allocation calls / bytes", f'{metrics["allocation_calls"]} / {metrics["allocated_bytes"]}'],
                    ["RSS before / after / delta", f'{metrics["rss_before_bytes"] / 1048576:.2f} / {metrics["rss_after_bytes"] / 1048576:.2f} / {metrics["rss_delta_bytes"] / 1048576:.2f} MiB'],
                    ["whole replay wall / CPU", f'{metrics["wall_seconds"]:.3f} / {metrics["cpu_seconds"]:.3f} s'],
                    ["GC collections", metrics["gc_collections"]],
                ],
            ),
            "",
            "## Atomic invalid-scale probes",
            "",
            *markdown_table(
                ["fault", "typed ValueError", "outputs unchanged"],
                [[row["fault"], row["typed_value_error"], row["outputs_unchanged"]] for row in metrics["faults"]["rows"]],
            ),
            "",
            "The report distinguishes hierarchy compromise from declared resource exhaustion. "
            "It still does not prove that a real actuator realizes the command; calibrated "
            "electrical/thermal models, observation transport, and plant tracking remain "
            "separate authority rows.",
            "",
        ]
    )


def main() -> int:
    args = parse_args()
    if args.samples < 1_000:
        raise ValueError("resource replay requires at least 1,000 states")
    model_path = pathlib.Path(args.model).resolve()
    output_path = pathlib.Path(args.output)
    output_path.mkdir(parents=True, exist_ok=True)
    session = new_session(model_path)
    joint_names = list(session.joint_names)
    contact_frames = ["left_wheel_center", "right_wheel_center"]
    all_frames = list(session.frame_names)
    frame_ids = np.asarray([all_frames.index(name) for name in contact_frames], np.int64)
    _, coordinates, coefficients = rolling_descriptors(
        model_path, joint_names, contact_frames
    )
    corpus = make_transition_corpus(
        model_path,
        args.samples,
        args.seed,
        joint_names,
        contact_frames,
        coordinates,
        coefficients,
    )
    acceleration, effort = availability(args.samples, len(joint_names) + 6, len(joint_names))
    ones_a = np.ones_like(acceleration)
    ones_e = np.ones_like(effort)
    input_fingerprint = sha256(model_path) + ":" + str(
        hash((corpus["q"].tobytes(), acceleration.tobytes(), effort.tobytes()))
    )
    cases = (
        ("nominal", ones_a, ones_e),
        ("acceleration_derated", acceleration, ones_e),
        ("effort_derated", ones_a, effort),
        ("combined", acceleration, effort),
    )
    outputs: dict[str, dict[str, np.ndarray]] = {}
    case_rows = []
    effort_limits = urdf_effort_limits(model_path, joint_names)
    gc.collect()
    gc_before = [entry["collections"] for entry in gc.get_stats()]
    rss_before = rss_bytes()
    wall_before = time.perf_counter_ns()
    cpu_before = time.process_time_ns()
    for name, acceleration_scales, effort_scales in cases:
        out = run_resource_case(
            model_path,
            corpus,
            frame_ids,
            coordinates,
            coefficients,
            acceleration_scales=acceleration_scales,
            effort_scales=effort_scales,
        )
        outputs[name] = out
        oracle, _ = independent_mode_oracle(
            model_path,
            joint_names,
            contact_frames,
            corpus,
            out,
            coordinates,
            coefficients,
        )
        case_rows.append(
            case_metrics(
                name,
                corpus,
                out,
                acceleration_scales,
                effort_scales,
                effort_limits,
                oracle,
            )
        )
    cpu_after = time.process_time_ns()
    wall_after = time.perf_counter_ns()
    rss_after = rss_bytes()
    gc_after = [entry["collections"] for entry in gc.get_stats()]
    absent = run_resource_case(
        model_path,
        corpus,
        frame_ids,
        coordinates,
        coefficients,
        acceleration_scales=None,
        effort_scales=None,
    )
    repeat = run_resource_case(
        model_path,
        corpus,
        frame_ids,
        coordinates,
        coefficients,
        acceleration_scales=acceleration,
        effort_scales=effort,
    )
    replay = combined_invariance(
        model_path,
        corpus,
        frame_ids,
        coordinates,
        coefficients,
        acceleration,
        effort,
        outputs["combined"],
    )
    faults = atomic_scale_faults(
        model_path, corpus, frame_ids, coordinates, coefficients
    )
    inputs_immutable = input_fingerprint == sha256(model_path) + ":" + str(
        hash((corpus["q"].tobytes(), acceleration.tobytes(), effort.tobytes()))
    )
    allocation_calls = sum(row["allocation_calls"] for row in case_rows)
    allocated_bytes = sum(row["allocated_bytes"] for row in case_rows)
    admission = bool(
        all(row["all_solved_or_slack"] and row["all_finite"] for row in case_rows)
        and all(row["minimum_acceleration_margin"] >= -2e-7 for row in case_rows)
        and all(row["minimum_effort_margin_nm"] >= -2e-7 for row in case_rows)
        and all(row["dynamics_linf"] <= 2e-6 for row in case_rows)
        and all(row["contact_linf"] <= 2e-6 for row in case_rows)
        and all(row["minimum_normal_force_n"] >= -1e-8 for row in case_rows)
        and all(row["minimum_friction_margin_n"] >= -2e-7 for row in case_rows)
        and all(row["inactive_force_linf"] == 0.0 for row in case_rows)
        and semantic_equal(outputs["nominal"], absent)
        and semantic_equal(outputs["combined"], repeat)
        and replay["reverse_order_exact"]
        and replay["four_chunk_exact"]
        and replay["allocation_calls"] == 0
        and replay["allocated_bytes"] == 0
        and allocation_calls == 0
        and allocated_bytes == 0
        and faults["all_typed_and_atomic"]
        and inputs_immutable
    )
    metrics = {
        "schema_version": 1,
        "revision": "upkie-resource-authority-r125",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "processor": platform.processor()},
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "samples": args.samples,
        "seed": hex(args.seed),
        "policy": None,
        "contact_estimator": None,
        "physics_rollout": None,
        "cases": case_rows,
        "nominal_transparency_exact": semantic_equal(outputs["nominal"], absent),
        "combined_repeat_exact": semantic_equal(outputs["combined"], repeat),
        "invariance": replay,
        "inputs_immutable": inputs_immutable,
        "faults": faults,
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
        "wall_seconds": (wall_after - wall_before) / 1e9,
        "cpu_seconds": (cpu_after - cpu_before) / 1e9,
        "rss_before_bytes": rss_before,
        "rss_after_bytes": rss_after,
        "rss_delta_bytes": rss_after - rss_before,
        "gc_collections": int(
            sum(after - before for before, after in zip(gc_before, gc_after, strict=True))
        ),
        "admission": admission,
    }
    raw = {
        "acceleration_scales": acceleration,
        "effort_scales": effort,
        **{
            f"{case}_{name}": value
            for case, values in outputs.items()
            for name, value in values.items()
        },
    }
    np.savez_compressed(output_path / "upkie-resource-authority-raw.npz", **raw)
    (output_path / "upkie-resource-authority-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    markdown = report_markdown(metrics)
    (output_path / "UPKIE_RESOURCE_AUTHORITY_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(
        render_report_html(markdown, title="Bonesaw Upkie resource-authority replay · r125")
    )
    print(json.dumps(metrics, indent=2))
    return 0 if admission else 1


if __name__ == "__main__":
    raise SystemExit(main())
