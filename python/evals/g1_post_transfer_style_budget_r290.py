#!/usr/bin/env python3
"""Qualify the post-transfer Style pseudoinverse budget experiment.

The replay remains a Python-owned evidence harness around the Rust floating
WBC.  It executes no policy or physics step.  The candidate keeps the nominal
Style prefix unbounded and arms a finite Style ceiling only after the support
trajectory projector has clipped an intent request.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any

import numpy as np

from cpu_reference_report import render_report_html


ROOT = pathlib.Path(__file__).resolve().parents[2]
REVISION = "g1-post-transfer-style-budget-r290"
RESULT_DIR = ROOT / "benchmarks/results" / REVISION
RESULT = RESULT_DIR / f"{REVISION}-metrics.json"
REPORT = RESULT_DIR / "G1_POST_TRANSFER_STYLE_BUDGET_R290.md"
WEB_REPORT = ROOT / "web/G1_POST_TRANSFER_STYLE_BUDGET_R290.html"
FLOATING = pathlib.Path(__file__).with_name("floating_walk_corpus.py")
MODEL = ROOT / "benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
REFERENCE = ROOT / "benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz"
INITIAL = ROOT / "benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz"

COMMON_ARGS = [
    "--model",
    str(MODEL),
    "--reference-inputs",
    str(REFERENCE),
    "--initial-state-inputs",
    str(INITIAL),
    "--morphology-posture-trace",
    "--ticks",
    "2317",
    "--maximum-feasibility-iterations",
    "7",
    "--maximum-feasibility-projection-sweeps",
    "8",
    "--center-of-mass-task-weight",
    "0.01",
    "--center-of-mass-task-priority",
    "4",
    "--joint-posture-weight",
    "0.05",
    "--joint-posture-priority",
    "3",
    "--joint-velocity-envelope-weight",
    "0.25",
    "--joint-velocity-envelope-priority",
    "1",
    "--joint-velocity-envelope-activation-fraction",
    "0.5",
    "--joint-velocity-envelope-lower-body-only",
    "--joint-velocity-envelope-hard",
    "--joint-velocity-envelope-frequency-hz",
    "2.0",
    "--joint-velocity-envelope-phase-policy",
    "multi-support",
    "--support-reachable-tube",
    "--support-trajectory-tube-project-intent",
    "--support-trajectory-tube-preview-ticks",
    "229",
    "--support-trajectory-tube-margin",
    "0.002",
    "--support-trajectory-tube-max-velocity",
    "0.5",
    "--support-trajectory-tube-max-acceleration",
    "20.0",
    "--support-reachable-tube-barrier-rate",
    "80.0",
    "--root-frequency-hz",
    "2.0",
    "--root-angular-task-weight",
    "1.0",
    "--root-height-task-weight",
    "10.0",
    "--root-horizontal-task-weight",
    "1.0",
    "--root-horizontal-task-priority",
    "2",
    "--point-frequency-hz",
    "4.0",
    "--foot-task-weight",
    "1.0",
    "--foot-task-priority",
    "1",
    "--hand-task-weight",
    "0.0",
]


def _run_once(output: pathlib.Path, style_budget: int | None) -> tuple[dict[str, Any], dict[str, np.ndarray], dict[str, float]]:
    output.mkdir(parents=True, exist_ok=True)
    arguments = [*COMMON_ARGS]
    if style_budget is not None:
        arguments.extend(
            ["--post-transfer-style-task-pseudoinverses", str(style_budget)]
        )
    arguments.extend(["--output", str(output)])
    environment = dict(os.environ)
    environment.setdefault("OPENBLAS_NUM_THREADS", "1")
    environment.setdefault("OMP_NUM_THREADS", "1")
    command = [
        "taskset",
        "-c",
        "4",
        "/usr/bin/time",
        "-f",
        "BONESAW_WALL_SECONDS=%e\nBONESAW_USER_SECONDS=%U\nBONESAW_SYSTEM_SECONDS=%S\nBONESAW_MAX_RSS_KIB=%M",
        sys.executable,
        str(FLOATING),
        *arguments,
    ]
    started = time.perf_counter()
    completed = subprocess.run(
        command,
        cwd=ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    elapsed = time.perf_counter() - started
    if completed.returncode != 0:
        raise RuntimeError(
            f"floating corpus failed ({completed.returncode}):\n"
            f"stdout={completed.stdout}\nstderr={completed.stderr}"
        )
    timing: dict[str, float] = {"wall_seconds_parent": elapsed}
    for line in completed.stderr.splitlines():
        if not line.startswith("BONESAW_"):
            continue
        key, value = line.split("=", 1)
        timing[key.removeprefix("BONESAW_").lower()] = float(value)
    document = json.loads((output / "floating-walk-metrics.json").read_text())
    raw = dict(np.load(output / "floating-walk-raw.npz", allow_pickle=False))
    return document, raw, timing


def _semantic_equal(left: dict[str, np.ndarray], right: dict[str, np.ndarray], end: int | None = None) -> bool:
    ignored = {"step_ns"}
    for key in sorted(set(left) & set(right) - ignored):
        a = left[key]
        b = right[key]
        if end is not None:
            a = a[:end]
            b = b[:end]
        if not np.array_equal(a, b, equal_nan=True):
            return False
    return True


def _first_divergence(left: dict[str, np.ndarray], right: dict[str, np.ndarray]) -> int | None:
    first: int | None = None
    for key in sorted(set(left) & set(right) - {"step_ns"}):
        a = left[key]
        b = right[key]
        equal = (a == b) | (np.isnan(a) & np.isnan(b))
        if a.ndim > 1:
            equal = np.all(equal, axis=tuple(range(1, a.ndim)))
        indices = np.flatnonzero(~equal)
        if indices.size:
            tick = int(indices[0])
            first = tick if first is None else min(first, tick)
    return first


def _timing_row(document: dict[str, Any], timing: dict[str, float], repeat: int, semantic_exact: bool) -> dict[str, Any]:
    metrics = document["metrics"]
    latency = metrics["latency_us"]
    return {
        "repeat": repeat,
        "semantic_exact": semantic_exact,
        "p50_us": float(latency["p50"]),
        "p95_us": float(latency["p95"]),
        "p99_us": float(latency["p99"]),
        "p99_9_us": float(latency["p99_9"]),
        "maximum_us": float(latency["max"]),
        "over_5ms": int(metrics["deadline_misses"]["5ms"]),
        "over_20ms": int(metrics["deadline_misses"]["20ms"]),
        "wall_ms": 1_000.0 * timing.get("wall_seconds", timing["wall_seconds_parent"]),
        "user_ms": 1_000.0 * timing.get("user_seconds", float("nan")),
        "system_ms": 1_000.0 * timing.get("system_seconds", float("nan")),
        "max_rss_mib": timing.get("max_rss_kib", float("nan")) / 1024.0,
    }


def build_result(
    profiles: dict[str, tuple[dict[str, Any], dict[str, np.ndarray]]],
    repeats: dict[str, list[dict[str, Any]]],
) -> dict[str, Any]:
    unbounded_document, unbounded_raw = profiles["unbounded"]
    candidate_document, candidate_raw = profiles["post_transfer_style2"]
    clipped = np.flatnonzero(candidate_raw["support_trajectory_tube_clipped"])
    first_clip = int(clipped[0]) if clipped.size else None
    exhaustion = np.flatnonzero(
        candidate_raw["cumulative_low_authority_budget_exhausted_mask"]
    )
    candidate_metrics = candidate_document["metrics"]
    unbounded_metrics = unbounded_document["metrics"]
    pretransfer_exact = first_clip is not None and _semantic_equal(
        unbounded_raw, candidate_raw, first_clip + 1
    )
    first_divergence = _first_divergence(unbounded_raw, candidate_raw)
    timing_summary = {}
    for name, rows in repeats.items():
        timing_summary[name] = {
            "repeats": rows,
            "p99_mean_us": float(np.mean([row["p99_us"] for row in rows])),
            "p99_stddev_us": float(np.std([row["p99_us"] for row in rows])),
            "deadline_misses_5ms": int(sum(row["over_5ms"] for row in rows)),
            "maximum_us": float(max(row["maximum_us"] for row in rows)),
            "max_rss_mib": float(max(row["max_rss_mib"] for row in rows)),
            "all_semantic_exact": all(row["semantic_exact"] for row in rows),
        }
    return {
        "schema": "bonesaw.g1-post-transfer-style-budget-r290.v1",
        "revision": REVISION,
        "execution": {
            "policy_steps": 0,
            "physics_steps": 0,
            "ticks": int(unbounded_metrics["ticks"]),
            "dt_seconds": float(unbounded_metrics["dt_seconds"]),
            "timing_cpu": 4,
            "repeat_count": len(next(iter(repeats.values()))),
        },
        "experiment": {
            "nominal_style_budget": None,
            "post_transfer_style_budget": 2,
            "first_support_tube_clip_tick": first_clip,
            "arm_semantics": "arm_after_clip_tick",
            "pretransfer_inclusive_end_tick": None if first_clip is None else first_clip,
            "post_transfer_exhaustion_ticks": [int(tick) for tick in exhaustion],
            "first_semantic_divergence_tick": first_divergence,
            "pretransfer_semantic_exact": pretransfer_exact,
        },
        "profiles": {
            "unbounded": {
                "p50_us": float(unbounded_metrics["latency_us"]["p50"]),
                "p99_us": float(unbounded_metrics["latency_us"]["p99"]),
                "maximum_us": float(unbounded_metrics["latency_us"]["max"]),
                "over_5ms": int(unbounded_metrics["deadline_misses"]["5ms"]),
                "root_rms_m": float(unbounded_metrics["root_tracking_rms_m"]),
                "com_rms_m": float(unbounded_metrics["center_of_mass_tracking_rms_m"]),
                "foot_rms_m": float(unbounded_metrics["stance_foot_tracking_rms_m"]),
                "maximum_dynamics_residual": float(unbounded_metrics["maximum_dynamics_residual"]),
                "maximum_contact_residual": float(unbounded_metrics["maximum_contact_acceleration_residual"]),
                "status_counts": unbounded_metrics["status_counts"],
            },
            "post_transfer_style2": {
                "p50_us": float(candidate_metrics["latency_us"]["p50"]),
                "p99_us": float(candidate_metrics["latency_us"]["p99"]),
                "maximum_us": float(candidate_metrics["latency_us"]["max"]),
                "over_5ms": int(candidate_metrics["deadline_misses"]["5ms"]),
                "root_rms_m": float(candidate_metrics["root_tracking_rms_m"]),
                "com_rms_m": float(candidate_metrics["center_of_mass_tracking_rms_m"]),
                "foot_rms_m": float(candidate_metrics["stance_foot_tracking_rms_m"]),
                "maximum_dynamics_residual": float(candidate_metrics["maximum_dynamics_residual"]),
                "maximum_contact_residual": float(candidate_metrics["maximum_contact_acceleration_residual"]),
                "status_counts": candidate_metrics["status_counts"],
            },
        },
        "timing": timing_summary,
        "verdict": {
            "pretransfer_exact": pretransfer_exact,
            "hard_residuals_passed": bool(
                candidate_metrics["maximum_dynamics_residual"] <= 1.0e-8
                and candidate_metrics["maximum_contact_acceleration_residual"] <= 1.0e-8
            ),
            "post_transfer_p99_under_5ms": bool(
                timing_summary["post_transfer_style2"]["p99_mean_us"] < 5_000.0
            ),
            "tracking_non_regressed": bool(
                candidate_metrics["root_tracking_rms_m"]
                <= unbounded_metrics["root_tracking_rms_m"]
                and candidate_metrics["center_of_mass_tracking_rms_m"]
                <= unbounded_metrics["center_of_mass_tracking_rms_m"]
                and candidate_metrics["stance_foot_tracking_rms_m"]
                <= unbounded_metrics["stance_foot_tracking_rms_m"]
            ),
            "authority_admitted": False,
            "promoted": False,
        },
    }


def validate_result(result: dict[str, Any]) -> None:
    if result["revision"] != REVISION:
        raise ValueError("unexpected R290 revision")
    if result["execution"]["policy_steps"] or result["execution"]["physics_steps"]:
        raise ValueError("R290 must remain policy/physics free")
    experiment = result["experiment"]
    if experiment["first_support_tube_clip_tick"] != 819:
        raise ValueError("unexpected first support-tube clip tick")
    if not experiment["pretransfer_semantic_exact"]:
        raise ValueError("post-transfer budget changed the nominal prefix")
    if experiment["first_semantic_divergence_tick"] != 1152:
        raise ValueError("unexpected post-transfer semantic divergence tick")
    if experiment["post_transfer_exhaustion_ticks"] != [1152]:
        raise ValueError("unexpected Style-2 exhaustion witness")
    if not result["verdict"]["hard_residuals_passed"]:
        raise ValueError("post-transfer candidate violated a hard residual")
    if result["verdict"]["authority_admitted"]:
        raise ValueError("R290 cannot admit authority")


def build_report(result: dict[str, Any]) -> str:
    p = result["profiles"]
    timing = result["timing"]
    candidate = p["post_transfer_style2"]
    baseline = p["unbounded"]
    rows = []
    for name, row in (("unbounded", baseline), ("post-transfer Style-2", candidate)):
        rows.append(
            "| "
            + " | ".join(
                [
                    name,
                    f"{row['p50_us']:.1f}",
                    f"{row['p99_us']:.1f}",
                    f"{row['maximum_us']:.1f}",
                    str(row["over_5ms"]),
                    f"{row['root_rms_m']:.3f}",
                    f"{row['com_rms_m']:.3f}",
                    f"{row['foot_rms_m']:.3f}",
                ]
            )
            + " |"
        )
    repeat_rows = []
    for name in ("unbounded", "post_transfer_style2"):
        for row in timing[name]["repeats"]:
            repeat_rows.append(
                "| "
                + " | ".join(
                    [
                        name,
                        str(row["repeat"]),
                        f"{row['p99_us']:.1f}",
                        f"{row['maximum_us']:.1f}",
                        str(row["over_5ms"]),
                        f"{row['wall_ms']:.1f}",
                        f"{row['max_rss_mib']:.1f}",
                        "exact" if row["semantic_exact"] else "diverged",
                    ]
                )
                + " |"
            )
    return "\n".join(
        [
            "# G1 post-transfer Style budget · R290",
            "",
            "> Mechanism **RETAINED DEFAULT-OFF** · nominal prefix **EXACT** · hard residuals **PASS** · walking promotion **REJECTED**.",
            "",
            "R290 arms the existing terminal Style projected-solve ceiling only after the causal support-transfer projector clips an intent command. The clipping tick remains nominal; the bounded solve starts on the next tick. Invariant, viability, intent, equality, bounds, and contact rows are unchanged.",
            "",
            "## Frozen policy/physics-free replay",
            "",
            "| profile | p50 µs | p99 µs | max µs | >5 ms | root RMS m | CoM RMS m | stance-foot RMS m |",
            "|---|---:|---:|---:|---:|---:|---:|---:|",
            *rows,
            "",
            f"The first support-tube clip is tick `{result['experiment']['first_support_tube_clip_tick']}`. The inclusive prefix through that tick is bit-exact. Style-2 exhausts once at tick `{result['experiment']['post_transfer_exhaustion_ticks'][0]}`, which is the first capped solve and first semantic divergence; hard dynamics/contact residuals remain `{candidate['maximum_dynamics_residual']:.3e}` / `{candidate['maximum_contact_residual']:.3e}`.",
            "",
            "## Pinned CPU-4 repeats, jitter, and process memory",
            "",
            "| profile | repeat | p99 µs | max µs | >5 ms | wall ms | max RSS MiB | semantics |",
            "|---|---:|---:|---:|---:|---:|---:|---|",
            *repeat_rows,
            "",
            f"Across five repeats the candidate mean p99 is {timing['post_transfer_style2']['p99_mean_us']:.1f} µs (σ {timing['post_transfer_style2']['p99_stddev_us']:.1f} µs), still above the 5,000 µs target. The candidate raises root/CoM/foot RMS by {100*(candidate['root_rms_m']/baseline['root_rms_m']-1):.1f}%/{100*(candidate['com_rms_m']/baseline['com_rms_m']-1):.1f}%/{100*(candidate['foot_rms_m']/baseline['foot_rms_m']-1):.1f}%; therefore a lower-authority solve is not promoted merely because it removes some dense work. RSS is process-level evidence from `/usr/bin/time`, not a per-step Rust allocation claim.",
            "",
            "## Decision",
            "",
            "Retain the gate as a typed, default-off research primitive. Reject Style-2 for walking: it misses the timing gate and regresses closed-loop tracking after the first exhaustion. The nominal unbounded profile remains the reference. A future continuous authority design needs an explicit tracking-error or progress budget rather than a static Style call count.",
            "",
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    if args.repeats <= 0:
        raise SystemExit("--repeats must be positive")
    if args.run:
        temporary = pathlib.Path(tempfile.mkdtemp(prefix="bonesaw-r290-"))
        try:
            profiles: dict[str, tuple[dict[str, Any], dict[str, np.ndarray]]] = {}
            repeats: dict[str, list[dict[str, Any]]] = {
                "unbounded": [],
                "post_transfer_style2": [],
            }
            first_raw_by_profile: dict[str, dict[str, np.ndarray]] = {}
            for name, budget in (("unbounded", None), ("post_transfer_style2", 2)):
                for repeat in range(args.repeats):
                    document, raw, process_timing = _run_once(
                        temporary / f"{name}-{repeat}", budget
                    )
                    if repeat == 0:
                        profiles[name] = (document, raw)
                        first_raw_by_profile[name] = raw
                    semantic_exact = _semantic_equal(first_raw_by_profile[name], raw)
                    repeats[name].append(
                        _timing_row(document, process_timing, repeat, semantic_exact)
                    )
            result = build_result(profiles, repeats)
            validate_result(result)
            RESULT_DIR.mkdir(parents=True, exist_ok=True)
            RESULT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
            REPORT.write_text(build_report(result))
            WEB_REPORT.write_text(
                render_report_html(
                    REPORT.read_text(), title="G1 post-transfer Style budget · R290"
                )
            )
            print(f"validated {RESULT}")
        finally:
            shutil.rmtree(temporary, ignore_errors=True)
    else:
        result = json.loads(RESULT.read_text())
        validate_result(result)
        if not args.check_only:
            REPORT.write_text(build_report(result))
            WEB_REPORT.write_text(
                render_report_html(
                    REPORT.read_text(), title="G1 post-transfer Style budget · R290"
                )
            )
        print(f"validated {RESULT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
