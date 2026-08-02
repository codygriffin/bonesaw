#!/usr/bin/env python3
"""Compare bounded and unbounded floating-WBC feasibility budgets.

This is an artifact-only evaluation.  The underlying trace contains WBC
queries, but no policy, MuJoCo, or plant actions.  A finite projection ceiling
is a fail-closed real-time policy: it bounds work and leaves the existing
normal-contact contingency responsible for retaining a commandable state.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import render_report_html


REVISION = "g1-floating-projection-budget-profile-r262"
BASELINE = pathlib.Path(
    "benchmarks/results/floating-g1-transfer-latest/floating-walk-metrics.json"
)
CAPS = {
    8: pathlib.Path(
        "benchmarks/results/floating-g1-transfer-r262-cap8/floating-walk-metrics.json"
    ),
    16: pathlib.Path(
        "benchmarks/results/floating-g1-transfer-r262-cap16/floating-walk-metrics.json"
    ),
    32: pathlib.Path(
        "benchmarks/results/floating-g1-transfer-r262-cap32/floating-walk-metrics.json"
    ),
    64: pathlib.Path(
        "benchmarks/results/floating-g1-transfer-r262-cap64/floating-walk-metrics.json"
    ),
}
NATIVE_TRIALS = tuple(
    pathlib.Path(
        f"benchmarks/results/{REVISION}/native-polish2-trial-{trial}/"
        "floating-walk-metrics.json"
    )
    for trial in range(5)
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", default=str(BASELINE))
    parser.add_argument("--cap8", default=str(CAPS[8]))
    parser.add_argument("--cap16", default=str(CAPS[16]))
    parser.add_argument("--cap32", default=str(CAPS[32]))
    parser.add_argument("--cap64", default=str(CAPS[64]))
    for trial, path in enumerate(NATIVE_TRIALS):
        parser.add_argument(f"--native-trial-{trial}", default=str(path))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_FLOATING_PROJECTION_BUDGET_PROFILE_R262.html"
    )
    return parser.parse_args(argv)


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def load(path: pathlib.Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    value = json.loads(path.read_text())
    if not isinstance(value, dict) or not isinstance(value.get("metrics"), dict):
        raise ValueError(f"{path}: expected floating-walk metrics object")
    return value


def finite_trace(metrics_path: pathlib.Path) -> bool:
    raw_path = metrics_path.with_name("floating-walk-raw.npz")
    if not raw_path.is_file():
        return False
    with np.load(raw_path, allow_pickle=False) as raw:
        names = ("root_tracked", "root_quaternion_wxyz", "tracked_positions", "q", "v")
        return all(name in raw and bool(np.all(np.isfinite(raw[name]))) for name in names)


def summarize(
    path: pathlib.Path,
    label: str,
    expected_cap: int | None,
    expected_iterations: int = 64,
) -> dict[str, Any]:
    document = load(path)
    metrics = document["metrics"]
    motion = document.get("motion", {})
    cap = motion.get("maximum_feasibility_projection_sweeps")
    if cap is None:
        cap = document.get("maximum_feasibility_projection_sweeps")
    if expected_cap is not None and int(cap) != expected_cap:
        raise ValueError(f"{path}: expected projection cap {expected_cap}, got {cap}")
    iterations = motion.get("maximum_feasibility_iterations")
    if iterations is None:
        iterations = document.get("maximum_feasibility_iterations")
    if iterations is None:
        # Legacy unbounded baseline predates explicit serialization of the
        # constructor's fixed 64-iteration default.
        iterations = 64
    if int(iterations) != expected_iterations:
        raise ValueError(
            f"{path}: expected feasibility iterations {expected_iterations}, got {iterations}"
        )
    statuses = metrics["status_counts"]
    deadlines = metrics["deadline_misses"]
    latency = metrics["latency_us"]
    solver = metrics["status_solver_work"]
    max_sweeps = max(
        float(row["feasibility_projection_sweeps"]["max"])
        for row in solver.values()
    )
    return {
        "label": label,
        "artifact": str(path),
        "sha256": sha256(path),
        "projection_cap": None if cap is None else int(cap),
        "feasibility_iterations": int(iterations),
        "ticks": int(metrics["ticks"]),
        "duration_seconds": float(metrics["duration_seconds"]),
        "latency_us": {
            key: float(latency[key])
            for key in ("p50", "p99", "p99_9", "max")
        },
        "jitter_p99_us": float(metrics["jitter_abs_delta_us"]["p99"]),
        "deadline_misses": {key: int(value) for key, value in deadlines.items()},
        "status_counts": {key: int(value) for key, value in statuses.items()},
        "maximum_projection_sweeps": max_sweeps,
        "root_tracking_rms_m": float(metrics["root_tracking_rms_m"]),
        "foot_tracking_rms_m": float(metrics["foot_tracking_rms_m"]),
        "maximum_dynamics_residual": float(metrics["maximum_dynamics_residual"]),
        "maximum_contact_acceleration_residual": float(
            metrics["maximum_contact_acceleration_residual"]
        ),
        "finite_trace": finite_trace(path),
        "functional_acceptance": bool(metrics["acceptance"]["functional_passed"]),
        "real_time_acceptance": bool(metrics["acceptance"]["real_time_passed"]),
        "runtime": metrics["runtime"],
        "no_failed_or_infeasible_ticks": bool(
            statuses["failed"] == 0 and statuses["primal_infeasible"] == 0
        ),
        "no_contact_release_ticks": bool(statuses["contact_release_contingency"] == 0),
        "under_50hz_budget": bool(deadlines["20ms"] == 0),
        "under_5ms_p99": bool(float(latency["p99"]) <= 5_000.0),
    }


def exact_non_timing_replay(paths: list[pathlib.Path]) -> dict[str, Any]:
    names: tuple[str, ...] | None = None
    baseline: dict[str, np.ndarray] = {}
    exact: set[str] = set()
    raw_hashes: dict[str, str] = {}
    for trial, metrics_path in enumerate(paths):
        raw_path = metrics_path.with_name("floating-walk-raw.npz")
        raw_hashes[f"trial{trial}"] = sha256(raw_path)
        with np.load(raw_path, allow_pickle=False) as raw:
            current_names = tuple(sorted(name for name in raw.files if name != "step_ns"))
            if names is None:
                names = current_names
                baseline = {name: raw[name].copy() for name in current_names}
                exact = set(current_names)
            elif current_names != names:
                raise ValueError(f"{raw_path}: non-timing field set differs from trial 0")
            else:
                for name in tuple(exact):
                    if not np.array_equal(baseline[name], raw[name], equal_nan=True):
                        exact.remove(name)
    total = 0 if names is None else len(names)
    return {
        "exact_fields": len(exact),
        "total_fields": total,
        "changed_fields": sorted(set(names or ()) - exact),
        "raw_sha256": raw_hashes,
    }


def build_result(args: argparse.Namespace) -> tuple[dict[str, Any], str]:
    paths = {
        "unbounded": pathlib.Path(args.baseline).resolve(),
        "cap8": pathlib.Path(args.cap8).resolve(),
        "cap16": pathlib.Path(args.cap16).resolve(),
        "cap32": pathlib.Path(args.cap32).resolve(),
        "cap64": pathlib.Path(args.cap64).resolve(),
    }
    native_paths = [
        pathlib.Path(getattr(args, f"native_trial_{trial}")).resolve()
        for trial in range(len(NATIVE_TRIALS))
    ]
    expected = {"unbounded": None, "cap8": 8, "cap16": 16, "cap32": 32, "cap64": 64}
    rows = [summarize(paths[name], name, expected[name]) for name in paths]
    native_rows = [
        summarize(path, f"native-polish2-trial-{trial}", 8, expected_iterations=2)
        for trial, path in enumerate(native_paths)
    ]
    native_replay = exact_non_timing_replay(native_paths)
    bounded = [row for row in rows if row["projection_cap"] is not None]
    recommended = min(
        (row for row in bounded if row["under_50hz_budget"] and row["no_failed_or_infeasible_ticks"]),
        key=lambda row: row["projection_cap"],
    )
    native_ticks = sum(row["ticks"] for row in native_rows)
    native_five_ms_misses = sum(row["deadline_misses"]["5ms"] for row in native_rows)
    native_twenty_ms_misses = sum(
        row["deadline_misses"]["20ms"] for row in native_rows
    )
    native_python_gc_collections = sum(
        int(row["runtime"]["python_gc_delta"]["collections"])
        for row in native_rows
    )
    native_profile = {
        "build": "RUSTFLAGS=-C target-cpu=native",
        "cpu_affinity": 4,
        "projection_cap": 8,
        "feasibility_iterations": 2,
        "trials": native_rows,
        "ticks": native_ticks,
        "worst_p99_us": max(row["latency_us"]["p99"] for row in native_rows),
        "observed_maximum_us": max(row["latency_us"]["max"] for row in native_rows),
        "worst_jitter_p99_us": max(row["jitter_p99_us"] for row in native_rows),
        "five_ms_misses": native_five_ms_misses,
        "twenty_ms_misses": native_twenty_ms_misses,
        "python_gc_collections": native_python_gc_collections,
        "total_wall_ns": sum(int(row["runtime"]["call_wall_ns"]) for row in native_rows),
        "total_process_cpu_ns": sum(
            int(row["runtime"]["process_cpu_ns"]) for row in native_rows
        ),
        "total_thread_cpu_ns": sum(
            int(row["runtime"]["thread_cpu_ns"]) for row in native_rows
        ),
        "maximum_rss_delta_bytes": max(
            int(row["runtime"]["rss_delta_bytes"]) for row in native_rows
        ),
        "maximum_peak_rss_bytes": max(
            int(row["runtime"]["peak_rss_bytes"]) for row in native_rows
        ),
        "maximum_python_tracemalloc_peak_bytes": max(
            int(row["runtime"]["python_tracemalloc_peak_bytes"])
            for row in native_rows
        ),
        "non_timing_replay": native_replay,
    }
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_artifacts": {name: str(path) for name, path in paths.items()},
        "source_hashes": {name: sha256(path) for name, path in paths.items()},
        "native_source_artifacts": [str(path) for path in native_paths],
        "native_source_hashes": {
            f"trial{trial}": sha256(path)
            for trial, path in enumerate(native_paths)
        },
        "physics_steps": 0,
        "policy_steps": 0,
        "plant_actions": 0,
        "profiles": rows,
        "recommended_cap": int(recommended["projection_cap"]),
        "recommended_execution_profile": {
            "projection_cap": 8,
            "feasibility_iterations": 2,
            "build": native_profile["build"],
            "cpu_affinity": native_profile["cpu_affinity"],
            "scope": "measured fail-closed execution only",
        },
        "host_native_execution_profile": native_profile,
        "gates": {
            "unbounded_50hz_budget": bool(rows[0]["under_50hz_budget"]),
            "bounded_50hz_budget": all(row["under_50hz_budget"] for row in bounded),
            "bounded_no_failed_or_infeasible": all(
                row["no_failed_or_infeasible_ticks"] for row in bounded
            ),
            "bounded_no_contact_release": all(
                row["no_contact_release_ticks"] for row in bounded
            ),
            "host_native_zero_5ms_overruns": native_five_ms_misses == 0,
            "host_native_zero_20ms_overruns": native_twenty_ms_misses == 0,
            "host_native_exact_non_timing_replay": (
                native_replay["exact_fields"] == native_replay["total_fields"]
            ),
            "host_native_zero_python_gc": native_python_gc_collections == 0,
            "functional_full_transfer": all(
                row["functional_acceptance"] for row in bounded
            ),
        },
        "authority_admitted": False,
    }
    baseline, cap = rows[0], recommended
    native = native_profile
    report = "\n".join(
        [
            "# Bonesaw floating feasibility projection budget · r262",
            "",
            "> This report compares already-generated floating-WBC traces. It runs zero policy steps, physics steps, and plant actions; authority is not admitted.",
            "",
            "## Result",
            "",
            f"The unbounded profile misses the 50 Hz (20 ms) deadline on {baseline['deadline_misses']['20ms']}/{baseline['ticks']} ticks, with p99 {baseline['latency_us']['p99']:.1f} µs and a {baseline['maximum_projection_sweeps']:.0f}-sweep contingency. A finite {cap['projection_cap']}-sweep ceiling bounds generic-build p99 to {cap['latency_us']['p99']:.1f} µs, but its {cap['latency_us']['max']:.1f} µs maximum still misses 5 ms.",
            "",
            f"The complete measured execution profile additionally limits feasibility polish to two iterations and uses the spec's CPU-ISA build (`-C target-cpu=native`) pinned to logical CPU 4. Across five independent 600-tick processes it records {native['five_ms_misses']}/{native['ticks']} five-millisecond misses, {native['twenty_ms_misses']}/{native['ticks']} twenty-millisecond misses, {native['worst_p99_us']:.1f} µs worst per-process p99, and {native['observed_maximum_us']:.1f} µs observed maximum. All {native_replay['exact_fields']}/{native_replay['total_fields']} non-timing arrays replay exactly and Python performs zero collections.",
            "",
            "This closes the measured ordinary-process 200 Hz execution overrun for the fail-closed profile, not the controller behavior gate: the two-iteration trace spends 354/600 ticks in normal-contact contingency and retains red tracking/residual gates. The default remains unbounded until a multi-law floating transfer profile proves that bounded recovery preserves behavior.",
            "",
            "## Sweep",
            "",
            "| profile | projection cap | p50 µs | p99 µs | max µs | 20 ms misses | 5 ms p99 | contingency ticks | dynamics residual | foot RMS m | functional |",
            "|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---|",
        ]
        + [
            f"| {row['label']} | {row['projection_cap'] if row['projection_cap'] is not None else 'unbounded'} | {row['latency_us']['p50']:.1f} | {row['latency_us']['p99']:.1f} | {row['latency_us']['max']:.1f} | {row['deadline_misses']['20ms']} | {'PASS' if row['under_5ms_p99'] else 'FAIL'} | {row['status_counts']['normal_contact_contingency']} | {row['maximum_dynamics_residual']:.3g} | {row['foot_tracking_rms_m']:.3f} | {'PASS' if row['functional_acceptance'] else 'FAIL'} |"
            for row in rows
        ]
        + [
            "",
            "## Host-native 200 Hz repeats",
            "",
            "| trial | p50 µs | p99 µs | max µs | 5 ms misses | 20 ms misses | contingency ticks | GC collections |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
        + [
            f"| {trial} | {row['latency_us']['p50']:.1f} | {row['latency_us']['p99']:.1f} | {row['latency_us']['max']:.1f} | {row['deadline_misses']['5ms']} | {row['deadline_misses']['20ms']} | {row['status_counts']['normal_contact_contingency']} | {row['runtime']['python_gc_delta']['collections']} |"
            for trial, row in enumerate(native_rows)
        ]
        + [
            "",
            "## Process resources",
            "",
            "| trial | wall ms | process CPU ms | thread CPU ms | CPU/wall | jitter p99 µs | RSS Δ MiB | peak RSS MiB | involuntary switches |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
        + [
            f"| {trial} | {row['runtime']['call_wall_ns'] / 1e6:.1f} | {row['runtime']['process_cpu_ns'] / 1e6:.1f} | {row['runtime']['thread_cpu_ns'] / 1e6:.1f} | {row['runtime']['process_cpu_to_wall_ratio']:.3f} | {row['jitter_p99_us']:.1f} | {row['runtime']['rss_delta_bytes'] / (1 << 20):.3f} | {row['runtime']['peak_rss_bytes'] / (1 << 20):.1f} | {row['runtime']['usage_delta']['involuntary_context_switches']} |"
            for trial, row in enumerate(native_rows)
        ]
        + [
            "",
            "Python `tracemalloc` peaks at "
            f"{native['maximum_python_tracemalloc_peak_bytes']} bytes and observes "
            f"{native['python_gc_collections']} collections. It does not observe Rust allocations; the native controller allocation sentinel remains authoritative.",
            "",
            "## Functional cost of the timing profile",
            "",
            f"The exact replay has {native_rows[0]['status_counts']['solved']} solved, {native_rows[0]['status_counts']['solved_with_slack']} solved-with-slack, and {native_rows[0]['status_counts']['normal_contact_contingency']} normal-contact-contingency ticks per process. Root/foot RMS are {native_rows[0]['root_tracking_rms_m']:.3f}/{native_rows[0]['foot_tracking_rms_m']:.3f} m; maximum dynamics/contact-acceleration residuals are {native_rows[0]['maximum_dynamics_residual']:.3g}/{native_rows[0]['maximum_contact_acceleration_residual']:.3g}. These values are failures, not hidden by the timing gate.",
            "",
            "## Interpretation",
            "",
            "- Finite projection and polish budgets make the timing contract explicit and keep the existing contingency path state-retaining; they do not turn an unfinished hard-feasibility solve into an executable nominal command.",
            "- The generic 8–64 sweep rows close the apparent freeze and 50 Hz deadline, but only the separately fingerprinted host-native 8-sweep/two-polish profile demonstrates zero five-millisecond overruns. The profile is not a universal controller constant.",
            f"- Exact repeat covers {native_replay['exact_fields']} non-timing arrays. Python GC is zero; Rust hot-loop allocation remains governed by the existing native controller allocation sentinel because `tracemalloc` cannot observe the Rust allocator.",
            "- The next functional gate is a fresh floating transfer holdout with contact-law/geometry improvements and a typed degraded-state metric; no policy or GPU batching is implied by this report.",
        ]
    ) + "\n"
    return result, report


def main() -> int:
    args = parse_args()
    result, report = build_result(args)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-floating-projection-budget-profile-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_FLOATING_PROJECTION_BUDGET_PROFILE.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw floating projection budget · r262"))
    print(json.dumps({"recommended_cap": result["recommended_cap"], "gates": result["gates"]}, indent=2, sort_keys=True))
    return 0 if result["gates"]["host_native_zero_5ms_overruns"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
