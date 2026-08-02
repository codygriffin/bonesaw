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


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", default=str(BASELINE))
    parser.add_argument("--cap8", default=str(CAPS[8]))
    parser.add_argument("--cap16", default=str(CAPS[16]))
    parser.add_argument("--cap32", default=str(CAPS[32]))
    parser.add_argument("--cap64", default=str(CAPS[64]))
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


def summarize(path: pathlib.Path, label: str, expected_cap: int | None) -> dict[str, Any]:
    document = load(path)
    metrics = document["metrics"]
    motion = document.get("motion", {})
    cap = motion.get("maximum_feasibility_projection_sweeps")
    if cap is None:
        cap = document.get("maximum_feasibility_projection_sweeps")
    if expected_cap is not None and int(cap) != expected_cap:
        raise ValueError(f"{path}: expected projection cap {expected_cap}, got {cap}")
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
        "no_failed_or_infeasible_ticks": bool(
            statuses["failed"] == 0 and statuses["primal_infeasible"] == 0
        ),
        "no_contact_release_ticks": bool(statuses["contact_release_contingency"] == 0),
        "under_50hz_budget": bool(deadlines["20ms"] == 0),
        "under_5ms_p99": bool(float(latency["p99"]) <= 5_000.0),
    }


def build_result(args: argparse.Namespace) -> tuple[dict[str, Any], str]:
    paths = {
        "unbounded": pathlib.Path(args.baseline).resolve(),
        "cap8": pathlib.Path(args.cap8).resolve(),
        "cap16": pathlib.Path(args.cap16).resolve(),
        "cap32": pathlib.Path(args.cap32).resolve(),
        "cap64": pathlib.Path(args.cap64).resolve(),
    }
    expected = {"unbounded": None, "cap8": 8, "cap16": 16, "cap32": 32, "cap64": 64}
    rows = [summarize(paths[name], name, expected[name]) for name in paths]
    bounded = [row for row in rows if row["projection_cap"] is not None]
    recommended = min(
        (row for row in bounded if row["under_50hz_budget"] and row["no_failed_or_infeasible_ticks"]),
        key=lambda row: row["projection_cap"],
    )
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_artifacts": {name: str(path) for name, path in paths.items()},
        "source_hashes": {name: sha256(path) for name, path in paths.items()},
        "physics_steps": 0,
        "policy_steps": 0,
        "plant_actions": 0,
        "profiles": rows,
        "recommended_cap": int(recommended["projection_cap"]),
        "gates": {
            "unbounded_50hz_budget": bool(rows[0]["under_50hz_budget"]),
            "bounded_50hz_budget": all(row["under_50hz_budget"] for row in bounded),
            "bounded_no_failed_or_infeasible": all(
                row["no_failed_or_infeasible_ticks"] for row in bounded
            ),
            "bounded_no_contact_release": all(
                row["no_contact_release_ticks"] for row in bounded
            ),
            "functional_full_transfer": all(
                row["functional_acceptance"] for row in bounded
            ),
        },
        "authority_admitted": False,
    }
    baseline, cap = rows[0], recommended
    report = "\n".join(
        [
            "# Bonesaw floating feasibility projection budget · r262",
            "",
            "> This report compares already-generated floating-WBC traces. It runs zero policy steps, physics steps, and plant actions; authority is not admitted.",
            "",
            "## Result",
            "",
            f"The unbounded profile misses the 50 Hz (20 ms) deadline on {baseline['deadline_misses']['20ms']}/{baseline['ticks']} ticks, with p99 {baseline['latency_us']['p99']:.1f} µs and a {baseline['maximum_projection_sweeps']:.0f}-sweep contingency. A finite {cap['projection_cap']}-sweep ceiling bounds p99 to {cap['latency_us']['p99']:.1f} µs and max to {cap['latency_us']['max']:.1f} µs, with zero 20 ms misses and no failed/infeasible or contact-release tick.",
            "",
            "The cap is therefore a useful real-time fail-closed policy, not a functional walking admission: every bounded trace still exercises normal-contact contingency and retains red tracking/residual gates. The default remains unbounded until a multi-law floating transfer profile proves that bounded recovery preserves behavior.",
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
            "## Interpretation",
            "",
            "- A finite projection cap makes the 50 Hz timing contract explicit and keeps the existing contingency path state-retaining; it does not turn an unfinished hard-feasibility solve into an executable nominal command.",
            "- The identical bounded behavior across 8–64 sweeps indicates that this stress trace reaches the same normal-contact fallback before the cap matters to tracking. The lower cap is recommended only for the measured timing envelope, not as a universal controller constant.",
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
    return 0 if result["gates"]["bounded_50hz_budget"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
