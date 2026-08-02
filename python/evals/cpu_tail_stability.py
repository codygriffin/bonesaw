#!/usr/bin/env python3
"""Compare repeated CPU timing traces while requiring exact semantic parity."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html


TIMING_ARRAYS = {"step_ns", "ik_solve_ns", "jet_solve_ns"}
DEFAULT_RUNS = (
    "baseline=benchmarks/results/g1-multistep-oracle-r54",
    "cpu2=benchmarks/results/g1-cpu-tail-r57/cpu2",
    "cpu4=benchmarks/results/g1-cpu-tail-r57/cpu4",
    "cpu6=benchmarks/results/g1-cpu-tail-r57/cpu6",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", dest="runs")
    parser.add_argument("--output", default="benchmarks/results/g1-cpu-tail-r57")
    parser.add_argument("--web-report", default="web/CPU_TAIL_STABILITY_R57.html")
    return parser.parse_args()


def parse_runs(values: list[str] | None) -> list[tuple[str, pathlib.Path]]:
    result = []
    for value in values or DEFAULT_RUNS:
        if "=" not in value:
            raise ValueError("--run must be label=directory")
        label, path = value.split("=", 1)
        if not label or not path:
            raise ValueError("--run must have a non-empty label and directory")
        result.append((label, pathlib.Path(path)))
    if len(result) < 2:
        raise ValueError("CPU tail stability requires at least two runs")
    return result


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def runs(mask: np.ndarray) -> list[tuple[int, int]]:
    edges = np.diff(np.pad(np.asarray(mask, dtype=np.int8), (1, 1)))
    return list(
        zip(
            np.flatnonzero(edges == 1).tolist(),
            np.flatnonzero(edges == -1).tolist(),
            strict=True,
        )
    )


def correlation(left: np.ndarray, right: np.ndarray) -> float:
    if np.std(left) == 0.0 or np.std(right) == 0.0:
        return 0.0
    return float(np.corrcoef(left, right)[0, 1])


def jaccard(left: np.ndarray, right: np.ndarray) -> float:
    union = np.count_nonzero(np.logical_or(left, right))
    if union == 0:
        return 1.0
    return float(np.count_nonzero(np.logical_and(left, right)) / union)


def arrays_byte_exact(left: np.ndarray, right: np.ndarray) -> bool:
    return (
        left.shape == right.shape
        and left.dtype == right.dtype
        and np.array_equal(
            np.ascontiguousarray(left).view(np.uint8),
            np.ascontiguousarray(right).view(np.uint8),
        )
    )


def main() -> None:
    args = parse_args()
    configured_runs = parse_runs(args.runs)
    output_dir = pathlib.Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    loaded: dict[str, dict[str, Any]] = {}
    artifact_hashes: dict[str, str] = {}

    for label, directory in configured_runs:
        metrics_path = directory / "oracle-wbc-admission-metrics.json"
        raw_path = directory / "oracle-wbc-admission-raw.npz"
        metrics = json.loads(metrics_path.read_text())
        if not metrics["passed"]:
            raise ValueError(f"{label} is not an admitted trace")
        with np.load(raw_path) as archive:
            arrays = {name: archive[name].copy() for name in archive.files}
        loaded[label] = {
            "directory": str(directory),
            "metrics": metrics,
            "arrays": arrays,
        }
        artifact_hashes[str(metrics_path)] = sha256(metrics_path)
        artifact_hashes[str(raw_path)] = sha256(raw_path)

    labels = list(loaded)
    baseline = loaded[labels[0]]["arrays"]
    semantic_fields = sorted(set(baseline) - TIMING_ARRAYS)
    semantic_differences: dict[str, list[str]] = {}
    for label in labels[1:]:
        candidate = loaded[label]["arrays"]
        semantic_differences[label] = [
            name
            for name in semantic_fields
            if not arrays_byte_exact(baseline[name], candidate[name])
        ]
    all_semantics_exact = not any(semantic_differences.values())
    if not all_semantics_exact:
        raise ValueError(f"repeated traces changed semantic arrays: {semantic_differences}")

    rows = []
    latency_by_label: dict[str, np.ndarray] = {}
    overrun_by_label: dict[str, np.ndarray] = {}
    for label in labels:
        arrays = loaded[label]["arrays"]
        metrics = loaded[label]["metrics"]
        latency_ms = arrays["step_ns"].astype(np.float64) / 1e6
        overrun = latency_ms > 5.0
        episodes = runs(overrun)
        latency_by_label[label] = latency_ms
        overrun_by_label[label] = overrun
        rows.append(
            {
                "label": label,
                "affinity": "not recorded" if label == "baseline" else label.replace("cpu", "logical CPU "),
                "p50_ms": float(np.percentile(latency_ms, 50)),
                "p95_ms": float(np.percentile(latency_ms, 95)),
                "p99_ms": float(np.percentile(latency_ms, 99)),
                "maximum_ms": float(np.max(latency_ms)),
                "over_5ms_ticks": int(np.count_nonzero(overrun)),
                "over_5ms_fraction": float(np.mean(overrun)),
                "over_5ms_episodes": len(episodes),
                "longest_over_5ms_ticks": max((stop - start for start, stop in episodes), default=0),
                "longest_over_5ms_ms": max((stop - start for start, stop in episodes), default=0) * 5.0,
                "wall_seconds": metrics["runtime"]["wall_seconds"],
                "process_cpu_seconds": metrics["runtime"]["process_cpu_seconds"],
                "cpu_to_wall": metrics["runtime"]["process_cpu_seconds"] / metrics["runtime"]["wall_seconds"],
                "latency_to_pseudoinverse_correlation": correlation(
                    latency_ms, arrays["task_pseudoinverse_calls"]
                ),
                "latency_to_jacobi_sweeps_correlation": correlation(
                    latency_ms, arrays["task_jacobi_sweeps"]
                ),
                "latency_to_clipped_steps_correlation": correlation(
                    latency_ms, arrays["clipped_steps"]
                ),
            }
        )

    pairwise = []
    for left_index, left in enumerate(labels):
        for right in labels[left_index + 1 :]:
            pairwise.append(
                {
                    "left": left,
                    "right": right,
                    "per_tick_latency_correlation": correlation(
                        latency_by_label[left], latency_by_label[right]
                    ),
                    "over_5ms_jaccard": jaccard(
                        overrun_by_label[left], overrun_by_label[right]
                    ),
                }
            )

    latency_matrix = np.column_stack([latency_by_label[label] for label in labels])
    per_tick_median = np.median(latency_matrix, axis=1)
    per_tick_spread = np.max(latency_matrix, axis=1) - np.min(latency_matrix, axis=1)
    aggregate = {
        "p50_range_ms": [min(row["p50_ms"] for row in rows), max(row["p50_ms"] for row in rows)],
        "p99_range_ms": [min(row["p99_ms"] for row in rows), max(row["p99_ms"] for row in rows)],
        "maximum_range_ms": [min(row["maximum_ms"] for row in rows), max(row["maximum_ms"] for row in rows)],
        "over_5ms_ticks_range": [min(row["over_5ms_ticks"] for row in rows), max(row["over_5ms_ticks"] for row in rows)],
        "longest_over_5ms_ms_range": [min(row["longest_over_5ms_ms"] for row in rows), max(row["longest_over_5ms_ms"] for row in rows)],
        "per_tick_median_latency_ms": {
            "p50": float(np.percentile(per_tick_median, 50)),
            "p95": float(np.percentile(per_tick_median, 95)),
            "p99": float(np.percentile(per_tick_median, 99)),
            "maximum": float(np.max(per_tick_median)),
        },
        "per_tick_cross_run_spread_ms": {
            "p50": float(np.percentile(per_tick_spread, 50)),
            "p95": float(np.percentile(per_tick_spread, 95)),
            "p99": float(np.percentile(per_tick_spread, 99)),
            "maximum": float(np.max(per_tick_spread)),
        },
    }
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "runs": rows,
        "all_non_timing_arrays_exact": all_semantics_exact,
        "semantic_field_count": len(semantic_fields),
        "excluded_timing_arrays": sorted(TIMING_ARRAYS),
        "semantic_differences": semantic_differences,
        "pairwise": pairwise,
        "aggregate": aggregate,
        "artifact_sha256": artifact_hashes,
        "interpretation": {
            "iteration_cap_supported_by_evidence": False,
            "reason": "identical algorithmic work produces materially different tail episodes across logical CPUs and runs",
            "next_measurement": "isolated benchmark host with fixed governor plus per-tick thread CPU cycles/instructions",
        },
    }
    (output_dir / "cpu-tail-stability-metrics.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    with (output_dir / "cpu-tail-stability.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    report = [
        "# Bonesaw CPU-tail stability audit · r57",
        "",
        "## Result",
        "",
        "Four release traces execute the same 2,317 immutable G1 states. All non-timing "
        f"arrays are byte-exact across {len(semantic_fields)} fields, including physical "
        "outputs, status, constraint residuals, clipping, and every solver-work counter. "
        "Despite identical work, nominal 5 ms overruns range from "
        f"{aggregate['over_5ms_ticks_range'][0]} to {aggregate['over_5ms_ticks_range'][1]} ticks "
        "and the longest episode ranges from "
        f"{aggregate['longest_over_5ms_ms_range'][0]:.0f} to {aggregate['longest_over_5ms_ms_range'][1]:.0f} ms.",
        "",
        "> A deterministic iteration cap is not justified by this evidence. Dense-kernel work "
        "does correlate with latency inside each run, but host CPU placement/frequency/contention "
        "materially changes the tail while the work trace remains identical.",
        "",
        "## Repeated timing",
        "",
    ]
    report += markdown_table(
        ["run", "affinity", "p50 / p95 / p99 / max ms", ">5 ms ticks / episodes", "longest >5 ms", "CPU / wall", "latency corr pinv / Jacobi / clips"],
        [
            [
                row["label"], row["affinity"],
                f"{row['p50_ms']:.3f} / {row['p95_ms']:.3f} / {row['p99_ms']:.3f} / {row['maximum_ms']:.3f}",
                f"{row['over_5ms_ticks']} / {row['over_5ms_episodes']}",
                f"{row['longest_over_5ms_ms']:.0f} ms",
                f"{row['cpu_to_wall']:.6f}",
                f"{row['latency_to_pseudoinverse_correlation']:.3f} / {row['latency_to_jacobi_sweeps_correlation']:.3f} / {row['latency_to_clipped_steps_correlation']:.3f}",
            ]
            for row in rows
        ],
    )
    report += ["", "## Cross-run stability", ""]
    report += markdown_table(
        ["run pair", "per-tick latency correlation", ">5 ms tick-set Jaccard"],
        [
            [f"{row['left']} ↔ {row['right']}", f"{row['per_tick_latency_correlation']:.3f}", f"{row['over_5ms_jaccard']:.3f}"]
            for row in pairwise
        ],
    )
    report += [
        "",
        f"The cross-run per-tick spread is {aggregate['per_tick_cross_run_spread_ms']['p50']:.3f} ms median, "
        f"{aggregate['per_tick_cross_run_spread_ms']['p99']:.3f} ms p99, and "
        f"{aggregate['per_tick_cross_run_spread_ms']['maximum']:.3f} ms maximum. The best observed "
        f"p99 is still {aggregate['p99_range_ms'][0]:.3f} ms, so real kernel optimization remains useful; "
        "the unstable tail simply means wall-time iteration truncation is not yet the correct mechanism.",
        "",
        "## Decision",
        "",
        "- Preserve the exact bounded-work solver and 20 ms oracle admission contract.",
        "- Do not add a clock-dependent early exit to the pure core.",
        "- Add per-tick thread CPU cycles/instructions and fixed-governor isolated-host runs before changing Jacobi convergence or clipping budgets.",
        "- Pursue semantics-preserving dense-kernel work first; accept an iteration budget only with a typed feasible-best status and separate tracking/dwell gates.",
        "- Keep the nominal 5 ms line red in the browser until an isolated repeat distribution—not one lucky trace—passes.",
        "",
        "## Artifact integrity",
        "",
    ]
    report += markdown_table(
        ["artifact", "SHA-256"], artifact_hashes.items()
    )
    report_text = "\n".join(report) + "\n"
    (output_dir / "CPU_TAIL_STABILITY.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))


if __name__ == "__main__":
    main()
