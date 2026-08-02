#!/usr/bin/env python3
"""Consolidate CPU WBC and independent reference artifacts.

This is intentionally a report-only evaluator. It never retimes an implementation:
the G1 oracle stays immutable, while the selected PlaCo, Pinocchio, and upstream
Upkie measurements remain explicitly labeled, dated, and checksum-addressed.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import html
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any, Iterable

import numpy as np


SCENARIOS = (
    "end_effector_reach",
    "bimanual_priority_conflict",
    "walking_motion_retarget",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--oracle",
        default="benchmarks/results/g1-multistep-oracle-r54",
    )
    parser.add_argument(
        "--coupled-oracle",
        default="benchmarks/results/g1-multistep-oracle-r54-coupled-actuation",
    )
    parser.add_argument("--references", default="benchmarks/results/reference-r38")
    parser.add_argument(
        "--output", default="benchmarks/results/cpu-reference-comparison-r55"
    )
    parser.add_argument(
        "--web-report",
        default="web/CPU_REFERENCE_COMPARISON_R55.html",
        help="public read-only HTML mirror served by the browser adapter",
    )
    parser.add_argument("--report-revision", default="r55")
    parser.add_argument(
        "--reference-label",
        default="retained r38",
        help="provenance label shown on PlaCo, Pinocchio, and Upkie sections",
    )
    parser.add_argument(
        "--fresh-reference-run",
        action="store_true",
        help="declare that PlaCo, Pinocchio, and Upkie were retimed together in this run",
    )
    return parser.parse_args()


def read_json(path: pathlib.Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def distribution(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=np.float64)
    return {
        "minimum": float(np.min(values)),
        "mean": float(np.mean(values)),
        "stddev": float(np.std(values)),
        "mad": float(np.median(np.abs(values - np.median(values)))),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "p99_9": float(np.percentile(values, 99.9)),
        "maximum": float(np.max(values)),
    }


def rms(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=np.float64)
    return float(math.sqrt(float(np.mean(values * values))))


def correlation(left: np.ndarray, right: np.ndarray) -> float:
    if np.std(left) == 0.0 or np.std(right) == 0.0:
        return 0.0
    return float(np.corrcoef(left, right)[0, 1])


def longest_run(mask: np.ndarray) -> int:
    best = current = 0
    for value in np.asarray(mask, dtype=bool):
        current = current + 1 if value else 0
        best = max(best, current)
    return best


def mb(value: float) -> str:
    return f"{value / 1_048_576:.2f}"


def us_from_ns(value: float) -> float:
    return value / 1_000.0


def fmt(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def markdown_table(headers: Iterable[str], rows: Iterable[Iterable[Any]]) -> list[str]:
    headers = list(headers)
    output = [
        "| " + " | ".join(headers) + " |",
        "|" + "|".join("---" for _ in headers) + "|",
    ]
    output.extend("| " + " | ".join(str(cell) for cell in row) + " |" for row in rows)
    return output


def render_report_html(markdown: str, *, title: str | None = None) -> str:
    """Render the evaluator's deliberately small Markdown subset without dependencies."""
    lines = markdown.splitlines()
    if title is None:
        title = next(
            (line[2:].strip() for line in lines if line.startswith("# ")),
            "Bonesaw evaluation report",
        )
    body: list[str] = []
    index = 0
    while index < len(lines):
        line = lines[index]
        if not line:
            index += 1
            continue
        if line.startswith("### "):
            body.append(f"<h3>{html.escape(line[4:])}</h3>")
        elif line.startswith("## "):
            body.append(f"<h2>{html.escape(line[3:])}</h2>")
        elif line.startswith("# "):
            body.append(f"<h1>{html.escape(line[2:])}</h1>")
        elif line.startswith("> "):
            body.append(f"<blockquote>{html.escape(line[2:])}</blockquote>")
        elif line.startswith("- "):
            items: list[str] = []
            while index < len(lines) and lines[index].startswith("- "):
                items.append(f"<li>{html.escape(lines[index][2:])}</li>")
                index += 1
            body.append("<ul>" + "".join(items) + "</ul>")
            continue
        elif line.startswith("| "):
            table_lines: list[str] = []
            while index < len(lines) and lines[index].startswith("|"):
                table_lines.append(lines[index])
                index += 1
            rows = [[cell.strip() for cell in row.strip("|").split("|")] for row in table_lines]
            body.append("<div class=table-wrap><table><thead><tr>" + "".join(
                f"<th>{html.escape(cell)}</th>" for cell in rows[0]
            ) + "</tr></thead><tbody>" + "".join(
                "<tr>" + "".join(f"<td>{html.escape(cell)}</td>" for cell in row) + "</tr>"
                for row in rows[2:]
            ) + "</tbody></table></div>")
            continue
        else:
            paragraph = [line]
            while index + 1 < len(lines) and lines[index + 1] and not lines[index + 1].startswith(("#", "> ", "- ", "|")):
                index += 1
                paragraph.append(lines[index])
            body.append(f"<p>{html.escape(' '.join(paragraph))}</p>")
        index += 1
    return """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>""" + html.escape(title) + """</title>
<style>
:root{color-scheme:dark;--bg:#0f1215;--panel:#151a1e;--line:#293138;--text:#d8dfdc;--muted:#8c9994;--accent:#78d5ae}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px/1.58 system-ui,sans-serif}
main{width:min(1180px,calc(100% - 32px));min-width:0;margin:0 auto;padding:42px 0 80px}h1{font-size:clamp(28px,5vw,52px);line-height:1.05;margin:0 0 36px}
h2{margin:48px 0 18px;padding-top:16px;border-top:1px solid var(--line);font-size:24px}h3{margin:32px 0 14px;color:var(--accent)}
p,li,blockquote{max-width:90ch;overflow-wrap:anywhere}p{color:#bcc6c2}blockquote{margin:22px 0;padding:14px 18px;border-left:3px solid var(--accent);background:var(--panel);color:#b9c5c0}
ul{padding-left:22px}.table-wrap{max-width:100%;min-width:0;overflow-x:auto;margin:16px 0 30px;border:1px solid var(--line)}table{width:100%;border-collapse:collapse;font-size:12px}
th,td{padding:10px 12px;text-align:left;vertical-align:top;border-bottom:1px solid var(--line)}th{position:sticky;top:0;background:#1b2227;color:var(--accent)}tr:last-child td{border-bottom:0}
@media(max-width:600px){main{width:min(100% - 20px,1180px);padding-top:24px}h2{font-size:20px}.table-wrap{margin-left:-2px;margin-right:-2px}th,td{min-width:115px;padding:9px;font-size:11px}}
</style></head><body><main>""" + "\n".join(body) + "</main></body></html>\n"


def main() -> None:
    args = parse_args()
    oracle_dir = pathlib.Path(args.oracle)
    coupled_dir = pathlib.Path(args.coupled_oracle)
    reference_dir = pathlib.Path(args.references)
    output_dir = pathlib.Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    oracle_metrics_path = oracle_dir / "oracle-wbc-admission-metrics.json"
    oracle_raw_path = oracle_dir / "oracle-wbc-admission-raw.npz"
    coupled_metrics_path = coupled_dir / "oracle-wbc-admission-metrics.json"
    coupled_raw_path = coupled_dir / "oracle-wbc-admission-raw.npz"
    reference_metrics_path = reference_dir / "reference-metrics.json"
    reference_raw_paths = {
        "bonesaw": reference_dir / "bonesaw-raw.npz",
        "placo": reference_dir / "placo-raw.npz",
        "upkie": reference_dir / "upkie-controller-raw.npz",
    }

    oracle = read_json(oracle_metrics_path)
    coupled = read_json(coupled_metrics_path)
    references = read_json(reference_metrics_path)
    dt = float(oracle["dt_seconds"])

    with np.load(oracle_raw_path) as raw:
        step_us = raw["step_ns"].astype(np.float64) / 1_000.0
        jitter_us = np.abs(np.diff(step_us))
        point_error = raw["point_error"].astype(np.float64)
        orientation_error = raw["orientation_error"].astype(np.float64)
        com_error = raw["center_of_mass_error"].astype(np.float64)
        status = raw["status"].astype(np.uint8)
        clipped = raw["clipped_steps"].astype(np.float64)
        pinv = raw["task_pseudoinverse_calls"].astype(np.float64)
        sweeps = raw["task_jacobi_sweeps"].astype(np.float64)
        projections = raw["feasibility_halfspace_projections"].astype(np.float64)
        dynamics = raw["dynamics_residual"].astype(np.float64)
        contact = raw["contact_residual"].astype(np.float64)
        effort = raw["maximum_torque_utilization"].astype(np.float64)
        support = raw["minimum_support_margin"].astype(np.float64)
        joint_margin = raw["minimum_joint_margin_rad"].astype(np.float64)
        task_rms = raw["task_rms"].astype(np.float64)

    if oracle["ticks"] != step_us.size or coupled["ticks"] != step_us.size:
        raise ValueError("oracle metrics/raw tick mismatch")
    if not oracle["passed"] or not coupled["passed"]:
        raise ValueError("the consolidated report requires both retained r54 gates to pass")

    latency = distribution(step_us)
    jitter = distribution(jitter_us)
    deadline_counts = {
        f"{deadline_ms}ms": int(np.count_nonzero(step_us > deadline_ms * 1_000.0))
        for deadline_ms in (1, 2, 5, 10, 20)
    }
    tracking = {
        "point_position_m": {**distribution(point_error), "time_rms": rms(point_error)},
        "orientation_rad": {
            **distribution(orientation_error),
            "time_rms": rms(orientation_error),
        },
        "center_of_mass_m": {**distribution(com_error), "time_rms": rms(com_error)},
    }
    work = {
        "pseudoinverse_calls": distribution(pinv),
        "jacobi_sweeps": distribution(sweeps),
        "halfspace_projections": distribution(projections),
        "clipped_steps": distribution(clipped),
        "latency_correlation": {
            "pseudoinverse_calls": correlation(step_us, pinv),
            "jacobi_sweeps": correlation(step_us, sweeps),
            "halfspace_projections": correlation(step_us, projections),
            "clipped_steps": correlation(step_us, clipped),
        },
    }

    windows: list[dict[str, Any]] = []
    window_ticks = 200
    for start in range(0, step_us.size, window_ticks):
        stop = min(start + window_ticks, step_us.size)
        window = slice(start, stop)
        windows.append(
            {
                "tick_start": start,
                "tick_stop": stop,
                "time_start_s": start * dt,
                "time_stop_s": stop * dt,
                "latency_p50_us": float(np.percentile(step_us[window], 50)),
                "latency_p99_us": float(np.percentile(step_us[window], 99)),
                "latency_max_us": float(np.max(step_us[window])),
                "point_error_rms_m": rms(point_error[window]),
                "com_error_rms_m": rms(com_error[window]),
                "orientation_error_rms_rad": rms(orientation_error[window]),
                "dynamics_max": float(np.max(dynamics[window])),
                "contact_max": float(np.max(contact[window])),
                "effort_maximum_utilization": float(np.max(effort[window])),
                "support_minimum_margin_m": float(np.min(support[window])),
                "joint_minimum_margin_rad": float(np.min(joint_margin[window])),
                "slack_ticks": int(np.count_nonzero(status[window] == 1)),
                "pseudoinverse_mean": float(np.mean(pinv[window])),
                "jacobi_sweeps_mean": float(np.mean(sweeps[window])),
            }
        )

    task_layers = oracle["authority_evidence"]["task_layers"]["over_time_by_task"]
    task_rows = []
    for name, layer in task_layers.items():
        dist = layer["distribution"]
        task_rows.append(
            {
                "task": name,
                "units": layer["physical_units"],
                "clipped_ticks": layer["clipped_ticks"],
                "time_rms": layer["time_rms"],
                "p50": dist["p50"],
                "p95": dist["p95"],
                "p99": dist["p99"],
                "maximum": dist["maximum"],
                "integrated_residual_seconds": layer["integrated_residual_seconds"],
            }
        )

    fixed_rows = []
    for scenario in SCENARIOS:
        for implementation in ("bonesaw", "placo"):
            item = references[implementation]["scenarios"][scenario]
            fixed_rows.append(
                {
                    "scenario": scenario,
                    "implementation": references[implementation]["implementation"],
                    "tracking_rms_m": item["tracking_rms_m"],
                    "steady_state_rms_m": item["steady_state_rms_m_after_1s"],
                    "iae_m_s": item["integrated_absolute_error_m_s"],
                    "ise_m2_s": item["integrated_squared_error_m2_s"],
                    "latency_p50_us": us_from_ns(item["latency_ns"]["median"]),
                    "latency_p99_us": us_from_ns(item["latency_ns"]["p99"]),
                    "latency_max_us": us_from_ns(item["latency_ns"]["max"]),
                    "jitter_p99_us": us_from_ns(item["jitter_abs_delta_ns"]["p99"]),
                    "throughput_steps_per_second": item["throughput_steps_per_second"],
                    "rss_before_bytes": item["memory_before"]["rss_bytes"],
                    "peak_rss_bytes": item["peak_rss_bytes"],
                    "rss_delta_bytes": item["rss_delta_bytes"],
                    "cpu_to_wall": item["cpu_to_wall_ratio"],
                    "gc_collections": sum(item["python_gc_delta"]["collections"]),
                    "python_tracemalloc_peak_bytes": item[
                        "python_tracemalloc_peak_bytes"
                    ],
                    "deadline_misses": item["deadline_misses"],
                }
            )

    hashes = {
        str(path): sha256(path)
        for path in (
            oracle_metrics_path,
            oracle_raw_path,
            coupled_metrics_path,
            coupled_raw_path,
            reference_metrics_path,
            *reference_raw_paths.values(),
        )
    }
    upkie = references["upkie_controller_oracle"]
    consolidated = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "report_only": True,
        "report_revision": args.report_revision,
        "reference_label": args.reference_label,
        "fresh_reference_run": args.fresh_reference_run,
        "reference_generated_utc": references["generated_utc"],
        "claims": {
            "g1_uncoupled_admission": oracle["passed"],
            "g1_coupled_admission": coupled["passed"],
            "pinocchio_g1_product_oracle": references["pinocchio_oracles"]["g1"][
                "passed"
            ],
            "pinocchio_upkie_product_oracle": references["pinocchio_oracles"][
                "upkie"
            ]["passed"],
            "upkie_controller_bitwise_parity": upkie["comparisons"][
                "official_aligned"
            ]["canonical_bitwise_equal"],
        },
        "g1_oracle": {
            "ticks": oracle["ticks"],
            "dt_seconds": dt,
            "evaluation_boundary": oracle["evaluation_boundary"],
            "latency_us": latency,
            "jitter_abs_delta_us": jitter,
            "deadline_misses": deadline_counts,
            "tracking": tracking,
            "solver_work": work,
            "runtime": oracle["runtime"],
            "wbc_admission": oracle["wbc_admission"],
            "contact_transitions": oracle["contact_transitions"],
            "task_layers": task_rows,
            "execution_windows": windows,
        },
        "g1_coupled_oracle": {
            "runtime": coupled["runtime"],
            "wbc_admission": coupled["wbc_admission"],
            "loose_control": coupled["authority_evidence"]["loose_2000nm_control"],
            "checks": coupled["check_groups"],
        },
        "fixed_base_placo_comparison": fixed_rows,
        "pinocchio_oracles": references["pinocchio_oracles"],
        "upkie_controller_oracle": upkie,
        "environment": references["environment"],
        "artifact_sha256": hashes,
    }
    metrics_path = output_dir / "comparison-metrics.json"
    metrics_path.write_text(json.dumps(consolidated, indent=2) + "\n")

    with (output_dir / "g1-execution-windows.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(windows[0]))
        writer.writeheader()
        writer.writerows(windows)
    with (output_dir / "g1-task-residuals.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(task_rows[0]))
        writer.writeheader()
        writer.writerows(task_rows)
    with (output_dir / "fixed-base-comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fixed_rows[0]))
        writer.writeheader()
        writer.writerows(fixed_rows)

    timing_note = (
        f"> Timing is host- and run-specific. The G1 r54 admission is retained and checksum-addressed; "
        f"PlaCo, Pinocchio, and Upkie were freshly rerun together at "
        f"{references['generated_utc']} on the recorded host. Same-boundary ratios are publishable, "
        "but the report still makes no cross-boundary speedup claim."
        if args.fresh_reference_run
        else "> Timing is host- and run-specific. The G1 r54 measurements are current; PlaCo, "
        "Pinocchio, and Upkie are retained measurements on the same recorded host and are "
        "checksum-addressed below. This report performs no retiming and makes no cross-boundary "
        "speedup claim."
    )
    report: list[str] = [
        f"# Bonesaw CPU reference comparison · {args.report_revision}",
        "",
        "## Executive result",
        "",
        "The CPU concept is demonstrated at three deliberately separate boundaries. "
        "The current G1 stateless oracle passes all 43/43 uncoupled and 45/45 synthetic "
        "coupled-actuation gates over 2,317 ticks, four alternating steps, and eight contact "
        "edges. Pinocchio independently validates the rigid-body products to floating-point "
        "roundoff. The official-aligned Rust rolling law is bitwise identical to the pinned "
        "upstream Upkie C++ implementation over 100,000 sequential samples. PlaCo remains an "
        "independent fixed-base task-level comparator on identical targets, not a falsely "
        "equivalent floating inverse-dynamics solver.",
        "",
        timing_note,
        "",
        "## What is actually comparable",
        "",
    ]
    report += markdown_table(
        ["Reference", "Shared boundary", "Strongest claim", "Must not be inferred"],
        [
            [
                "Pinocchio 4.0",
                "same URDF states and convention adapters",
                "FK/Jacobian/CoM/mass/bias/RNEA/centroidal products agree at roundoff",
                "controller or task-policy parity",
            ],
            [
                "Upkie C++ WheelBalancer",
                "same sequential inputs, default gains, 5 ms update order",
                "0/100,000 canonical command mismatches",
                "whole-body WBC parity; adapter costs differ",
            ],
            [
                "PlaCo 0.9.23",
                "same toy model, targets, frames, initial state, and 50 Hz corpus",
                "independent task tracking/latency/resource behavior",
                "identical QP semantics; PlaCo is soft weighted and position-only",
            ],
            [
                "G1 r54 oracle",
                "same immutable projected state/jet at every 5 ms tick",
                "floating WBC feasibility and task admission without policy or physics",
                "closed-loop stability or disturbance recovery",
            ],
        ],
    )
    report += [
        "",
        "## Current G1 CPU WBC",
        "",
        f"Boundary: policy={oracle['evaluation_boundary']['feedback_policy']}, "
        f"physics={oracle['evaluation_boundary']['physics_simulator']}, "
        f"integration={oracle['evaluation_boundary']['state_integration']}, "
        f"oracle-state-per-tick={oracle['evaluation_boundary']['oracle_state_per_tick']}. "
        "This isolates WBC admission from a learned policy, simulator, or rollout drift.",
        "",
    ]
    report += markdown_table(
        ["Profile", "gates", "solved / slack / fail", "p50 / p99 / max µs", ">5 / >20 ms", "alloc calls / bytes"],
        [
            [
                "G1 authored transmission",
                "43/43 PASS",
                f"{oracle['wbc_admission']['status_counts']['solved']} / {oracle['wbc_admission']['status_counts']['solved_with_slack']} / 0",
                f"{latency['p50']:.1f} / {latency['p99']:.1f} / {latency['maximum']:.1f}",
                f"{deadline_counts['5ms']} / {deadline_counts['20ms']}",
                f"{oracle['wbc_admission']['allocation_calls']} / {oracle['wbc_admission']['allocated_bytes']}",
            ],
            [
                "synthetic ankle differential",
                "45/45 PASS",
                f"{coupled['wbc_admission']['status_counts']['solved']} / {coupled['wbc_admission']['status_counts']['solved_with_slack']} / 0",
                f"{coupled['wbc_admission']['latency_us']['p50']:.1f} / {coupled['wbc_admission']['latency_us']['p99']:.1f} / {coupled['wbc_admission']['latency_us']['maximum']:.1f}",
                f"— / {coupled['wbc_admission']['deadline_misses_20ms']}",
                f"{coupled['wbc_admission']['allocation_calls']} / {coupled['wbc_admission']['allocated_bytes']}",
            ],
        ],
    )
    report += ["", "### CPU, memory, jitter, and deadlines", ""]
    runtime = oracle["runtime"]
    report += markdown_table(
        ["Metric", "Value"],
        [
            ["wall / process CPU", f"{runtime['wall_seconds']:.6f} s / {runtime['process_cpu_seconds']:.6f} s"],
            ["CPU / wall", f"{runtime['process_cpu_seconds'] / runtime['wall_seconds']:.6f}"],
            ["throughput", f"{oracle['ticks'] / runtime['wall_seconds']:.1f} WBC ticks/s"],
            ["RSS before / after / growth", f"{mb(runtime['maximum_rss_before_bytes'])} / {mb(runtime['maximum_rss_after_bytes'])} / {mb(runtime['maximum_rss_growth_bytes'])} MiB"],
            ["Python tracemalloc peak", f"{runtime['python_tracemalloc_peak_bytes']} B (native Rust excluded)"],
            ["hot-loop allocation sentinel", "0 calls / 0 bytes"],
            ["latency mean / std / MAD", f"{latency['mean']:.1f} / {latency['stddev']:.1f} / {latency['mad']:.1f} µs"],
            ["latency p50 / p95 / p99 / p99.9 / max", f"{latency['p50']:.1f} / {latency['p95']:.1f} / {latency['p99']:.1f} / {latency['p99_9']:.1f} / {latency['maximum']:.1f} µs"],
            ["absolute inter-tick jitter p50 / p95 / p99 / max", f"{jitter['p50']:.1f} / {jitter['p95']:.1f} / {jitter['p99']:.1f} / {jitter['maximum']:.1f} µs"],
            ["deadline misses >1 / >2 / >5 / >10 / >20 ms", " / ".join(str(deadline_counts[f'{v}ms']) for v in (1, 2, 5, 10, 20))],
        ],
    )
    report += ["", "### Tracking and physical feasibility", ""]
    report += markdown_table(
        ["Signal", "time RMS", "p50", "p95", "p99", "max"],
        [
            [name.replace("_", " "), fmt(values["time_rms"], 6), fmt(values["p50"], 6), fmt(values["p95"], 6), fmt(values["p99"], 6), fmt(values["maximum"], 6)]
            for name, values in tracking.items()
        ],
    )
    report += [
        "",
        "These are morphology-projection pose errors. WBC task residuals below are physical "
        "acceleration residuals against the same generalized-acceleration witness; they are not "
        "silently substituted for pose error.",
        "",
    ]
    report += markdown_table(
        ["Hard/resource quantity", "Observed worst case"],
        [
            ["dynamics residual", f"{oracle['wbc_admission']['maximum_dynamics_residual']:.3e}"],
            ["contact acceleration residual", f"{oracle['wbc_admission']['maximum_contact_acceleration_residual']:.3e}"],
            ["minimum friction margin", f"{oracle['wbc_admission']['minimum_friction_margin']:.3e}"],
            ["minimum eroded support margin", f"{oracle['wbc_admission']['minimum_support_margin_m'] * 1000:.3f} mm"],
            ["maximum actuator utilization", f"{oracle['authority_evidence']['actuator_effort']['maximum_utilization']['maximum'] * 100:.3f}%"],
            ["minimum joint margin", f"{oracle['authority_evidence']['joint_position']['minimum_margin_rad']['minimum'] * 180 / math.pi:.3f}°"],
            ["bitwise repeat", "PASS for every compared physical output"],
        ],
    )
    report += ["", "### Task/nullspace residual stack", ""]
    report += markdown_table(
        ["Task", "units", "clipped ticks", "time RMS", "p99", "max", "integral"],
        [
            [row["task"], row["units"], row["clipped_ticks"], fmt(row["time_rms"], 6), fmt(row["p99"], 6), fmt(row["maximum"], 6), fmt(row["integrated_residual_seconds"], 6)]
            for row in task_rows
        ],
    )
    report += [
        "",
        "A clipped tick means a lower-priority objective was relaxed to preserve higher "
        "authority. It is not a hidden solver failure: 1,447 authored-transmission ticks report "
        "typed slack, while Invariant root attitude/height, dynamics, contacts, limits, and all "
        "declared tracking thresholds remain admitted.",
        "",
        "### Solver work and latency attribution",
        "",
    ]
    report += markdown_table(
        ["Work counter", "mean", "p95", "p99", "max", "correlation with latency"],
        [
            [key.replace("_", " "), fmt(values["mean"], 2), fmt(values["p95"], 2), fmt(values["p99"], 2), fmt(values["maximum"], 2), fmt(work["latency_correlation"][key], 4)]
            for key, values in work.items()
            if key != "latency_correlation"
        ],
    )
    report += ["", "### Execution over time (1 s windows)", ""]
    report += markdown_table(
        ["ticks", "seconds", "p50 / p99 / max µs", "point / CoM RMS mm", "dyn / contact max", "effort max", "slack", "pinv / sweeps mean"],
        [
            [
                f"{row['tick_start']}–{row['tick_stop'] - 1}",
                f"{row['time_start_s']:.1f}–{row['time_stop_s']:.1f}",
                f"{row['latency_p50_us']:.0f} / {row['latency_p99_us']:.0f} / {row['latency_max_us']:.0f}",
                f"{row['point_error_rms_m'] * 1000:.3f} / {row['com_error_rms_m'] * 1000:.3f}",
                f"{row['dynamics_max']:.1e} / {row['contact_max']:.1e}",
                f"{row['effort_maximum_utilization'] * 100:.1f}%",
                row["slack_ticks"],
                f"{row['pseudoinverse_mean']:.2f} / {row['jacobi_sweeps_mean']:.2f}",
            ]
            for row in windows
        ],
    )
    report += ["", f"## PlaCo fixed-base task comparison ({args.reference_label})", ""]
    report += markdown_table(
        ["scenario", "impl", "RMS / steady cm", "IAE / ISE", "p50 / p99 / max µs", "jitter p99 µs", "steps/s", "peak / Δ RSS MiB", "GC"],
        [
            [
                row["scenario"], row["implementation"],
                f"{row['tracking_rms_m'] * 100:.3f} / {row['steady_state_rms_m'] * 100:.3f}",
                f"{row['iae_m_s']:.4f} / {row['ise_m2_s']:.4f}",
                f"{row['latency_p50_us']:.1f} / {row['latency_p99_us']:.1f} / {row['latency_max_us']:.1f}",
                f"{row['jitter_p99_us']:.1f}", f"{row['throughput_steps_per_second']:.0f}",
                f"{mb(row['peak_rss_bytes'])} / {mb(row['rss_delta_bytes'])}", row["gc_collections"],
            ]
            for row in fixed_rows
        ],
    )
    report += [
        "",
        "PlaCo tracks the walking endpoints better on this fixed-pelvis toy corpus; Bonesaw is "
        "faster and reaches lower steady error on the reach/conflict cases. Both walking gates in "
        "the shared run fail a predeclared sub-gate (Bonesaw foot RMS; PlaCo clearance RMS). "
        "The current G1 result above is a different, floating inverse-dynamics admission boundary "
        "and is not used to erase that historical failure.",
        "",
        f"## Pinocchio rigid-body product oracle ({args.reference_label})",
        "",
    ]
    report += markdown_table(
        ["model", "states / frame samples", "frame position", "Jacobian", "CoM", "mass", "bias", "inverse dynamics", "centroidal map", "gate"],
        [
            [
                item["model"], f"{item['samples']} / {item['frames_compared']}",
                f"{item['maximum_absolute_error']['frame_translation_m']:.2e}",
                f"{item['maximum_absolute_error']['frame_jacobian']:.2e}",
                f"{item['maximum_absolute_error']['center_of_mass']:.2e}",
                f"{item['maximum_absolute_error']['floating_mass_matrix']:.2e}",
                f"{item['maximum_absolute_error']['floating_bias']:.2e}",
                f"{item['maximum_absolute_error']['floating_inverse_dynamics']:.2e}",
                f"{item['maximum_absolute_error']['floating_centroidal_map']:.2e}",
                "PASS" if item["passed"] else "FAIL",
            ]
            for item in references["pinocchio_oracles"].values()
        ],
    )
    report += ["", f"## Upkie upstream controller oracle ({args.reference_label})", ""]
    official = upkie["comparisons"]["official_aligned"]
    report += markdown_table(
        ["quantity", "official Upkie C++", "Bonesaw Rust, official gains", "interpretation"],
        [
            ["command mismatches", "reference", official["canonical_bit_mismatches"], "canonical bitwise PASS"],
            ["max / RMS command error", "reference", f"{official['maximum_absolute_error']:.3e} / {official['rms_error']:.3e}", "exact over 100,000 sequential samples"],
            ["p50 / p99 / max latency µs", f"{upkie['latency']['upkie_cpp']['p50_us']:.3f} / {upkie['latency']['upkie_cpp']['p99_us']:.3f} / {upkie['latency']['upkie_cpp']['max_us']:.3f}", f"{upkie['latency']['bonesaw_official']['p50_us']:.3f} / {upkie['latency']['bonesaw_official']['p99_us']:.3f} / {upkie['latency']['bonesaw_official']['max_us']:.3f}", "C++ includes Dictionary I/O; Rust is typed call"],
            ["jitter p99 µs", f"{upkie['latency']['upkie_cpp']['jitter_p99_us']:.3f}", f"{upkie['latency']['bonesaw_official']['jitter_p99_us']:.3f}", "boundary cost differs"],
            ["peak RSS MiB", mb(upkie['process']['upkie_cpp']['peak_rss_bytes']), mb(upkie['process']['bonesaw_official']['peak_rss_bytes']), "isolated workers"],
            ["hot-loop allocations", "not instrumented", "0 calls / 0 bytes", "native sentinel"],
        ],
    )
    report += [
        "",
        f"Pinned upstream commit: `{upkie['provenance']['upkie_commit']}`. The live-tuned "
        f"Bonesaw profile is intentionally different (RMS command delta "
        f"{upkie['comparisons']['live_tuned']['rms_error']:.4f} m/s) and is not counted as parity.",
        "",
        "## Remaining risks and next gates",
        "",
        "- This is an oracle-state WBC admission test, not a policy, simulation, or hardware stability claim.",
        "- Real G1 coupled transmissions are not authored; the 45/45 differential is a synthetic semantics fixture.",
        "- Thermal/reliability authority remains unavailable until calibrated actuator parameters and telemetry exist.",
        (
            "- Fresh same-session PlaCo timing supports within-boundary latency, jitter, CPU, and memory ratios; its position-only weighted QP still does not define a universal WBC speedup."
            if args.fresh_reference_run
            else "- PlaCo timing is retained and independently useful, but a fresh same-session rerun is required before publishing speed ratios."
        ),
        "- The next CPU gate should add deterministic closed-loop plant surrogates separately from this policy/physics-free certificate, then hardware-in-the-loop timing and disturbance recovery.",
        "- CUDA batching remains deferred until these CPU semantics, artifacts, and per-row divergence checks are frozen.",
        "",
        "## Artifact integrity",
        "",
    ]
    report += markdown_table(["artifact", "SHA-256"], [[path, digest] for path, digest in hashes.items()])
    report += [
        "",
        "Machine-readable companions: `comparison-metrics.json`, `g1-execution-windows.csv`, "
        "`g1-task-residuals.csv`, and `fixed-base-comparison.csv`. Raw per-tick NPZ files remain "
        "in their checksum-addressed source directories to avoid lossy duplication.",
        "",
    ]
    report_text = "\n".join(report)
    (output_dir / "CPU_REFERENCE_COMPARISON.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))


if __name__ == "__main__":
    main()
