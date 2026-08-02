#!/usr/bin/env python3
"""Continuous authority and persistence evaluation over immutable WBC states.

The evaluator does not roll state, invoke a policy, or run a simulator. It
normalizes each typed Rust diagnostic independently, then reports exposure and
duration curves without combining them into a single health score.
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
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html


DT = 0.005
TASK_NAMES = (
    "root_angular",
    "root_horizontal",
    "root_height",
    "joint_posture",
    "protected_joint_acceleration",
    "center_of_mass",
    "centroidal_angular_momentum_rate",
    "contact_force_style",
    "actuator_torque_style",
    "frame_angular_0",
    "point_0",
    "point_1",
    "point_2",
    "point_3",
)
DWELL_TICKS = (1, 2, 4, 20, 100)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--oracle", default="benchmarks/results/g1-multistep-oracle-r54"
    )
    parser.add_argument(
        "--output", default="benchmarks/results/g1-authority-over-time-r56"
    )
    parser.add_argument(
        "--web-report", default="web/AUTHORITY_OVER_TIME_R56.html"
    )
    return parser.parse_args()


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
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "maximum": float(np.max(values)),
    }


def runs(mask: np.ndarray) -> list[tuple[int, int]]:
    mask = np.asarray(mask, dtype=bool)
    edges = np.diff(np.pad(mask.astype(np.int8), (1, 1)))
    starts = np.flatnonzero(edges == 1)
    stops = np.flatnonzero(edges == -1)
    return list(zip(starts.tolist(), stops.tolist(), strict=True))


def dwell_curve(mask: np.ndarray) -> list[dict[str, int | float]]:
    episodes = runs(mask)
    result = []
    for dwell in DWELL_TICKS:
        retained = [(start, stop) for start, stop in episodes if stop - start >= dwell]
        result.append(
            {
                "minimum_dwell_ticks": dwell,
                "minimum_dwell_ms": dwell * DT * 1_000.0,
                "episode_count": len(retained),
                "ticks_in_retained_episodes": sum(stop - start for start, stop in retained),
            }
        )
    return result


def ema(values: np.ndarray, time_constant_seconds: float) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    alpha = 1.0 - math.exp(-DT / time_constant_seconds)
    output = np.empty_like(values)
    output[0] = values[0]
    for index in range(1, values.size):
        output[index] = output[index - 1] + alpha * (values[index] - output[index - 1])
    return output


def signal_summary(
    pressure: np.ndarray,
    warning: float,
    critical: float,
    semantics: str,
) -> dict[str, Any]:
    warning_mask = pressure >= warning
    critical_mask = pressure >= critical
    warning_runs = runs(warning_mask)
    critical_runs = runs(critical_mask)
    exposures = {
        f"{time_constant:g}s": distribution(ema(pressure, time_constant))
        for time_constant in (0.25, 1.0, 5.0)
    }
    return {
        "semantics": semantics,
        "warning_threshold": warning,
        "critical_threshold": critical,
        "pressure": distribution(pressure),
        "warning": {
            "ticks": int(np.count_nonzero(warning_mask)),
            "fraction": float(np.mean(warning_mask)),
            "episodes": len(warning_runs),
            "longest_run_ticks": max((stop - start for start, stop in warning_runs), default=0),
            "longest_run_seconds": max((stop - start for start, stop in warning_runs), default=0) * DT,
            "dwell_curve": dwell_curve(warning_mask),
        },
        "critical": {
            "ticks": int(np.count_nonzero(critical_mask)),
            "fraction": float(np.mean(critical_mask)),
            "episodes": len(critical_runs),
            "longest_run_ticks": max((stop - start for start, stop in critical_runs), default=0),
            "longest_run_seconds": max((stop - start for start, stop in critical_runs), default=0) * DT,
            "dwell_curve": dwell_curve(critical_mask),
        },
        "leaky_exposure": exposures,
    }


def heatmap_svg(
    pressure_by_signal: dict[str, np.ndarray],
    thresholds: dict[str, tuple[float, float]],
) -> str:
    labels = list(pressure_by_signal)
    bins = 232
    width = 1_120
    label_width = 220
    plot_width = width - label_width - 20
    row_height = 26
    height = 42 + row_height * len(labels)
    parts = [
        f'<section class="authority-heatmap"><h2>Continuous authority heatmap</h2>',
        '<p>Green is below warning, amber is warning pressure, and red is the signal-specific critical boundary. Rows are independent; this is not an aggregate score.</p>',
        f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="Authority pressure over time">',
        '<text x="220" y="20" class="axis">0 s</text>',
        f'<text x="{width - 54}" y="20" class="axis">11.6 s</text>',
    ]
    for row, label in enumerate(labels):
        values = pressure_by_signal[label]
        warning, critical = thresholds[label]
        y = 34 + row * row_height
        parts.append(f'<text x="4" y="{y + 14}" class="label">{html.escape(label.replace("_", " "))}</text>')
        for column in range(bins):
            start = column * values.size // bins
            stop = max(start + 1, (column + 1) * values.size // bins)
            value = float(np.max(values[start:stop]))
            if value >= critical:
                color = "#ef756a"
            elif value >= warning:
                color = "#e1af5b"
            else:
                ratio = min(1.0, value / max(warning, 1e-12))
                color = "#315b4d" if ratio > 0.5 else "#203b34"
            x = label_width + column * plot_width / bins
            cell_width = plot_width / bins + 0.3
            parts.append(f'<rect x="{x:.2f}" y="{y}" width="{cell_width:.2f}" height="18" fill="{color}"/>')
    parts.append("</svg></section>")
    return "".join(parts)


def main() -> None:
    args = parse_args()
    oracle_dir = pathlib.Path(args.oracle)
    output_dir = pathlib.Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = oracle_dir / "oracle-wbc-admission-metrics.json"
    raw_path = oracle_dir / "oracle-wbc-admission-raw.npz"
    metrics = json.loads(metrics_path.read_text())
    if not metrics["passed"]:
        raise ValueError("continuous authority baseline must be an admitted oracle")
    if float(metrics["dt_seconds"]) != DT:
        raise ValueError("authority evaluator currently requires the 5 ms oracle contract")

    with np.load(raw_path) as raw:
        task_rms = raw["task_rms"].astype(np.float64)
        task_clipped = raw["task_clipped"].astype(bool)
        maximum_hard_residual = np.maximum.reduce(
            (
                raw["dynamics_residual"],
                raw["contact_residual"],
                raw["maximum_constraint_violation"],
            )
        ).astype(np.float64)
        support_margin = raw["minimum_support_margin"].astype(np.float64)
        joint_margin_deg = np.rad2deg(raw["minimum_joint_margin_rad"].astype(np.float64))
        actuator_utilization = raw["maximum_torque_utilization"].astype(np.float64)
        step_us = raw["step_ns"].astype(np.float64) / 1_000.0

    task = {name: task_rms[:, index] for index, name in enumerate(TASK_NAMES)}
    clipped = {name: task_clipped[:, index] for index, name in enumerate(TASK_NAMES)}
    support_reserve = support_margin - float(metrics["configuration"]["minimum_contact_cop_margin_m"])
    pressure_by_signal = {
        "hard_constraints": maximum_hard_residual / 1e-8,
        "finite_support": 1.0 - np.clip(support_reserve / 0.005, 0.0, 1.0),
        "invariant_root": np.maximum(task["root_angular"], task["root_height"]),
        "viability_transfer": np.maximum(task["root_horizontal"], task["center_of_mass"]),
        "viability_effectors": np.maximum.reduce(
            (
                task["point_0"] / 0.1,
                task["point_1"] / 0.1,
                task["frame_angular_0"],
            )
        ),
        "joint_position": 1.0 - np.clip(joint_margin_deg / 15.0, 0.0, 1.0),
        "actuator_effort": actuator_utilization,
        "solver_5ms": step_us / 5_000.0,
        "solver_20ms": step_us / 20_000.0,
        "preference_clipping": clipped["joint_posture"].astype(np.float64),
        "style_clipping": np.logical_or(
            clipped["contact_force_style"], clipped["actuator_torque_style"]
        ).astype(np.float64),
    }
    thresholds = {
        "hard_constraints": (0.1, 1.0),
        "finite_support": (0.5, 1.0),
        "invariant_root": (0.8, 1.0),
        "viability_transfer": (0.8, 1.0),
        "viability_effectors": (0.8, 1.0),
        "joint_position": (2.0 / 3.0, 1.0),
        "actuator_effort": (0.8, 1.0),
        "solver_5ms": (0.8, 1.0),
        "solver_20ms": (0.8, 1.0),
        "preference_clipping": (1.0, 2.0),
        "style_clipping": (1.0, 2.0),
    }
    semantics = {
        "hard_constraints": "maximum rigid-body/contact/original-row residual divided by 1e-8",
        "finite_support": "consumption of the first 5 mm reserve beyond the mandatory 5 mm erosion",
        "invariant_root": "maximum root attitude/height task RMS divided by its 1-unit contract",
        "viability_transfer": "maximum horizontal-root/CoM task RMS divided by its 1 m/s² contract",
        "viability_effectors": "maximum linear/angular effector residual divided by its task contract",
        "joint_position": "consumption of a 15 degree evaluation headroom band; zero margin is critical",
        "actuator_effort": "maximum authored actuator-limit utilization",
        "solver_5ms": "tick time divided by the nominal 5 ms servo period; observational, not r54 admission",
        "solver_20ms": "tick time divided by the declared r54 admission deadline",
        "preference_clipping": "binary legal clipping of the joint-posture Preference task",
        "style_clipping": "binary legal clipping of force or torque Style tasks",
    }
    summaries = {
        name: signal_summary(values, *thresholds[name], semantics[name])
        for name, values in pressure_by_signal.items()
    }

    names = list(pressure_by_signal)
    pressure_matrix = np.column_stack([pressure_by_signal[name] for name in names])
    dominant_indices = np.argmax(pressure_matrix, axis=1)
    dominant_names = np.asarray(names, dtype=object)[dominant_indices]
    dominant_counts = {
        name: int(np.count_nonzero(dominant_names == name)) for name in names
    }
    output = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "evaluation_boundary": metrics["evaluation_boundary"],
        "ticks": int(pressure_matrix.shape[0]),
        "dt_seconds": DT,
        "duration_seconds": pressure_matrix.shape[0] * DT,
        "no_aggregate_health_score": True,
        "dominant_signal_is_view_ordering_not_a_rollup": True,
        "signals": summaries,
        "dominant_signal_counts": dominant_counts,
        "source_artifacts": {
            str(metrics_path): sha256(metrics_path),
            str(raw_path): sha256(raw_path),
        },
    }
    (output_dir / "authority-over-time-metrics.json").write_text(
        json.dumps(output, indent=2) + "\n"
    )

    csv_path = output_dir / "authority-over-time.csv"
    with csv_path.open("w", newline="") as stream:
        fieldnames = ["tick", "time_seconds", "dominant_signal", *names]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for tick in range(pressure_matrix.shape[0]):
            writer.writerow(
                {
                    "tick": tick,
                    "time_seconds": tick * DT,
                    "dominant_signal": dominant_names[tick],
                    **{
                        name: float(pressure_by_signal[name][tick]) for name in names
                    },
                }
            )

    report = [
        "# Bonesaw continuous authority over time · r56",
        "",
        "## Result",
        "",
        "The same 2,317 immutable G1 oracle states are reinterpreted as independent "
        "continuous capability signals. No policy, state integration, simulator, or physics "
        "is introduced. Hard feasibility remains instantaneous; soft tracking, compute "
        "pressure, and legal Preference/Style clipping additionally receive episode-duration, "
        "dwell, and 0.25/1/5 second leaky-exposure evidence.",
        "",
        "> There is deliberately no aggregate health score. “Dominant signal” only orders "
        "rows for display at one tick; it cannot hide a hard failure or convert unrelated units.",
        "",
        "## Signal summary",
        "",
    ]
    report += markdown_table(
        [
            "signal",
            "max pressure",
            "warning ticks / longest",
            "critical ticks / longest",
            "1 s exposure p99",
            "meaning",
        ],
        [
            [
                name,
                f"{summary['pressure']['maximum']:.3f}",
                f"{summary['warning']['ticks']} / {summary['warning']['longest_run_seconds'] * 1000:.0f} ms",
                f"{summary['critical']['ticks']} / {summary['critical']['longest_run_seconds'] * 1000:.0f} ms",
                f"{summary['leaky_exposure']['1s']['p99']:.3f}",
                summary["semantics"],
            ]
            for name, summary in summaries.items()
        ],
    )
    report += [
        "",
        "## Persistence curves",
        "",
        "Each row counts warning episodes that survive at least the stated continuous dwell. "
        "This lets an adapter distinguish a one-tick soft excursion from sustained loss of "
        "authority. Hard constraint criticality still has zero permitted dwell.",
        "",
    ]
    persistence_rows = []
    for name, summary in summaries.items():
        curve = summary["warning"]["dwell_curve"]
        persistence_rows.append(
            [name, *[entry["episode_count"] for entry in curve]]
        )
    report += markdown_table(
        ["signal", "≥5 ms", "≥10 ms", "≥20 ms", "≥100 ms", "≥500 ms"],
        persistence_rows,
    )
    report += [
        "",
        "## Interpretation",
        "",
        "- Hard constraints never reach their critical boundary; no waiting policy is legal or needed there.",
        "- The finite-support row stays at its critical display boundary because the solver deliberately consumes the complete optional reserve while preserving the mandatory 5 mm erosion. This is low geometric reserve, not a CoP violation.",
        "- Root Invariant and effector tracking remain far below their contracts. Viability transfer approaches but does not cross its contract, which makes it the meaningful continuous tracking pressure.",
        "- The nominal 5 ms servo period has 76 overruns, while the declared 20 ms admission deadline has none. Any 5 ms deployment needs further CPU-tail work or a slower supervisory cadence; the evidence must not be relabeled.",
        "- Preference and Style clipping persist for long runs. They are legal nullspace relaxation and must remain visible even though physical admission is green.",
        "",
        "## Artifacts",
        "",
        "`authority-over-time.csv` retains one row per tick and per signal. "
        "`authority-over-time-metrics.json` retains thresholds, distributions, warning/critical "
        "episodes, dwell curves, and leaky exposures. Source NPZ and JSON hashes are embedded.",
        "",
    ]
    report_text = "\n".join(report)
    (output_dir / "AUTHORITY_OVER_TIME.md").write_text(report_text)
    rendered = render_report_html(report_text)
    chart = heatmap_svg(pressure_by_signal, thresholds)
    rendered = rendered.replace(
        "</main></body>",
        chart
        + "<style>.authority-heatmap{margin-top:48px}.authority-heatmap svg{display:block;width:100%;height:auto;background:#151a1e;border:1px solid #293138}.authority-heatmap .label,.authority-heatmap .axis{fill:#aeb9b5;font:12px system-ui,sans-serif}.authority-heatmap .axis{fill:#6f7b77;font-size:10px}</style></main></body>",
    )
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(rendered)


if __name__ == "__main__":
    main()
