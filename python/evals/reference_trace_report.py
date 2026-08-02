#!/usr/bin/env python3
"""Render the fresh reference run with binned views of every retained step trace."""

from __future__ import annotations

import argparse
import csv
import hashlib
import html
import json
import pathlib
from datetime import datetime, timezone
from typing import Any, Callable

import numpy as np

from cpu_reference_report import render_report_html


SCENARIOS = (
    "end_effector_reach",
    "bimanual_priority_conflict",
    "walking_motion_retarget",
)
LABELS = {
    "end_effector_reach": "End-effector reach",
    "bimanual_priority_conflict": "Bimanual priority conflict",
    "walking_motion_retarget": "CMU walking retarget",
}
COLORS = {
    "Bonesaw p50": "#78d5ae",
    "Bonesaw p99": "#b8f3dc",
    "PlaCo p50": "#e0b15a",
    "PlaCo p99": "#f5d899",
    "Bonesaw RMS": "#78d5ae",
    "PlaCo RMS": "#e0b15a",
    "Upkie C++ p50": "#e0b15a",
    "Upkie C++ p99": "#f5d899",
    "Bonesaw Rust p50": "#78d5ae",
    "Bonesaw Rust p99": "#b8f3dc",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="benchmarks/results/reference-r89")
    parser.add_argument(
        "--output", default="benchmarks/results/reference-trace-report-r89"
    )
    parser.add_argument(
        "--web-report", default="web/REFERENCE_COMPARISON_R89.html"
    )
    parser.add_argument("--revision", default="r89")
    parser.add_argument("--windows", type=int, default=100)
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def window_reduce(
    values: np.ndarray, windows: int, reducer: Callable[[np.ndarray], float]
) -> tuple[np.ndarray, np.ndarray, list[tuple[int, int]]]:
    values = np.asarray(values, dtype=np.float64)
    edges = np.linspace(0, values.size, windows + 1, dtype=np.int64)
    reduced = np.empty(windows, np.float64)
    centers = np.empty(windows, np.float64)
    ranges: list[tuple[int, int]] = []
    for index, (start, stop) in enumerate(zip(edges[:-1], edges[1:])):
        reduced[index] = reducer(values[start:stop])
        centers[index] = 0.5 * (start + stop - 1)
        ranges.append((int(start), int(stop)))
    return centers, reduced, ranges


def svg_plot(
    title: str,
    subtitle: str,
    y_label: str,
    series: list[tuple[str, np.ndarray, np.ndarray]],
) -> str:
    width, height = 760, 260
    left, right, top, bottom = 70, 18, 50, 38
    plot_width = width - left - right
    plot_height = height - top - bottom
    all_x = np.concatenate([item[1] for item in series])
    all_y = np.concatenate([item[2] for item in series])
    x_min, x_max = float(np.min(all_x)), float(np.max(all_x))
    y_min = 0.0
    y_max = float(np.max(all_y))
    if y_max <= 0.0:
        y_max = 1.0

    def project_x(value: float) -> float:
        return left + (value - x_min) / max(x_max - x_min, 1e-12) * plot_width

    def project_y(value: float) -> float:
        return top + (1.0 - (value - y_min) / (y_max - y_min)) * plot_height

    output = [
        f'<svg class="trace-chart" viewBox="0 0 {width} {height}" role="img" '
        f'aria-label="{html.escape(title)}">',
        f'<text x="{left}" y="20" class="chart-title">{html.escape(title)}</text>',
        f'<text x="{left}" y="38" class="chart-subtitle">{html.escape(subtitle)}</text>',
    ]
    for tick in range(5):
        fraction = tick / 4
        y = top + fraction * plot_height
        value = y_max * (1.0 - fraction)
        output.append(
            f'<line x1="{left}" x2="{width-right}" y1="{y:.2f}" y2="{y:.2f}" class="chart-grid"/>'
        )
        output.append(
            f'<text x="{left-8}" y="{y+4:.2f}" text-anchor="end" class="chart-axis">{value:.2f}</text>'
        )
    for tick in range(5):
        fraction = tick / 4
        x = left + fraction * plot_width
        value = x_min + fraction * (x_max - x_min)
        output.append(
            f'<text x="{x:.2f}" y="{height-12}" text-anchor="middle" class="chart-axis">{value:.0f}</text>'
        )
    output.append(
        f'<text x="14" y="{top + plot_height/2:.2f}" transform="rotate(-90 14 {top + plot_height/2:.2f})" '
        f'text-anchor="middle" class="chart-axis">{html.escape(y_label)}</text>'
    )
    for index, (name, x_values, y_values) in enumerate(series):
        points = " ".join(
            f"{project_x(float(x)):.2f},{project_y(float(y)):.2f}"
            for x, y in zip(x_values, y_values)
        )
        dashed = " p99" in name
        output.append(
            f'<polyline points="{points}" fill="none" stroke="{COLORS[name]}" '
            f'stroke-width="{1.2 if dashed else 2.0}" '
            f'{"stroke-dasharray=\"4 4\"" if dashed else ""}/>'
        )
        legend_x = left + index * 150
        output.append(
            f'<line x1="{legend_x}" x2="{legend_x+24}" y1="{height-27}" y2="{height-27}" '
            f'stroke="{COLORS[name]}" stroke-width="2" '
            f'{"stroke-dasharray=\"4 4\"" if dashed else ""}/>'
        )
        output.append(
            f'<text x="{legend_x+30}" y="{height-23}" class="chart-axis">{html.escape(name)}</text>'
        )
    output.append("</svg>")
    return "".join(output)


def ratio_cards(metrics: dict[str, Any]) -> str:
    cards: list[str] = []
    for scenario in SCENARIOS:
        bonesaw = metrics["bonesaw"]["scenarios"][scenario]
        placo = metrics["placo"]["scenarios"][scenario]
        latency_ratio = placo["latency_ns"]["median"] / bonesaw["latency_ns"]["median"]
        memory_ratio = placo["peak_rss_bytes"] / bonesaw["peak_rss_bytes"]
        tracking_ratio = placo["tracking_rms_m"] / bonesaw["tracking_rms_m"]
        cards.append(
            '<article class="ratio-card">'
            f'<span>{html.escape(LABELS[scenario])}</span>'
            f'<strong>{latency_ratio:.2f}×</strong><small>PlaCo/Bonesaw p50 latency</small>'
            f'<b>PlaCo/Bonesaw: {memory_ratio:.2f}× memory · '
            f'{tracking_ratio:.2f}× tracking RMS</b>'
            "</article>"
        )
    return '<div class="ratio-cards">' + "".join(cards) + "</div>"


def comparison_summary(metrics: dict[str, Any]) -> dict[str, Any]:
    scenarios: dict[str, Any] = {}
    for scenario in SCENARIOS:
        bonesaw = metrics["bonesaw"]["scenarios"][scenario]
        placo = metrics["placo"]["scenarios"][scenario]
        scenarios[scenario] = {
            "ticks": bonesaw["ticks"],
            "placo_over_bonesaw": {
                "p50_latency_ratio": (
                    placo["latency_ns"]["median"]
                    / bonesaw["latency_ns"]["median"]
                ),
                "peak_rss_ratio": (
                    placo["peak_rss_bytes"] / bonesaw["peak_rss_bytes"]
                ),
                "tracking_rms_ratio": (
                    placo["tracking_rms_m"] / bonesaw["tracking_rms_m"]
                ),
            },
            "bonesaw": {
                "p50_latency_us": bonesaw["latency_ns"]["median"] / 1_000.0,
                "p99_latency_us": bonesaw["latency_ns"]["p99"] / 1_000.0,
                "peak_rss_mib": bonesaw["peak_rss_bytes"] / (1024.0 * 1024.0),
                "tracking_rms_cm": bonesaw["tracking_rms_m"] * 100.0,
            },
            "placo": {
                "p50_latency_us": placo["latency_ns"]["median"] / 1_000.0,
                "p99_latency_us": placo["latency_ns"]["p99"] / 1_000.0,
                "peak_rss_mib": placo["peak_rss_bytes"] / (1024.0 * 1024.0),
                "tracking_rms_cm": placo["tracking_rms_m"] * 100.0,
            },
        }
    upkie = metrics["upkie_controller_oracle"]
    return {
        "ratio_direction": "PlaCo divided by Bonesaw",
        "fixed_base_scenarios": scenarios,
        "upkie_controller_oracle": {
            "ticks": upkie["ticks"],
            "canonical_bit_mismatches": upkie["comparisons"]["official_aligned"][
                "canonical_bit_mismatches"
            ],
            "upkie_cpp_p50_us": upkie["latency"]["upkie_cpp"]["p50_us"],
            "bonesaw_official_p50_us": upkie["latency"]["bonesaw_official"][
                "p50_us"
            ],
        },
    }


def build_trace_section(
    metrics: dict[str, Any],
    bonesaw_raw: Any,
    placo_raw: Any,
    upkie_raw: Any,
    windows: int,
) -> tuple[str, list[dict[str, Any]]]:
    charts: list[str] = []
    rows: list[dict[str, Any]] = []
    for scenario in SCENARIOS:
        latency_series: list[tuple[str, np.ndarray, np.ndarray]] = []
        tracking_series: list[tuple[str, np.ndarray, np.ndarray]] = []
        for implementation, raw in (("Bonesaw", bonesaw_raw), ("PlaCo", placo_raw)):
            prefix = implementation.lower()
            latency = raw[f"{prefix}_{scenario}_step_ns"].astype(np.float64) / 1_000.0
            errors = raw[f"{prefix}_{scenario}_tracking_error_m"].astype(np.float64)
            per_tick_rms = np.sqrt(np.mean(errors * errors, axis=1)) * 100.0
            x, median, ranges = window_reduce(latency, windows, np.median)
            _, p99, _ = window_reduce(latency, windows, lambda v: float(np.percentile(v, 99)))
            _, tracking, _ = window_reduce(
                per_tick_rms, windows, lambda v: float(np.sqrt(np.mean(v * v)))
            )
            seconds = x * 0.02
            latency_series.extend(
                [(f"{implementation} p50", seconds, median), (f"{implementation} p99", seconds, p99)]
            )
            tracking_series.append((f"{implementation} RMS", seconds, tracking))
            for index, (start, stop) in enumerate(ranges):
                rows.append(
                    {
                        "scenario": scenario,
                        "implementation": implementation,
                        "tick_start": start,
                        "tick_stop": stop,
                        "time_start_s": start * 0.02,
                        "time_stop_s": stop * 0.02,
                        "latency_p50_us": float(median[index]),
                        "latency_p99_us": float(p99[index]),
                        "tracking_rms_cm": float(tracking[index]),
                    }
                )
        charts.append(
            '<div class="trace-pair">'
            + svg_plot(
                f"{LABELS[scenario]} · execution time",
                f"{metrics['bonesaw']['scenarios'][scenario]['ticks']:,} raw steps; 1 s windows",
                "latency µs",
                latency_series,
            )
            + svg_plot(
                f"{LABELS[scenario]} · tracking over time",
                "window RMS over the identical target corpus",
                "tracking RMS cm",
                tracking_series,
            )
            + "</div>"
        )

    upkie_series: list[tuple[str, np.ndarray, np.ndarray]] = []
    for label, key in (
        ("Upkie C++", "upkie_latency_ns"),
        ("Bonesaw Rust", "rust_official_latency_ns"),
    ):
        values = upkie_raw[key].astype(np.float64) / 1_000.0
        x, median, _ = window_reduce(values, windows, np.median)
        _, p99, _ = window_reduce(values, windows, lambda v: float(np.percentile(v, 99)))
        upkie_series.extend([(f"{label} p50", x, median), (f"{label} p99", x, p99)])
    charts.append(
        '<div class="trace-pair single">'
        + svg_plot(
            "Upkie controller-law latency over 100,000 steps",
            "100 consecutive windows; official commands are bitwise identical",
            "latency µs",
            upkie_series,
        )
        + "</div>"
    )
    section = (
        '<section class="trace-overview"><span class="eyebrow">Fresh same-session R89</span>'
        '<h2>Reference performance over execution time</h2>'
        '<p>Every line below is derived from the retained per-step NPZ traces. Fixed-base charts '
        'use one-second windows for readability; the full 5,000-step arrays remain checksum-addressed. '
        'Ratios are valid only within the stated shared boundary.</p>'
        + ratio_cards(metrics)
        + "".join(charts)
        + "</section>"
    )
    return section, rows


def main() -> None:
    args = parse_args()
    source = pathlib.Path(args.source)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    metrics_path = source / "reference-metrics.json"
    markdown_path = source / "REFERENCE_COMPARISON.md"
    bonesaw_path = source / "bonesaw-raw.npz"
    placo_path = source / "placo-raw.npz"
    upkie_path = source / "upkie-controller-raw.npz"
    metrics = json.loads(metrics_path.read_text())
    with np.load(bonesaw_path) as bonesaw_raw, np.load(placo_path) as placo_raw, np.load(
        upkie_path
    ) as upkie_raw:
        trace_html, rows = build_trace_section(
            metrics, bonesaw_raw, placo_raw, upkie_raw, args.windows
        )

    analysis = {
        "schema": 1,
        "revision": f"reference-trace-report-{args.revision}",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "source_generated_utc": metrics["generated_utc"],
        "windows": args.windows,
        "artifacts": {
            str(path): sha256(path)
            for path in (metrics_path, markdown_path, bonesaw_path, placo_path, upkie_path)
        },
        "window_rows": len(rows),
        "comparison_summary": comparison_summary(metrics),
    }
    (output / "reference-trace-report-metrics.json").write_text(
        json.dumps(analysis, indent=2) + "\n"
    )
    with (output / "fixed-base-time-windows.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    markdown = markdown_path.read_text().replace(
        "# Bonesaw reference implementation comparison",
        f"# Bonesaw reference implementation comparison · {args.revision}",
        1,
    )
    rendered = render_report_html(markdown)
    extra_style = """
.trace-overview{margin:0 0 56px;padding:24px;border:1px solid var(--line);background:#12171a}.eyebrow{color:var(--accent);font:700 11px/1.2 ui-monospace,monospace;letter-spacing:.12em;text-transform:uppercase}.trace-overview h2{margin:10px 0 12px;padding:0;border:0}.ratio-cards{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px;margin:22px 0}.ratio-card{display:grid;gap:4px;padding:14px;background:var(--panel);border:1px solid var(--line)}.ratio-card span{color:var(--muted);font-size:12px}.ratio-card strong{font-size:28px;color:var(--accent)}.ratio-card small,.ratio-card b{font-size:11px;font-weight:500}.ratio-card b{color:#bfc9c5}.trace-pair{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin-top:18px}.trace-pair.single{grid-template-columns:minmax(0,1fr)}.trace-chart{display:block;width:100%;height:auto;background:#0f1417;border:1px solid var(--line)}.chart-title{fill:#d8dfdc;font:700 14px system-ui}.chart-subtitle,.chart-axis{fill:#84918c;font:10px ui-monospace,monospace}.chart-grid{stroke:#273139;stroke-width:1}@media(max-width:760px){.trace-overview{padding:14px}.ratio-cards,.trace-pair{grid-template-columns:1fr}.ratio-card{grid-template-columns:1fr auto}.ratio-card strong{grid-row:1/4;grid-column:2}.trace-chart{min-width:0}}
"""
    rendered = rendered.replace("</style>", extra_style + "</style>", 1)
    rendered = rendered.replace("<body><main>", "<body><main>" + trace_html, 1)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(rendered)
    print(web_report)


if __name__ == "__main__":
    main()
