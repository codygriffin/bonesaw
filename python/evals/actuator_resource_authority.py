#!/usr/bin/env python3
"""Persistent actuator-resource authority fixture.

This evaluator intentionally contains no robot policy, state integration,
rigid-body physics, or contact simulation. It drives a declared synthetic
electrical/thermal fixture with immutable actuator-space effort/velocity
arrays, while Rust owns every stateful resource step and output sample.
The fixture proves semantics and dataflow; it is not an Upkie/G1 calibration.
"""

from __future__ import annotations

import argparse
import gc
import json
import os
import pathlib
import platform
import resource
import sys
import time
from typing import Any

import numpy as np

from bonesaw import ActuatorResourceSession


DT_SECONDS = 0.005
DURATION_SECONDS = 120.0
ACTUATOR_NAMES = ["cool_continuous", "sustained_derating", "intermittent_pulse"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default="benchmarks/results/actuator-resource-authority-r50",
    )
    return parser.parse_args()


def distribution(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=np.float64)
    return {
        "minimum": float(np.min(values)),
        "p01": float(np.percentile(values, 1)),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "maximum": float(np.max(values)),
        "mean": float(np.mean(values)),
    }


def threshold_curve(
    values: np.ndarray, thresholds: list[float], *, adverse: str
) -> list[dict[str, float | int]]:
    rows = []
    for threshold in thresholds:
        mask = values > threshold if adverse == "above" else values < threshold
        rows.append(
            {
                "threshold": threshold,
                "adverse_ticks": int(np.count_nonzero(mask)),
                "adverse_fraction": float(np.mean(mask)),
            }
        )
    return rows


def longest_true_run(mask: np.ndarray) -> int:
    longest = 0
    current = 0
    for active in np.asarray(mask, dtype=np.bool_):
        current = current + 1 if active else 0
        longest = max(longest, current)
    return longest


def allocate_outputs(ticks: int, actuators: int) -> dict[str, np.ndarray]:
    shape = (ticks, actuators)
    return {
        "current_a": np.empty(shape, dtype=np.float64),
        "copper_loss_w": np.empty(shape, dtype=np.float64),
        "mechanical_power_w": np.empty(shape, dtype=np.float64),
        "ideal_electrical_power_w": np.empty(shape, dtype=np.float64),
        "winding_temperature_c": np.empty(shape, dtype=np.float64),
        "effort_scale": np.empty(shape, dtype=np.float64),
        "derated_effort_limit_nm": np.empty(shape, dtype=np.float64),
        "effort_headroom_nm": np.empty(shape, dtype=np.float64),
        "effort_utilization": np.empty(shape, dtype=np.float64),
        "step_ns": np.empty(ticks, dtype=np.uint64),
        "allocation_calls": np.empty(ticks, dtype=np.uint64),
        "allocated_bytes": np.empty(ticks, dtype=np.uint64),
    }


def write_timeline_svg(
    path: pathlib.Path,
    time_s: np.ndarray,
    effort: np.ndarray,
    temperature: np.ndarray,
    effort_scale: np.ndarray,
) -> None:
    width, height = 960, 520
    left, right = 72, 28
    top, panel_height, gap = 42, 178, 62
    plot_width = width - left - right
    sample = np.arange(0, len(time_s), max(1, len(time_s) // 1400))

    def points(values: np.ndarray, y_min: float, y_max: float, y_top: float) -> str:
        x = left + plot_width * time_s[sample] / time_s[-1]
        y = y_top + panel_height * (1.0 - (values[sample] - y_min) / (y_max - y_min))
        return " ".join(f"{px:.2f},{py:.2f}" for px, py in zip(x, y, strict=True))

    temp_top = top
    scale_top = top + panel_height + gap
    temp_points = points(temperature, 20.0, 110.0, temp_top)
    scale_points = points(effort_scale, 0.0, 1.05, scale_top)
    effort_normalized = np.abs(effort) / np.max(np.abs(effort))
    effort_points = points(effort_normalized, 0.0, 1.05, scale_top)

    grid = []
    for seconds in range(0, 121, 20):
        x = left + plot_width * seconds / 120.0
        grid.append(
            f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{scale_top + panel_height}" class="grid"/>'
        )
        grid.append(
            f'<text x="{x:.1f}" y="{scale_top + panel_height + 28}" text-anchor="middle">{seconds}</text>'
        )

    def horizontal(value: float, y_min: float, y_max: float, y_top: float, label: str) -> str:
        y = y_top + panel_height * (1.0 - (value - y_min) / (y_max - y_min))
        return (
            f'<line x1="{left}" y1="{y:.1f}" x2="{width-right}" y2="{y:.1f}" class="threshold"/>'
            f'<text x="{width-right-4}" y="{y-5:.1f}" text-anchor="end" class="threshold-label">{label}</text>'
        )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<style>
  rect {{ fill:#11161a; }} text {{ fill:#9eaaa5; font:12px system-ui,sans-serif; }}
  .title {{ fill:#d9e3de; font-size:17px; font-weight:600; }}
  .axis {{ stroke:#59645f; stroke-width:1; }} .grid {{ stroke:#29312f; stroke-width:1; }}
  .temperature {{ fill:none; stroke:#ef8b7f; stroke-width:2; }}
  .scale {{ fill:none; stroke:#7fe0b0; stroke-width:2.2; }}
  .demand {{ fill:none; stroke:#6c7d96; stroke-width:1.2; stroke-dasharray:5 4; }}
  .threshold {{ stroke:#c29b56; stroke-width:1; stroke-dasharray:4 4; }}
  .threshold-label {{ fill:#c9a96c; font-size:10px; }}
</style>
<rect width="100%" height="100%"/>
<text x="{left}" y="25" class="title">Persistent actuator authority · synthetic sustained-load fixture</text>
{''.join(grid)}
<line x1="{left}" y1="{temp_top + panel_height}" x2="{width-right}" y2="{temp_top + panel_height}" class="axis"/>
<line x1="{left}" y1="{scale_top + panel_height}" x2="{width-right}" y2="{scale_top + panel_height}" class="axis"/>
{horizontal(70.0, 20.0, 110.0, temp_top, 'derating begins · 70 °C')}
{horizontal(95.0, 20.0, 110.0, temp_top, 'declared shutdown · 95 °C')}
<polyline points="{temp_points}" class="temperature"/>
<polyline points="{effort_points}" class="demand"/>
<polyline points="{scale_points}" class="scale"/>
<text x="14" y="{temp_top + panel_height/2:.1f}" transform="rotate(-90 14 {temp_top + panel_height/2:.1f})" text-anchor="middle">winding temperature · °C</text>
<text x="14" y="{scale_top + panel_height/2:.1f}" transform="rotate(-90 14 {scale_top + panel_height/2:.1f})" text-anchor="middle">normalized demand / effort scale</text>
<text x="{width/2}" y="{height-10}" text-anchor="middle">execution time · seconds</text>
<text x="{left+8}" y="{scale_top+18}" fill="#7fe0b0">available effort scale</text>
<text x="{left+8}" y="{scale_top+35}" fill="#7d8ca4">instantaneous effort demand</text>
</svg>"""
    path.write_text(svg)


def run_session(
    profiles: np.ndarray,
    initial_temperature: np.ndarray,
    effort: np.ndarray,
    velocity: np.ndarray,
) -> dict[str, np.ndarray]:
    session = ActuatorResourceSession(profiles, initial_temperature)
    outputs = allocate_outputs(effort.shape[0], effort.shape[1])
    session.run_trace(
        effort,
        velocity,
        DT_SECONDS,
        outputs["current_a"],
        outputs["copper_loss_w"],
        outputs["mechanical_power_w"],
        outputs["ideal_electrical_power_w"],
        outputs["winding_temperature_c"],
        outputs["effort_scale"],
        outputs["derated_effort_limit_nm"],
        outputs["effort_headroom_nm"],
        outputs["effort_utilization"],
        outputs["step_ns"],
        outputs["allocation_calls"],
        outputs["allocated_bytes"],
    )
    return outputs


def main() -> int:
    args = parse_args()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    ticks = round(DURATION_SECONDS / DT_SECONDS)
    time_s = np.arange(ticks, dtype=np.float64) * DT_SECONDS

    # Columns: Kt, R, Rth, tau, ambient, derating start, shutdown,
    # minimum effort fraction, base effort limit.
    profiles = np.asarray(
        [
            [0.80, 0.12, 0.45, 35.0, 25.0, 70.0, 90.0, 0.20, 40.0],
            [0.60, 0.18, 0.80, 25.0, 25.0, 70.0, 95.0, 0.10, 30.0],
            [0.70, 0.15, 0.65, 20.0, 25.0, 70.0, 90.0, 0.20, 25.0],
        ],
        dtype=np.float64,
    )
    initial_temperature = profiles[:, 4].copy()
    effort = np.zeros((ticks, len(ACTUATOR_NAMES)), dtype=np.float64)
    velocity = np.zeros_like(effort)

    effort[:, 0] = 8.0 + 2.0 * np.sin(2.0 * np.pi * time_s / 7.0)
    velocity[:, 0] = 2.0 * np.sin(2.0 * np.pi * time_s / 5.0)
    sustained_load = (time_s < 50.0) | (time_s >= 80.0)
    effort[sustained_load, 1] = 15.0
    velocity[sustained_load, 1] = 1.5
    pulse = np.mod(time_s, 5.0) < 2.0
    effort[pulse, 2] = 18.0
    velocity[:, 2] = np.where(np.mod(time_s, 10.0) < 5.0, 2.5, -2.5)

    gc.collect()
    gc_before = gc.get_count()
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    wall_before = time.perf_counter_ns()
    cpu_before = time.process_time_ns()
    first = run_session(profiles, initial_temperature, effort, velocity)
    cpu_after = time.process_time_ns()
    wall_after = time.perf_counter_ns()
    rss_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    gc_after = gc.get_count()
    repeat = run_session(profiles, initial_temperature, effort, velocity)

    physical_fields = [
        "current_a",
        "copper_loss_w",
        "mechanical_power_w",
        "ideal_electrical_power_w",
        "winding_temperature_c",
        "effort_scale",
        "derated_effort_limit_nm",
        "effort_headroom_nm",
        "effort_utilization",
    ]
    exact_fields = {
        name: bool(np.array_equal(first[name], repeat[name])) for name in physical_fields
    }
    sustained_end = round(50.0 / DT_SECONDS) - 1
    cooling_start = sustained_end + 1
    cooling_end = round(80.0 / DT_SECONDS) - 1
    hot_probe = round(45.0 / DT_SECONDS)
    cool_probe = round(5.0 / DT_SECONDS)
    constant_loss = (15.0 / profiles[1, 0]) ** 2 * profiles[1, 1]
    steady_temperature = profiles[1, 4] + constant_loss * profiles[1, 2]
    analytic_temperature = steady_temperature + (
        initial_temperature[1] - steady_temperature
    ) * np.exp(-(sustained_end + 1) * DT_SECONDS / profiles[1, 3])

    energy_identity_error = np.max(
        np.abs(
            first["ideal_electrical_power_w"]
            - first["mechanical_power_w"]
            - first["copper_loss_w"]
        )
    )
    per_actuator = []
    for actuator, name in enumerate(ACTUATOR_NAMES):
        per_actuator.append(
            {
                "name": name,
                "maximum_temperature_c": float(
                    np.max(first["winding_temperature_c"][:, actuator])
                ),
                "minimum_effort_scale": float(np.min(first["effort_scale"][:, actuator])),
                "maximum_effort_utilization": float(
                    np.max(first["effort_utilization"][:, actuator])
                ),
                "maximum_current_a": float(np.max(first["current_a"][:, actuator])),
                "maximum_copper_loss_w": float(
                    np.max(first["copper_loss_w"][:, actuator])
                ),
                "mechanical_power_w": distribution(
                    first["mechanical_power_w"][:, actuator]
                ),
            }
        )

    checks = {
        "all_physical_outputs_are_finite": all(
            np.all(np.isfinite(first[name])) for name in physical_fields
        ),
        "rust_resource_loop_has_zero_allocations": int(
            np.sum(first["allocation_calls"], dtype=np.uint64)
        )
        == 0
        and int(np.sum(first["allocated_bytes"], dtype=np.uint64)) == 0,
        "physical_outputs_are_bitwise_repeatable": all(exact_fields.values()),
        "ideal_electrical_power_identity_le_1e_12w": energy_identity_error <= 1e-12,
        "constant_load_matches_exact_closed_form_le_1e_9c": abs(
            first["winding_temperature_c"][sustained_end, 1] - analytic_temperature
        )
        <= 1e-9,
        "cool_fixture_never_derates": bool(np.all(first["effort_scale"][:, 0] == 1.0)),
        "sustained_fixture_reaches_declared_minimum_scale": float(
            np.min(first["effort_scale"][:, 1])
        )
        <= profiles[1, 7] + 1e-12,
        "zero_effort_cooling_is_strictly_monotonic": bool(
            np.all(
                np.diff(
                    first["winding_temperature_c"][cooling_start : cooling_end + 1, 1]
                )
                < 0.0
            )
        ),
        "same_demand_has_less_authority_when_hot": effort[hot_probe, 1]
        == effort[cool_probe, 1]
        and velocity[hot_probe, 1] == velocity[cool_probe, 1]
        and first["effort_scale"][hot_probe, 1]
        < first["effort_scale"][cool_probe, 1],
        "derating_is_continuous_per_5ms_tick": float(
            np.max(np.abs(np.diff(first["effort_scale"][:, 1])))
        )
        <= 0.002,
        "resource_p99_latency_le_50us": float(np.percentile(first["step_ns"], 99))
        <= 50_000.0,
        "resource_maximum_latency_le_1ms": int(np.max(first["step_ns"])) <= 1_000_000,
    }
    checks = {name: bool(value) for name, value in checks.items()}

    metrics: dict[str, Any] = {
        "schema": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "evaluation_boundary": {
            "robot_policy": False,
            "state_integration": False,
            "rigid_body_physics": False,
            "contact_simulator": False,
            "synthetic_parameter_fixture": True,
            "upkie_or_g1_calibration_claim": False,
            "rust_owns_persistent_state": True,
        },
        "configuration": {
            "dt_seconds": DT_SECONDS,
            "duration_seconds": DURATION_SECONDS,
            "ticks": ticks,
            "actuator_names": ACTUATOR_NAMES,
            "profile_columns": [
                "torque_constant_nm_per_amp",
                "winding_resistance_ohm",
                "thermal_resistance_c_per_w",
                "thermal_time_constant_s",
                "ambient_temperature_c",
                "derating_start_temperature_c",
                "shutdown_temperature_c",
                "minimum_effort_fraction",
                "base_effort_limit_nm",
            ],
            "profiles": profiles.tolist(),
        },
        "authority_evidence": {
            "per_actuator": per_actuator,
            "maximum_temperature_c": distribution(first["winding_temperature_c"]),
            "minimum_effort_scale": distribution(first["effort_scale"]),
            "maximum_effort_utilization": distribution(first["effort_utilization"]),
            "temperature_threshold_curve": threshold_curve(
                np.max(first["winding_temperature_c"], axis=1),
                [40.0, 55.0, 70.0, 85.0, 95.0],
                adverse="above",
            ),
            "effort_scale_threshold_curve": threshold_curve(
                np.min(first["effort_scale"], axis=1),
                [1.0, 0.8, 0.5, 0.2],
                adverse="below",
            ),
            "longest_derating_run_ticks": longest_true_run(
                np.min(first["effort_scale"], axis=1) < 1.0
            ),
            "hot_vs_cool_same_demand": {
                "cool_tick": cool_probe,
                "hot_tick": hot_probe,
                "effort_nm": float(effort[hot_probe, 1]),
                "velocity_rad_s": float(velocity[hot_probe, 1]),
                "cool_temperature_c": float(
                    first["winding_temperature_c"][cool_probe, 1]
                ),
                "hot_temperature_c": float(
                    first["winding_temperature_c"][hot_probe, 1]
                ),
                "cool_effort_scale": float(first["effort_scale"][cool_probe, 1]),
                "hot_effort_scale": float(first["effort_scale"][hot_probe, 1]),
            },
        },
        "numerics": {
            "constant_load_analytic_temperature_c": float(analytic_temperature),
            "constant_load_rust_temperature_c": float(
                first["winding_temperature_c"][sustained_end, 1]
            ),
            "constant_load_absolute_error_c": float(
                abs(
                    first["winding_temperature_c"][sustained_end, 1]
                    - analytic_temperature
                )
            ),
            "maximum_power_identity_error_w": float(energy_identity_error),
            "maximum_effort_scale_delta_per_tick": float(
                np.max(np.abs(np.diff(first["effort_scale"][:, 1])))
            ),
        },
        "runtime": {
            "latency_us": distribution(first["step_ns"].astype(np.float64) / 1_000.0),
            "allocation_calls": int(
                np.sum(first["allocation_calls"], dtype=np.uint64)
            ),
            "allocated_bytes": int(np.sum(first["allocated_bytes"], dtype=np.uint64)),
            "wall_seconds": (wall_after - wall_before) / 1e9,
            "process_cpu_seconds": (cpu_after - cpu_before) / 1e9,
            "maximum_rss_growth_bytes": max(0, rss_after - rss_before),
            "python_gc_count_before": list(gc_before),
            "python_gc_count_after": list(gc_after),
            "python": sys.version.split()[0],
            "platform": platform.platform(),
        },
        "exact_repeat_fields": exact_fields,
    }
    (output / "actuator-resource-authority-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "actuator-resource-authority-raw.npz",
        time_s=time_s,
        profiles=profiles,
        actuator_effort_nm=effort,
        actuator_velocity_rad_s=velocity,
        **first,
    )
    write_timeline_svg(
        output / "actuator-resource-timeline.svg",
        time_s,
        effort[:, 1],
        first["winding_temperature_c"][:, 1],
        first["effort_scale"][:, 1],
    )

    report = [
        "# Actuator resource authority",
        "",
        f"**{'PASS' if metrics['passed'] else 'RED'} · {sum(checks.values())}/{len(checks)} gates**",
        "",
        "This is a synthetic parameter fixture for the persistent actuator-resource mechanism. It is deliberately policy-free, integration-free, rigid-body-physics-free, and contact-free. The values do not claim to identify Upkie or G1 hardware. Python owns the immutable workload and report; Rust owns every 5 ms electrical/thermal update, derating sample, allocation count, and timed step.",
        "",
        "## Measured resource stack",
        "",
        "![Persistent actuator authority over execution time](actuator-resource-timeline.svg)",
        "",
        "| actuator fixture | max temperature | min effort scale | max utilization after derating | max current | max copper loss |",
        "|---|---:|---:|---:|---:|---:|",
        *[
            f"| `{row['name']}` | {row['maximum_temperature_c']:.2f} °C | {row['minimum_effort_scale'] * 100:.2f}% | {row['maximum_effort_utilization'] * 100:.2f}% | {row['maximum_current_a']:.2f} A | {row['maximum_copper_loss_w']:.2f} W |"
            for row in per_actuator
        ],
        "",
        f"At the exact same `15 Nm / 1.5 rad/s` demand, the sustained fixture has {metrics['authority_evidence']['hot_vs_cool_same_demand']['cool_effort_scale'] * 100:.2f}% available effort at {metrics['authority_evidence']['hot_vs_cool_same_demand']['cool_temperature_c']:.2f} °C, but only {metrics['authority_evidence']['hot_vs_cool_same_demand']['hot_effort_scale'] * 100:.2f}% at {metrics['authority_evidence']['hot_vs_cool_same_demand']['hot_temperature_c']:.2f} °C. Instantaneous QP feasibility and persistent actuator authority are therefore observably different signals.",
        "",
        "## Capability curves",
        "",
        "| maximum winding temperature | adverse ticks | fraction |",
        "|---:|---:|---:|",
        *[
            f"| {row['threshold']:.0f} °C | {row['adverse_ticks']} | {row['adverse_fraction'] * 100:.2f}% |"
            for row in metrics["authority_evidence"]["temperature_threshold_curve"]
        ],
        "",
        "| minimum available effort scale | adverse ticks | fraction |",
        "|---:|---:|---:|",
        *[
            f"| {row['threshold'] * 100:.0f}% | {row['adverse_ticks']} | {row['adverse_fraction'] * 100:.2f}% |"
            for row in metrics["authority_evidence"]["effort_scale_threshold_curve"]
        ],
        "",
        "## Numerical and execution evidence",
        "",
        f"The constant-load exact closed-form temperature error is `{metrics['numerics']['constant_load_absolute_error_c']:.3e} °C`; the maximum power-identity error is `{metrics['numerics']['maximum_power_identity_error_w']:.3e} W`. Per-step effort-scale change is bounded at `{metrics['numerics']['maximum_effort_scale_delta_per_tick']:.6f}`. The three-actuator Rust update costs `{metrics['runtime']['latency_us']['p50']:.3f} / {metrics['runtime']['latency_us']['p99']:.3f} / {metrics['runtime']['latency_us']['maximum']:.3f} µs` p50/p99/max, allocates `{metrics['runtime']['allocation_calls']}` calls / `{metrics['runtime']['allocated_bytes']}` bytes, and every physical output is bitwise repeatable: `{all(exact_fields.values())}`.",
        "",
        "## Gates",
        "",
        *[f"- {'PASS' if passed else 'FAIL'} `{name}`" for name, passed in checks.items()],
        "",
        "## Interpretation",
        "",
        "The derated effort limit is a physical model output, not a UI color threshold. Warning thresholds, corpus percentiles, and hardware promotion criteria remain separate. A real robot profile must identify torque constant, winding resistance, thermal resistance/time constant, ambient measurement, derating curve, and driver/regeneration losses before this row can stop reporting UNMODELED in the live Upkie editor.",
        "",
    ]
    (output / "ACTUATOR_RESOURCE_AUTHORITY.md").write_text("\n".join(report))
    print(json.dumps({"passed": metrics["passed"], "checks": checks, "output": str(output)}, indent=2))
    return 0 if metrics["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
