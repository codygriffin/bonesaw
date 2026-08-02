#!/usr/bin/env python3
"""Physics-free actuator realization envelope over the admitted r54 WBC trace."""

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
from cpu_tail_stability import arrays_byte_exact, runs


DT_SECONDS = 0.005
SCENARIOS = (
    {"name": "ideal_control", "bandwidth_hz": np.inf, "effort_rate_nm_s": np.inf, "availability_fraction": 1.0},
    {"name": "fast_synthetic", "bandwidth_hz": 20.0, "effort_rate_nm_s": 5_000.0, "availability_fraction": 1.0},
    {"name": "medium_synthetic", "bandwidth_hz": 10.0, "effort_rate_nm_s": 1_000.0, "availability_fraction": 1.0},
    {"name": "slow_synthetic", "bandwidth_hz": 5.0, "effort_rate_nm_s": 250.0, "availability_fraction": 1.0},
    {"name": "half_available", "bandwidth_hz": 10.0, "effort_rate_nm_s": 1_000.0, "availability_fraction": 0.5},
    {"name": "quarter_available_slow", "bandwidth_hz": 5.0, "effort_rate_nm_s": 250.0, "availability_fraction": 0.25},
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--oracle", default="benchmarks/results/g1-multistep-oracle-r54")
    parser.add_argument("--output", default="benchmarks/results/g1-actuator-realization-r62")
    parser.add_argument("--web-report", default="web/G1_ACTUATOR_REALIZATION_R62.html")
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
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "maximum": float(np.max(values)),
        "mean": float(np.mean(values)),
    }


def longest_true_run(mask: np.ndarray) -> int:
    return max((stop - start for start, stop in runs(mask)), default=0)


def contact_edges(contacts: np.ndarray) -> np.ndarray:
    return np.flatnonzero(np.any(contacts[1:] != contacts[:-1], axis=1)) + 1


def edge_recovery_ticks(
    pressure: np.ndarray,
    edges: np.ndarray,
    threshold: float,
    dwell_ticks: int = 5,
) -> list[int | None]:
    recoveries: list[int | None] = []
    for edge_index, edge in enumerate(edges):
        stop = int(edges[edge_index + 1]) if edge_index + 1 < len(edges) else len(pressure)
        recovery = None
        for tick in range(int(edge), max(int(edge), stop - dwell_ticks + 1)):
            if np.all(pressure[tick : tick + dwell_ticks] <= threshold):
                recovery = tick - int(edge)
                break
        recoveries.append(recovery)
    return recoveries


def run_session(
    bonesaw: Any,
    requested: np.ndarray,
    available: np.ndarray,
    profiles: np.ndarray,
) -> dict[str, np.ndarray]:
    ticks, actuators = requested.shape
    output = {
        "limited_target": np.empty_like(requested),
        "realized": np.empty_like(requested),
        "error": np.empty_like(requested),
        "availability_clipped": np.empty((ticks, actuators), dtype=np.uint8),
        "slew_limited": np.empty((ticks, actuators), dtype=np.uint8),
        "step_ns": np.empty(ticks, dtype=np.uint64),
        "allocation_calls": np.empty(ticks, dtype=np.uint64),
        "allocated_bytes": np.empty(ticks, dtype=np.uint64),
    }
    session = bonesaw.ActuatorRealizationSession(profiles, requested[0].copy())
    session.run_trace(
        requested,
        available,
        DT_SECONDS,
        output["limited_target"],
        output["realized"],
        output["error"],
        output["availability_clipped"],
        output["slew_limited"],
        output["step_ns"],
        output["allocation_calls"],
        output["allocated_bytes"],
    )
    return output


def main() -> None:
    args = parse_args()
    try:
        import bonesaw
    except ImportError as error:
        raise RuntimeError("build the release PyO3 extension before running this evaluator") from error

    oracle = pathlib.Path(args.oracle)
    raw_path = oracle / "oracle-wbc-admission-raw.npz"
    metrics_path = oracle / "oracle-wbc-admission-metrics.json"
    source_hash_before = sha256(raw_path)
    metrics_hash_before = sha256(metrics_path)
    oracle_metrics = json.loads(metrics_path.read_text())
    if not oracle_metrics["passed"]:
        raise ValueError("the source WBC trace is not admitted")
    with np.load(raw_path) as archive:
        requested = archive["actuator_torque"].copy()
        velocity = archive["v"].copy()
        effort_limits = archive["actuator_effort_limits"].copy()
        contacts = archive["contacts"].copy()
    ticks, actuators = requested.shape
    per_actuator = oracle_metrics["authority_evidence"]["actuator_effort"]["per_actuator"]
    actuator_names = [""] * actuators
    for item in per_actuator:
        actuator_names[item["coordinate"]] = item["name"]
    if any(not name for name in actuator_names):
        raise ValueError("oracle metrics do not name every actuator coordinate")

    edges = contact_edges(contacts)
    case_metrics = []
    raw_outputs: dict[str, np.ndarray] = {}
    all_repeat_exact = True
    for scenario in SCENARIOS:
        profiles = np.empty((actuators, 2), dtype=np.float64)
        profiles[:, 0] = scenario["bandwidth_hz"]
        profiles[:, 1] = scenario["effort_rate_nm_s"]
        available = np.broadcast_to(
            effort_limits * scenario["availability_fraction"], requested.shape
        ).copy()
        first = run_session(bonesaw, requested, available, profiles)
        second = run_session(bonesaw, requested, available, profiles)
        repeat_exact = all(
            arrays_byte_exact(first[name], second[name])
            for name in first
            if name != "step_ns"
        )
        all_repeat_exact &= repeat_exact
        normalized_error = np.abs(first["error"]) / effort_limits[None, :]
        pressure = np.max(normalized_error, axis=1)
        requested_power = requested * velocity
        realized_power = first["realized"] * velocity
        power_error = requested_power - realized_power
        availability_mask = first["availability_clipped"].astype(bool)
        slew_mask = first["slew_limited"].astype(bool)
        recoveries = edge_recovery_ticks(pressure, edges, 0.05)
        finite_recoveries = [value for value in recoveries if value is not None]
        per_coordinate = []
        for coordinate, name in enumerate(actuator_names):
            per_coordinate.append({
                "coordinate": coordinate,
                "name": name,
                "rms_error_nm": float(np.sqrt(np.mean(first["error"][:, coordinate] ** 2))),
                "maximum_error_nm": float(np.max(np.abs(first["error"][:, coordinate]))),
                "maximum_normalized_error": float(np.max(normalized_error[:, coordinate])),
                "availability_clipped_samples": int(np.count_nonzero(availability_mask[:, coordinate])),
                "slew_limited_samples": int(np.count_nonzero(slew_mask[:, coordinate])),
            })
        per_coordinate.sort(key=lambda item: item["maximum_normalized_error"], reverse=True)
        metrics = {
            "name": scenario["name"],
            "profile": {
                "bandwidth_hz": "infinite" if np.isinf(scenario["bandwidth_hz"]) else scenario["bandwidth_hz"],
                "effort_rate_nm_s": "infinite" if np.isinf(scenario["effort_rate_nm_s"]) else scenario["effort_rate_nm_s"],
                "availability_fraction": scenario["availability_fraction"],
            },
            "repeat_exact": repeat_exact,
            "effort_error_nm": {
                "rms": float(np.sqrt(np.mean(first["error"] ** 2))),
                "absolute": distribution(np.abs(first["error"])),
                "normalized_absolute": distribution(normalized_error),
                "per_tick_maximum_normalized": distribution(pressure),
            },
            "pressure_dwell": {
                str(threshold): {
                    "ticks": int(np.count_nonzero(pressure > threshold)),
                    "longest_ticks": longest_true_run(pressure > threshold),
                    "longest_ms": 5 * longest_true_run(pressure > threshold),
                }
                for threshold in (0.01, 0.05, 0.1, 0.2)
            },
            "limiting": {
                "availability_clipped_samples": int(np.count_nonzero(availability_mask)),
                "availability_clipped_ticks": int(np.count_nonzero(np.any(availability_mask, axis=1))),
                "slew_limited_samples": int(np.count_nonzero(slew_mask)),
                "slew_limited_ticks": int(np.count_nonzero(np.any(slew_mask, axis=1))),
            },
            "mechanical_power_w": {
                "requested_absolute": distribution(np.abs(requested_power)),
                "realized_absolute": distribution(np.abs(realized_power)),
                "error_absolute": distribution(np.abs(power_error)),
                "requested_positive_energy_j": float(np.sum(np.maximum(requested_power, 0.0)) * DT_SECONDS),
                "realized_positive_energy_j": float(np.sum(np.maximum(realized_power, 0.0)) * DT_SECONDS),
                "requested_regeneration_j": float(-np.sum(np.minimum(requested_power, 0.0)) * DT_SECONDS),
                "realized_regeneration_j": float(-np.sum(np.minimum(realized_power, 0.0)) * DT_SECONDS),
            },
            "contact_edge_recovery": {
                "edges": len(edges),
                "threshold_fraction_of_authored_limit": 0.05,
                "dwell_ticks": 5,
                "recovery_ticks": recoveries,
                "recovered_edges": len(finite_recoveries),
                "maximum_recovery_ms": None if not finite_recoveries else 5 * max(finite_recoveries),
            },
            "runtime": {
                "step_us": distribution(first["step_ns"].astype(np.float64) / 1e3),
                "allocation_calls": int(np.sum(first["allocation_calls"])),
                "allocated_bytes": int(np.sum(first["allocated_bytes"])),
            },
            "worst_actuators": per_coordinate[:8],
        }
        case_metrics.append(metrics)
        prefix = scenario["name"]
        for name, array in first.items():
            raw_outputs[f"{prefix}_{name}"] = array

    ideal = case_metrics[0]
    ideal_exact = (
        ideal["effort_error_nm"]["absolute"]["maximum"] == 0.0
        and ideal["limiting"]["availability_clipped_samples"] == 0
        and ideal["limiting"]["slew_limited_samples"] == 0
    )
    source_immutable = source_hash_before == sha256(raw_path) and metrics_hash_before == sha256(metrics_path)
    zero_allocations = all(case["runtime"]["allocation_calls"] == 0 and case["runtime"]["allocated_bytes"] == 0 for case in case_metrics)
    checks = {
        "source_oracle_remains_immutable": source_immutable,
        "ideal_control_is_bit_exact": ideal_exact,
        "all_scenarios_repeat_bit_exactly": all_repeat_exact,
        "all_rust_steps_are_allocation_free": zero_allocations,
        "all_realized_efforts_respect_declared_availability": all(
            np.all(np.abs(raw_outputs[f"{case['name']}_realized"]) <= effort_limits[None, :] * case["profile"]["availability_fraction"] + 1e-12)
            for case in case_metrics
        ),
    }
    if not all(checks.values()):
        raise ValueError(f"actuator realization mechanism checks failed: {checks}")

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output / "actuator-realization-raw.npz", **raw_outputs)
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "passed": True,
        "checks": checks,
        "evaluation_boundary": {
            "source": "admitted r54 WBC actuator effort and oracle joint velocity arrays",
            "robot_policy": False,
            "robot_state_integration": False,
            "rigid_body_physics": False,
            "contact_simulation": False,
            "persistent_state": "realized actuator effort only",
            "hardware_calibration": False,
        },
        "ticks": ticks,
        "actuators": actuators,
        "dt_seconds": DT_SECONDS,
        "contact_edges": edges.tolist(),
        "scenarios": case_metrics,
        "source_sha256": {str(raw_path): source_hash_before, str(metrics_path): metrics_hash_before},
    }
    (output / "actuator-realization-metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    with (output / "actuator-realization-scenarios.csv").open("w", newline="") as stream:
        fieldnames = [
            "name", "bandwidth_hz", "effort_rate_nm_s", "availability_fraction",
            "rms_error_nm", "p99_normalized_error", "maximum_normalized_error",
            "availability_clipped_ticks", "slew_limited_ticks", "longest_over_5pct_ms",
            "maximum_edge_recovery_ms", "p99_step_us", "allocation_calls", "allocated_bytes",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for case in case_metrics:
            writer.writerow({
                "name": case["name"],
                **case["profile"],
                "rms_error_nm": case["effort_error_nm"]["rms"],
                "p99_normalized_error": case["effort_error_nm"]["normalized_absolute"]["p99"],
                "maximum_normalized_error": case["effort_error_nm"]["normalized_absolute"]["maximum"],
                "availability_clipped_ticks": case["limiting"]["availability_clipped_ticks"],
                "slew_limited_ticks": case["limiting"]["slew_limited_ticks"],
                "longest_over_5pct_ms": case["pressure_dwell"]["0.05"]["longest_ms"],
                "maximum_edge_recovery_ms": case["contact_edge_recovery"]["maximum_recovery_ms"],
                "p99_step_us": case["runtime"]["step_us"]["p99"],
                "allocation_calls": case["runtime"]["allocation_calls"],
                "allocated_bytes": case["runtime"]["allocated_bytes"],
            })

    report = [
        "# Bonesaw G1 actuator-realization envelope · r62",
        "",
        "## Result",
        "",
        "The admitted r54 WBC effort trace is replayed through six declared Rust command-response envelopes without a robot policy, state integration, rigid-body physics, or contact simulation. The only persistent state is realized actuator effort. Infinite-bandwidth/infinite-slew control reproduces every requested effort bit-exactly. Finite profiles expose bandwidth lag, slew limiting, availability clipping, power mismatch, persistence, and contact-edge recovery as separate signals.",
        "",
        "> All finite profiles are synthetic sensitivity cases, not Unitree G1 actuator calibration. This report quantifies how much command-path authority would be required; it does not claim that the real robot has any listed bandwidth, slew rate, or derating fraction.",
        "",
        "## Capability matrix",
        "",
    ]
    report += markdown_table(
        ["case", "bandwidth / rate / availability", "RMS error Nm", "p99 / max normalized error", "availability / slew ticks", "longest >5%", "edge recovery max", "p99 Rust step"],
        [[
            case["name"],
            f"{case['profile']['bandwidth_hz']} Hz / {case['profile']['effort_rate_nm_s']} Nm/s / {100 * case['profile']['availability_fraction']:.0f}%",
            f"{case['effort_error_nm']['rms']:.4f}",
            f"{100 * case['effort_error_nm']['normalized_absolute']['p99']:.3f}% / {100 * case['effort_error_nm']['normalized_absolute']['maximum']:.3f}%",
            f"{case['limiting']['availability_clipped_ticks']} / {case['limiting']['slew_limited_ticks']}",
            f"{case['pressure_dwell']['0.05']['longest_ms']} ms",
            "N/A" if case['contact_edge_recovery']['maximum_recovery_ms'] is None else f"{case['contact_edge_recovery']['maximum_recovery_ms']} ms",
            f"{case['runtime']['step_us']['p99']:.3f} µs",
        ] for case in case_metrics],
    )
    report += [
        "",
        "## Interpretation",
        "",
        "- Stateless r54 admission remains the authority for instantaneous dynamics/contact/friction/CoP/effort feasibility; realization is a separately scored downstream boundary.",
        "- Normalized effort error uses each actuator's authored URDF effort limit, so joints with different units of authority are comparable without forming one aggregate health score.",
        "- Availability clipping and slew limiting are causal flags from the Rust step, while bandwidth lag is retained continuously in the error trace and dwell curves.",
        "- Mechanical power is observational `effort × oracle velocity`; electrical/thermal/reliability claims still require calibrated motor and driver profiles plus telemetry.",
        "- A future constrained forward-dynamics surrogate may map realized effort error to acceleration tracking, but it must remain separate from this physics-free command-path certificate and from integrated simulation.",
        "",
        "## Mechanism gates",
        "",
    ]
    report += [f"- {'PASS' if passed else 'FAIL'} `{name}`" for name, passed in checks.items()]
    report += [
        "",
        "## Artifacts",
        "",
        "`actuator-realization-raw.npz` retains every realized effort, error, causal clipping flag, timing, and allocation trace for all six cases. `actuator-realization-metrics.json` retains per-actuator leaders, power/energy observations, dwell, edge recovery, evaluation boundary, and source checksums. The CSV is the compact capability curve.",
        "",
    ]
    report_text = "\n".join(report)
    (output / "G1_ACTUATOR_REALIZATION.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))
    print(json.dumps({"passed": True, "checks": checks, "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
