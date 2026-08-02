#!/usr/bin/env python3
"""Aggregate the pinned WBC behavior/reference evidence into one eval manifest.

This report intentionally consumes completed artifacts instead of rerunning long
simulations. It keeps the Python evaluation shell responsible for benchmark
composition while the measured Rust products remain the implementation under
test. A red behavior row is retained as evidence, never hidden by a passing
microbenchmark.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

from cpu_reference_report import render_report_html


REVISION = "wbc-benchmark-manifest-r260"
REFERENCE = pathlib.Path("benchmarks/results/reference-r89/reference-metrics.json")
UPKIE = pathlib.Path("benchmarks/results/reference-r89/upkie-controller-metrics.json")
FLOATING = {
    "moving_liftoff": pathlib.Path(
        "benchmarks/results/floating-g1-liftoff-latest/floating-walk-metrics.json"
    ),
    "full_transfer_stress": pathlib.Path(
        "benchmarks/results/floating-g1-transfer-latest/floating-walk-metrics.json"
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", default=str(REFERENCE))
    parser.add_argument("--upkie", default=str(UPKIE))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/WBC_BENCHMARK_MANIFEST_R260.html"
    )
    return parser.parse_args()


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
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return value


def fixed_base_summary(reference: dict[str, Any]) -> dict[str, Any]:
    scenarios: dict[str, Any] = {}
    for scenario in ("end_effector_reach", "bimanual_priority_conflict", "walking_motion_retarget"):
        implementations: dict[str, Any] = {}
        for implementation in ("bonesaw", "placo"):
            row = reference[implementation]["scenarios"][scenario]
            summary: dict[str, Any] = {
                "ticks": int(row["ticks"]),
                "tracking_rms_m": float(row["tracking_rms_m"]),
                "steady_state_rms_m": float(row["steady_state_rms_m_after_1s"]),
                "latency_us": {
                    "p50": float(row["latency_ns"]["median"]) / 1_000.0,
                    **{
                        key: float(row["latency_ns"][key]) / 1_000.0
                        for key in ("p95", "p99", "p99_9", "max")
                    },
                },
                "jitter_p99_us": float(row["jitter_abs_delta_ns"]["p99"]) / 1_000.0,
                "deadline_misses": row["deadline_misses"],
                "throughput_steps_per_second": float(row["throughput_steps_per_second"]),
            }
            if scenario == "walking_motion_retarget":
                walking = row["walking_retarget"]
                summary["walking"] = {
                    "foot_tracking_rms_m": float(walking["foot_tracking_rms_m"]),
                    "hand_tracking_rms_m": float(walking["hand_tracking_rms_m"]),
                    "stance_foot_tracking_rms_m": float(
                        walking["stance_foot_tracking_rms_m"]
                    ),
                    "swing_foot_tracking_rms_m": float(
                        walking["swing_foot_tracking_rms_m"]
                    ),
                    "acceptance": walking["acceptance"],
                }
            implementations[implementation] = summary
        scenarios[scenario] = implementations
    return scenarios


def floating_summary(paths: dict[str, pathlib.Path]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for profile, path in paths.items():
        metrics = load(path)["metrics"]
        nominal = metrics["nominal_prefix"]
        output[profile] = {
            "artifact": str(path),
            "ticks": int(metrics["ticks"]),
            "duration_seconds": float(metrics["duration_seconds"]),
            "nominal_prefix": nominal,
            "full_trace": {
                "root_tracking_rms_m": float(metrics["root_tracking_rms_m"]),
                "foot_tracking_rms_m": float(metrics["foot_tracking_rms_m"]),
                "hand_tracking_rms_m": float(metrics["hand_tracking_rms_m"]),
                "maximum_dynamics_residual": float(metrics["maximum_dynamics_residual"]),
                "maximum_contact_acceleration_residual": float(
                    metrics["maximum_contact_acceleration_residual"]
                ),
                "latency_p99_us": float(metrics["latency_us"]["p99"]),
                "jitter_p99_us": float(metrics["jitter_abs_delta_us"]["p99"]),
                "deadline_misses": metrics["deadline_misses"],
                "status_counts": metrics["status_counts"],
            },
            "acceptance": metrics["acceptance"],
        }
    return output


def upkie_summary(metrics: dict[str, Any]) -> dict[str, Any]:
    aligned = metrics["comparisons"]["official_aligned"]
    official = metrics["latency"]["bonesaw_official"]
    upstream = metrics["latency"]["upkie_cpp"]
    return {
        "ticks": int(metrics["ticks"]),
        "official_aligned": {
            "canonical_bitwise_equal": bool(aligned["canonical_bitwise_equal"]),
            "canonical_bit_mismatches": int(aligned["canonical_bit_mismatches"]),
            "maximum_absolute_error": float(aligned["maximum_absolute_error"]),
        },
        "latency_us": {
            "upstream_p50": float(upstream["p50_us"]),
            "upstream_p99": float(upstream["p99_us"]),
            "bonesaw_p50": float(official["p50_us"]),
            "bonesaw_p99": float(official["p99_us"]),
        },
        "zero_hot_loop_allocation": metrics["bonesaw_hot_loop_allocations"]["official"],
        "temporal_windows": len(metrics["temporal_windows"]),
    }


def build_result(args: argparse.Namespace) -> tuple[dict[str, Any], str]:
    reference_path = pathlib.Path(args.reference).resolve()
    upkie_path = pathlib.Path(args.upkie).resolve()
    floating_paths = {name: path.resolve() for name, path in FLOATING.items()}
    paths = {"reference": reference_path, "upkie": upkie_path, **floating_paths}
    if any(not path.is_file() for path in paths.values()):
        missing = [str(path) for path in paths.values() if not path.is_file()]
        raise FileNotFoundError(", ".join(missing))
    reference = load(reference_path)
    upkie = load(upkie_path)
    fixed_realtime_ok = all(
        all(int(value) == 0 for value in reference["bonesaw"]["scenarios"][scenario]["deadline_misses"].values())
        for scenario in ("end_effector_reach", "bimanual_priority_conflict")
    )
    result: dict[str, Any] = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_hashes": {name: sha256(path) for name, path in paths.items()},
        "source_artifacts": {name: str(path) for name, path in paths.items()},
        "fixed_base_reference": fixed_base_summary(reference),
        "floating_measured_feedback": floating_summary(floating_paths),
        "upkie_reference": upkie_summary(upkie),
        "gates": {
            "fixed_base_end_effector_and_conflict": fixed_realtime_ok,
            "fixed_base_walking_retarget": bool(
                reference["bonesaw"]["scenarios"]["walking_motion_retarget"]
                ["walking_retarget"]["acceptance"]["passed"]
            ),
            "floating_moving_liftoff": bool(
                load(floating_paths["moving_liftoff"])["metrics"]["acceptance"]["passed"]
            ),
            "floating_full_transfer": bool(
                load(floating_paths["full_transfer_stress"])["metrics"]["acceptance"]["passed"]
            ),
            "upkie_official_controller_parity": bool(
                upkie["comparisons"]["official_aligned"]["canonical_bitwise_equal"]
                and upkie["bonesaw_hot_loop_allocations"]["official"]["allocated_bytes"] == 0
            ),
        },
        "authority_admitted": False,
    }
    report_lines = [
        "# Bonesaw WBC benchmark manifest · r260",
        "",
        "> This report composes pinned Python evaluation artifacts; it does not rerun policy or plant code and admits no authority.",
        "",
        "## Coverage matrix",
        "",
        "| Boundary | Evidence | Result |",
        "|---|---|---|",
        "| End-effector reach | fixed-base Bonesaw vs PlaCo, 5,000 ticks | recorded |",
        "| Bimanual priority conflict | shared target with strict-vs-soft reference | recorded |",
        "| Walking retarget | CMU subject 37/01, three cadences | "
        + ("PASS" if result["gates"]["fixed_base_walking_retarget"] else "FAIL (retained red gate)")
        + " |",
        "| Floating moving liftoff | measured q/v, contact transition, 250/50 Hz contract | "
        + ("PASS" if result["gates"]["floating_moving_liftoff"] else "FAIL")
        + " |",
        "| Floating full transfer | same contract through support transfer | "
        + ("PASS" if result["gates"]["floating_full_transfer"] else "FAIL (retained stress case)")
        + " |",
        "| Upkie WheelBalancer | pinned upstream C++ parity and resource profile | "
        + ("PASS" if result["gates"]["upkie_official_controller_parity"] else "FAIL")
        + " |",
        "",
        "## Fixed-base tracking and timing",
        "",
        "| Scenario | Bonesaw RMS | PlaCo RMS | Bonesaw p99 | PlaCo p99 |",
        "|---|---:|---:|---:|---:|",
    ]
    for scenario, rows in result["fixed_base_reference"].items():
        report_lines.append(
            f"| {scenario} | {rows['bonesaw']['tracking_rms_m'] * 100:.3f} cm | "
            f"{rows['placo']['tracking_rms_m'] * 100:.3f} cm | "
            f"{rows['bonesaw']['latency_us']['p99']:.1f} µs | "
            f"{rows['placo']['latency_us']['p99']:.1f} µs |"
        )
    report_lines.extend(
        [
            "",
            "The fixed-base reach and conflict rows are useful end-effector and priority-regression sentinels. Walking remains a deliberate red behavior row: Bonesaw's recorded overall foot RMS is 5.549 cm against the 5 cm gate, while the independent PlaCo row misses swing-clearance RMS at 3.301 cm. Neither failure is hidden by the faster Rust path.",
            "",
            "## Floating measured-feedback behavior",
            "",
            "| Profile | Ticks | Nominal prefix | Full p99 | Full dynamics residual | Full acceptance |",
            "|---|---:|---:|---:|---:|---|",
        ]
    )
    for profile, row in result["floating_measured_feedback"].items():
        report_lines.append(
            f"| {profile} | {row['ticks']} | {row['nominal_prefix']['ticks']} | "
            f"{row['full_trace']['latency_p99_us']:.1f} µs | "
            f"{row['full_trace']['maximum_dynamics_residual']:.3g} | "
            f"{'PASS' if row['acceptance']['passed'] else 'FAIL'} |"
        )
    report_lines.extend(
        [
            "",
            "The moving-liftoff prefix passes the existing behavior and 5 ms p99 gate. The 600-tick transfer stress remains red: the measured-feedback path records contact contingency, infeasible timing/residual windows, and a 281-tick 20 ms deadline miss. This is the highest-value WBC behavior gap and is retained as a required contact-mode/retargeting improvement, not converted into a reset or authority decision.",
            "",
            "## Upkie reference",
            "",
            f"The official aligned law is canonical-bitwise equal with {result['upkie_reference']['official_aligned']['canonical_bit_mismatches']} mismatches; Bonesaw p50/p99 is {result['upkie_reference']['latency_us']['bonesaw_p50']:.3f}/{result['upkie_reference']['latency_us']['bonesaw_p99']:.3f} µs against upstream {result['upkie_reference']['latency_us']['upstream_p50']:.3f}/{result['upkie_reference']['latency_us']['upstream_p99']:.3f} µs. The measured Rust hot loop reports zero allocation calls and bytes. Ten temporal windows are retained for drift/jitter inspection.",
            "",
            "No row in this manifest authorizes a plant command. The next behavior checkpoint should improve the floating support-transition path, then rerun the same immutable metric schema with over-time windows, contact/CoP/friction margins, actuator/resource health, and reference parity intact.",
            "",
        ]
    )
    return result, "\n".join(report_lines)


def main() -> int:
    args = parse_args()
    result, report = build_result(args)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "wbc-benchmark-manifest-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "WBC_BENCHMARK_MANIFEST.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw WBC benchmark manifest · r260"))
    print(json.dumps({"revision": REVISION, "authority_admitted": False, "gates": result["gates"]}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
