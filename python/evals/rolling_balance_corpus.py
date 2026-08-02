#!/usr/bin/env python3
"""Deterministic Upkie rolling-balance acceptance corpus.

Python owns scenario design, process isolation, acceptance policy, and report
rendering. Each measured servo loop remains inside the Rust evaluator.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import subprocess
import time
from dataclasses import asdict, dataclass
from typing import Any


@dataclass(frozen=True)
class Scenario:
    name: str
    split: str
    initial_ground_velocity_mps: float


SCENARIOS = (
    Scenario("nominal", "train", 0.0),
    Scenario("slow_forward", "train", 0.01),
    Scenario("slow_backward", "train", -0.01),
    Scenario("medium_forward", "train", 0.03),
    Scenario("medium_backward", "train", -0.03),
    Scenario("heldout_forward", "heldout", 0.02),
    Scenario("heldout_backward", "heldout", -0.02),
    Scenario("heldout_fast_forward", "heldout", 0.05),
    Scenario("heldout_fast_backward", "heldout", -0.05),
)

THRESHOLDS = {
    "infeasible_ticks": 0,
    "final_root_height_error_abs_m": 0.015,
    "root_height_tracking_rms_m": 0.020,
    "root_horizontal_tracking_rms_m": 0.045,
    "final_root_horizontal_error_m": 0.015,
    "maximum_center_of_mass_tracking_error_m": 0.065,
    "maximum_constrained_contact_drift_m": 0.002,
    "maximum_root_rotation_rad": 0.15,
    "minimum_friction_margin": -1.0e-7,
    "minimum_torque_margin": -1.0e-7,
    "allocations_per_step": 0.01,
}
TIMING_FIELDS = {
    "mean_step_us",
    "p50_step_us",
    "p99_step_us",
    "max_step_us",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", default="target/release/bonesaw-eval")
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--ticks", type=int, default=500)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--endurance-ticks", type=int, default=5_000)
    parser.add_argument(
        "--output",
        default="benchmarks/results/rolling-balance-latest",
    )
    return parser.parse_args()


def checks(report: dict[str, Any]) -> dict[str, bool]:
    return {
        "feasible": report["infeasible_ticks"] <= THRESHOLDS["infeasible_ticks"],
        "final_height": abs(report["final_root_height_error_m"])
        <= THRESHOLDS["final_root_height_error_abs_m"],
        "height_tracking": report["root_height_tracking_rms_m"]
        <= THRESHOLDS["root_height_tracking_rms_m"],
        "horizontal_tracking": report["root_horizontal_tracking_rms_m"]
        <= THRESHOLDS["root_horizontal_tracking_rms_m"],
        "horizontal_recovery": report["final_root_horizontal_error_m"]
        <= THRESHOLDS["final_root_horizontal_error_m"],
        "com_tracking": report["maximum_center_of_mass_tracking_error_m"]
        <= THRESHOLDS["maximum_center_of_mass_tracking_error_m"],
        "contact_drift": report["maximum_constrained_contact_drift_m"]
        <= THRESHOLDS["maximum_constrained_contact_drift_m"],
        "root_rotation": report["maximum_root_rotation_rad"]
        <= THRESHOLDS["maximum_root_rotation_rad"],
        "friction_margin": report["minimum_friction_margin"]
        >= THRESHOLDS["minimum_friction_margin"],
        "torque_margin": report["minimum_torque_margin"]
        >= THRESHOLDS["minimum_torque_margin"],
        "allocation_rate": report["allocations_per_step"]
        <= THRESHOLDS["allocations_per_step"],
    }


def run_scenario(
    binary: pathlib.Path,
    model: pathlib.Path,
    ticks: int,
    scenario: Scenario,
    repeats: int,
) -> dict[str, Any]:
    environment = os.environ.copy()
    environment["BONESAW_SQUAT_PERTURB"] = str(
        scenario.initial_ground_velocity_mps
    )
    command = [
        str(binary),
        "--model",
        str(model),
        "--ticks",
        str(ticks),
        "--squat-only",
        "--json",
    ]
    reports = []
    wall_seconds = []
    for _ in range(repeats):
        started = time.perf_counter()
        completed = subprocess.run(
            command,
            check=True,
            capture_output=True,
            text=True,
            env=environment,
        )
        wall_seconds.append(time.perf_counter() - started)
        reports.append(json.loads(completed.stdout))
    report = reports[0]
    deterministic_fields = sorted(set(report) - TIMING_FIELDS)
    mismatch_fields = [
        field
        for field in deterministic_fields
        if any(other[field] != report[field] for other in reports[1:])
    ]
    acceptance = checks(report)
    acceptance["bitwise_repeat"] = not mismatch_fields
    return {
        "scenario": asdict(scenario),
        "wall_seconds": wall_seconds,
        "repeats": repeats,
        "deterministic": not mismatch_fields,
        "deterministic_mismatch_fields": mismatch_fields,
        "accepted": all(acceptance.values()),
        "checks": acceptance,
        "metrics": report,
    }


def render_markdown(payload: dict[str, Any]) -> str:
    endurance = payload["endurance"]
    lines = [
        "# Bonesaw rolling-balance corpus",
        "",
        f"Overall: **{'PASS' if payload['accepted'] else 'FAIL'}**  ",
        f"Ticks/scenario: {payload['ticks']} at 5 ms  ",
        f"Isolated repeats/scenario: {payload['repeats']}  ",
        "Initial velocity cases set root and both wheel rates to satisfy the "
        "no-slip rows exactly.",
        "",
        "| split | scenario | v₀ (m/s) | pass | infeasible | height RMS (m) | "
        "horizontal RMS (m) | CoM max (m) | drift (m) | rotation (rad) | "
        "p99 (µs) | alloc/step |",
        "|---|---|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for result in payload["results"]:
        scenario = result["scenario"]
        metrics = result["metrics"]
        lines.append(
            f"| {scenario['split']} | {scenario['name']} | "
            f"{scenario['initial_ground_velocity_mps']:.3f} | "
            f"{'PASS' if result['accepted'] else 'FAIL'} | "
            f"{metrics['infeasible_ticks']} | "
            f"{metrics['root_height_tracking_rms_m']:.6f} | "
            f"{metrics['root_horizontal_tracking_rms_m']:.6f} | "
            f"{metrics['maximum_center_of_mass_tracking_error_m']:.6f} | "
            f"{metrics['maximum_constrained_contact_drift_m']:.6f} | "
            f"{metrics['maximum_root_rotation_rad']:.6f} | "
            f"{metrics['p99_step_us']:.1f} | "
            f"{metrics['allocations_per_step']:.4f} |"
        )
    lines.extend(
        [
            "",
            "## Nominal endurance",
            "",
            "This single long run uses the same raw integrated WBC path after all "
            "train and held-out perturbation cases.",
            "",
            "| ticks | achieved lowering cm | root z RMS cm | root xy RMS cm | CoM max cm | constrained drift mm | wheel travel cm | dynamics max | contact max | infeasible | p50 µs | p99 µs | max µs | alloc/step |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
            f"| {endurance['ticks']:,} | {endurance['achieved_lowering_m'] * 100:.3f} | "
            f"{endurance['root_height_tracking_rms_m'] * 100:.3f} | "
            f"{endurance['root_horizontal_tracking_rms_m'] * 100:.3f} | "
            f"{endurance['maximum_center_of_mass_tracking_error_m'] * 100:.3f} | "
            f"{endurance['maximum_constrained_contact_drift_m'] * 1000:.4f} | "
            f"{endurance['maximum_permitted_rolling_travel_m'] * 100:.3f} | "
            f"{endurance['maximum_dynamics_residual']:.2e} | "
            f"{endurance['maximum_contact_acceleration_residual']:.2e} | "
            f"{endurance['infeasible_ticks']:,} | {endurance['p50_step_us']:.1f} | "
            f"{endurance['p99_step_us']:.1f} | {endurance['max_step_us']:.1f} | "
            f"{endurance['allocations_per_step']:.4f} |",
            "",
            "## Acceptance thresholds",
            "",
            "```json",
            json.dumps(payload["thresholds"], indent=2),
            "```",
            "",
            "A scenario passes only when every named check passes. Latency is "
            "retained for diagnosis, but does not become an acceptance metric "
            "until the controller completes every scenario feasibly.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    if args.ticks < 100:
        raise SystemExit("--ticks must be at least 100")
    if args.repeats < 2:
        raise SystemExit("--repeats must be at least 2")
    if args.endurance_ticks < 100:
        raise SystemExit("--endurance-ticks must be at least 100")
    binary = pathlib.Path(args.binary).resolve()
    model = pathlib.Path(args.model).resolve()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    results = [
        run_scenario(binary, model, args.ticks, scenario, args.repeats)
        for scenario in SCENARIOS
    ]
    endurance = run_scenario(
        binary, model, args.endurance_ticks, SCENARIOS[0], 1
    )["metrics"]
    payload = {
        "schema": 1,
        "ticks": args.ticks,
        "repeats": args.repeats,
        "timestep_seconds": 0.005,
        "binary": str(binary),
        "model": str(model),
        "thresholds": THRESHOLDS,
        "accepted": all(result["accepted"] for result in results),
        "training_accepted": all(
            result["accepted"]
            for result in results
            if result["scenario"]["split"] == "train"
        ),
        "heldout_accepted": all(
            result["accepted"]
            for result in results
            if result["scenario"]["split"] == "heldout"
        ),
        "results": results,
        "endurance": endurance,
    }
    (output / "rolling-balance-corpus.json").write_text(
        json.dumps(payload, indent=2) + "\n"
    )
    (output / f"nominal-endurance-{args.endurance_ticks}.json").write_text(
        json.dumps(endurance, indent=2) + "\n"
    )
    markdown = render_markdown(payload)
    (output / "ROLLING_BALANCE_CORPUS.md").write_text(markdown)
    print(markdown)


if __name__ == "__main__":
    main()
