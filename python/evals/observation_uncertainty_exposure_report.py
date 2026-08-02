#!/usr/bin/env python3
"""Retain the policy-/physics-free reconstruction-exposure CPU audit."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
import subprocess
from datetime import datetime, timezone
from typing import Any

from cpu_reference_report import markdown_table as markdown_table_rows, render_report_html


def table(headers: list[str], rows: list[list[Any]]) -> str:
    return "\n".join(markdown_table_rows(headers, rows))


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def run_audit(binary: pathlib.Path, model: pathlib.Path, repeats: int) -> dict[str, Any]:
    completed = subprocess.run(
        [str(binary), str(model), str(repeats)],
        check=False,
        capture_output=True,
        text=True,
    )
    try:
        audit = json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(
            f"uncertainty audit returned no JSON (exit {completed.returncode}): "
            f"{completed.stderr.strip()}"
        ) from error
    audit["process_exit_code"] = completed.returncode
    audit["process_stderr"] = completed.stderr.strip()
    return audit


def validate_audit(audit: dict[str, Any]) -> None:
    assert audit["status"] == "pass"
    assert audit["process_exit_code"] == 0
    assert audit["execution"] == "fixed_state_without_policy_physics_or_integration"
    sweep = audit["sweep"]
    assert [row["exposure_ns"] for row in sweep] == sorted(
        row["exposure_ns"] for row in sweep
    )
    assert sweep[0]["exposure_ns"] == 0
    assert sweep[0]["self_collision_margin_erosion_m"] == 0.0
    previous = None
    for row in sweep:
        error = row["error_bound"]
        if previous is not None:
            assert error["joint_position_error_rad"] > previous["error_bound"][
                "joint_position_error_rad"
            ]
            assert error["represented_point_position_error_m"] > previous[
                "error_bound"
            ]["represented_point_position_error_m"]
            assert row["joint_stopping_upper_erosion_rad_s2"] > previous[
                "joint_stopping_upper_erosion_rad_s2"
            ]
            assert row["self_collision_robust_margin_m"] < previous[
                "self_collision_robust_margin_m"
            ]
        assert abs(
            row["self_collision_margin_erosion_m"]
            - 2.0 * error["represented_point_position_error_m"]
        ) < 1e-12
        assert abs(
            row["self_collision_raw_margin_m"]
            - audit["nominal"]["collision_margin_m"]
        ) < 1e-12
        assert row["error_bound_allocation_calls"] == 0
        assert row["error_bound_allocated_bytes"] == 0
        assert row["solve"]["allocation_calls"] == 0
        assert row["solve"]["allocated_bytes"] == 0
        assert row["solve"]["bitwise_repeat"] is True
        assert row["inside_live_prediction_horizon"] == (row["exposure_ns"] <= 5_000_000)
        previous = row


def row_for_exposure(audit: dict[str, Any], exposure_ns: int) -> dict[str, Any]:
    return next(row for row in audit["sweep"] if row["exposure_ns"] == exposure_ns)


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    sweep = audit["sweep"]
    live = row_for_exposure(audit, 5_000_000)
    live_nominal = live["solve"]["nominal_timing_us"]
    live_robust = live["solve"]["robust_timing_us"]
    rows = []
    for row in sweep:
        error = row["error_bound"]
        solve = row["solve"]
        rows.append(
            [
                f"{row['exposure_ns'] / 1e6:.1f}",
                "yes" if row["inside_live_prediction_horizon"] else "no · diagnostic only",
                f"{error['joint_position_error_rad'] * 1e3:.5f}",
                f"{error['represented_point_position_error_m'] * 1e3:.5f}",
                f"{row['joint_stopping_upper_erosion_rad_s2']:.6f}",
                f"{row['self_collision_robust_margin_m'] * 1e3:.5f}",
                f"{row['required_normal_acceleration_mps2']:.6f}",
                f"{row['error_bound_ns_per_call']:.2f}",
                f"{solve['nominal_timing_us']['p50']:.3f} / {solve['robust_timing_us']['p50']:.3f}",
                f"{solve['nominal_timing_us']['p99']:.3f} / {solve['robust_timing_us']['p99']:.3f}",
            ]
        )
    return f"""# Bonesaw reconstruction-exposure CPU authority · r120

## Outcome

**PASS.** A fixed-state, policy-/physics-free Rust audit sweeps reconstruction
exposure from 0 to 10 ms. It calls the same typed error-growth primitive used by
the live boundary, intersects the joint-stopping interval over every
`q ± error, v ± error` corner, and executes the actual floating WBC with a tight
self-collision barrier. No state is integrated and no output is called plant
response.

The production live prediction horizon ends at 5 ms. The 7.5 and 10 ms rows are
diagnostic continuations of the growth curve only; live reconstruction would
withhold them before hard rows.

{table(['exposure ms', 'live-eligible', 'q error mrad', 'point error mm', 'stopping upper loss rad/s²', 'robust self margin mm', 'required normal accel m/s²', 'bound ns/call', 'nominal/robust p50 µs', 'nominal/robust p99 µs'], rows)}

## Five-millisecond live boundary

{table(['signal', 'result'], [
    ['joint position / velocity error', f"{live['error_bound']['joint_position_error_rad'] * 1e3:.6f} mrad / {live['error_bound']['joint_velocity_error_rad_s']:.6f} rad/s"],
    ['joint-stopping upper-bound loss', f"{live['joint_stopping_upper_erosion_rad_s2']:.6f} rad/s²"],
    ['self-collision raw → robust margin', f"{live['self_collision_raw_margin_m'] * 1e3:.6f} → {live['self_collision_robust_margin_m'] * 1e3:.6f} mm"],
    ['barrier required acceleration', f"{audit['nominal']['required_normal_acceleration_mps2']:.6f} → {live['required_normal_acceleration_mps2']:.6f} m/s²"],
    ['error-bound cost', f"{live['error_bound_ns_per_call']:.2f} ns/call"],
    ['nominal / robust p50', f"{live_nominal['p50']:.3f} / {live_robust['p50']:.3f} µs (Δ {live['solve']['p50_delta_us']:+.3f})"],
    ['nominal / robust p99', f"{live_nominal['p99']:.3f} / {live_robust['p99']:.3f} µs (Δ {live['solve']['p99_delta_us']:+.3f})"],
    ['nominal / robust p99−p50', f"{live_nominal['jitter_p99_minus_p50']:.3f} / {live_robust['jitter_p99_minus_p50']:.3f} µs"],
])}

The alternating A/B loop changes call order every repeat so warmup and scheduler
drift do not systematically favor nominal or robust. Timing is descriptive for
this host. A small signed delta is not evidence that uncertainty makes the
dense solve intrinsically faster or slower; both paths emit the same number of
rows and differ only in scalar margin values.

## Allocation, memory, and determinism

{table(['boundary', 'result'], [
    [f"error growth · {audit['error_bound_repeats']:,} calls/exposure", '0 allocation calls · 0 bytes'],
    [f"paired WBC · {audit['solve_repeats']:,} nominal + robust calls/exposure", '0 allocation calls · 0 bytes'],
    ['complete generalized-acceleration replay', 'bitwise identical at every exposure'],
    ['reconstruction evidence value', f"{audit['fixed_value_bytes']['reconstruction_evidence']} bytes"],
    ['growth configuration value', f"{audit['fixed_value_bytes']['error_growth']} bytes"],
    ['derived error-bound value', f"{audit['fixed_value_bytes']['error_bound']} bytes"],
])}

These are fixed Rust values embedded beside already preallocated WBC scratch and
output; the audit does not infer memory from process RSS. It directly counts
allocator calls and bytes around the hot primitives.

## Interpretation

- Error and consumed margins grow continuously and monotonically with exposure.
- The raw 10 mm collision geometry is invariant; only robust authority shrinks.
- At 5 ms, the represented-point radius consumes 0.525 mm and increases the
  required outward acceleration from 3.0000 to 3.0525 m/s².
- The near-limit joint witness loses 11.3474 rad/s² of its safe upper
  acceleration interval at 5 ms, unlike the live squat trace whose ±200 rad/s²
  cap masked this effect.
- The growth rates remain caller-authored deterministic contracts—not
  covariance, statistical confidence, calibrated estimator residuals, network
  evidence, hardware safety certification, or realized plant behavior.

## Retained audit

```json
{json.dumps(audit, indent=2, sort_keys=True)}
```
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--binary", default="target/release/bonesaw-observation-uncertainty-audit"
    )
    parser.add_argument("--model", default="models/tight_avoidance_toy.urdf")
    parser.add_argument("--repeats", type=int, default=5000)
    parser.add_argument(
        "--output", default="benchmarks/results/observation-uncertainty-exposure-r120"
    )
    parser.add_argument(
        "--web-report", default="web/OBSERVATION_UNCERTAINTY_EXPOSURE_R120.html"
    )
    args = parser.parse_args()
    if args.repeats < 20:
        raise ValueError("--repeats must be at least 20")

    binary = pathlib.Path(args.binary)
    model = pathlib.Path(args.model)
    audit = run_audit(binary, model, args.repeats)
    validate_audit(audit)
    source = pathlib.Path(
        "crates/bonesaw-tools/src/bin/observation_uncertainty_audit.rs"
    )
    metrics = {
        "schema": 1,
        "revision": "observation-uncertainty-exposure-r120",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {
            str(path): sha256(path) for path in (binary, model, source)
        },
        "audit": audit,
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "audit.json").write_text(json.dumps(metrics, indent=2) + "\n")
    report = markdown(metrics)
    (output / "OBSERVATION_UNCERTAINTY_EXPOSURE_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw reconstruction-exposure CPU authority · r120")
    )
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
