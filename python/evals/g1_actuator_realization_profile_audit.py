#!/usr/bin/env python3
"""R249 Rust-owned actuator bandwidth/slew profile audit.

R248 showed that a five-step held effort is not a safe proxy for an actuator
command.  This audit keeps the failed plant evidence immutable and exercises
only the existing Rust first-order bandwidth/slew realization boundary on its
spent selected-effort rows.  It performs no policy query and no rigid-body
physics step.  The profile family is declared before reading the outcomes; the
next fresh plant gate must still test the frozen choice without retuning.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_terminal_box_wbc_action_audit import sha256


REVISION = "g1-actuator-realization-profile-audit-r249"
SOURCE_REVISION = "g1-terminal-box-wbc-plant-ab-r248"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-terminal-box-wbc-plant-ab-r248/"
    "g1-terminal-box-wbc-plant-ab.npz"
)
SOURCE_METRICS = pathlib.Path(
    "benchmarks/results/g1-terminal-box-wbc-plant-ab-r248/"
    "g1-terminal-box-wbc-plant-ab-metrics.json"
)
MODEL = pathlib.Path("benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf")
CONTROL_DT_SECONDS = 0.020
PROFILE_NAMES = (
    "ideal_no_bandwidth_no_slew",
    "bandwidth_50hz_no_slew",
    "bandwidth_25hz_slew_1000_nm_s",
    "bandwidth_12hz_slew_500_nm_s",
)
# These are declared sensitivity profiles, not G1 calibration.  The first
# finite profile preserves a 50 Hz closed-loop response; the remaining two
# make the lag/slew tradeoff explicit at the 50 Hz WBC cadence.
PROFILE_PARAMETERS = np.asarray(
    [
        [np.inf, np.inf],
        [50.0, np.inf],
        [25.0, 1_000.0],
        [12.0, 500.0],
    ],
    np.float64,
)
PROFILE_COUNT = len(PROFILE_NAMES)
SAMPLE_COUNT = 96
DOF = 23


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=str(MODEL))
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--source-metrics", default=str(SOURCE_METRICS))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_ACTUATOR_REALIZATION_PROFILE_AUDIT_R249.html"
    )
    return parser.parse_args()


def load_requests(path: pathlib.Path) -> np.ndarray:
    with np.load(path) as replay:
        keys = [
            "compliant_pyramidal_implicitfast_plant_r248_selected_torque",
            "stiff_pyramidal_rk4_plant_r248_selected_torque",
        ]
        requests = np.concatenate([np.asarray(replay[key], np.float64) for key in keys])
    if requests.shape != (SAMPLE_COUNT, DOF) or not np.isfinite(requests).all():
        raise ValueError(f"R248 selected-effort replay must have shape [{SAMPLE_COUNT}, {DOF}]")
    return requests


def output_buffers(ticks: int, dof: int) -> dict[str, np.ndarray]:
    return {
        "limited": np.empty((ticks, dof), np.float64),
        "realized": np.empty((ticks, dof), np.float64),
        "tracking_error": np.empty((ticks, dof), np.float64),
        "availability_clipped": np.empty((ticks, dof), np.uint8),
        "slew_limited": np.empty((ticks, dof), np.uint8),
        "step_ns": np.empty(ticks, np.uint64),
        "allocation_calls": np.empty(ticks, np.uint64),
        "allocated_bytes": np.empty(ticks, np.uint64),
    }


def serial_parameters(parameters: np.ndarray) -> list[float | None]:
    return [None if np.isinf(value) else float(value) for value in parameters]


def run_profile(
    bonesaw: Any,
    requested: np.ndarray,
    effort_limits: np.ndarray,
    parameters: np.ndarray,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    dof = requested.shape[1]
    profiles = np.repeat(parameters[None, :], dof, axis=0)
    session = bonesaw.ActuatorRealizationSession(profiles, np.zeros(dof, np.float64))
    buffers = output_buffers(len(requested), dof)
    zero = np.zeros(dof, np.float64)
    available = np.asarray(effort_limits, np.float64)[None, :]
    for tick in range(len(requested)):
        # R248 rows are reset-every-sample states.  Reset is a Rust-owned,
        # complete-vector atomic operation, so no lag state leaks across rows.
        session.reset(zero)
        one = {key: value[tick : tick + 1] for key, value in buffers.items()}
        session.run_trace(
            requested[tick : tick + 1],
            available,
            CONTROL_DT_SECONDS,
            one["limited"],
            one["realized"],
            one["tracking_error"],
            one["availability_clipped"],
            one["slew_limited"],
            one["step_ns"],
            one["allocation_calls"],
            one["allocated_bytes"],
        )
    error = buffers["tracking_error"]
    realized = buffers["realized"]
    requested_norm = np.linalg.norm(requested, axis=1)
    response_ratio = np.divide(
        np.linalg.norm(realized, axis=1),
        requested_norm,
        out=np.ones_like(requested_norm),
        where=requested_norm > 0.0,
    )
    metrics = {
        "parameters": serial_parameters(parameters),
        "maximum_abs_tracking_error_nm": float(np.max(np.abs(error))),
        "tracking_error_nm": distribution(error.reshape(-1)),
        "response_ratio": distribution(response_ratio),
        "slew_limited_rows": int(np.count_nonzero(np.any(buffers["slew_limited"], axis=1))),
        "slew_limited_coordinates": int(np.count_nonzero(buffers["slew_limited"])),
        "availability_clipped_coordinates": int(
            np.count_nonzero(buffers["availability_clipped"])
        ),
        "p99_step_ns": float(np.percentile(buffers["step_ns"], 99)),
        "zero_rust_allocation": bool(
            np.all(buffers["allocation_calls"] == 0)
            and np.all(buffers["allocated_bytes"] == 0)
        ),
        "deadline_passed": bool(np.percentile(buffers["step_ns"], 99) <= 5_000_000.0),
    }
    return buffers, metrics


def semantic_repeat(first: dict[str, np.ndarray], second: dict[str, np.ndarray]) -> tuple[int, int]:
    keys = sorted(key for key in first if key not in ("step_ns",))
    exact = sum(np.array_equal(first[key], second[key]) for key in keys)
    return exact, len(keys)


def main() -> int:
    import bonesaw

    args = parse_args()
    replay = pathlib.Path(args.source_replay).resolve()
    source_metrics_path = pathlib.Path(args.source_metrics).resolve()
    model_path = pathlib.Path(args.model).resolve()
    if not replay.is_file() or not source_metrics_path.is_file() or not model_path.is_file():
        raise SystemExit("R249 requires immutable R248 evidence and the pinned URDF")
    source_metrics = json.loads(source_metrics_path.read_text())
    if source_metrics.get("revision") != SOURCE_REVISION:
        raise ValueError("R249 source revision mismatch")
    source_hashes_before = {
        "replay": sha256(replay),
        "metrics": sha256(source_metrics_path),
    }
    requested = load_requests(replay)
    # The effort limits are the URDF's declared limits as exposed by the Rust
    # WBC model product; no limit is inferred from a realized plant outcome.
    wbc = bonesaw.FloatingWbcSession(str(model_path), maximum_contacts=8)
    effort_limits = np.asarray(wbc.actuator_effort_limits, np.float64)
    if effort_limits.shape != (DOF,) or np.any(effort_limits <= 0.0):
        raise ValueError("pinned G1 URDF did not expose positive effort limits")
    first_outputs: dict[str, dict[str, np.ndarray]] = {}
    second_outputs: dict[str, dict[str, np.ndarray]] = {}
    profile_metrics: list[dict[str, Any]] = []
    for name, parameters in zip(PROFILE_NAMES, PROFILE_PARAMETERS, strict=True):
        first_outputs[name], first_metrics = run_profile(
            bonesaw, requested, effort_limits, parameters
        )
        second_outputs[name], _ = run_profile(
            bonesaw, requested, effort_limits, parameters
        )
        exact, compared = semantic_repeat(first_outputs[name], second_outputs[name])
        first_metrics.update(
            {
                "name": name,
                "semantic_arrays_bitwise_exact": exact,
                "semantic_arrays_compared": compared,
                "repeat_passed": exact == compared,
            }
        )
        profile_metrics.append(first_metrics)
    source_hashes_after = {
        "replay": sha256(replay),
        "metrics": sha256(source_metrics_path),
    }
    source_immutable = source_hashes_before == source_hashes_after
    mechanism_passed = bool(
        source_immutable
        and all(
            row["repeat_passed"]
            and row["zero_rust_allocation"]
            and row["deadline_passed"]
            for row in profile_metrics
        )
    )
    # This is a construction freeze, not an authority decision.  The
    # conservative finite candidate is retained for the next fresh plant
    # matrix; the ideal row remains an explicit reference, never a command.
    selected_profile = "bandwidth_25hz_slew_1000_nm_s"
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_hashes": source_hashes_before,
        "source_immutable": source_immutable,
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "control_dt_seconds": CONTROL_DT_SECONDS,
        "samples": int(len(requested)),
        "physics_steps": 0,
        "policy_steps": 0,
        "plant_actions": 0,
        "profile_names": PROFILE_NAMES,
        "profile_parameters": [serial_parameters(row) for row in PROFILE_PARAMETERS],
        "selected_profile_for_fresh_plant": selected_profile,
        "profile_selection_is_authority": False,
        "mechanism_passed": mechanism_passed,
        "authority_admitted": False,
        "profiles": profile_metrics,
    }
    rows = [
        [
            row["name"],
            f"{row['response_ratio']['p50']:.3f} / {row['response_ratio']['p99']:.3f}",
            f"{row['maximum_abs_tracking_error_nm']:.3f}",
            str(row["slew_limited_coordinates"]),
            f"{row['p99_step_ns'] / 1e3:.2f}",
            "PASS" if row["repeat_passed"] and row["zero_rust_allocation"] else "FAIL",
        ]
        for row in profile_metrics
    ]
    report = "\n".join(
        [
            "# Bonesaw actuator realization profile audit · r249",
            "",
            f"> Rust realization mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · plant **NOT RUN** · authority **NOT ADMITTED**.",
            "",
            "R249 replays 96 selected R248 efforts through Rust's first-order bandwidth and slew boundary at the 20 ms / 50 Hz WBC cadence. Every row resets the complete realized-effort state before evaluation. No policy or MuJoCo step runs; the failed R248 plant labels remain immutable.",
            "",
            *markdown_table(
                ["profile", "response norm · p50 / p99", "max |error| Nm", "slew-limited coordinates", "p99 µs", "decision"],
                rows,
            ),
            "",
            f"The selected construction profile for the next fresh plant matrix is **{selected_profile}**. This is a declared sensitivity profile, not G1 actuator calibration and not authority; the next gate must apply it without retuning on new plant laws and offsets.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-actuator-realization-profile-audit-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-actuator-realization-profile-audit.npz",
        requested_effort=requested,
        **{
            f"{name}_realized_effort": first_outputs[name]["realized"]
            for name in PROFILE_NAMES
        },
        **{
            f"{name}_tracking_error": first_outputs[name]["tracking_error"]
            for name in PROFILE_NAMES
        },
    )
    (output / "G1_ACTUATOR_REALIZATION_PROFILE_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw actuator realization · r249"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "selected_profile_for_fresh_plant": selected_profile,
                "plant_actions": 0,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
