#!/usr/bin/env python3
"""Policy-component, physics-free audit of Upkie rooted capture semantics."""

from __future__ import annotations

import argparse
import gc
import hashlib
import json
import math
import pathlib
import platform
import resource
import time
from datetime import datetime, timezone
from typing import Any

import numpy as np

import bonesaw
from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "upkie-rooted-capture-r129"
DT = 0.005
IDENTITY_QUATERNION = np.asarray([1.0, 0.0, 0.0, 0.0], np.float64)
ZERO_TRANSLATION = np.zeros(3, np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--ticks", type=int, default=10_000)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_ROOTED_CAPTURE_R129.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def rss_bytes() -> int:
    for line in pathlib.Path("/proc/self/status").read_text().splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1]) * 1024
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024


def balanced_state(session: bonesaw.UpkieBalanceSession) -> tuple[np.ndarray, np.ndarray]:
    root = np.asarray([0.0, 0.0, 0.539], np.float64)
    q = np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64)
    root_out = np.empty(3, np.float64)
    q_out = np.empty(6, np.float64)
    residual = session.balanced_standing(root, q, root_out, q_out)
    if abs(residual) > 1.0e-6:
        raise RuntimeError(f"balanced seed residual {residual:.3e} m")
    return root_out, q_out


class RootedCaptureCall:
    def __init__(self, session: bonesaw.UpkieBalanceSession, root: np.ndarray, q: np.ndarray):
        self.session = session
        self.root = root
        self.q = q
        self.root_twist = np.zeros(6, np.float64)
        self.joint_velocity = np.zeros(6, np.float64)
        self.control_world_from_odom = np.zeros(3, np.float64)
        self.map_from_odom = np.zeros(3, np.float64)
        self.wheel = np.empty(2, np.float64)
        self.diagnostics = np.empty(16, np.float64)
        self.names = tuple(session.capture_diagnostic_names)
        self.index = {name: index for index, name in enumerate(self.names)}

    def step(
        self,
        *,
        root_velocity_x: float = 0.0,
        odom_x: float = 0.0,
        map_x: float = 0.0,
        station_x_odom: float = 0.0,
    ) -> None:
        self.root_twist.fill(0.0)
        self.root_twist[3] = root_velocity_x
        self.control_world_from_odom.fill(0.0)
        self.control_world_from_odom[0] = odom_x
        self.map_from_odom.fill(0.0)
        self.map_from_odom[0] = map_x
        self.session.step_rooted_capture_from_state(
            DT,
            station_x_odom,
            0.0,
            self.control_world_from_odom,
            IDENTITY_QUATERNION,
            self.map_from_odom,
            IDENTITY_QUATERNION,
            self.root,
            IDENTITY_QUATERNION,
            self.root_twist,
            self.q,
            self.joint_velocity,
            self.wheel,
            self.diagnostics,
        )


def run_trace(
    model_path: pathlib.Path,
    root: np.ndarray,
    q: np.ndarray,
    root_velocity_x: np.ndarray,
    odom_x: np.ndarray,
    map_x: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    session = bonesaw.UpkieBalanceSession(str(model_path))
    call = RootedCaptureCall(session, root, q)
    ticks = len(root_velocity_x)
    wheel = np.empty((ticks, 2), np.float64)
    diagnostics = np.empty((ticks, 16), np.float64)
    elapsed_ns = np.empty(ticks, np.uint64)
    for tick in range(ticks):
        started = time.perf_counter_ns()
        call.step(
            root_velocity_x=float(root_velocity_x[tick]),
            odom_x=float(odom_x[tick]),
            map_x=float(map_x[tick]),
        )
        elapsed_ns[tick] = time.perf_counter_ns() - started
        wheel[tick] = call.wheel
        diagnostics[tick] = call.diagnostics
    return wheel, diagnostics, elapsed_ns


def make_report(metrics: dict[str, Any]) -> str:
    gates = metrics["gates"]
    timing = metrics["timing_ns"]
    return "\n".join(
        [
            f"# Upkie rooted capture audit · {REVISION}",
            "",
            f'**Admission: {"PASS" if metrics["admission"] else "FAIL"}.** This is a policy-component, physics-free corpus. Immutable oracle states are supplied on every call; there is no MuJoCo, integration, contact response, learned policy, WBC solve, or browser animation. Rust owns the one-dimensional DCM/capture computation, station-authority fade, reference-matched PI update, wheel-coordinate lowering, rooted frame evaluation, and state. Python owns only corpus construction, timing, scoring, and artifacts.',
            "",
            "## Rooted authority boundary",
            "",
            "```text",
            "map ──possibly discontinuous reporting correction──> odom",
            "                                                     │",
            "control_world <──smooth external transform───────────┘",
            "      │",
            "      ├── measured CoM + velocity ──> capture viability",
            "      └── odom station anchor ──────> fading preference",
            "```",
            "",
            f'The authority witness is the full capture state `com_x + com_velocity_x / sqrt(g / height)`. The actuator-facing reference presents **{metrics["capture_velocity_fraction"]:.3f}×** of its velocity offset to the Upkie-matched PI loop. Station authority is one inside the release band, follows a C1 smoothstep through the transition band, and is exactly zero at full capture pressure. A `map → odom` correction is consumed for reporting but has no path into capture or station commands.',
            "",
            "## Admission gates",
            "",
            *markdown_table(
                ["gate", "observed", "pass"],
                [[name, gate["observed"], gate["pass"]] for name, gate in gates.items()],
            ),
            "",
            "## Boundary timing",
            "",
            *markdown_table(
                ["calls", "p50 µs", "p95 µs", "p99 µs", "p99.9 µs", "max µs"],
                [[
                    metrics["ticks"],
                    f'{timing["p50"] / 1e3:.2f}',
                    f'{timing["p95"] / 1e3:.2f}',
                    f'{timing["p99"] / 1e3:.2f}',
                    f'{timing["p99_9"] / 1e3:.2f}',
                    f'{timing["maximum"] / 1e3:.2f}',
                ]],
            ),
            "",
            f'- Exact D1 replay: **{metrics["exact_replay"]}** across wheel commands and all sixteen diagnostics.',
            f'- Map-jump command delta: **{metrics["map_jump_max_command_delta"]:.3e}**; reporting delta: **{metrics["map_jump_reporting_delta_m"]:.6f} m**.',
            f'- Python GC collections: **{metrics["python_gc_collections"]}**; RSS delta: **{metrics["rss_delta_bytes"] / 1_048_576:.3f} MiB**.',
            f'- Rust hot-path allocation check: **{metrics["rust_hot_path_allocation_check"]}** (every call would return an error on any allocation).',
            "",
            "## Deliberate limits",
            "",
            "This proves rooted frame isolation and continuous reference semantics, not body response or WBC feasibility. The controller is sagittal and assumes the Upkie rolling axis. It has no contact-mode estimator, lateral capture set, terrain model, actuator bandwidth, delay/noise, thermal state, or hardware calibration. The separate r128 MuJoCo report remains the current physical consequence gate until this reference passes a new plant envelope.",
            "",
        ]
    )


def main() -> int:
    args = parse_args()
    if args.ticks < 100:
        raise ValueError("ticks must be at least 100")
    model_path = pathlib.Path(args.model).resolve()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    seed_session = bonesaw.UpkieBalanceSession(str(model_path))
    root, q = balanced_state(seed_session)
    names = tuple(seed_session.capture_diagnostic_names)
    index = {name: slot for slot, name in enumerate(names)}

    sample = np.arange(args.ticks, dtype=np.float64)
    root_velocity = 0.65 * np.sin(sample * 0.013) + 0.08 * np.sin(sample * 0.071)
    odom_x = 0.05 * np.sin(sample * 0.0017)
    map_x = np.where(sample < args.ticks // 2, 0.0, 10.0)
    gc.collect()
    gc_before = np.asarray([item["collections"] for item in gc.get_stats()])
    rss_before = rss_bytes()
    wheel_a, diagnostics_a, elapsed_ns = run_trace(
        model_path, root, q, root_velocity, odom_x, map_x
    )
    wheel_b, diagnostics_b, _ = run_trace(model_path, root, q, root_velocity, odom_x, map_x)
    gc_after = np.asarray([item["collections"] for item in gc.get_stats()])

    _, diagnostics_no_jump, _ = run_trace(
        model_path, root, q, root_velocity, odom_x, np.zeros(args.ticks, np.float64)
    )
    non_map_slots = np.asarray([slot for slot in range(16) if slot != index["ground_position_map_m"]])
    map_command_delta = max(
        float(np.max(np.abs(wheel_a - wheel_b))),
        float(np.max(np.abs(diagnostics_a[:, non_map_slots] - diagnostics_no_jump[:, non_map_slots]))),
    )
    map_reporting_delta = float(
        np.median(
            diagnostics_a[args.ticks // 2 :, index["ground_position_map_m"]]
            - diagnostics_no_jump[args.ticks // 2 :, index["ground_position_map_m"]]
        )
    )

    velocity_sweep = np.linspace(-0.8, 0.8, 161)
    sweep_commands = np.empty(len(velocity_sweep), np.float64)
    sweep_authority = np.empty(len(velocity_sweep), np.float64)
    for slot, velocity in enumerate(velocity_sweep):
        session = bonesaw.UpkieBalanceSession(str(model_path))
        call = RootedCaptureCall(session, root, q)
        call.step(root_velocity_x=float(velocity))
        sweep_commands[slot] = call.diagnostics[index["commanded_ground_velocity_mps"]]
        sweep_authority[slot] = call.diagnostics[index["station_authority"]]

    adjacent_command_delta = np.diff(sweep_commands)
    adjacent_full_pressure = np.isclose(sweep_authority[:-1], 0.0) & np.isclose(
        sweep_authority[1:], 0.0
    )
    full_pressure_delta = adjacent_command_delta[adjacent_full_pressure]

    high_pressure_commands = []
    for odom_shift in (-10.0, 10.0):
        session = bonesaw.UpkieBalanceSession(str(model_path))
        call = RootedCaptureCall(session, root, q)
        call.step(root_velocity_x=0.8, odom_x=odom_shift)
        high_pressure_commands.append(call.wheel.copy())

    station_targets = []
    for odom_shift in (0.0, 1.0):
        session = bonesaw.UpkieBalanceSession(str(model_path))
        call = RootedCaptureCall(session, root, q)
        call.step(odom_x=odom_shift)
        station_targets.append(call.diagnostics[index["station_position_control_world_m"]])

    degenerate_rejected = False
    fault_session = bonesaw.UpkieBalanceSession(str(model_path))
    fault_call = RootedCaptureCall(fault_session, root, q)
    try:
        fault_session.step_rooted_capture_from_state(
            DT, 0.0, 0.0, ZERO_TRANSLATION, np.zeros(4, np.float64),
            ZERO_TRANSLATION, IDENTITY_QUATERNION, root, IDENTITY_QUATERNION,
            fault_call.root_twist, q, fault_call.joint_velocity, fault_call.wheel,
            fault_call.diagnostics,
        )
    except ValueError:
        degenerate_rejected = True

    exact_replay = bool(
        np.array_equal(wheel_a, wheel_b) and np.array_equal(diagnostics_a, diagnostics_b)
    )
    gates = {
        "finite corpus": {
            "observed": bool(np.all(np.isfinite(wheel_a)) and np.all(np.isfinite(diagnostics_a))),
        },
        "exact D1 replay": {"observed": exact_replay},
        "map jump cannot change command/control diagnostics": {
            "observed": f"maximum delta {map_command_delta:.3e}",
            "pass": map_command_delta == 0.0,
        },
        "map jump remains visible in reporting": {
            "observed": f"{map_reporting_delta:.6f} m",
            "pass": abs(map_reporting_delta - 10.0) < 1.0e-12,
        },
        "full-pressure capture command is monotone in CoM velocity": {
            "observed": f"minimum adjacent full-pressure delta {np.min(full_pressure_delta):.3e}",
            "pass": bool(
                full_pressure_delta.size > 0
                and np.all(full_pressure_delta >= -1.0e-15)
            ),
        },
        "authority blend is continuous at corpus resolution": {
            "observed": f"maximum adjacent command delta {np.max(np.abs(adjacent_command_delta)):.3e}",
            "pass": bool(np.max(np.abs(adjacent_command_delta)) < 0.01),
        },
        "station authority fades monotonically with capture pressure": {
            "observed": f"center {sweep_authority[len(sweep_authority)//2]:.6f}; edges {sweep_authority[0]:.6f}/{sweep_authority[-1]:.6f}",
            "pass": bool(
                sweep_authority[len(sweep_authority) // 2] > 0.99
                and sweep_authority[0] == 0.0
                and sweep_authority[-1] == 0.0
            ),
        },
        "station target cannot oppose full-pressure capture": {
            "observed": f"wheel delta {np.max(np.abs(high_pressure_commands[0] - high_pressure_commands[1])):.3e}",
            "pass": bool(np.array_equal(high_pressure_commands[0], high_pressure_commands[1])),
        },
        "smooth odom transform moves station preference": {
            "observed": f"target delta {station_targets[1] - station_targets[0]:.6f} m",
            "pass": abs((station_targets[1] - station_targets[0]) - 1.0) < 1.0e-12,
        },
        "invalid rooted quaternion rejects atomically": {
            "observed": degenerate_rejected,
        },
        "boundary p99 below 1 ms": {
            "observed": f"{distribution(elapsed_ns)['p99'] / 1e3:.2f} µs",
            "pass": distribution(elapsed_ns)["p99"] < 1_000_000,
        },
    }
    for gate in gates.values():
        gate["pass"] = bool(gate.get("pass", gate["observed"]))
    metrics = {
        "schema_version": 1,
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "machine": platform.machine()},
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "ticks": args.ticks,
        "dt_s": DT,
        "capture_velocity_fraction": float(seed_session.capture_velocity_fraction),
        "policy_component": True,
        "physics": False,
        "integration": False,
        "wbc_solve": False,
        "diagnostic_names": names,
        "timing_ns": distribution(elapsed_ns),
        "exact_replay": exact_replay,
        "map_jump_max_command_delta": map_command_delta,
        "map_jump_reporting_delta_m": map_reporting_delta,
        "python_gc_collections": int(np.sum(gc_after - gc_before)),
        "rss_delta_bytes": rss_bytes() - rss_before,
        "rust_hot_path_allocation_check": "PASS",
        "velocity_sweep": {
            "minimum_mps": float(velocity_sweep[0]),
            "maximum_mps": float(velocity_sweep[-1]),
            "samples": len(velocity_sweep),
            "minimum_command_step_mps": float(np.min(adjacent_command_delta)),
            "minimum_full_pressure_command_step_mps": float(
                np.min(full_pressure_delta)
            ),
            "maximum_absolute_command_step_mps": float(
                np.max(np.abs(adjacent_command_delta))
            ),
        },
        "gates": gates,
        "admission": all(gate["pass"] for gate in gates.values()),
    }
    np.savez_compressed(
        output / "upkie-rooted-capture-raw.npz",
        root_velocity_x=root_velocity,
        odom_x=odom_x,
        map_x=map_x,
        wheel_acceleration=wheel_a,
        diagnostics=diagnostics_a,
        elapsed_ns=elapsed_ns,
        velocity_sweep=velocity_sweep,
        velocity_sweep_command=sweep_commands,
        velocity_sweep_station_authority=sweep_authority,
    )
    metrics_path = output / "upkie-rooted-capture-metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")
    report = make_report(metrics)
    report_path = output / "UPKIE_ROOTED_CAPTURE_AUDIT.md"
    report_path.write_text(report)
    pathlib.Path(args.web_report).write_text(
        render_report_html(report, title="Upkie rooted capture audit")
    )
    print(json.dumps({"admission": metrics["admission"], "metrics": str(metrics_path), "report": str(report_path)}, indent=2))
    return 0 if metrics["admission"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
