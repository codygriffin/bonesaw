#!/usr/bin/env python3
"""Measure sequential Upkie contacts from MuJoCo geometry without integration.

The fixture authors root poses, never contact masks. MuJoCo forward collision
detection produces every 250 Hz wheel/ground observation; the newest of each
five-frame window is passed into the existing persistent Rust WBC adapter at
50 Hz. No policy is queried and ``mj_step`` is forbidden.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import pathlib
import sys
from datetime import datetime, timezone
from typing import Any
from unittest.mock import patch

import mujoco
import numpy as np

import upkie_mujoco_plant_report as plant
from upkie_live_plant_worker import LiveUpkiePlant


REVISION = "upkie-measured-contact-kinematic-r299"
PHYSICS_HZ = 250
CONTROL_HZ = 50
SUBSTEPS = PHYSICS_HZ // CONTROL_HZ

# name, roll, root-z offset, 50 Hz repetitions, expected measured mask
PHASES: tuple[tuple[str, float, float, int, tuple[int, int]], ...] = (
    ("double", 0.0, 0.0, 3, (1, 1)),
    ("left_only", -0.08, 0.012, 2, (1, 0)),
    ("right_only", 0.08, 0.012, 3, (0, 1)),
    ("flight", 0.0, 0.5, 2, (0, 0)),
)


def control_poses() -> list[dict[str, Any]]:
    return [
        {
            "phase": name,
            "roll_rad": roll,
            "root_z_offset_m": z_offset,
            "expected_mask": list(mask),
        }
        for name, roll, z_offset, count, mask in PHASES
        for _ in range(count)
    ]


def _set_pose(
    worker: LiveUpkiePlant,
    base_qpos: np.ndarray,
    root_qpos: int,
    pose: dict[str, Any],
) -> None:
    worker.data.qpos[:] = base_qpos
    worker.data.qvel.fill(0.0)
    worker.data.qpos[root_qpos + 2] += float(pose["root_z_offset_m"])
    half_roll = 0.5 * float(pose["roll_rad"])
    worker.data.qpos[root_qpos + 3 : root_qpos + 7] = (
        math.cos(half_roll),
        math.sin(half_roll),
        0.0,
        0.0,
    )
    mujoco.mj_forward(worker.model, worker.data)


def run_replay(model_path: pathlib.Path) -> dict[str, Any]:
    worker = LiveUpkiePlant(model_path)
    root_joint = mujoco.mj_name2id(
        worker.model, mujoco.mjtObj.mjOBJ_JOINT, "root"
    )
    root_qpos = int(worker.model.jnt_qposadr[root_joint])
    base_qpos = worker.data.qpos.copy()
    poses = control_poses()
    previous = poses[0]
    scratch = np.empty(2, np.uint8)
    physics_frames: list[dict[str, Any]] = []
    states: list[dict[str, Any]] = []
    initial_time = float(worker.data.time)

    def forbidden_step(*_args: Any, **_kwargs: Any) -> None:
        raise AssertionError("R299 forbids MuJoCo dynamics integration")

    with patch.object(mujoco, "mj_step", forbidden_step):
        for tick, pose in enumerate(poses):
            window: list[list[int]] = []
            for substep in range(SUBSTEPS):
                frame_pose = previous if substep < SUBSTEPS - 1 else pose
                _set_pose(worker, base_qpos, root_qpos, frame_pose)
                plant.measured_wheel_ground_contacts_into(
                    worker.model,
                    worker.data,
                    worker.wheel_contact_body_sets,
                    scratch,
                )
                measured = scratch.astype(int).tolist()
                window.append(measured)
                physics_frames.append(
                    {
                        "frame": tick * SUBSTEPS + substep,
                        "control_tick": tick,
                        "substep": substep,
                        "phase": frame_pose["phase"],
                        "roll_rad": frame_pose["roll_rad"],
                        "root_z_offset_m": frame_pose["root_z_offset_m"],
                        "measured_mask": measured,
                        "contact_count": int(worker.data.ncon),
                        "minimum_distance_m": min(
                            (
                                float(contact.dist)
                                for contact in worker.data.contact[: worker.data.ncon]
                            ),
                            default=0.0,
                        ),
                    }
                )

            _set_pose(worker, base_qpos, root_qpos, pose)
            root_position, root_quaternion, root_twist, q, v = plant.read_state(
                worker.model, worker.data
            )
            result = worker.controller.solve(
                root_position,
                root_quaternion,
                root_twist,
                q,
                v,
                float(np.mean(worker.data.xpos[worker.wheel_bodies, 0])),
                float(np.mean(worker.data.xpos[worker.wheel_bodies, 2])),
                observed_contact_active=np.asarray(window[-1], np.uint8),
                observed_contact_available=True,
            )
            states.append(
                {
                    "tick": tick,
                    "phase": pose["phase"],
                    "roll_rad": pose["roll_rad"],
                    "root_z_offset_m": pose["root_z_offset_m"],
                    "physics_masks": window,
                    "observed": window[-1],
                    "debounced": worker.controller.contact_debounced.astype(int).tolist(),
                    "hard": worker.controller.contact_active[0].astype(int).tolist(),
                    "contact_observation_status": int(result["contact_observation_status"]),
                    "contact_observation_provenance": int(result["contact_observation_provenance"]),
                    "contact_observation_flags": int(result["contact_observation_flags"]),
                    "wbc_status": int(result["status"]),
                    "raw_wbc_status": int(result["raw_wbc_status"]),
                    "maximum_constraint_violation": float(result["maximum_constraint_violation"]),
                    "dynamics_residual": float(result["dynamics_residual"]),
                    "contact_residual": float(result["contact_residual"]),
                    "step_ns": int(result["step_ns"]),
                    "allocation_calls": int(result["allocation_calls"]),
                    "allocated_bytes": int(result["allocated_bytes"]),
                }
            )
            previous = pose

    return {
        "revision": REVISION,
        "model": str(model_path.resolve()),
        "physics_hz": PHYSICS_HZ,
        "control_hz": CONTROL_HZ,
        "physics_substeps_per_control": SUBSTEPS,
        "policy_steps": 0,
        "physics_integration_steps": 0,
        "initial_simulator_time_s": initial_time,
        "final_simulator_time_s": float(worker.data.time),
        "poses": poses,
        "physics_frames": physics_frames,
        "states": states,
    }


def _semantic(result: dict[str, Any]) -> dict[str, Any]:
    copy = json.loads(json.dumps(result))
    for state in copy["states"]:
        state.pop("step_ns")
    return copy


def evaluate(result: dict[str, Any], replay: dict[str, Any]) -> dict[str, bool]:
    states = result["states"]
    expected = [pose["expected_mask"] for pose in result["poses"]]
    observed = [state["observed"] for state in states]
    debounced = [state["debounced"] for state in states]
    hard = [state["hard"] for state in states]
    finite_keys = (
        "maximum_constraint_violation",
        "dynamics_residual",
        "contact_residual",
    )
    return {
        "actual_mujoco_contact_extractor": observed == expected,
        "all_contact_patterns_measured": {tuple(mask) for mask in observed}
        == {(1, 1), (1, 0), (0, 1), (0, 0)},
        "fifty_measured_250hz_frames": len(result["physics_frames"])
        == len(states) * SUBSTEPS
        == 50,
        "newest_frame_enters_wbc": all(
            state["observed"] == state["physics_masks"][-1] for state in states
        ),
        "edges_exist_inside_windows": any(
            len({tuple(mask) for mask in state["physics_masks"]}) > 1
            for state in states
        ),
        "activation_debounce": debounced[:3] == [[0, 0], [0, 0], [1, 1]]
        and debounced[7] == [0, 1],
        "deactivation_debounce": debounced[3:7]
        == [[1, 1], [1, 0], [1, 0], [0, 0]],
        "hard_rows_are_raw_subset": all(
            all(h <= r for h, r in zip(hard_tick, raw_tick, strict=True))
            for hard_tick, raw_tick in zip(hard, observed, strict=True)
        ),
        "flight_fails_closed": hard[8:] == [[0, 0], [0, 0]],
        "accepted_exact_observations": all(
            state["contact_observation_status"] == 0
            and state["contact_observation_provenance"] == 0
            for state in states
        ),
        "wbc_outputs_finite": all(
            math.isfinite(float(state[key])) for state in states for key in finite_keys
        ),
        "rust_hot_loop_zero_allocations": all(
            state["allocation_calls"] == 0 and state["allocated_bytes"] == 0
            for state in states
        ),
        "zero_policy_zero_physics_integration": result["policy_steps"] == 0
        and result["physics_integration_steps"] == 0
        and result["initial_simulator_time_s"] == result["final_simulator_time_s"],
        "exact_semantic_replay": _semantic(result) == _semantic(replay),
    }


def render_markdown(result: dict[str, Any], gates: dict[str, bool]) -> str:
    states = result["states"]
    lines = [
        "# Upkie kinematic measured-contact replay · R299",
        "",
        f"> {'PASS' if all(gates.values()) else 'FAIL'} · actual MuJoCo collision masks · zero policy · zero dynamics integration · authority closed.",
        "",
        "R299 authors only root poses. Each of 50 nominal 250 Hz frames runs `mj_forward` and the production wheel-subtree/ground contact extractor. The newest observation in each five-frame window enters the existing persistent Rust adapter and WBC at 50 Hz. `mj_step` is patched to fail, so no plant trajectory or physics consequence is claimed.",
        "",
        "| tick | phase | roll rad | z offset m | measured | debounced | hard | WBC status | residual | step µs |",
        "|---:|---|---:|---:|:---:|:---:|:---:|---:|---:|---:|",
    ]
    for state in states:
        lines.append(
            f"| {state['tick']} | {state['phase']} | {state['roll_rad']:.3f} | {state['root_z_offset_m']:.3f} | `{state['observed'][0]}{state['observed'][1]}` | `{state['debounced'][0]}{state['debounced'][1]}` | `{state['hard'][0]}{state['hard'][1]}` | {state['wbc_status']} | {state['maximum_constraint_violation']:.3e} | {state['step_ns'] / 1000.0:.1f} |"
        )
    lines.extend(["", "## Gates", "", "| gate | result |", "|---|:---:|"])
    lines.extend(
        f"| {name} | {'PASS' if passed else 'FAIL'} |"
        for name, passed in gates.items()
    )
    lines.extend(
        [
            "",
            "This admits the collision-extraction → observation → debounce → hard-row seam for a deterministic kinematic fixture. It does not admit a physically realized transfer, tracking performance, contact-force accuracy, hardware sensing, or command authority.",
            "",
        ]
    )
    return "\n".join(lines)


def render_html(report: str, passed: bool) -> str:
    return "\n".join(
        (
            "<!doctype html>",
            "<meta name='viewport' content='width=device-width,initial-scale=1'>",
            "<title>Bonesaw Upkie measured-contact replay R299</title>",
            "<style>body{font:15px system-ui,sans-serif;max-width:1050px;margin:2rem auto;padding:0 1rem;background:#111827;color:#e5e7eb}pre{white-space:pre-wrap;line-height:1.45;background:#1f2937;padding:1rem;border-radius:8px}h1{color:#93c5fd}.pass{color:#86efac}.fail{color:#fca5a5}</style>",
            f"<h1 class='{'pass' if passed else 'fail'}'>Upkie kinematic measured-contact replay · R299</h1>",
            f"<pre>{html.escape(report)}</pre>",
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_MEASURED_CONTACT_KINEMATIC_R299.html")
    args = parser.parse_args()
    result = run_replay(pathlib.Path(args.model))
    replay = run_replay(pathlib.Path(args.model))
    gates = evaluate(result, replay)
    passed = all(gates.values())
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "passed": passed,
        "gates": gates,
        "trace": result,
    }
    (output / "upkie-measured-contact-kinematic-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    report = render_markdown(result, gates)
    (output / "UPKIE_MEASURED_CONTACT_KINEMATIC_R299.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_html(report, passed))
    print(json.dumps({"passed": passed, "gates": gates}, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
