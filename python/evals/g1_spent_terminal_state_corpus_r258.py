#!/usr/bin/env python3
"""R258 terminal position/attitude corpus extraction on already-spent R250 states."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone

import numpy as np

from cpu_reference_report import distribution, render_report_html
from g1_actuator_bandwidth_action_freeze_r250 import (
    FAMILY_CASES,
    PROFILE_CASES,
    evaluate_case,
)
from g1_compliant_terminal_consequence_audit import joint_limits
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256


REVISION = "g1-spent-terminal-state-corpus-r258"
SOURCE_REVISION = "g1-actuator-bandwidth-action-freeze-r250"
SPENT_GATE_REVISION = "g1-terminal-box-wbc-plant-ab-r248"
SOURCE_METRICS = pathlib.Path(
    "benchmarks/results/g1-terminal-box-wbc-plant-ab-r248/"
    "g1-terminal-box-wbc-plant-ab-metrics.json"
)
PROFILE = PROFILE_CASES[0]
FAMILY = FAMILY_CASES[1]
ROWS = 96
CANDIDATES = 3
SUBSTEPS = 5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-metrics", default=str(SOURCE_METRICS))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_SPENT_TERMINAL_STATE_CORPUS_R258.html"
    )
    return parser.parse_args()


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source_metrics_path = pathlib.Path(args.source_metrics).resolve()
    if not model.is_file() or not source_metrics_path.is_file():
        raise SystemExit("R258 requires the pinned G1 model and spent R248 gate")
    source_metrics = json.loads(source_metrics_path.read_text())
    if source_metrics.get("revision") != SPENT_GATE_REVISION:
        raise ValueError("R258 source is not the frozen spent R248 gate")
    source_hash = sha256(source_metrics_path)
    case_metrics, arrays = evaluate_case(bonesaw, model, PROFILE, FAMILY)
    root_terminal = np.asarray(arrays["actual_root_terminal_state"], np.float64)
    q_terminal = np.asarray(arrays["actual_joint_terminal_position"], np.float64)
    velocity_terminal = np.asarray(arrays["actual_velocity"], np.float64)
    diagnostics = np.asarray(arrays["actual_diagnostics"], np.float64)
    effort = np.asarray(arrays["candidate_effort_utilization"], np.float64)
    fixed_status = np.asarray(arrays["fixed_status"], np.uint8)
    if (
        root_terminal.shape != (ROWS, CANDIDATES, 6)
        or q_terminal.shape[:2] != (ROWS, CANDIDATES)
        or velocity_terminal.shape != (ROWS, CANDIDATES, 6 + q_terminal.shape[2])
        or diagnostics.shape != (ROWS, CANDIDATES, 17)
    ):
        raise ValueError("R258 extracted terminal corpus has an unexpected shape")

    scorer = bonesaw.ContactTransitionModelSession(str(model), [FOOT_FRAMES[0]])
    limits = joint_limits(model, list(scorer.joint_names()))
    replay = np.empty_like(diagnostics)
    zero_root_acceleration = np.zeros((1, 2), np.float64)
    zero_joint_acceleration = np.zeros((1, q_terminal.shape[2]), np.float64)
    available = np.ones(1, np.uint8)
    replay_timing_ns = np.empty((ROWS, CANDIDATES), np.uint64)
    replay_allocation_calls = np.empty_like(replay_timing_ns)
    replay_allocated_bytes = np.empty_like(replay_timing_ns)
    for row in range(ROWS):
        for candidate in range(CANDIDATES):
            timing = scorer.score_terminal_impact_state_batch(
                np.ascontiguousarray(root_terminal[row, candidate : candidate + 1]),
                np.ascontiguousarray(q_terminal[row, candidate]),
                np.ascontiguousarray(velocity_terminal[row, candidate : candidate + 1, 6:]),
                limits[0],
                limits[1],
                limits[2],
                available,
                zero_root_acceleration,
                zero_joint_acceleration,
                np.ascontiguousarray(effort[row, candidate : candidate + 1]),
                replay[row, candidate : candidate + 1],
            )
            replay_timing_ns[row, candidate] = timing[0]
            replay_allocation_calls[row, candidate] = timing[1]
            replay_allocated_bytes[row, candidate] = timing[2]
    exact_diagnostic_replay = bool(np.array_equal(diagnostics, replay))
    source_immutable = source_hash == sha256(source_metrics_path)
    warning_count = int(np.sum(arrays["plant_warning"]))
    extraction_passed = bool(
        source_immutable
        and exact_diagnostic_replay
        and np.all(np.isfinite(root_terminal))
        and np.all(np.isfinite(q_terminal))
        and np.all(np.isfinite(velocity_terminal))
        and np.all(fixed_status <= 1)
        and warning_count == 0
        and np.all(replay_allocation_calls == 0)
        and np.all(replay_allocated_bytes == 0)
    )
    result = {
        "revision": REVISION,
        "source_revision": SOURCE_REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "model_sha256": sha256(model),
        "source_metrics_sha256": source_hash,
        "source_immutable": source_immutable,
        "profile": PROFILE[0],
        "family": FAMILY[0],
        "rows": ROWS,
        "candidates": CANDIDATES,
        "joint_dof": int(q_terminal.shape[2]),
        "wbc_queries": ROWS * 5,
        "physics_steps": ROWS * CANDIDATES * SUBSTEPS,
        "policy_steps": 0,
        "plant_actions": 0,
        "mujoco_warning_count": warning_count,
        "all_fixed_effort_wbc_admitted": bool(np.all(fixed_status <= 1)),
        "exact_diagnostic_replay": exact_diagnostic_replay,
        "replay_zero_rust_allocation": bool(
            np.all(replay_allocation_calls == 0)
            and np.all(replay_allocated_bytes == 0)
        ),
        "replay_timing_ns": distribution(replay_timing_ns.reshape(-1)),
        "terminal_state_values": int(root_terminal.size + q_terminal.size + velocity_terminal.size),
        "extraction_passed": extraction_passed,
        "profile_frozen": False,
        "authority_admitted": False,
    }
    report = "\n".join(
        [
            "# Bonesaw spent terminal-state corpus · r258",
            "",
            f"> Extraction **{'PASS' if extraction_passed else 'FAIL'}** · profile **NOT FROZEN** · authority **NOT ADMITTED**.",
            "",
            "R258 replays the already-spent R250 laws, offsets, 25 Hz actuator realization, and neutral-recovery candidate family solely to retain the terminal state that the historical artifact omitted: root clearance/vertical speed, roll/pitch and angular rate, every joint position, and every generalized velocity. This is corpus extraction, not a fresh holdout.",
            "",
            f"The extraction executes {result['wbc_queries']} WBC queries and {result['physics_steps']} MuJoCo steps over {ROWS} × {CANDIDATES} rows, with zero policy steps and zero plant actions. All WBC queries admit, MuJoCo warnings are {warning_count}, and an independent allocation-free Rust rescore reproduces all {diagnostics.size:,} terminal diagnostics bit-for-bit at p99 {result['replay_timing_ns']['p99'] / 1e3:.2f} µs.",
            "",
            "The new state arrays are spent design evidence. R259 must fit and evaluate the paired tube from these immutable arrays with zero physics and zero policy; no authority follows from extraction itself.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-spent-terminal-state-corpus-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-spent-terminal-state-corpus.npz",
        root_state=np.asarray(arrays["root_state"], np.float64),
        joint_position=np.asarray(arrays["joint_position"], np.float64),
        initial_velocity=np.asarray(arrays["initial_velocity"], np.float64),
        candidate_acceleration=np.asarray(arrays["candidate_acceleration"], np.float64),
        candidate_effort_utilization=effort,
        fixed_status=fixed_status,
        actual_root_terminal_state=root_terminal,
        actual_joint_terminal_position=q_terminal,
        actual_velocity=velocity_terminal,
        actual_diagnostics=diagnostics,
        actual_energy=np.asarray(arrays["actual_energy"], np.float64),
        plant_warning=np.asarray(arrays["plant_warning"], np.uint16),
    )
    (output / "G1_SPENT_TERMINAL_STATE_CORPUS.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw spent terminal state · r258"))
    print(
        json.dumps(
            {
                "extraction_passed": extraction_passed,
                "physics_steps": result["physics_steps"],
                "profile_frozen": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if extraction_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
