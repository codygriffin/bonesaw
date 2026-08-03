#!/usr/bin/env python3
"""Replay synthetic 250 Hz contact chunks through the live Upkie ingress.

This is an evaluation harness, not a contact estimator or a controller.  The
worker still owns normal MuJoCo integration and calls the existing Rust
adapter; the harness replaces only the named contact extraction boundary with
a deterministic synthetic source buffer. Five authored source frames are
chunked for every 50 Hz WBC tick so downsampling semantics, debounce, hard-row
admission, and lifecycle fail-closed behavior can be checked without a server
or policy rollout. This is not a recorded MuJoCo contact trace.
"""

from __future__ import annotations

import argparse
import html
import json
import pathlib
import sys
from datetime import datetime, timezone
from typing import Any
from unittest.mock import patch

import numpy as np


REVISION = "upkie-synthetic-contact-ingress-r298"
PHYSICS_HZ = 250
CONTROL_HZ = 50
PHYSICS_SUBSTEPS = PHYSICS_HZ // CONTROL_HZ
PHYSICS_DT = 1.0 / PHYSICS_HZ
CONTROL_DT = 1.0 / CONTROL_HZ

# Three activation samples and two release samples are the Rust adapter's
# default contact-observation contract.  The fourth phase intentionally
# includes both one-contact orientations and a zero-contact frame.
PHASES: tuple[tuple[str, tuple[int, int], int], ...] = (
    ("double", (1, 1), 3),
    ("left_only", (1, 0), 2),
    ("right_only", (0, 1), 3),
    ("airborne", (0, 0), 2),
)


def control_masks() -> list[tuple[int, int]]:
    """Return the deterministic 50 Hz mask schedule."""

    return [mask for _, mask, count in PHASES for _ in range(count)]


class ReplayContactSource:
    """Synthetic 250 Hz buffer sampled at the live worker's contact boundary."""

    def __init__(self, masks_50hz: list[tuple[int, int]]) -> None:
        self.masks_250hz: list[tuple[int, int]] = []
        previous = masks_50hz[0]
        for mask in masks_50hz:
            # Place each edge on the newest 250 Hz frame in its control
            # window.  This exercises the sampling rule instead of merely
            # repeating a 50 Hz value five times.
            self.masks_250hz.extend(
                [previous] * (PHYSICS_SUBSTEPS - 1) + [mask]
            )
            previous = mask
        self.cursor = 0
        self.calls = 0
        self.startup_observation: list[int] | None = None
        self.records: list[dict[str, Any]] = []

    def __call__(
        self,
        _model: Any,
        _data: Any,
        _body_sets: Any,
        active: np.ndarray,
    ) -> np.ndarray:
        """Write one synthetic 250 Hz observation into worker-owned storage."""

        # The first call is the frame-zero startup observation. It is not part
        # of a completed physics window and therefore does not advance the
        # source cursor; the following five calls fill the first live window.
        if self.calls == 0:
            selected = np.asarray(self.masks_250hz[0], dtype=np.uint8)
            self.startup_observation = selected.tolist()
            self.calls += 1
            active[...] = selected
            return active
        if self.cursor >= len(self.masks_250hz):
            raise AssertionError(
                f"contact source overrun: requested frame {self.cursor} of "
                f"{len(self.masks_250hz)} frames"
            )
        frame_index = self.cursor
        selected = np.asarray(self.masks_250hz[frame_index], dtype=np.uint8)
        active[...] = selected
        self.cursor += 1
        self.calls += 1
        if self.cursor % PHYSICS_SUBSTEPS == 0:
            start = self.cursor - PHYSICS_SUBSTEPS
            chunk = self.masks_250hz[start : self.cursor]
            self.records.append(
                {
                    "control_tick": len(self.records),
                    "physics_indices": list(range(start, self.cursor)),
                    "selected": selected.tolist(),
                    "chunk": [list(frame) for frame in chunk],
                }
            )
        return active


def _state_summary(state: dict[str, Any]) -> dict[str, Any]:
    metrics = state["metrics"]
    simulator = state["simulator"]
    return {
        "tick": int(state["tick"]),
        "time_s": float(simulator["time_s"]),
        "observed": [int(value) for value in state["wbc_observed_contact_active"]],
        "debounced": [int(value) for value in state["wbc_debounced_contact_active"]],
        "hard": [int(value) for value in state["wbc_hard_contact_active"]],
        "available": bool(metrics["wbc_observed_contact_available"]),
        "status": int(metrics["wbc_contact_observation_status"]),
        "provenance": int(metrics["wbc_contact_observation_provenance"]),
        "flags": int(metrics["wbc_contact_observation_flags"]),
        "support_count": int(metrics["wbc_support_active_count"]),
        "reset_epoch": int(state["reset_epoch"]),
        "numeric_reset": bool(state["numeric_reset"]),
        "automatic_reset_reason": state["automatic_reset_reason"],
        "automatic_reset_pending": state["automatic_reset_pending"],
    }


def run_replay(model_path: pathlib.Path) -> dict[str, Any]:
    """Run one deterministic source trace through one fresh live worker."""

    # Keep the import here so this module remains importable for report tooling
    # in environments that do not have the optional MuJoCo/PyO3 runtime.
    import upkie_live_plant_worker as worker_module

    masks_50hz = control_masks()
    source = ReplayContactSource(masks_50hz)
    with patch.object(
        worker_module.plant,
        "measured_wheel_ground_contacts_into",
        source,
    ):
        worker = worker_module.LiveUpkiePlant(model_path)
        initial_epoch = int(worker.reset_epoch)
        states: list[dict[str, Any]] = []
        for command_id in range(len(masks_50hz)):
            states.append(
                worker.step(
                    {
                        "type": "step",
                        "command_id": command_id,
                    }
                )
            )
        # A lifecycle heartbeat is deliberately sampled after the zero-contact
        # phase.  No solve runs while paused, so retained debounce state must be
        # hidden and support authority must be explicitly zero.
        paused = worker.step(
            {
                "type": "step",
                "command_id": len(masks_50hz),
                "paused": True,
            }
        )

    summaries = [_state_summary(state) for state in states]
    paused_summary = _state_summary(paused)
    return {
        "revision": REVISION,
        "hello": {
            "physics_hz": PHYSICS_HZ,
            "control_hz": CONTROL_HZ,
            "physics_substeps_per_control": PHYSICS_SUBSTEPS,
        },
        "initial_reset_epoch": initial_epoch,
        "masks_50hz": [list(mask) for mask in masks_50hz],
        "source": {
            "physics_hz": PHYSICS_HZ,
            "control_hz": CONTROL_HZ,
            "physics_substeps_per_control": PHYSICS_SUBSTEPS,
            "physics_frame_count": len(source.masks_250hz),
            "cursor": source.cursor,
            "calls": source.calls,
            "startup_observation": source.startup_observation,
            "records": source.records,
        },
        "states": summaries,
        "paused": paused_summary,
    }


def _canonical_trace(result: dict[str, Any]) -> list[tuple[Any, ...]]:
    """Drop wall-clock timing while retaining all causal authority witnesses."""

    fields = (
        "tick",
        "observed",
        "debounced",
        "hard",
        "available",
        "status",
        "provenance",
        "flags",
        "support_count",
        "reset_epoch",
        "numeric_reset",
        "automatic_reset_reason",
        "automatic_reset_pending",
    )
    return [tuple(summary[field] if isinstance(summary[field], list) else summary[field] for field in fields) for summary in result["states"]]


def evaluate(result: dict[str, Any], replay: dict[str, Any]) -> dict[str, bool]:
    states = result["states"]
    masks = result["masks_50hz"]
    records = result["source"]["records"]
    observed = [tuple(state["observed"]) for state in states]
    hard = [tuple(state["hard"]) for state in states]
    debounced = [tuple(state["debounced"]) for state in states]
    patterns = set(observed)
    # The production worker consumes the terminal mask from the completed
    # prior physics window. Frame zero is the startup observation; every later
    # WBC call therefore sees the preceding 50 Hz source mask.
    expected = [tuple(masks[0]), *(tuple(mask) for mask in masks[:-1])]
    paused = result["paused"]
    initial_epoch = result["initial_reset_epoch"]

    return {
        "rate_split_declared": result["hello"]
        == {
            "physics_hz": PHYSICS_HZ,
            "control_hz": CONTROL_HZ,
            "physics_substeps_per_control": PHYSICS_SUBSTEPS,
        },
        "all_contact_patterns": {(1, 1), (1, 0), (0, 1), (0, 0)}.issubset(patterns),
        "raw_mask_passthrough": observed == expected,
        "source_consumes_exactly_five_frames_per_tick": result["source"]["cursor"]
        == len(states) * PHYSICS_SUBSTEPS
        and all(
            record["physics_indices"]
            == list(
                range(
                    tick * PHYSICS_SUBSTEPS,
                    (tick + 1) * PHYSICS_SUBSTEPS,
                )
            )
            for tick, record in enumerate(records)
        ),
        "source_chunks_select_newest_frame": all(
            len(record["chunk"]) == PHYSICS_SUBSTEPS
            and record["selected"] == record["chunk"][-1]
            for record in records
        ),
        "source_replays_edges_inside_control_window": any(
            len({tuple(frame) for frame in record["chunk"]}) > 1
            for record in records
        ),
        "activation_debounce": debounced[:3] == [(0, 0), (0, 0), (1, 1)]
        and debounced[8] == (0, 1),
        "deactivation_debounce": debounced[3:7]
        == [(1, 1), (1, 1), (1, 0), (1, 0)],
        "hard_rows_intersect_raw": all(
            all(hard_value <= raw_value for hard_value, raw_value in zip(hard_tick, raw_tick, strict=True))
            for hard_tick, raw_tick in zip(hard, observed, strict=True)
        ),
        "hard_rows_fail_closed_on_zero_contact": hard[9] == (0, 0),
        "running_observation_is_exact": all(
            state["available"] and state["status"] == 0 and state["provenance"] == 0
            for state in states
        ),
        "pause_fails_closed_without_reset": not paused["available"]
        and paused["debounced"] == [0, 0]
        and paused["hard"] == [0, 0]
        and paused["support_count"] == 0
        and paused["reset_epoch"] == initial_epoch
        and not paused["numeric_reset"]
        and paused["automatic_reset_reason"] is None,
        "no_reset_during_trace": all(
            state["reset_epoch"] == initial_epoch
            and not state["numeric_reset"]
            and state["automatic_reset_reason"] is None
            and state["automatic_reset_pending"] is None
            for state in states
        ),
        "exact_replay": _canonical_trace(result) == _canonical_trace(replay)
        and result["source"]["records"] == replay["source"]["records"],
    }


def render_markdown(result: dict[str, Any], gates: dict[str, bool]) -> str:
    passed = all(gates.values())
    lines = [
        f"# Bonesaw Upkie synthetic contact-ingress replay · {REVISION}",
        "",
        f"> Evaluation **{'PASS' if passed else 'FAIL'}**. This report drives the existing Python MuJoCo worker and persistent Rust adapter with a deterministic synthetic contact buffer; it adds no policy, solver change, or server.",
        "",
        "## Contract",
        "",
        f"The harness replays {len(result['source']['records'])} WBC ticks at {CONTROL_HZ} Hz. Each completed tick supplies exactly {PHYSICS_SUBSTEPS} source frames at {PHYSICS_HZ} Hz; the following WBC call consumes the terminal frame from the preceding window, with frame zero as startup. Each phase edge is placed on a terminal frame to exercise the sampling boundary. The sequence covers `11`, `10`, `01`, and `00`; Rust's default three-sample activation and two-sample deactivation debounce is observed at the WBC boundary.",
        "",
        "The replacement is limited to `measured_wheel_ground_contacts_into`: the worker still performs its normal five MuJoCo integration substeps and calls the existing `RustWbcAdapter.solve` path. The synthetic source supplies one frame per 250 Hz contact sample and groups the resulting five frames for each 50 Hz boundary; it does not prove measured MuJoCo transfer or a hardware contact estimator.",
        "",
        "## Trace",
        "",
        "| tick | source mask | debounced | hard | status | provenance | flags | support | reset epoch |",
        "|---:|:---:|:---:|:---:|---:|---:|---:|---:|---:|",
    ]
    for state in result["states"]:
        lines.append(
            f"| {state['tick']} | `{' '.join(str(x) for x in state['observed'])}` | `{' '.join(str(x) for x in state['debounced'])}` | `{' '.join(str(x) for x in state['hard'])}` | {state['status']} | {state['provenance']} | {state['flags']} | {state['support_count']} | {state['reset_epoch']} |"
        )
    paused = result["paused"]
    lines.extend(
        [
            f"| {paused['tick']} (paused) | — | `{' '.join(str(x) for x in paused['debounced'])}` | `{' '.join(str(x) for x in paused['hard'])}` | {paused['status']} | {paused['provenance']} | {paused['flags']} | {paused['support_count']} | {paused['reset_epoch']} |",
            "",
            "## Gates",
            "",
            "| gate | result |",
            "|:---|:---:|",
        ]
    )
    lines.extend(f"| {name} | {'PASS' if value else 'FAIL'} |" for name, value in gates.items())
    lines.extend(
        [
            "",
            "The paused heartbeat intentionally publishes no observed-contact availability, debounced mask, hard rows, or support count. The trace must complete without a numeric/fall reset or a reset-epoch change.",
            "",
        ]
    )
    return "\n".join(lines)


def render_html(markdown: str, result: dict[str, Any], gates: dict[str, bool]) -> str:
    passed = all(gates.values())
    return "\n".join(
        [
            "<!doctype html>",
            "<meta name='viewport' content='width=device-width,initial-scale=1'>",
            f"<title>Bonesaw {html.escape(REVISION)}</title>",
            "<style>body{font:15px system-ui,sans-serif;max-width:1000px;margin:2rem auto;padding:0 1rem;background:#111827;color:#e5e7eb}pre{white-space:pre-wrap;line-height:1.45;background:#1f2937;padding:1rem;border-radius:8px}h1{color:#93c5fd}.pass{color:#86efac}.fail{color:#fca5a5}</style>",
            f"<h1 class='{ 'pass' if passed else 'fail' }'>Bonesaw synthetic contact ingress · {html.escape(REVISION)}</h1>",
            f"<pre>{html.escape(markdown)}</pre>",
        ]
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument(
        "--output", default=f"benchmarks/results/{REVISION}"
    )
    parser.add_argument(
        "--web-report", default=f"web/UPKIE_SYNTHETIC_CONTACT_INGRESS_R298.html"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    result = run_replay(model)
    replay = run_replay(model)
    gates = evaluate(result, replay)
    passed = all(gates.values())
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "passed": passed,
        "gates": gates,
        "trace": result,
    }
    report = render_markdown(result, gates)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-synthetic-contact-ingress-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_SYNTHETIC_CONTACT_INGRESS_R298.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_html(report, result, gates))
    print(json.dumps({"passed": passed, "gates": gates, "web_report": str(web_report)}, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
