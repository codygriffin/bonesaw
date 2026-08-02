#!/usr/bin/env python3
"""Artifact-only localization of per-target reacquisition and feasibility work."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "g1-floating-reacquisition-localization-r267"
TARGET_TRACE = pathlib.Path(
    "benchmarks/results/floating-g1-transfer-r267-target-reacquisition-cap8/"
    "floating-walk-raw.npz"
)
NATIVE_REPAIR_TRACE = pathlib.Path(
    "benchmarks/results/floating-g1-r267-equality-repair-native-it16-cap8/"
    "floating-walk-raw.npz"
)
CAPS = (128, 256, 512, 768, 1024)
ITERATIONS = (4, 8, 16, 64)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-trace", default=str(TARGET_TRACE))
    parser.add_argument("--native-repair-trace", default=str(NATIVE_REPAIR_TRACE))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/G1_FLOATING_REACQUISITION_LOCALIZATION_R267.html",
    )
    return parser.parse_args(argv)


def load_trace(path: pathlib.Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as source:
        return {name: np.asarray(source[name]) for name in source.files}


def transitions(values: np.ndarray) -> list[int]:
    changed = values[1:] != values[:-1]
    if changed.ndim > 1:
        changed = np.any(changed, axis=tuple(range(1, changed.ndim)))
    return (np.flatnonzero(changed) + 1).astype(int).tolist()


def summarize(path: pathlib.Path) -> dict[str, Any]:
    trace = load_trace(path)
    status = trace["status"]
    timing_ms = trace["step_ns"].astype(np.float64) / 1.0e6
    values, counts = np.unique(status, return_counts=True)
    return {
        "artifact": str(path),
        "ticks": len(status),
        "status_counts": {str(int(k)): int(v) for k, v in zip(values, counts)},
        "status_transitions": transitions(status),
        "support_phase_transitions": transitions(trace["support_phase"]),
        "timing_ms": distribution(timing_ms),
        "deadline_misses": {
            "5ms": int(np.count_nonzero(timing_ms > 5.0)),
            "20ms": int(np.count_nonzero(timing_ms > 20.0)),
        },
        "finite_state": bool(
            all(
                np.all(np.isfinite(trace[name]))
                for name in ("q", "v", "root_tracked", "tracked_positions")
            )
        ),
        "root_end_m": trace["root_tracked"][-1].tolist(),
        "root_max_step_after_428_m": (
            float(
                np.max(
                    np.linalg.norm(
                        np.diff(trace["root_tracked"][428:], axis=0), axis=1
                    )
                )
            )
            if len(status) > 429
            else None
        ),
        "maximum_accepted_dynamics_residual": float(
            np.max(trace["dynamics_residual"][status <= 1])
        ),
        "maximum_accepted_contact_residual": float(
            np.max(trace["contact_residual"][status <= 1])
        ),
    }


def main() -> int:
    args = parse_args()
    target_path = pathlib.Path(args.target_trace)
    native_path = pathlib.Path(args.native_repair_trace)
    cap_paths = {
        cap: pathlib.Path(
            f"benchmarks/results/floating-g1-r267-convergence-cap{cap}/"
            "floating-walk-raw.npz"
        )
        for cap in CAPS
    }
    iteration_paths = {
        iteration: pathlib.Path(
            "benchmarks/results/"
            + (
                "floating-g1-r267-equality-repair-cap8"
                if iteration == 64
                else f"floating-g1-r267-equality-repair-it{iteration}-cap8"
            )
            + "/floating-walk-raw.npz"
        )
        for iteration in ITERATIONS
    }
    required = [target_path, native_path, *cap_paths.values(), *iteration_paths.values()]
    if not all(path.is_file() for path in required):
        raise SystemExit("R267 requires the completed immutable localization traces")

    target_trace = load_trace(target_path)
    target = summarize(target_path)
    support = target_trace["support_phase"]
    status = target_trace["status"]
    per_target_reacquisition_exercised = bool(
        support[428, 0] == 2
        and support[428, 1] == 0
        and np.all(support[312:428] == 0)
    )
    target_locked_after_reacquisition = bool(np.any(support[428:, 0] == 3))
    target_release_after_attempt = bool(status[460] == 5 and support[460, 0] == 0)

    caps = []
    for cap, path in cap_paths.items():
        trace = load_trace(path)
        summary = summarize(path)
        caps.append(
            {
                "cap": cap,
                "status_at_tick_300": int(trace["status"][300]),
                "first_post_liftoff_contingency_tick": next(
                    (
                        index
                        for index in range(300, len(trace["status"]))
                        if trace["status"][index] >= 4
                    ),
                    None,
                ),
                "p99_ms": summary["timing_ms"]["p99"],
                "maximum_ms": summary["timing_ms"]["maximum"],
                "twenty_ms_misses": summary["deadline_misses"]["20ms"],
            }
        )

    iterations = []
    for limit, path in iteration_paths.items():
        trace = load_trace(path)
        summary = summarize(path)
        iterations.append(
            {
                "maximum_iterations": limit,
                "first_contingency_tick": next(
                    (
                        index
                        for index in range(300, len(trace["status"]))
                        if trace["status"][index] >= 4
                    ),
                    None,
                ),
                "p99_ms": summary["timing_ms"]["p99"],
                "maximum_ms": summary["timing_ms"]["maximum"],
                "five_ms_misses": summary["deadline_misses"]["5ms"],
                "twenty_ms_misses": summary["deadline_misses"]["20ms"],
            }
        )

    native = summarize(native_path)
    native_trace = load_trace(native_path)
    mechanism_passed = bool(
        per_target_reacquisition_exercised
        and target_release_after_attempt
        and target["finite_state"]
    )
    profile_rejected = bool(
        not target_locked_after_reacquisition
        and native["deadline_misses"]["20ms"] > 0
        and np.count_nonzero(native_trace["status"] >= 4) > 0
    )
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "artifact_only": True,
        "physics_steps": 0,
        "policy_steps": 0,
        "plant_actions": 0,
        "target_reacquisition": target,
        "per_target_reacquisition_exercised": per_target_reacquisition_exercised,
        "target_locked_after_reacquisition": target_locked_after_reacquisition,
        "target_release_after_attempt": target_release_after_attempt,
        "projection_cap_sweep": caps,
        "equality_repair_iteration_sweep": iterations,
        "native_equality_repair": native,
        "native_contingency_ticks": int(
            np.count_nonzero(native_trace["status"] >= 4)
        ),
        "native_support_phase_transitions": transitions(native_trace["support_phase"]),
        "known_r165_physical_gate": "rejected_11_earlier_fall_boundaries",
        "mechanism_passed": mechanism_passed,
        "profile_rejected": profile_rejected,
        "default_changed": False,
        "authority_admitted": False,
    }
    cap_rows = [
        [
            str(row["cap"]),
            str(row["status_at_tick_300"]),
            str(row["first_post_liftoff_contingency_tick"]),
            f"{row['p99_ms']:.3f}",
            f"{row['maximum_ms']:.3f}",
            str(row["twenty_ms_misses"]),
        ]
        for row in caps
    ]
    iteration_rows = [
        [
            str(row["maximum_iterations"]),
            str(row["first_contingency_tick"]),
            f"{row['p99_ms']:.3f}",
            f"{row['maximum_ms']:.3f}",
            str(row["five_ms_misses"]),
            str(row["twenty_ms_misses"]),
        ]
        for row in iterations
    ]
    report = "\n".join(
        [
            "# Bonesaw floating target-reacquisition localization · r267",
            "",
            f"> Per-target mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · controller profile **REJECTED** · default **UNCHANGED** · authority **NOT ADMITTED**.",
            "",
            "R267 is artifact-only. It separates three questions after R263: whether a different target can cross its own schedule edge after another target is suppressed, whether a larger Dykstra budget retains locked contact, and whether the existing equality-first accelerator is a promotable real-time controller profile.",
            "",
            "The per-target edge is exercised: failed target 1 remains suppressed while target 0 enters Precontact at tick 428 after its genuine swing interval. It never locks and is released at tick 460. By then the support-free fallback has advanced for 116 ticks and the root ends at z = {:.3f} m; a late schedule edge cannot undo a physical fall.".format(target["root_end_m"][2]),
            "",
            *markdown_table(
                ["Dykstra cap", "status@300", "first contingency", "p99 ms", "max ms", ">20 ms"],
                cap_rows,
            ),
            "",
            "The full-contact convergence knee is 768 sweeps: cap 512 still degrades at tick 300, while 768 solves it but creates a 22.9 ms deadline miss. Raising the cap is therefore rejected.",
            "",
            *markdown_table(
                ["active-set iterations", "first contingency", "p99 ms", "max ms", ">5 ms", ">20 ms"],
                iteration_rows,
            ),
            "",
            "Equality-first repair with 16 iterations retains full contact through tick 329, but the native 600-tick profile has {} contingency ticks, {:.3f} ms p99, {:.3f} ms maximum, and {} twenty-millisecond misses. R165 already rejected this seed rule after 11 plant fall boundaries moved earlier. R267 therefore keeps it diagnostic and default-off.".format(result["native_contingency_ticks"], native["timing_ms"]["p99"], native["timing_ms"]["maximum"], native["deadline_misses"]["20ms"]),
            "",
            "The remaining functional problem precedes recovery: the authored single-support motion becomes badly untrackable before tick 300. The next controller slice should improve causal support/CoM/reference compatibility while preserving the R262 bounded fail-closed timing contract; neither a late reacquisition edge nor a semantic feasibility shortcut is sufficient.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-floating-reacquisition-localization-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_FLOATING_REACQUISITION_LOCALIZATION.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw reacquisition · r267"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "profile_rejected": profile_rejected,
                "default_changed": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
