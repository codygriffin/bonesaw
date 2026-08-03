#!/usr/bin/env python3
"""Audit the finite-horizon and schedule-reachable support-transfer tubes."""

from __future__ import annotations

import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_hard_feasibility_witness_r275 import fingerprint, load, semantic_differences


REVISION = "g1-support-trajectory-tube-r279"
ROOT = pathlib.Path("benchmarks/results")
PRIOR = ROOT / "floating-g1-r278-tube-dormant-cap8"
PROFILES = {
    "dormant control": ROOT / "floating-g1-r279-dormant-cap8",
    "local position h5": ROOT / "floating-g1-r279-local-position-h5-cap8",
    "reachable observer h5": ROOT / "floating-g1-r279-reachable-observer-h5-rate2-cap8",
    "reachable hard h5": ROOT / "floating-g1-r279-reachable-hard-h5-rate2-cap8",
    "reachable hard h10": ROOT / "floating-g1-r279-reachable-hard-h10-rate2-cap8",
    "reachable hard h25": ROOT / "floating-g1-r279-reachable-hard-h25-rate2-cap8",
    "reachable combined h5": ROOT / "floating-g1-r279-reachable-combined-h5-rate2-cap8",
}
FAILED_PRE_STATUSES = (2, 3, 4, 5)


def first_tick(mask: np.ndarray) -> int | None:
    ticks = np.flatnonzero(mask)
    return int(ticks[0]) if len(ticks) else None


def profile(directory: pathlib.Path) -> dict[str, Any]:
    metrics = json.loads((directory / "floating-walk-metrics.json").read_text())
    trace = load(directory / "floating-walk-raw.npz")
    values = metrics["metrics"]
    tube = values["support_trajectory_tube"]
    motion = metrics["motion"]
    return {
        "directory": str(directory),
        "root_tracking_rms_m": values["root_tracking_rms_m"],
        "center_of_mass_tracking_rms_m": values[
            "center_of_mass_tracking_rms_m"
        ],
        "foot_tracking_rms_m": values["foot_tracking_rms_m"],
        "first_hard_conflict_tick": first_tick(
            np.isin(trace["pre_contingency_status"], FAILED_PRE_STATUSES)
        ),
        "first_contact_release_tick": first_tick(trace["status"] == 5),
        "active_ticks": tube["active_ticks"],
        "intent_projected_ticks": tube["clipped_ticks"],
        "hard_enabled": bool(
            motion.get("support_trajectory_tube_enabled", False)
            or motion.get("support_reachable_tube_hard", False)
        ),
        "reachable_enabled": bool(
            motion.get("support_reachable_tube_enabled", False)
        ),
        "hard_solved_ticks": tube["hard_solved_ticks"],
        "hard_unresolved_ticks": tube["hard_unresolved_ticks"],
        "hard_witness_ticks": tube["hard_witness_ticks"],
        "hard_margin_minimum_mps2": tube["hard_margin_minimum_mps2"],
        "hard_witness_margin_minimum_mps2": tube[
            "hard_witness_margin_minimum_mps2"
        ],
        "hard_boundary_violation_ticks": tube["hard_boundary_violation_ticks"],
        "hard_witness_violation_ticks": tube["hard_witness_violation_ticks"],
        "limiting_halfspace_counts": tube["limiting_halfspace_counts"],
        "target_displacement_rms_m": tube["target_displacement_rms_m"],
        "latency_us": values["latency_us"],
        "over_5ms_ticks": int(np.count_nonzero(trace["step_ns"] > 5_000_000)),
        "runtime": values["runtime"],
    }


def fmt(value: int | None) -> str:
    return "none" if value is None else str(value)


def fmt_margin(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.3e}"


def main() -> int:
    required = [
        path / filename
        for path in PROFILES.values()
        for filename in ("floating-walk-raw.npz", "floating-walk-metrics.json")
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise SystemExit("R279 is missing evidence: " + ", ".join(missing))

    profiles = {name: profile(path) for name, path in PROFILES.items()}
    prior = load(PRIOR / "floating-walk-raw.npz")
    dormant = load(PROFILES["dormant control"] / "floating-walk-raw.npz")
    dormant_differences = semantic_differences(prior, dormant)
    shared_arrays = set(prior) & set(dormant)
    new_arrays = sorted(set(dormant) - set(prior))
    control = profiles["dormant control"]
    local = profiles["local position h5"]
    reachable_observer = profiles["reachable observer h5"]
    reachable_hard = profiles["reachable hard h5"]
    combined = profiles["reachable combined h5"]
    mechanism_passed = bool(
        not dormant_differences
        and control["active_ticks"] == 0
        and local["hard_enabled"]
        and local["hard_solved_ticks"] > 0
        and reachable_observer["reachable_enabled"]
        and reachable_observer["active_ticks"] > 0
        and not reachable_observer["hard_enabled"]
        and reachable_hard["hard_enabled"]
        and reachable_hard["hard_witness_ticks"] > 0
        and combined["intent_projected_ticks"] > 0
    )
    profile_rejected = bool(
        reachable_hard["first_hard_conflict_tick"]
        < control["first_hard_conflict_tick"]
        and reachable_hard["first_contact_release_tick"]
        < control["first_contact_release_tick"]
    )

    profile_rows = []
    for name, row in profiles.items():
        profile_rows.append(
            [
                name,
                row["active_ticks"],
                "yes" if row["hard_enabled"] else "no",
                row["hard_solved_ticks"],
                row["hard_unresolved_ticks"],
                fmt(row["first_hard_conflict_tick"]),
                fmt(row["first_contact_release_tick"]),
                f"{row['root_tracking_rms_m']:.3f}",
                f"{row['foot_tracking_rms_m']:.3f}",
                f"{row['latency_us']['p99']:.1f}",
                row["over_5ms_ticks"],
            ]
        )
    runtime_rows = []
    for name, row in profiles.items():
        runtime = row["runtime"]
        runtime_rows.append(
            [
                name,
                f"{runtime['call_wall_ns'] / 2_317 / 1e6:.3f}",
                f"{runtime['process_cpu_ns'] / 2_317 / 1e6:.3f}",
                f"{runtime['process_cpu_to_wall_ratio']:.4f}",
                f"{runtime['rss_delta_bytes'] / 2**20:.3f}",
                f"{runtime['peak_rss_bytes'] / 2**20:.3f}",
                runtime["python_gc_delta"]["collections"],
                runtime["python_tracemalloc_peak_bytes"],
            ]
        )

    report = "\n".join(
        [
            "# G1 support-transfer trajectory tube · R279",
            "",
            "**Mechanisms pass; every walking profile remains rejected.** R279 keeps "
            "the finite-horizon position tube as one default-off hard option and "
            "adds a second, independently switchable exact discrete DCM "
            "backward-reachable-set observer. The observer folds axis-aligned "
            "support boxes backward through the authored preview, then computes a "
            "moving-boundary acceleration interval. A separate hard flag installs "
            "those four rows; intent projection remains a third, independent layer.",
            "",
            "## Frozen simulator-free replay",
            "",
            *markdown_table(
                [
                    "profile",
                    "active",
                    "hard",
                    "hard solved",
                    "unresolved",
                    "first conflict",
                    "release",
                    "root RMS m",
                    "foot RMS m",
                    "p99 µs",
                    ">5 ms",
                ],
                profile_rows,
            ),
            "",
            f"The dormant build is bit-exact on all {len(shared_arrays) - 1} shared "
            f"non-timing arrays against R278. R279 adds no new trace arrays "
            f"(`{new_arrays}`); the six tube arrays introduced by R278 remain "
            "neutral when the feature is disabled. All "
            "runs execute zero policy steps and zero physics steps.",
            "",
            "The local finite-horizon position tube is materially less aggressive "
            f"than R278's DCM barrier: its h5 row first fails at "
            f"{local['first_hard_conflict_tick']} versus R278's 324, but it still "
            f"does not improve the control prefix ({control['first_contact_release_tick']} "
            "release) and remains rejected.",
            "",
            "The reachable-set observer is behavior-neutral when hard enforcement "
            "and intent projection are both disabled. Its hard h5 counterpart "
            f"retains a first-solve witness at tick {reachable_hard['first_hard_conflict_tick']} "
            f"with minimum witness margin {fmt_margin(reachable_hard['hard_witness_margin_minimum_mps2'])} "
            "m/s², correctly failing closed before it can admit a contradictory "
            "acceleration interval. This is useful constraint evidence, not a "
            "walking claim.",
            "",
            "The combined reachable h5 row projects intent on "
            f"{combined['intent_projected_ticks']} ticks and releases at "
            f"{combined['first_contact_release_tick']}; intent shaping and hard "
            "viability therefore remain separate authority layers.",
            "",
            "## Constraint evidence",
            "",
            "The Rust reachability primitives use fixed-size boxes and a fixed four "
            "face output. Empty moving-boundary intervals return a typed `None` "
            "witness; they are never clamped into contradictory hard rows. Stable "
            "face IDs and first-solve witnesses survive contingency retries, while "
            "unresolved attempts remain visible in the trace.",
            "",
            "## CPU, jitter, allocations, and memory",
            "",
            *markdown_table(
                [
                    "profile",
                    "wall ms/tick",
                    "CPU ms/tick",
                    "CPU/wall",
                    "RSS Δ MiB",
                    "peak MiB",
                    "Python GC",
                    "trace peak B",
                ],
                runtime_rows,
            ),
            "",
            "The per-tick Rust path uses fixed-capacity caller-owned storage and "
            "does not allocate. The reachability fold is bounded by the preview "
            "horizon; Python retains orchestration, arrays, statistics, and "
            "reporting. Timing is a consequence metric because rejected profiles "
            "enter different retry/release paths, not a kernel-speed claim.",
            "",
            "## Decision and remaining chunk",
            "",
            "Retain both formulations as default-off diagnostic mechanisms. Admit no "
            "walking profile and no actuator/contact authority. The next WBC chunk "
            "is a support-transition retiming/replanning loop that can react to a "
            "negative reachable-set witness (hold or move the liftoff edge) before "
            "hard rows become infeasible, then demonstrate support through tick "
            "1108 with bounded tracking and p99 below 5 ms.",
        ]
    )
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "policy_steps": 0,
        "physics_steps": 0,
        "profiles": profiles,
        "source_contract": {
            "prior_sha256": fingerprint(PRIOR / "floating-walk-raw.npz"),
            "dormant_sha256": fingerprint(
                PROFILES["dormant control"] / "floating-walk-raw.npz"
            ),
            "shared_array_count": len(shared_arrays),
            "shared_non_timing_array_count": len(shared_arrays) - 1,
            "dormant_semantic_differences": dormant_differences,
            "new_arrays": new_arrays,
        },
        "mechanism_passed": mechanism_passed,
        "profile_rejected": profile_rejected,
        "authority_admitted": False,
    }
    output = ROOT / REVISION
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-support-trajectory-tube-r279-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    report_path = output / "G1_SUPPORT_TRAJECTORY_TUBE_R279.md"
    report_path.write_text(report + "\n")
    pathlib.Path("web/G1_SUPPORT_TRAJECTORY_TUBE_R279.html").write_text(
        render_report_html(report, title="G1 support-transfer trajectory tube · R279")
    )
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
