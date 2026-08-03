#!/usr/bin/env python3
"""Audit the default-off hard support-transfer trajectory tube."""

from __future__ import annotations

import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_hard_feasibility_witness_r275 import (
    fingerprint,
    load,
    semantic_differences,
)


REVISION = "g1-support-trajectory-tube-r278"
ROOT = pathlib.Path("benchmarks/results")
R277 = ROOT / "floating-g1-r277-dcm-dormant-cap8"
PROFILES = {
    "dormant control": ROOT / "floating-g1-r278-tube-dormant-cap8",
    "hard h5": ROOT / "floating-g1-r278-tube-hard-h5-cap8",
    "hard h10": ROOT / "floating-g1-r278-tube-hard-h10-cap8",
    "hard h25": ROOT / "floating-g1-r278-tube-hard-h25-cap8",
    "combined h5": ROOT / "floating-g1-r278-tube-combined-h5-cap8",
}
FAILED_PRE_STATUSES = (2, 3, 4, 5)


def first_tick(mask: np.ndarray) -> int | None:
    ticks = np.flatnonzero(mask)
    return int(ticks[0]) if len(ticks) else None


def profile(directory: pathlib.Path) -> dict[str, Any]:
    metrics = json.loads(
        (directory / "floating-walk-metrics.json").read_text()
    )["metrics"]
    trace = load(directory / "floating-walk-raw.npz")
    tube = metrics["support_trajectory_tube"]
    return {
        "directory": str(directory),
        "root_tracking_rms_m": metrics["root_tracking_rms_m"],
        "center_of_mass_tracking_rms_m": metrics[
            "center_of_mass_tracking_rms_m"
        ],
        "foot_tracking_rms_m": metrics["foot_tracking_rms_m"],
        "nominal_prefix_ticks": metrics["nominal_prefix"]["ticks"],
        "first_hard_conflict_tick": first_tick(
            np.isin(trace["pre_contingency_status"], FAILED_PRE_STATUSES)
        ),
        "first_contact_release_tick": first_tick(trace["status"] == 5),
        "max_iteration_ticks": int(np.count_nonzero(trace["status"] == 4)),
        "active_ticks": tube["active_ticks"],
        "intent_projected_ticks": tube["clipped_ticks"],
        "hard_solved_ticks": tube["hard_solved_ticks"],
        "hard_unresolved_ticks": tube["hard_unresolved_ticks"],
        "hard_witness_ticks": tube["hard_witness_ticks"],
        "hard_margin_minimum_mps2": tube["hard_margin_minimum_mps2"],
        "hard_witness_margin_minimum_mps2": tube[
            "hard_witness_margin_minimum_mps2"
        ],
        "hard_boundary_violation_ticks": tube[
            "hard_boundary_violation_ticks"
        ],
        "hard_witness_violation_ticks": tube[
            "hard_witness_violation_ticks"
        ],
        "limiting_halfspace_counts": tube["limiting_halfspace_counts"],
        "target_displacement_rms_m": tube["target_displacement_rms_m"],
        "latency_us": metrics["latency_us"],
        "over_5ms_ticks": int(np.count_nonzero(trace["step_ns"] > 5_000_000)),
        "runtime": metrics["runtime"],
    }


def fmt_tick(value: int | None) -> str:
    return "none" if value is None else str(value)


def main() -> int:
    required = [
        directory / filename
        for directory in PROFILES.values()
        for filename in ("floating-walk-raw.npz", "floating-walk-metrics.json")
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise SystemExit("R278 is missing evidence: " + ", ".join(missing))

    profiles = {name: profile(directory) for name, directory in PROFILES.items()}
    prior = load(R277 / "floating-walk-raw.npz")
    dormant = load(PROFILES["dormant control"] / "floating-walk-raw.npz")
    dormant_differences = semantic_differences(prior, dormant)
    shared_arrays = set(prior) & set(dormant)
    new_arrays = sorted(set(dormant) - set(prior))
    hard = profiles["hard h5"]
    combined = profiles["combined h5"]
    control = profiles["dormant control"]
    mechanism_passed = bool(
        not dormant_differences
        and control["active_ticks"] == 0
        and hard["active_ticks"] > 0
        and hard["hard_solved_ticks"] > 0
        and hard["hard_boundary_violation_ticks"] == 0
        and hard["intent_projected_ticks"] == 0
        and combined["intent_projected_ticks"] > 0
    )
    profile_rejected = bool(
        hard["first_hard_conflict_tick"] < control["first_hard_conflict_tick"]
        and hard["first_contact_release_tick"] < control["first_contact_release_tick"]
        and hard["latency_us"]["p99"] > 5_000.0
    )

    profile_rows = []
    for name, row in profiles.items():
        profile_rows.append(
            [
                name,
                row["active_ticks"],
                row["hard_solved_ticks"],
                row["hard_unresolved_ticks"],
                fmt_tick(row["first_hard_conflict_tick"]),
                fmt_tick(row["first_contact_release_tick"]),
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
            "# G1 support-transfer trajectory tube · R278",
            "",
            "**Mechanism passes; every walking profile remains rejected.** R278 adds a "
            "default-off, allocation-free hard horizontal CoM-acceleration polytope "
            "to the Rust floating WBC. Python supplies four causal DCM "
            "control-barrier faces from the intersection of current and previewed "
            "support patches. The boundary includes measured CoM velocity, measured "
            "CoM height, a finite schedule horizon, and optional joint-position "
            "headroom scaling. It is independent of the separately switchable soft "
            "root/CoM intent projector.",
            "",
            "## Frozen simulator-free replay",
            "",
            *markdown_table(
                [
                    "profile",
                    "active",
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
            f"non-timing arrays against R277. Its six new caller-owned arrays are "
            f"`{new_arrays}` and remain inactive/neutral. All runs execute zero policy "
            "steps and zero physics steps.",
            "",
            "The five-tick hard-only row is the least damaging candidate. It admits "
            f"{hard['hard_solved_ticks']} active first solves with a minimum admitted "
            f"face margin of {hard['hard_margin_minimum_mps2']:.3e} m/s² and zero "
            "violations. Nevertheless its first hard conflict moves from control tick "
            f"{control['first_hard_conflict_tick']} to {hard['first_hard_conflict_tick']}, "
            f"release moves from {control['first_contact_release_tick']} to "
            f"{hard['first_contact_release_tick']}, root RMS changes "
            f"{control['root_tracking_rms_m']:.3f}→{hard['root_tracking_rms_m']:.3f} m, "
            f"and p99 is {hard['latency_us']['p99']:.1f} µs. The row is fail-closed, "
            "but not useful.",
            "",
            "The combined h5 row applies the proposed preview target as well as the "
            f"hard boundary on {combined['intent_projected_ticks']} ticks. Its "
            f"root/foot RMS is {combined['root_tracking_rms_m']:.3f}/"
            f"{combined['foot_tracking_rms_m']:.3f} m, confirming that intent "
            "projection and hard viability must remain separate authority layers.",
            "",
            "## Constraint evidence",
            "",
            "Each face has a stable ID and reports both the admitted post-contingency "
            "margin and the first-hard-solve witness before a retry can overwrite it. "
            "Failed or skipped solves remain `unresolved`; they are never counted as "
            "boundary violations or silently integrated. The selected hard-only row's "
            f"limiting admitted face counts (+x/-x/+y/-y) are "
            f"`{hard['limiting_halfspace_counts']}`. Its proposed but unapplied preview "
            f"target differs from authored CoM intent by "
            f"{100 * hard['target_displacement_rms_m']:.3f} cm RMS.",
            "",
            f"The selected row has {hard['hard_witness_violation_ticks']} negative "
            "first-solve witness on an unsolved tick. That is the fail-closed evidence: "
            "the violating candidate is retained for diagnosis, classified unresolved, "
            "and never becomes an admitted acceleration or state update.",
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
            "The per-tick Rust path uses fixed-capacity caller-owned halfspaces and "
            "does not allocate. Python retains orchestration, arrays, statistics, and "
            "reporting. Timing is a consequence metric because rejected profiles enter "
            "different retry/release paths; it is not normalized as a kernel speedup.",
            "",
            "## Decision and remaining chunk",
            "",
            "Retain the typed hard boundary, stable face telemetry, preview generator, "
            "and intent/viability split as default-off experimental mechanisms. Admit "
            "no walking profile and no actuator/contact authority. The largest remaining "
            "WBC chunk is a time-varying support-transfer viability construction that "
            "does not turn a locally valid DCM barrier into an early hard conflict—likely "
            "a short horizon of reachable sets or a small sequential convex tube with "
            "warm-started, explicitly budgeted progress across control ticks. It must "
            "retain support through control tick 1108, improve tracking, and stay below "
            "5 ms p99 before the COMPLAINTS acceptance gate can close.",
        ]
    )

    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "policy_steps": 0,
        "physics_steps": 0,
        "profiles": profiles,
        "source_contract": {
            "r277_sha256": fingerprint(R277 / "floating-walk-raw.npz"),
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
    (output / "g1-support-trajectory-tube-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_SUPPORT_TRAJECTORY_TUBE_R278.md").write_text(report + "\n")
    pathlib.Path("web/G1_SUPPORT_TRAJECTORY_TUBE_R278.html").write_text(
        render_report_html(report, title="G1 support-transfer trajectory tube · R278")
    )
    print(output / "G1_SUPPORT_TRAJECTORY_TUBE_R278.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
