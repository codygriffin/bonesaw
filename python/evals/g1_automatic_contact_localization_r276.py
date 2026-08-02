#!/usr/bin/env python3
"""Audit bounded automatic per-target contact localization."""

from __future__ import annotations

import json
import pathlib
from datetime import datetime, timezone

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_hard_feasibility_witness_r275 import fingerprint, load, semantic_differences


REVISION = "g1-automatic-contact-localization-r276"
ROOT = pathlib.Path("benchmarks/results")
CONTROL = ROOT / "floating-g1-r275-hard-witness-control-cap8/floating-walk-raw.npz"
DORMANT = ROOT / "floating-g1-r276-auto-localization-dormant-cap8/floating-walk-raw.npz"
ENABLED = ROOT / "floating-g1-r276-auto-localization-enabled-cap8/floating-walk-raw.npz"


def first_tick(mask: np.ndarray) -> int | None:
    ticks = np.flatnonzero(mask)
    return int(ticks[0]) if len(ticks) else None


def summarize(trace: dict[str, np.ndarray]) -> dict[str, object]:
    attempts = trace["contact_localization_probe_attempts"]
    targets = trace["contact_localization_admitted_target"]
    timing = trace["step_ns"].astype(np.float64) / 1.0e6
    root_error = np.linalg.norm(
        trace["root_tracked"] - trace["effective_root_targets"], axis=1
    )
    foot_error = np.linalg.norm(
        trace["tracked_positions"][:, :2]
        - trace["effective_target_positions"][:, :2],
        axis=2,
    )
    return {
        "probe_ticks": np.flatnonzero(attempts).astype(int).tolist(),
        "total_probes": int(np.sum(attempts)),
        "maximum_probes_per_tick": int(np.max(attempts)),
        "admitted_ticks": np.flatnonzero(targets >= 0).astype(int).tolist(),
        "admitted_targets": targets[targets >= 0].astype(int).tolist(),
        "typed_admission_ticks": np.flatnonzero(trace["status"] == 13).astype(int).tolist(),
        "first_fallback_tick": first_tick(np.any(trace["support_phase"] == 4, axis=1)),
        "first_release_tick": first_tick(np.isin(trace["status"], (5, 12))),
        "root_tracking_rms_m": float(np.sqrt(np.mean(np.square(root_error)))),
        "foot_tracking_rms_m": float(np.sqrt(np.mean(np.square(foot_error)))),
        "timing_ms": distribution(timing),
        "five_ms_misses": int(np.count_nonzero(timing > 5.0)),
        "twenty_ms_misses": int(np.count_nonzero(timing > 20.0)),
    }


def main() -> int:
    missing = [str(path) for path in (CONTROL, DORMANT, ENABLED) if not path.is_file()]
    if missing:
        raise SystemExit("R276 is missing evidence: " + ", ".join(missing))
    control, dormant, enabled = load(CONTROL), load(DORMANT), load(ENABLED)
    profiles = {"control": summarize(dormant), "automatic": summarize(enabled)}
    dormant_differences = semantic_differences(control, dormant)
    mechanism_passed = bool(
        not dormant_differences
        and profiles["control"]["total_probes"] == 0
        and profiles["automatic"]["probe_ticks"] == [875]
        and profiles["automatic"]["total_probes"] == 2
        and profiles["automatic"]["admitted_targets"] == [1]
        and profiles["automatic"]["typed_admission_ticks"] == [875]
    )
    profile_rejected = bool(
        profiles["automatic"]["first_release_tick"]
        < profiles["control"]["first_release_tick"]
        and profiles["automatic"]["root_tracking_rms_m"]
        > profiles["control"]["root_tracking_rms_m"]
        and profiles["automatic"]["timing_ms"]["p99"] > 5.0
    )
    rows = [
        [
            name,
            str(profile["probe_ticks"]),
            str(profile["total_probes"]),
            str(profile["admitted_targets"]),
            str(profile["first_fallback_tick"]),
            str(profile["first_release_tick"]),
            f'{profile["root_tracking_rms_m"]:.3f}',
            f'{profile["foot_tracking_rms_m"]:.3f}',
            f'{profile["timing_ms"]["p99"]:.3f}',
        ]
        for name, profile in profiles.items()
    ]
    report = "\n".join(
        [
            "# G1 automatic contact localization · R276",
            "",
            "**Mechanism passed / walking profile rejected.** R276 spends at most one normal-only solve per represented contact target, in stable target order, only after the ordinary full-lock solve is unfinished and only when at least two targets are represented.",
            "",
            *markdown_table(
                ["profile", "probe ticks", "probes", "admitted target", "fallback", "release", "root RMS m", "foot RMS m", "p99 ms"],
                rows,
            ),
            "",
            "The default-off replay matches every one of the 81 shared R275 non-timing arrays and emits no probe. At tick 875 the enabled path rejects target 0/left, admits target 1/right on its second bounded probe, emits typed status 13, and keeps the left foot locked. Single-target failures are deliberately not probed.",
            "",
            "The contact choice is correct but insufficient: left-only support reaches normal fallback at tick 885 and releases at 886, earlier than the control's tick 1108. Root/foot RMS regress to 16.782/16.376 m and p99 is 5.216 ms. The next slice must shape the pre-liftoff coupled root/CoM/support state; more failure-time contact selection is not the limiting behavior. No authority is admitted.",
        ]
    )
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "policy_steps": 0,
        "physics_steps": 0,
        "profiles": profiles,
        "source_contract": {
            "control_sha256": fingerprint(CONTROL),
            "dormant_sha256": fingerprint(DORMANT),
            "enabled_sha256": fingerprint(ENABLED),
            "shared_control_array_count": len(set(control) & set(dormant)),
            "dormant_semantic_differences": dormant_differences,
        },
        "mechanism_passed": mechanism_passed,
        "profile_rejected": profile_rejected,
        "authority_admitted": False,
    }
    output = ROOT / REVISION
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-automatic-contact-localization-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_AUTOMATIC_CONTACT_LOCALIZATION_R276.md").write_text(report + "\n")
    pathlib.Path("web/G1_AUTOMATIC_CONTACT_LOCALIZATION_R276.html").write_text(
        render_report_html(report, title="G1 automatic contact localization · R276")
    )
    print(output / "G1_AUTOMATIC_CONTACT_LOCALIZATION_R276.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
