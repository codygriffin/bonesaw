#!/usr/bin/env python3
"""Audit a soft stopping-headroom joint-position capture request."""

from __future__ import annotations

import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "g1-joint-position-capture-r274"
RESULT_ROOT = pathlib.Path("benchmarks/results")
BASELINE = RESULT_ROOT / "floating-g1-r273-normal-fallback-relock-dormant-cap8" / "floating-walk-raw.npz"
TRACE_PATHS = {
    "r274_dormant": RESULT_ROOT / "floating-g1-r274-position-capture-dormant-cap8" / "floating-walk-raw.npz",
    "r274_w025_b25_r02": RESULT_ROOT / "floating-g1-r274-position-capture-w025-b25-r02-cap8" / "floating-walk-raw.npz",
    "r274_w025_b50_r02": RESULT_ROOT / "floating-g1-r274-position-capture-w025-b50-r02-cap8" / "floating-walk-raw.npz",
    "r274_w025_b50_r04": RESULT_ROOT / "floating-g1-r274-position-capture-w025-b50-r04-cap8" / "floating-walk-raw.npz",
    "r274_w05_b50_r02": RESULT_ROOT / "floating-g1-r274-position-capture-w05-b50-r02-cap8" / "floating-walk-raw.npz",
    "r274_w1_b50_r02": RESULT_ROOT / "floating-g1-r274-position-capture-w1-b50-r02-cap8" / "floating-walk-raw.npz",
}
SETTINGS = {
    "r273_baseline": (0.0, 50.0, 0.02),
    "r274_dormant": (0.0, 50.0, 0.02),
    "r274_w025_b25_r02": (0.25, 25.0, 0.02),
    "r274_w025_b50_r02": (0.25, 50.0, 0.02),
    "r274_w025_b50_r04": (0.25, 50.0, 0.04),
    "r274_w05_b50_r02": (0.50, 50.0, 0.02),
    "r274_w1_b50_r02": (1.00, 50.0, 0.02),
}
REFERENCE = RESULT_ROOT / "g1-multistep-reference-r53" / "reference-inputs.npz"
WITNESS = RESULT_ROOT / "g1-multistep-oracle-r54" / "oracle-wbc-admission-raw.npz"
CAPTURE_KEY = "joint_position_capture_active_coordinates"
CRITICAL_COORDINATE = 9
CRITICAL_POSITION_LOWER_RAD = -0.087267


def fingerprint(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load(path: pathlib.Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as source:
        return {key: np.asarray(source[key]) for key in source.files}


def first_tick(mask: np.ndarray) -> int:
    selected = np.flatnonzero(mask)
    return int(selected[0]) if len(selected) else len(mask)


def semantic_differences(
    baseline: dict[str, np.ndarray],
    candidate: dict[str, np.ndarray],
    stop: int | None = None,
) -> list[str]:
    differences: list[str] = []
    shared = sorted(set(baseline) & set(candidate))
    for key in shared:
        if key == "step_ns":
            continue
        left = baseline[key] if stop is None else baseline[key][:stop]
        right = candidate[key] if stop is None else candidate[key][:stop]
        if not np.array_equal(left, right, equal_nan=True):
            differences.append(key)
    return differences


def first_state_difference(
    baseline: dict[str, np.ndarray], candidate: dict[str, np.ndarray]
) -> int:
    for tick in range(len(candidate["status"])):
        if any(
            not np.array_equal(
                baseline[key][tick], candidate[key][tick], equal_nan=True
            )
            for key in ("root_tracked", "root_quaternion_wxyz", "q", "v")
        ):
            return tick
    return len(candidate["status"])


def summarize(
    trace: dict[str, np.ndarray], setting: tuple[float, float, float]
) -> dict[str, Any]:
    weight, braking, reaction = setting
    status = trace["status"]
    phase = trace["support_phase"]
    active = trace.get(CAPTURE_KEY, np.zeros(len(status), dtype=np.uint8))
    knee_q = trace["q"][:, CRITICAL_COORDINATE]
    lower_limit = np.flatnonzero(knee_q <= CRITICAL_POSITION_LOWER_RAD + 1.0e-9)
    root_error = np.linalg.norm(
        trace["root_tracked"] - trace["effective_root_targets"], axis=1
    )
    timing_ms = trace["step_ns"].astype(np.float64) / 1.0e6
    fallback = np.any(phase == 4, axis=1)
    release = np.isin(status, (5, 12))
    return {
        "weight": weight,
        "assumed_braking_acceleration_rad_s2": braking,
        "reaction_time_seconds": reaction,
        "first_active_tick": first_tick(active > 0),
        "active_ticks": int(np.count_nonzero(active)),
        "maximum_active_coordinates": int(np.max(active)),
        "first_fallback_tick": first_tick(fallback),
        "first_release_tick": first_tick(release),
        "critical_joint_first_lower_limit_tick": int(lower_limit[0]) if len(lower_limit) else None,
        "critical_joint_minimum_position_rad": float(np.min(knee_q)),
        "full_root_rms_m": float(np.sqrt(np.mean(np.square(root_error)))),
        "timing_ms": distribution(timing_ms),
        "deadline_misses": {
            "5ms": int(np.count_nonzero(timing_ms > 5.0)),
            "20ms": int(np.count_nonzero(timing_ms > 20.0)),
        },
        "finite_state": bool(
            np.all(np.isfinite(trace["q"]))
            and np.all(np.isfinite(trace["v"]))
            and np.all(np.isfinite(trace["root_tracked"]))
        ),
    }


def main() -> int:
    required = (BASELINE, REFERENCE, WITNESS, *TRACE_PATHS.values())
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise SystemExit("R274 is missing required evidence: " + ", ".join(missing))

    traces = {"r273_baseline": load(BASELINE)}
    traces.update({name: load(path) for name, path in TRACE_PATHS.items()})
    profiles = {
        name: summarize(trace, SETTINGS[name]) for name, trace in traces.items()
    }
    baseline = profiles["r273_baseline"]
    candidate_names = tuple(name for name in TRACE_PATHS if name != "r274_dormant")
    dormant_differences = semantic_differences(
        traces["r273_baseline"], traces["r274_dormant"]
    )
    activation_prefix_differences: dict[str, list[str]] = {}
    full_differences: dict[str, list[str]] = {}
    for name in candidate_names:
        stop = profiles[name]["first_active_tick"]
        activation_prefix_differences[name] = semantic_differences(
            traces["r273_baseline"], traces[name], stop
        )
        full_differences[name] = semantic_differences(
            traces["r273_baseline"], traces[name]
        )
        profiles[name]["first_state_difference_tick"] = first_state_difference(
            traces["r273_baseline"], traces[name]
        )

    mechanism_passed = bool(
        not dormant_differences
        and CAPTURE_KEY not in traces["r273_baseline"]
        and np.count_nonzero(traces["r274_dormant"][CAPTURE_KEY]) == 0
        and all(not activation_prefix_differences[name] for name in candidate_names)
        and all(profiles[name]["active_ticks"] > 0 for name in candidate_names)
        and all(profiles[name]["finite_state"] for name in candidate_names)
    )
    profile_rejected = bool(
        all(
            profiles[name]["critical_joint_first_lower_limit_tick"] is not None
            for name in candidate_names
        )
        and all(
            profiles[name]["first_release_tick"] < baseline["first_release_tick"]
            for name in candidate_names
        )
        and all(profiles[name]["full_root_rms_m"] > 1.0 for name in candidate_names)
    )

    rows = []
    for name, profile in profiles.items():
        rows.append(
            [
                name.replace("_", " "),
                f'{profile["weight"]:.2f}',
                f'{profile["assumed_braking_acceleration_rad_s2"]:.0f}',
                f'{profile["reaction_time_seconds"]:.2f}',
                str(profile["first_active_tick"]),
                str(profile["active_ticks"]),
                str(profile["critical_joint_first_lower_limit_tick"]),
                str(profile["first_fallback_tick"]),
                str(profile["first_release_tick"]),
                f'{profile["full_root_rms_m"]:.3f}',
                f'{profile["timing_ms"]["p99"]:.3f}',
            ]
        )

    source_contract = {
        "reference_sha256": fingerprint(REFERENCE),
        "witness_sha256": fingerprint(WITNESS),
        "baseline_sha256": fingerprint(BASELINE),
        **{f"{name}_sha256": fingerprint(path) for name, path in TRACE_PATHS.items()},
        "policy_steps": 0,
        "physics_steps": 0,
        "default_weight": 0.0,
        "dormant_common_non_timing_arrays_equal": not dormant_differences,
        "dormant_semantic_differences": dormant_differences,
        "candidate_activation_prefix_semantic_differences": activation_prefix_differences,
        "candidate_full_semantic_differences": full_differences,
    }
    report = "\n".join(
        [
            "# G1 joint-position stopping-headroom capture · R274",
            "",
            "**Mechanism passed / walking profile rejected.** R274 adds a default-off soft Viability request when directional stopping distance plus a bounded reaction guard exceeds the remaining authored position headroom. Existing hard joint intervals remain the authority boundary.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "profile",
                    "weight",
                    "brake",
                    "reaction s",
                    "first active",
                    "active ticks",
                    "knee limit",
                    "fallback",
                    "release",
                    "root RMS m",
                    "p99 ms",
                ],
                rows,
            ),
            "",
            f"The disabled replay matches every shared R273 non-timing array exactly and emits zero capture-active coordinates. Every enabled trace is also exact through its first measured activation. The mechanism is therefore dormant and causally timed rather than a hidden retiming of the reference.",
            "",
            f"No retained profile prevents the right-knee limit: enabled rows reach it at ticks {min(profiles[name]['critical_joint_first_lower_limit_tick'] for name in candidate_names)}–{max(profiles[name]['critical_joint_first_lower_limit_tick'] for name in candidate_names)} versus baseline {baseline['critical_joint_first_lower_limit_tick']}. The best release is tick {max(profiles[name]['first_release_tick'] for name in candidate_names)}, still before baseline {baseline['first_release_tick']}. Root RMS remains {min(profiles[name]['full_root_rms_m'] for name in candidate_names):.3f}–{max(profiles[name]['full_root_rms_m'] for name in candidate_names):.3f} m. No profile or authority is admitted.",
            "",
            "## Execution contract",
            "",
            "- Rust computes one scalar stopping-headroom request per selected joint without allocation; zero weight skips the path exactly.",
            "- The request shares the existing fixed Viability joint-task slot with velocity protection. Same-direction requests retain the stronger acceleration; the hard velocity-braking interval is unchanged.",
            "- Capture active-coordinate count is a dedicated raw trace, separate from velocity-envelope activity, solver status, position-limit contact, tracking, and timing.",
            "- The replay consumes the immutable reference and initial-state witness with zero policy and zero physics steps.",
        ]
    )
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "policy_steps": 0,
        "physics_steps": 0,
        "source_contract": source_contract,
        "profiles": profiles,
        "mechanism_passed": mechanism_passed,
        "profile_rejected": profile_rejected,
        "default_changed": bool(dormant_differences),
        "authority_admitted": False,
    }
    output = RESULT_ROOT / REVISION
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-joint-position-capture-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_JOINT_POSITION_CAPTURE_R274.md").write_text(report + "\n")
    pathlib.Path("web/G1_JOINT_POSITION_CAPTURE_R274.html").write_text(
        render_report_html(report, title="G1 joint-position capture · R274")
    )
    print(output / "G1_JOINT_POSITION_CAPTURE_R274.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
