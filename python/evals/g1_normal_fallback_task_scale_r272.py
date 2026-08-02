#!/usr/bin/env python3
"""Causally split the soft point task retained during NormalFallback."""

from __future__ import annotations

import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "g1-normal-fallback-task-scale-r272"
RESULT_ROOT = pathlib.Path("benchmarks/results")
BASELINE = RESULT_ROOT / "floating-g1-r270-lower-body-hard-cap8" / "floating-walk-raw.npz"
DORMANT = RESULT_ROOT / "floating-g1-r272-normal-fallback-task-dormant-cap8" / "floating-walk-raw.npz"
SCALE0 = RESULT_ROOT / "floating-g1-r272-normal-fallback-task-scale0-cap8" / "floating-walk-raw.npz"
SCALE025 = RESULT_ROOT / "floating-g1-r272-normal-fallback-task-scale025-cap8" / "floating-walk-raw.npz"
SCALE05 = RESULT_ROOT / "floating-g1-r272-normal-fallback-task-scale05-cap8" / "floating-walk-raw.npz"
REFERENCE = RESULT_ROOT / "g1-multistep-reference-r53" / "reference-inputs.npz"
WITNESS = RESULT_ROOT / "g1-multistep-oracle-r54" / "oracle-wbc-admission-raw.npz"
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


def first_tick(status: np.ndarray, codes: tuple[int, ...]) -> int:
    selected = np.isin(status, codes)
    return int(np.flatnonzero(selected)[0]) if np.any(selected) else len(status)


def semantic_differences(
    baseline: dict[str, np.ndarray], candidate: dict[str, np.ndarray], stop: int | None = None
) -> list[str]:
    differences: list[str] = []
    for key in sorted(set(baseline) | set(candidate)):
        if key == "step_ns":
            continue
        if key not in baseline or key not in candidate:
            differences.append(key)
            continue
        left = baseline[key] if stop is None else baseline[key][:stop]
        right = candidate[key] if stop is None else candidate[key][:stop]
        if not np.array_equal(left, right, equal_nan=True):
            differences.append(key)
    return differences


def summarize(trace: dict[str, np.ndarray], scale: float) -> dict[str, Any]:
    status = trace["status"]
    timing_ms = trace["step_ns"].astype(np.float64) / 1.0e6
    root_error = np.linalg.norm(
        trace["root_tracked"] - trace["effective_root_targets"], axis=1
    )
    attitude = 2.0 * np.arccos(
        np.clip(np.abs(trace["root_quaternion_wxyz"][:, 0]), 0.0, 1.0)
    )
    knee_q = trace["q"][:, CRITICAL_COORDINATE]
    knee_v = trace["v"][:, CRITICAL_COORDINATE]
    lower_limit = np.flatnonzero(knee_q <= CRITICAL_POSITION_LOWER_RAD + 1.0e-9)
    first_fallback = first_tick(status, (4, 5, 8, 9))
    first_release = first_tick(status, (5,))
    values, counts = np.unique(status, return_counts=True)
    return {
        "ticks": int(len(status)),
        "status_counts": {str(int(k)): int(v) for k, v in zip(values, counts)},
        "scale": scale,
        "first_fallback_tick": first_fallback,
        "first_release_tick": first_release,
        "prefix_root_rms_m": float(
            np.sqrt(np.mean(np.square(root_error[:first_fallback])))
        ),
        "full_root_rms_m": float(np.sqrt(np.mean(np.square(root_error)))),
        "maximum_root_attitude_deg": float(np.degrees(np.max(attitude))),
        "critical_joint_velocity_min_rad_s": float(np.min(knee_v[850:876])),
        "full_joint_velocity_min_rad_s": float(np.min(knee_v)),
        "critical_joint_first_lower_limit_tick": (
            int(lower_limit[0]) if len(lower_limit) else None
        ),
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
    required = (
        BASELINE,
        DORMANT,
        SCALE0,
        SCALE025,
        SCALE05,
        REFERENCE,
        WITNESS,
    )
    if not all(path.is_file() for path in required):
        missing = [str(path) for path in required if not path.is_file()]
        raise SystemExit("R272 is missing required evidence: " + ", ".join(missing))
    traces = {
        "r270_baseline": load(BASELINE),
        "r272_dormant": load(DORMANT),
        "r272_scale0": load(SCALE0),
        "r272_scale025": load(SCALE025),
        "r272_scale05": load(SCALE05),
    }
    scales = {
        "r270_baseline": 1.0,
        "r272_dormant": 1.0,
        "r272_scale0": 0.0,
        "r272_scale025": 0.25,
        "r272_scale05": 0.5,
    }
    profiles = {
        name: summarize(trace, scales[name]) for name, trace in traces.items()
    }
    dormant_differences = semantic_differences(
        traces["r270_baseline"], traces["r272_dormant"]
    )
    dormant_equal = not dormant_differences
    baseline = profiles["r270_baseline"]
    candidate_names = ("r272_scale0", "r272_scale025", "r272_scale05")
    candidates = [profiles[name] for name in candidate_names]
    causal_prefix_stop = baseline["first_fallback_tick"] + 1
    prefix_differences = {
        name: semantic_differences(
            traces["r270_baseline"], traces[name], causal_prefix_stop
        )
        for name in candidate_names
    }
    full_differences = {
        name: semantic_differences(traces["r270_baseline"], traces[name])
        for name in candidate_names
    }
    mechanism_passed = bool(
        dormant_equal
        and all(row["finite_state"] for row in candidates)
        and all(not prefix_differences[name] for name in candidate_names)
        and all(full_differences[name] for name in candidate_names)
    )
    profile_rejected = bool(
        all(row["first_release_tick"] < baseline["first_release_tick"] for row in candidates)
        and all(row["first_fallback_tick"] == baseline["first_fallback_tick"] for row in candidates)
        and all(
            row["critical_joint_first_lower_limit_tick"]
            == baseline["critical_joint_first_lower_limit_tick"]
            for row in candidates
        )
        and all(row["full_root_rms_m"] > 1.0 for row in candidates)
    )
    rows = [
        [
            name.replace("_", " "),
            f'{row["scale"]:.2f}',
            str(row["first_fallback_tick"]),
            str(row["first_release_tick"]),
            f'{row["critical_joint_velocity_min_rad_s"]:.3f}',
            str(row["critical_joint_first_lower_limit_tick"]),
            f'{row["full_root_rms_m"]:.3f}',
            f'{row["timing_ms"]["p99"]:.3f}',
        ]
        for name, row in profiles.items()
    ]
    source_paths = {
        "reference": REFERENCE,
        "witness": WITNESS,
        "baseline": BASELINE,
        "dormant": DORMANT,
        "scale0": SCALE0,
        "scale025": SCALE025,
        "scale05": SCALE05,
    }
    source_contract = {
        **{f"{name}_sha256": fingerprint(path) for name, path in source_paths.items()},
        "critical_coordinate": CRITICAL_COORDINATE,
        "critical_joint_name": "right_knee_joint",
        "policy_steps": 0,
        "physics_steps": 0,
        "dormant_non_timing_arrays_equal": dormant_equal,
        "dormant_semantic_differences": dormant_differences,
        "default_scale": 1.0,
        "causal_prefix_stop_exclusive": causal_prefix_stop,
        "candidate_prefix_semantic_differences": prefix_differences,
        "candidate_full_semantic_differences": full_differences,
    }
    report = "\n".join(
        [
            "# G1 NormalFallback point-task scale causal split · R272",
            "",
            "**Mechanism partial / walking profile rejected.** R272 makes the viability point-task weight retained during `NormalFallback` an explicit, default-preserving scale. The scale is applied only to the already typed fallback task; it adds no solver rows, policy step, or physics simulation.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "profile",
                    "scale",
                    "first fallback",
                    "first release",
                    "critical v min",
                    "knee limit tick",
                    "root RMS m",
                    "p99 ms",
                ],
                rows,
            ),
            "",
            f"The scale-1 dormant replay is bitwise equal to the R270 baseline on all {len(traces['r270_baseline']) - 1} retained non-timing arrays: {dormant_equal}. More importantly, every reduced-scale candidate is bitwise equal to baseline through tick {baseline['first_fallback_tick']} on every non-timing array. All four profiles therefore reach the right-knee lower limit at tick {baseline['critical_joint_first_lower_limit_tick']}, enter fallback at tick {baseline['first_fallback_tick']}, and share the same {baseline['critical_joint_velocity_min_rad_s']:.3f} rad/s critical-window knee minimum. A post-fallback task cannot repair its own pre-fallback cause.",
            "",
            f"After that common boundary, the scale is observably causal but strictly worse for support duration: scale 0 releases at tick {profiles['r272_scale0']['first_release_tick']}, scale 0.25 at {profiles['r272_scale025']['first_release_tick']}, and scale 0.5 at {profiles['r272_scale05']['first_release_tick']}, all before the default's {baseline['first_release_tick']}. Full root RMS remains {min(row['full_root_rms_m'] for row in candidates):.3f}–{max(row['full_root_rms_m'] for row in candidates):.3f} m. Timing is reported per trace but cannot rescue the behavioral rejection. No scale is admitted.",
            "",
            "## Contract",
            "",
            "- The field defaults to 1.0, is finite/nonnegative, and only scales the existing soft point task after a measured `NormalFallback` phase.",
            "- Hard contact rows, force cones, acceleration bounds, and fail-closed release semantics are unchanged.",
            "- The replay consumes only the immutable reference and initial morphology witness; policy and physics step counts are zero.",
            f"- Reference SHA-256: `{source_contract['reference_sha256']}`; witness SHA-256: `{source_contract['witness_sha256']}`.",
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
        "default_changed": not dormant_equal,
        "authority_admitted": False,
    }
    output = RESULT_ROOT / REVISION
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-normal-fallback-task-scale-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_NORMAL_FALLBACK_TASK_SCALE_R272.md").write_text(report + "\n")
    pathlib.Path("web/G1_NORMAL_FALLBACK_TASK_SCALE_R272.html").write_text(
        render_report_html(report, title="G1 NormalFallback task scale · R272")
    )
    print(output / "G1_NORMAL_FALLBACK_TASK_SCALE_R272.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
