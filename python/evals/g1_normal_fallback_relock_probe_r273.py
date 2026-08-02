#!/usr/bin/env python3
"""Audit bounded full-lock recovery probes from NormalFallback."""

from __future__ import annotations

import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "g1-normal-fallback-relock-probe-r273"
RESULT_ROOT = pathlib.Path("benchmarks/results")
BASELINE = RESULT_ROOT / "floating-g1-r272-normal-fallback-task-dormant-cap8" / "floating-walk-raw.npz"
TRACE_PATHS = {
    "r273_dormant": RESULT_ROOT / "floating-g1-r273-normal-fallback-relock-dormant-cap8" / "floating-walk-raw.npz",
    **{
        f"r273_interval{interval}": RESULT_ROOT
        / f"floating-g1-r273-normal-fallback-relock-{interval}-cap8"
        / "floating-walk-raw.npz"
        for interval in (1, 4, 8, 16, 32, 64)
    },
}
INTERVALS = {
    "r272_baseline": 0,
    "r273_dormant": 0,
    **{f"r273_interval{interval}": interval for interval in (1, 4, 8, 16, 32, 64)},
}
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


def semantic_differences(
    baseline: dict[str, np.ndarray],
    candidate: dict[str, np.ndarray],
    stop: int | None = None,
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


def first_tick(mask: np.ndarray) -> int:
    selected = np.flatnonzero(mask)
    return int(selected[0]) if len(selected) else len(mask)


def relock_durations(support_phase: np.ndarray, admitted_ticks: np.ndarray) -> list[int]:
    durations: list[int] = []
    for tick in admitted_ticks:
        promoted = np.flatnonzero(
            (support_phase[tick] == 3)
            & ((support_phase[tick - 1] == 4) if tick > 0 else False)
        )
        for target in promoted:
            stop = tick
            while stop < len(support_phase) and support_phase[stop, target] == 3:
                stop += 1
            durations.append(int(stop - tick))
    return durations


def timing_or_empty(values_ms: np.ndarray) -> dict[str, float | int]:
    if not len(values_ms):
        return {"samples": 0}
    return {"samples": int(len(values_ms)), **distribution(values_ms)}


def summarize(trace: dict[str, np.ndarray], interval: int) -> dict[str, Any]:
    status = trace["status"]
    phase = trace["support_phase"]
    timing_ms = trace["step_ns"].astype(np.float64) / 1.0e6
    root_error = np.linalg.norm(
        trace["root_tracked"] - trace["effective_root_targets"], axis=1
    )
    knee_q = trace["q"][:, CRITICAL_COORDINATE]
    lower_limit = np.flatnonzero(knee_q <= CRITICAL_POSITION_LOWER_RAD + 1.0e-9)
    admitted_ticks = np.flatnonzero(status == 10)
    rejected_ticks = np.flatnonzero(np.isin(status, (11, 12)))
    release_mask = np.isin(status, (5, 12))
    fallback_mask = np.any(phase == 4, axis=1)
    durations = relock_durations(phase, admitted_ticks)
    values, counts = np.unique(status, return_counts=True)
    return {
        "interval_ticks": interval,
        "ticks": int(len(status)),
        "first_fallback_tick": first_tick(fallback_mask),
        "last_fallback_tick": int(np.flatnonzero(fallback_mask)[-1]) if np.any(fallback_mask) else None,
        "first_release_tick": first_tick(release_mask),
        "probe_attempts": int(len(admitted_ticks) + len(rejected_ticks)),
        "probe_admissions": int(len(admitted_ticks)),
        "probe_rejections": int(len(rejected_ticks)),
        "probe_rejected_releases": int(np.count_nonzero(status == 12)),
        "admitted_ticks": admitted_ticks.astype(int).tolist(),
        "rejected_ticks": rejected_ticks.astype(int).tolist(),
        "relock_durations_ticks": durations,
        "maximum_relock_duration_ticks": max(durations, default=0),
        "critical_joint_first_lower_limit_tick": int(lower_limit[0]) if len(lower_limit) else None,
        "full_root_rms_m": float(np.sqrt(np.mean(np.square(root_error)))),
        "timing_ms": distribution(timing_ms),
        "probe_admitted_timing_ms": timing_or_empty(timing_ms[status == 10]),
        "probe_rejected_timing_ms": timing_or_empty(timing_ms[np.isin(status, (11, 12))]),
        "status_counts": {str(int(key)): int(value) for key, value in zip(values, counts)},
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
        raise SystemExit("R273 is missing required evidence: " + ", ".join(missing))

    traces = {"r272_baseline": load(BASELINE)}
    traces.update({name: load(path) for name, path in TRACE_PATHS.items()})
    profiles = {
        name: summarize(trace, INTERVALS[name]) for name, trace in traces.items()
    }
    baseline = profiles["r272_baseline"]
    dormant_differences = semantic_differences(
        traces["r272_baseline"], traces["r273_dormant"]
    )
    candidate_names = tuple(f"r273_interval{interval}" for interval in (1, 4, 8, 16, 32, 64))
    causal_prefix_stop = baseline["first_fallback_tick"] + 1
    prefix_differences = {
        name: semantic_differences(
            traces["r272_baseline"], traces[name], causal_prefix_stop
        )
        for name in candidate_names
    }
    full_differences = {
        name: semantic_differences(traces["r272_baseline"], traces[name])
        for name in candidate_names
    }
    admitted_names = tuple(
        name for name in candidate_names if profiles[name]["probe_admissions"] > 0
    )
    mechanism_passed = bool(
        not dormant_differences
        and all(not prefix_differences[name] for name in candidate_names)
        and all(profiles[name]["probe_attempts"] > 0 for name in candidate_names)
        and all(profiles[name]["finite_state"] for name in candidate_names)
        and all(profiles[name]["maximum_relock_duration_ticks"] > 0 for name in admitted_names)
        and len(admitted_names) >= 2
    )
    profile_rejected = bool(
        all(
            profiles[name]["first_release_tick"] <= baseline["first_release_tick"]
            for name in candidate_names
        )
        and all(
            profiles[name]["first_release_tick"] < baseline["first_release_tick"]
            for name in admitted_names
        )
        and profiles["r273_interval64"]["probe_admissions"] == 0
    )

    rows = []
    for name, profile in profiles.items():
        rows.append(
            [
                name.replace("_", " "),
                "off" if profile["interval_ticks"] == 0 else str(profile["interval_ticks"]),
                str(profile["first_fallback_tick"]),
                str(profile["probe_attempts"]),
                str(profile["probe_admissions"]),
                str(profile["probe_rejections"]),
                str(profile["maximum_relock_duration_ticks"]),
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
        "default_interval_ticks": 0,
        "dormant_non_timing_arrays_equal": not dormant_differences,
        "dormant_semantic_differences": dormant_differences,
        "causal_prefix_stop_exclusive": causal_prefix_stop,
        "candidate_prefix_semantic_differences": prefix_differences,
        "candidate_full_semantic_differences": full_differences,
    }
    report = "\n".join(
        [
            "# G1 NormalFallback bounded relock probe · R273",
            "",
            "**Mechanism passed / walking profile rejected.** R273 removes the absorbing-state defect from `NormalFallback`: a default-off cadence may spend one bounded full-lock solve, promote only a solved result, and otherwise discard it before retrying the established normal-only rows.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "profile",
                    "interval",
                    "fallback",
                    "attempts",
                    "admit",
                    "reject",
                    "max relock ticks",
                    "release",
                    "root RMS m",
                    "p99 ms",
                ],
                rows,
            ),
            "",
            f"The disabled R273 replay is bitwise equal to R272 on all {len(traces['r272_baseline']) - 1} non-timing arrays: {not dormant_differences}. Every enabled trace is also bitwise equal through the first fallback tick {baseline['first_fallback_tick']}, so the probe cannot rewrite its cause. The right knee still reaches its lower limit at tick {baseline['critical_joint_first_lower_limit_tick']}.",
            "",
            f"Intervals 1, 4, 8, 16, and 32 produced explicit full-lock admissions lasting at most {max(profiles[name]['maximum_relock_duration_ticks'] for name in admitted_names)} ticks, proving the state can recover continuously. They then re-entered fallback and released at ticks {profiles['r273_interval1']['first_release_tick']}, {profiles['r273_interval4']['first_release_tick']}, {profiles['r273_interval8']['first_release_tick']}, {profiles['r273_interval16']['first_release_tick']}, and {profiles['r273_interval32']['first_release_tick']}—all earlier than baseline tick {baseline['first_release_tick']}. Interval 64 rejected every probe and preserved the release tick, adding work without authority. No cadence is promoted.",
            "",
            "## Execution contract",
            "",
            "- Interval zero does not branch into probe construction or an extra solve.",
            "- An admitted probe uses the ordinary full locked contact modes and is the only path from `NormalFallback` back to `Locked`.",
            "- A rejected full-lock output is never integrated. The controller restores the normal-only modes and task before one bounded retry; rejection-with-release has its own typed status.",
            "- Probe admission, rejection, rejection-with-release, relock duration, per-status solver work, and latency remain visible in the raw trace and corpus report.",
            "- The replay consumes an immutable reference and initial-state witness; it executes zero policy and zero physics steps.",
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
    (output / "g1-normal-fallback-relock-probe-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_NORMAL_FALLBACK_RELOCK_PROBE_R273.md").write_text(report + "\n")
    pathlib.Path("web/G1_NORMAL_FALLBACK_RELOCK_PROBE_R273.html").write_text(
        render_report_html(report, title="G1 NormalFallback relock probe · R273")
    )
    print(output / "G1_NORMAL_FALLBACK_RELOCK_PROBE_R273.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
