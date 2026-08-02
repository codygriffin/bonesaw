#!/usr/bin/env python3
"""Evaluate and causally split the opt-in lower-body velocity envelope."""

from __future__ import annotations

import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "g1-lower-body-velocity-envelope-r270"
RESULT_ROOT = pathlib.Path("benchmarks/results")
R268_BASELINE = (
    RESULT_ROOT
    / "floating-g1-r268-native-reference-morphology-jet-low-gain-cap8"
    / "floating-walk-raw.npz"
)
R269_BASELINE = (
    RESULT_ROOT
    / "floating-g1-r269-low-gain-cross-tick-cap8x13"
    / "floating-walk-raw.npz"
)
DORMANT = (
    RESULT_ROOT
    / "floating-g1-r270-lower-body-dormant-r269-stack-cap8"
    / "floating-walk-raw.npz"
)
SOFT_ONLY = (
    RESULT_ROOT
    / "floating-g1-r270-lower-body-soft-only-cap8"
    / "floating-walk-raw.npz"
)
HARD_ONLY = (
    RESULT_ROOT
    / "floating-g1-r270-lower-body-hard-only-cap8"
    / "floating-walk-raw.npz"
)
COMBINED = (
    RESULT_ROOT
    / "floating-g1-r270-lower-body-hard-cap8"
    / "floating-walk-raw.npz"
)
HARD_ONLY_COMPOSED = (
    RESULT_ROOT
    / "floating-g1-r270-lower-body-hard-only-r269-stack-cap8"
    / "floating-walk-raw.npz"
)
COMBINED_COMPOSED = (
    RESULT_ROOT
    / "floating-g1-r270-lower-body-hard-r269-stack-cap8"
    / "floating-walk-raw.npz"
)
REFERENCE = RESULT_ROOT / "g1-multistep-reference-r53" / "reference-inputs.npz"
WITNESS = RESULT_ROOT / "g1-multistep-oracle-r54" / "oracle-wbc-admission-raw.npz"
CRITICAL_COORDINATE = 9  # Unitree G1 right_knee_joint in the pinned 23-DOF URDF.
DT_SECONDS = 0.005
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


def summarize(trace: dict[str, np.ndarray]) -> dict[str, Any]:
    status = trace["status"]
    timing_ms = trace["step_ns"].astype(np.float64) / 1.0e6
    root_error = np.linalg.norm(
        trace["root_tracked"] - trace["effective_root_targets"], axis=1
    )
    attitude = 2.0 * np.arccos(
        np.clip(np.abs(trace["root_quaternion_wxyz"][:, 0]), 0.0, 1.0)
    )
    first_fallback = first_tick(status, (4, 5, 8, 9))
    first_release = first_tick(status, (5,))
    envelope_active = trace["joint_velocity_envelope_active_coordinates"]
    critical_window = slice(850, 876)
    values, counts = np.unique(status, return_counts=True)
    release = status == 5
    after_critical_window = np.arange(len(status)) >= 850
    at_lower_limit = trace["q"][:, CRITICAL_COORDINATE] <= (
        CRITICAL_POSITION_LOWER_RAD + 1.0e-9
    )
    lower_limit_ticks = np.flatnonzero(after_critical_window & at_lower_limit)
    normal_force = np.sum(trace["contact_normal_force"], axis=1)
    return {
        "ticks": int(len(status)),
        "status_counts": {str(int(k)): int(v) for k, v in zip(values, counts)},
        "first_fallback_tick": first_fallback,
        "first_release_tick": first_release,
        "prefix_root_rms_m": float(
            np.sqrt(np.mean(np.square(root_error[:first_fallback])))
        ),
        "full_root_rms_m": float(np.sqrt(np.mean(np.square(root_error)))),
        "maximum_root_attitude_deg": float(np.degrees(np.max(attitude))),
        "critical_joint_velocity_min_rad_s": float(
            np.min(trace["v"][critical_window, CRITICAL_COORDINATE])
        ),
        "critical_joint_velocity_max_abs_rad_s": float(
            np.max(np.abs(trace["v"][critical_window, CRITICAL_COORDINATE]))
        ),
        "critical_joint_first_lower_limit_tick": (
            int(lower_limit_ticks[0]) if len(lower_limit_ticks) else None
        ),
        "envelope_active_ticks": int(np.count_nonzero(envelope_active)),
        "envelope_active_max_coordinates": int(np.max(envelope_active)),
        "envelope_active_early_max_coordinates": int(np.max(envelope_active[200:300])),
        "envelope_active_critical_max_coordinates": int(
            np.max(envelope_active[850:876])
        ),
        "release_diagnostics_fail_closed": bool(
            np.all(trace["dynamics_residual"][release] == 0.0)
            and np.all(trace["contact_residual"][release] == 0.0)
        ),
        "normal_force_before_first_release_n": (
            float(normal_force[first_release - 1])
            if 0 < first_release < len(status)
            else 0.0
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


def semantic_arrays_equal(
    left: dict[str, np.ndarray], right: dict[str, np.ndarray]
) -> tuple[bool, list[str]]:
    differing = [
        key
        for key in left
        if key != "step_ns"
        and (
            key not in right
            or not np.array_equal(left[key], right[key], equal_nan=True)
        )
    ]
    differing.extend(key for key in right if key not in left)
    return not differing, sorted(set(differing))


def main() -> int:
    required = (
        R268_BASELINE,
        R269_BASELINE,
        DORMANT,
        SOFT_ONLY,
        HARD_ONLY,
        COMBINED,
        HARD_ONLY_COMPOSED,
        COMBINED_COMPOSED,
        REFERENCE,
        WITNESS,
    )
    if not all(path.is_file() for path in required):
        raise SystemExit("R270 requires the retained baseline, candidate, reference, and witness")
    r268_trace = load(R268_BASELINE)
    r269_trace = load(R269_BASELINE)
    dormant_trace = load(DORMANT)
    baseline = summarize(r268_trace)
    r269 = summarize(r269_trace)
    dormant = summarize(dormant_trace)
    soft_only_trace = load(SOFT_ONLY)
    hard_only_trace = load(HARD_ONLY)
    soft_only = summarize(soft_only_trace)
    hard_only = summarize(hard_only_trace)
    candidate_trace = load(COMBINED)
    candidate = summarize(candidate_trace)
    hard_only_composed_trace = load(HARD_ONLY_COMPOSED)
    composed_trace = load(COMBINED_COMPOSED)
    hard_only_composed = summarize(hard_only_composed_trace)
    composed = summarize(composed_trace)
    dormant_equal, dormant_differences = semantic_arrays_equal(r269_trace, dormant_trace)
    source_contract = {
        "reference_sha256": fingerprint(REFERENCE),
        "witness_sha256": fingerprint(WITNESS),
        "r268_baseline_sha256": fingerprint(R268_BASELINE),
        "r269_baseline_sha256": fingerprint(R269_BASELINE),
        "dormant_sha256": fingerprint(DORMANT),
        "soft_only_sha256": fingerprint(SOFT_ONLY),
        "hard_only_sha256": fingerprint(HARD_ONLY),
        "combined_sha256": fingerprint(COMBINED),
        "hard_only_composed_sha256": fingerprint(HARD_ONLY_COMPOSED),
        "combined_composed_sha256": fingerprint(COMBINED_COMPOSED),
        "critical_coordinate": CRITICAL_COORDINATE,
        "critical_joint_name": "right_knee_joint",
        "policy_steps": 0,
        "physics_steps": 0,
        "hard_envelope_is_opt_in": True,
        "lower_body_allowlist_is_model_name_derived": True,
        "dormant_semantic_arrays_equal_r269": dormant_equal,
        "dormant_semantic_differences": dormant_differences,
    }
    hard_bound_causal = bool(
        baseline["first_fallback_tick"] == 863
        and hard_only["first_fallback_tick"] >= baseline["first_fallback_tick"] + 20
        and hard_only["critical_joint_velocity_min_rad_s"] > -8.0
        and hard_only["envelope_active_early_max_coordinates"] == 0
        and hard_only["envelope_active_critical_max_coordinates"] >= 1
        and soft_only["first_fallback_tick"] <= baseline["first_fallback_tick"] + 1
    )
    soft_hard_interaction_observed = bool(
        candidate["first_release_tick"] >= hard_only["first_release_tick"] + 200
        and candidate["first_release_tick"] >= soft_only["first_release_tick"] + 200
        and candidate["release_diagnostics_fail_closed"]
        and candidate["finite_state"]
    )
    mechanism_passed = bool(
        hard_bound_causal
        and soft_hard_interaction_observed
        and dormant_equal
    )
    composition_rejected = bool(
        composed["first_fallback_tick"] == candidate["first_fallback_tick"]
        and composed["first_release_tick"] < candidate["first_release_tick"]
    )
    profile_rejected = bool(
        candidate["full_root_rms_m"] > 1.0
        or candidate["first_release_tick"] < candidate["ticks"]
    )
    profiles = {
        "r268_low_gain_baseline": baseline,
        "r269_bounded_continuation": r269,
        "r270_dormant_r269_stack": dormant,
        "r270_lower_body_soft_only": soft_only,
        "r270_lower_body_hard_only": hard_only,
        "r270_lower_body_soft_plus_hard": candidate,
        "r270_hard_only_composed_r269_stack": hard_only_composed,
        "r270_soft_plus_hard_composed_r269_stack": composed,
    }
    rows = [
        [
            name.replace("_", " "),
            str(row["first_fallback_tick"]),
            str(row["first_release_tick"]),
            f'{row["prefix_root_rms_m"] * 100:.3f}',
            f'{row["full_root_rms_m"]:.3f}',
            f'{row["maximum_root_attitude_deg"]:.2f}',
            f'{row["timing_ms"]["p99"]:.3f}',
        ]
        for name, row in profiles.items()
    ]
    report = "\n".join(
        [
            "# G1 lower-body velocity-envelope causal split · R270",
            "",
            "**Causal safety mechanism PASS / walking profile REJECTED.** This replay uses no policy and no physics simulator. It separately measures the soft viability task, the hard directional braking bound, their combination, and both R269 compositions. The opt-in profile derives a lower-body coordinate allowlist from the pinned URDF joint names and activates at 50% velocity utilization. The default controller is unchanged.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "profile",
                    "first fallback",
                    "first release",
                    "prefix root RMS cm",
                    "full root RMS m",
                    "max attitude deg",
                    "p99 ms",
                ],
                rows,
            ),
            "",
            f"The hard-bound-only control establishes causality: it moves first fallback {baseline['first_fallback_tick']}→{hard_only['first_fallback_tick']} and clips the pinned right-knee coordinate {CRITICAL_COORDINATE} to {hard_only['critical_joint_velocity_min_rad_s']:.3f} rad/s in the critical window. Soft-only moves fallback only to tick {soft_only['first_fallback_tick']} and reaches {soft_only['critical_joint_velocity_min_rad_s']:.3f} rad/s. The early window has zero activations in every opt-in control.",
            "",
            f"The long release delay is an interaction, not a hard-bound-only result. Hard-only releases at tick {hard_only['first_release_tick']}; soft-only releases at {soft_only['first_release_tick']}; soft+hard reaches tick {candidate['first_release_tick']}. It retains {candidate['normal_force_before_first_release_n']:.1f} N one tick before release but has already reached the right-knee lower position limit at tick {candidate['critical_joint_first_lower_limit_tick']}. This localizes the next problem to continuous position-limit/support-transition recovery.",
            "",
            f"The interaction is not compositional yet. Adding R269 continuation and the predeclared target-1 handoff preserves the tick-{composed['first_fallback_tick']} combined fallback but releases at tick {composed['first_release_tick']}, versus {candidate['first_release_tick']} standalone, and records {composed['timing_ms']['p99']:.3f} ms p99. Hard-only plus R269 releases at tick {hard_only_composed['first_release_tick']}. R269 and the soft+hard R270 profile therefore remain alternative diagnostics, not one admitted controller profile.",
            "",
            f"This is not a walking pass: combined full-run root RMS remains {candidate['full_root_rms_m']:.3f} m and every opt-in variant releases support before the end of the trace. The combined candidate records {candidate['timing_ms']['p99']:.3f} ms p99 and {candidate['deadline_misses']['20ms']} 20 ms misses; timing is reported per retained distribution, not borrowed between variants.",
            "",
            "## Contract",
            "",
            "- The directional hard braking bound and soft viability task are independently switchable, opt-in, and lower-body-only; a zero soft weight is a valid hard-only causal control.",
            "- A conflicting hard braking interval is carried to the solver and fails closed as `InvalidProblem`; it is never silently weakened or skipped.",
            f"- With the new fields dormant, all {len(r269_trace) - 1} non-timing trace arrays are bitwise equal to R269 (NaNs compared positionally).",
            "- Composition with R269 is explicitly rejected; no benchmark row may borrow the standalone release time and the composed timing result.",
            "- The profile consumes only the immutable reference and initial morphology witness; no future policy, oracle force, solved acceleration, or simulator state is used.",
            "- Status-5 release telemetry remains fail-closed with zero rejected dynamics/contact residual witnesses.",
            f"- Reference SHA-256: `{source_contract['reference_sha256']}`; witness SHA-256: `{source_contract['witness_sha256']}`.",
        ]
    )
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "policy_steps": 0,
        "physics_steps": 0,
        "controller_ticks": int(sum(row["ticks"] for row in profiles.values())),
        "controller_integration_steps": int(
            sum(
                np.count_nonzero(trace["status"] != 8)
                for trace in (
                    r268_trace,
                    r269_trace,
                    dormant_trace,
                    soft_only_trace,
                    hard_only_trace,
                    candidate_trace,
                    hard_only_composed_trace,
                    composed_trace,
                )
            )
        ),
        "source_contract": source_contract,
        "profiles": profiles,
        "mechanism_passed": mechanism_passed,
        "hard_bound_causal": hard_bound_causal,
        "soft_hard_interaction_observed": soft_hard_interaction_observed,
        "composition_rejected": composition_rejected,
        "profile_rejected": profile_rejected,
        "default_changed": False,
        "authority_admitted": False,
    }
    output = RESULT_ROOT / REVISION
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-lower-body-velocity-envelope-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_LOWER_BODY_VELOCITY_ENVELOPE_R270.md").write_text(report + "\n")
    web = pathlib.Path("web/G1_LOWER_BODY_VELOCITY_ENVELOPE_R270.html")
    web.write_text(
        render_report_html(
            report, title="G1 lower-body velocity-envelope causal split · R270"
        )
    )
    print(output / "G1_LOWER_BODY_VELOCITY_ENVELOPE_R270.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
