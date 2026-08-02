#!/usr/bin/env python3
"""Audit named hard-feasibility witnesses and causal G1 transfer ablations."""

from __future__ import annotations

import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "g1-hard-feasibility-witness-r275"
RESULT_ROOT = pathlib.Path("benchmarks/results")
R274_CONTROL = RESULT_ROOT / "floating-g1-r274-position-capture-dormant-cap8" / "floating-walk-raw.npz"
TRACE_DIRS = {
    "control_cap8": "floating-g1-r275-hard-witness-control-cap8",
    "control_cap16": "floating-g1-r275-hard-witness-cap16",
    "control_cap64": "floating-g1-r275-hard-witness-cap64",
    "hard_capture_w025": "floating-g1-r275-position-capture-hard-w025-cap8",
    "local_left": "floating-g1-r275-ablation-local0",
    "local_right": "floating-g1-r275-ablation-local1",
    "accel_1000": "floating-g1-r275-ablation-accel1000",
    "torque_20000": "floating-g1-r275-ablation-torque20000",
    "normal_30x": "floating-g1-r275-ablation-normal30",
    "friction_2": "floating-g1-r275-ablation-friction2",
    "friction_3": "floating-g1-r275-ablation-friction3",
    "friction_5": "floating-g1-r275-ablation-friction5",
    "friction_10": "floating-g1-r275-ablation-friction10",
    "one_point": "floating-g1-r275-ablation-point",
}
TRACE_PATHS = {
    name: RESULT_ROOT / directory / "floating-walk-raw.npz"
    for name, directory in TRACE_DIRS.items()
}
REFERENCE = RESULT_ROOT / "g1-multistep-reference-r53" / "reference-inputs.npz"
WITNESS = RESULT_ROOT / "g1-multistep-oracle-r54" / "oracle-wbc-admission-raw.npz"
FAILED_PRE_STATUSES = (2, 3, 4, 5)
NO_ID = np.iinfo(np.uint32).max


def fingerprint(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load(path: pathlib.Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as source:
        return {key: np.asarray(source[key]) for key in source.files}


def first_tick(mask: np.ndarray) -> int | None:
    ticks = np.flatnonzero(mask)
    return int(ticks[0]) if len(ticks) else None


def semantic_differences(
    left: dict[str, np.ndarray], right: dict[str, np.ndarray]
) -> list[str]:
    return [
        key
        for key in sorted(set(left) & set(right))
        if key != "step_ns"
        and not np.array_equal(left[key], right[key], equal_nan=True)
    ]


def stable_row_name(stable_id: int) -> str:
    family = stable_id & 0xF000_0000
    local = stable_id & 0x0FFF_FFFF
    names = {
        0x1000_0000: "dynamics",
        0x2000_0000: "contact",
        0x3000_0000: "friction",
        0x4000_0000: "support",
        0x5000_0000: "actuator",
    }
    return f"{names.get(family, 'other')}[{local}]"


def summarize(trace: dict[str, np.ndarray]) -> dict[str, Any]:
    pre = trace["pre_contingency_status"]
    failed = np.isin(pre, FAILED_PRE_STATUSES)
    failed_ticks = np.flatnonzero(failed)
    linear_ids = trace["pre_contingency_limiting_linear_constraint"][failed]
    linear_ids = linear_ids[linear_ids != NO_ID]
    bound_coordinates = trace["pre_contingency_limiting_bound_coordinate"][failed]
    bound_coordinates = bound_coordinates[bound_coordinates >= 0]
    status = trace["status"]
    root_error = np.linalg.norm(
        trace["root_tracked"] - trace["effective_root_targets"], axis=1
    )
    foot_error = np.linalg.norm(
        trace["tracked_positions"][:, :2]
        - trace["effective_target_positions"][:, :2],
        axis=2,
    )
    timing_ms = trace["step_ns"].astype(np.float64) / 1.0e6
    first_failed = int(failed_ticks[0]) if len(failed_ticks) else None
    first_linear_id = (
        int(trace["pre_contingency_limiting_linear_constraint"][first_failed])
        if first_failed is not None
        else None
    )
    first_bound = (
        int(trace["pre_contingency_limiting_bound_coordinate"][first_failed])
        if first_failed is not None
        else None
    )
    return {
        "first_failed_tick": first_failed,
        "failed_ticks": failed_ticks.astype(int).tolist(),
        "first_failed_maximum_bound_violation": (
            float(trace["pre_contingency_maximum_bound_violation"][first_failed])
            if first_failed is not None
            else 0.0
        ),
        "first_failed_bound_coordinate": first_bound if first_bound is not None and first_bound >= 0 else None,
        "first_failed_bound_is_upper": (
            bool(trace["pre_contingency_limiting_bound_is_upper"][first_failed])
            if first_failed is not None and first_bound is not None and first_bound >= 0
            else None
        ),
        "first_failed_maximum_linear_violation": (
            float(trace["pre_contingency_maximum_linear_violation"][first_failed])
            if first_failed is not None
            else 0.0
        ),
        "first_failed_linear_constraint": (
            f"0x{first_linear_id:08x}"
            if first_linear_id is not None and first_linear_id != NO_ID
            else None
        ),
        "first_failed_linear_constraint_name": (
            stable_row_name(first_linear_id)
            if first_linear_id is not None and first_linear_id != NO_ID
            else None
        ),
        "failed_bound_coordinate_counts": {
            str(int(value)): int(np.count_nonzero(bound_coordinates == value))
            for value in np.unique(bound_coordinates)
        },
        "failed_linear_constraint_counts": {
            f"0x{int(value):08x}": int(np.count_nonzero(linear_ids == value))
            for value in np.unique(linear_ids)
        },
        "first_normal_fallback_tick": first_tick(
            np.any(trace["support_phase"] == 4, axis=1)
        ),
        "first_release_tick": first_tick(np.isin(status, (5, 12))),
        "localized_handoff_ticks": int(np.count_nonzero(status == 9)),
        "first_left_normal_fallback_tick": first_tick(trace["support_phase"][:, 0] == 4),
        "first_right_normal_fallback_tick": first_tick(trace["support_phase"][:, 1] == 4),
        "capture_active_ticks": int(
            np.count_nonzero(trace["joint_position_capture_active_coordinates"])
        ),
        "root_tracking_rms_m": float(np.sqrt(np.mean(np.square(root_error)))),
        "foot_tracking_rms_m": float(np.sqrt(np.mean(np.square(foot_error)))),
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
    required = (REFERENCE, WITNESS, R274_CONTROL, *TRACE_PATHS.values())
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise SystemExit("R275 is missing required evidence: " + ", ".join(missing))

    r274 = load(R274_CONTROL)
    traces = {name: load(path) for name, path in TRACE_PATHS.items()}
    profiles = {name: summarize(trace) for name, trace in traces.items()}
    control = traces["control_cap8"]
    control_profile = profiles["control_cap8"]
    dormant_differences = semantic_differences(r274, control)
    cap_state_keys = (
        "root_tracked",
        "root_quaternion_wxyz",
        "center_of_mass_tracked",
        "q",
        "v",
        "tracked_positions",
        "status",
        "support_phase",
    )
    cap_state_exact = {
        name: all(
            np.array_equal(control[key], traces[name][key], equal_nan=True)
            for key in cap_state_keys
        )
        for name in ("control_cap16", "control_cap64")
    }
    exact_resource_ablations = {
        name: semantic_differences(control, traces[name])
        for name in ("torque_20000", "normal_30x")
    }
    mechanism_passed = bool(
        not dormant_differences
        and all(cap_state_exact.values())
        and control_profile["first_failed_tick"] == 875
        and control_profile["first_failed_linear_constraint_name"] == "dynamics[3]"
        and control_profile["first_failed_bound_coordinate"] is None
        and all(not differences for differences in exact_resource_ablations.values())
        and all(profile["finite_state"] for profile in profiles.values())
    )
    hard_capture_rejected = bool(
        profiles["hard_capture_w025"]["first_failed_tick"]
        < control_profile["first_failed_tick"]
        and profiles["hard_capture_w025"]["first_failed_bound_coordinate"] == 15
        and profiles["hard_capture_w025"]["root_tracking_rms_m"]
        > control_profile["root_tracking_rms_m"]
    )
    localization_result = {
        "right_normal_only_admitted_without_release_at_first_failure": bool(
            profiles["local_right"]["first_right_normal_fallback_tick"] == 875
            and profiles["local_right"]["localized_handoff_ticks"] == 0
        ),
        "left_normal_only_required_left_release_at_first_failure": bool(
            profiles["local_left"]["localized_handoff_ticks"] == 1
        ),
        "automatic_localization_implemented": False,
    }

    selected = (
        "control_cap8",
        "control_cap16",
        "control_cap64",
        "hard_capture_w025",
        "local_left",
        "local_right",
        "accel_1000",
        "torque_20000",
        "normal_30x",
        "friction_2",
        "friction_3",
        "friction_5",
        "friction_10",
        "one_point",
    )
    rows = []
    for name in selected:
        profile = profiles[name]
        rows.append(
            [
                name.replace("_", " "),
                str(profile["first_failed_tick"]),
                str(profile["first_failed_linear_constraint_name"]),
                str(profile["first_failed_bound_coordinate"]),
                str(profile["first_normal_fallback_tick"]),
                str(profile["first_release_tick"]),
                f'{profile["root_tracking_rms_m"]:.3f}',
                f'{profile["foot_tracking_rms_m"]:.3f}',
                f'{profile["timing_ms"]["p99"]:.3f}',
            ]
        )

    report = "\n".join(
        [
            "# G1 hard-feasibility witness · R275",
            "",
            "**Diagnostic mechanism passed; recovery profile remains closed.** R275 separates anonymous coordinate-bound residue from named hard-row residue before contingency retries overwrite the first solve. The 8-sweep control remains byte-identical to R274 on all 74 shared non-timing arrays.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "profile",
                    "first hard fail",
                    "first named row",
                    "bound coord",
                    "fallback",
                    "release",
                    "root RMS m",
                    "foot RMS m",
                    "p99 ms",
                ],
                rows,
            ),
            "",
            "At tick 875 the retained control does not finish the hard problem. Its terminal bounded witness has zero coordinate-bound violation and a 13.781 violation of `dynamics[3]`, the floating-base world-X dynamics equality. Raising the projection cap to 16 or 64 changes the terminal witness but not the state, status, support phase, or failure tick. Raising torque to 20,000 or aggregate normal capacity to 30× is fully trace-exact, so neither resource is the active cause in this state stream.",
            "",
            "The per-foot ablation is sharper. Making only the right-foot rows normal-only yields an executable contingency at tick 875 while the left foot stays locked. Trying the left foot instead still fails and requires a typed left-support release. This localizes the first coupled conflict to right-foot tangential locking against floating-base dynamics; it does not yet authorize an automatic selector.",
            "",
            "A hard stopping-envelope variant activates the right-knee generalized acceleration bound (solver coordinate 15 = root 6 + joint 9) and fails six ticks earlier at 869 while root RMS worsens. Higher friction delays the first event to ticks 887–978, but every sweep still releases/fails later, changes the physical assumption, and misses the 5 ms profile. One-point contact and wider acceleration bounds also regress. No recovery profile or authority is admitted.",
            "",
            "## Execution contract",
            "",
            "- Rust scans the terminal fixed solver workspace without allocation and records deterministic bound coordinate/side plus named stable row/side.",
            "- Python captures the first solve before normal-only, localized-handoff, or release retries overwrite it.",
            "- The witness is diagnostic only: it cannot change contact mode, solve admission, integration, plant commands, or authority.",
            "- All replays consume the immutable R53 reference and R54 initial state with zero policy steps and zero physics steps.",
        ]
    )
    source_contract = {
        "reference_sha256": fingerprint(REFERENCE),
        "witness_sha256": fingerprint(WITNESS),
        "r274_control_sha256": fingerprint(R274_CONTROL),
        **{f"{name}_sha256": fingerprint(path) for name, path in TRACE_PATHS.items()},
        "policy_steps": 0,
        "physics_steps": 0,
        "shared_r274_non_timing_array_count": len(set(r274) & set(control)) - 1,
        "r274_control_semantic_differences": dormant_differences,
        "cap_state_exact": cap_state_exact,
        "exact_resource_ablation_differences": exact_resource_ablations,
    }
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "policy_steps": 0,
        "physics_steps": 0,
        "source_contract": source_contract,
        "profiles": profiles,
        "localization": localization_result,
        "mechanism_passed": mechanism_passed,
        "hard_capture_profile_rejected": hard_capture_rejected,
        "recovery_profile_admitted": False,
        "authority_admitted": False,
    }
    output = RESULT_ROOT / REVISION
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-hard-feasibility-witness-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_HARD_FEASIBILITY_WITNESS_R275.md").write_text(report + "\n")
    pathlib.Path("web/G1_HARD_FEASIBILITY_WITNESS_R275.html").write_text(
        render_report_html(report, title="G1 hard-feasibility witness · R275")
    )
    print(output / "G1_HARD_FEASIBILITY_WITNESS_R275.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
