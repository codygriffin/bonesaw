#!/usr/bin/env python3
"""Render the R40 G1 balance/coupled-phase causal ablation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


CASES = {
    "r39": "g1-transfer-r39-retiming-release100-guard020",
    "posture_intent": "g1-transfer-r40-retiming-posture-intent",
    "root_preference": "g1-transfer-r40-root-horizontal-preference",
    "support_preview": "g1-transfer-r40-support-preview-retiming",
    "scale_010": "g1-transfer-r40-scale010",
    "upper_intent": "g1-transfer-r40-scale010-upper-intent1",
    "upper_viability": "g1-transfer-r40-scale010-upper-viability010",
    "balance_inside": "g1-transfer-r40-balance-phase-default",
    "balance_hard": "g1-transfer-r40-balance-phase-outside020",
    "balance_smooth": "g1-transfer-r40-balance-phase-outside020-engage50",
    "balance_floor": "g1-transfer-r40-balance-phase-min050-engage50",
    "balance_floor_800": "g1-transfer-r40-balance-phase-min050-engage50-800",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", default="benchmarks/results")
    parser.add_argument(
        "--output",
        default="benchmarks/results/g1-balance-phase-r40/G1_BALANCE_PHASE_CAUSAL_REPORT.md",
    )
    return parser.parse_args()


def load_case(root: Path, name: str) -> tuple[dict[str, Any], Any, dict[str, Any]]:
    directory = root / name
    payload = json.loads((directory / "floating-walk-metrics.json").read_text())
    return payload["metrics"], np.load(directory / "floating-walk-raw.npz"), payload["motion"]


def non_nominal(metrics: dict[str, Any]) -> int:
    status = metrics["status_counts"]
    return sum(
        status[key]
        for key in (
            "normal_contact_contingency",
            "contact_release_contingency",
            "primal_infeasible",
            "failed",
        )
    )


def phase(metrics: dict[str, Any], key: str, default: float = 0.0) -> float:
    return float((metrics.get("touchdown_phase_retiming") or {}).get(key, default))


def first_tick(mask: np.ndarray) -> str:
    indices = np.flatnonzero(mask)
    return "—" if len(indices) == 0 else str(int(indices[0]))


def main() -> None:
    args = parse_args()
    root = Path(args.results_root)
    loaded = {key: load_case(root, name) for key, name in CASES.items()}
    metrics = {key: value[0] for key, value in loaded.items()}
    traces = {key: value[1] for key, value in loaded.items()}
    motion = {key: value[2] for key, value in loaded.items()}

    old = np.load(
        root / "g1-synthetic-step-r39-retiming-disabled-smoke" / "floating-walk-raw.npz"
    )
    new = np.load(
        root
        / "g1-synthetic-step-r40-balance-phase-disabled-smoke"
        / "floating-walk-raw.npz"
    )
    shared = (set(old.files) & set(new.files)) - {"step_ns"}
    inactive_exact = all(
        np.array_equal(old[key], new[key], equal_nan=True)
        if old[key].dtype.kind == "f"
        else np.array_equal(old[key], new[key])
        for key in shared
    )

    labels = (
        ("R39 touchdown cursor", "r39"),
        ("whole posture at Intent", "posture_intent"),
        ("pelvis horizontal at Preference", "root_preference"),
        ("static support-preview + cursor", "support_preview"),
        ("0.10× spatial retarget", "scale_010"),
        ("0.10× + upper posture Intent", "upper_intent"),
        ("0.10× + upper posture Viability", "upper_viability"),
        ("margin hold 0 / full +2 cm", "balance_inside"),
        ("outside taper, immediate", "balance_hard"),
        ("outside taper, 50-tick engage", "balance_smooth"),
        ("outside taper, 0.5× floor", "balance_floor"),
        ("0.5× floor, 800 ticks", "balance_floor_800"),
    )

    lines = [
        "# Bonesaw R40 G1 balance-phase causal report",
        "",
        "This is an explicit rejected-policy report for the sustained CMU-to-G1 "
        "transfer. It asks whether the R39 Rust-owned source cursor can also use "
        "measured DCM support margin to recover balance. Python owns experiment "
        "selection and reporting; every cursor update, target sample, solve, state "
        "transition, and diagnostic tick remains in Rust.",
        "",
        "## Result",
        "",
        "**The balance-phase mechanism is not promoted.** A smooth capture-margin "
        "rate improves prefix and tracking relative to abrupt engagement, but no "
        "case admits touchdown or keeps the sustained transfer upright. The 800-tick "
        "extension diverges rather than converging.",
        "",
        f"Inactive-path parity is **{'PASS' if inactive_exact else 'FAIL'}** across "
        f"`{len(shared)}` shared non-timing arrays (NaNs compared in place). Defaults "
        "remain behaviorally unchanged.",
        "",
        "## Consumer audit",
        "",
        "The Rust batch path uses `reference_tick`, `reference_next_tick`, and one "
        "fraction for root, CoM, four endpoint jets, target activation, contact intent, "
        "precontact lookahead, prospective landing anchor, and future root reach. The "
        "DCM controller consumes the already time-warped CoM position/velocity jet. "
        "No remaining floating-WBC reference consumer was found indexing physical "
        "output tick after R39.",
        "",
        "The earliest R39 symptoms precede touchdown retiming: root error crosses "
        "5 cm at physical tick 197, the first liftoff is tick 199, measured DCM margin "
        "turns negative at tick 244, joint speed reaches 7.9 rad/s at tick 282, while "
        "touchdown geometry does not limit phase until tick 348. This motivated a "
        "separate measured-margin phase signal.",
        "",
        "## Controller/reference matrix",
        "",
        "| case | ticks | clean prefix | contingency/rejected | final source tick | min rate | root RMS | stance RMS | swing RMS | max rotation | DCM RMS |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for label, key in labels:
        item = metrics[key]
        dcm_rms = item["dcm_balance"].get("tracking_rms_m")
        dcm_cell = "—" if dcm_rms is None else f"{dcm_rms * 100:.2f} cm"
        lines.append(
            f"| {label} | {item['ticks']} | {item['nominal_prefix']['ticks']} | "
            f"{non_nominal(item)} | {phase(item, 'final_source_tick', item['ticks'] - 1):.3f} | "
            f"{phase(item, 'applied_rate_minimum', 1.0):.3f} | "
            f"{item['root_tracking_rms_m'] * 100:.2f} cm | "
            f"{item['stance_foot_tracking_rms_m'] * 100:.2f} cm | "
            f"{item['swing_foot_tracking_rms_m'] * 100:.2f} cm | "
            f"{np.rad2deg(item['maximum_root_rotation_rad']):.1f}° | "
            f"{dcm_cell} |"
        )

    lines += [
        "",
        "## Causal findings",
        "",
        "- Moving whole-body posture from Preference to Intent makes root attitude "
        "fail earlier. Moving pelvis-horizontal intent down to Preference is worse "
        "again. The strict-priority conflict is real, but those reversals are not the "
        "solution.",
        "- Reducing the spatial retarget from 0.35× to 0.10× cuts DCM RMS and swing "
        "error, but does not change the failure class. A Viability upper-body task "
        "creates 64 rejected solves and is rejected outright.",
        "- Requiring +2 cm DCM margin for full rate is geometrically impossible after "
        "liftoff: the G1 sole half-width is 2.75 cm and the configured erosion is 1 cm, "
        "leaving at most 1.75 cm. The policy defaults therefore taper only outside "
        "support (full at 0, hold at -2 cm).",
        "- Immediate rate engagement creates a 56.1 s⁻¹ phase-acceleration impulse "
        "through the exact chain rule and cuts the clean prefix to 268 ticks. A "
        "50-tick engagement bounds that term to 4.0 s⁻¹ and restores a 479-tick prefix.",
        "- Adding a 0.5× floor reaches authored touchdown tick 428 and extends the "
        "clean prefix to 490 ticks, versus 482 for R39. Swing RMS falls from 81.2 cm "
        "to 46.3 cm and non-nominal ticks from 118 to 110.",
        "- That apparent progress is not physical touchdown progress: at the contact "
        "edge the minimum sole error is 0.639 m and minimum whole-patch tangential "
        "speed is 6.044 m/s. Both remain far outside the immutable 0.025 m / 0.20 m/s "
        "admission envelope.",
        "",
        "## Failure over execution time",
        "",
        "| trace | first 7.9 rad/s | first 5° root | first contingency | delayed touchdown ticks | minimum landing error | minimum tangential speed |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for label, key in (
        ("R39", "r39"),
        ("smooth outside taper", "balance_smooth"),
        ("0.5× floor / 600", "balance_floor"),
        ("0.5× floor / 800", "balance_floor_800"),
    ):
        item = metrics[key]
        trace = traces[key]
        joint_speed = np.max(np.abs(trace["v"]), axis=1)
        root_angle = 2.0 * np.arccos(
            np.abs(trace["root_quaternion_wxyz"][:, 0]).clip(0.0, 1.0)
        )
        viability = item["touchdown_viability"]
        if viability["observed_target_ticks"]:
            landing_error = f"{viability['position_error_minimum_m']:.3f} m"
            tangent_speed = f"{viability['tangential_speed_minimum_mps']:.3f} m/s"
        else:
            landing_error = "—"
            tangent_speed = "—"
        lines.append(
            f"| {label} | {first_tick(joint_speed > 7.9)} | "
            f"{first_tick(root_angle > np.deg2rad(5.0))} | "
            f"{first_tick(np.isin(trace['status'], (4, 5)))} | "
            f"{item['delayed_touchdown_admission_ticks']} | "
            f"{landing_error} | {tangent_speed} |"
        )

    lines += [
        "",
        "The 800-tick extension retains 239 delayed-touchdown observations. Its "
        "minimum error never improves below 0.642 m and the trajectory grows to "
        "metre-scale tracking error, proving non-convergence rather than insufficient "
        "observation time.",
        "",
        "## CPU, jitter, memory, and solver work",
        "",
        "| case | p50 | p99 | jitter p99 | thread CPU | RSS Δ | GC | pseudoinverses/tick mean |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for label, key in (
        ("R39", "r39"),
        ("smooth outside taper", "balance_smooth"),
        ("0.5× floor / 600", "balance_floor"),
        ("0.5× floor / 800", "balance_floor_800"),
    ):
        item = metrics[key]
        runtime = item["runtime"]
        lines.append(
            f"| {label} | {item['latency_us']['p50']:.1f} µs | "
            f"{item['latency_us']['p99']:.1f} µs | "
            f"{item['jitter_abs_delta_us']['p99']:.1f} µs | "
            f"{runtime['thread_cpu_ns'] / 1e6:.1f} ms | "
            f"{runtime['rss_delta_bytes'] / 1024:.1f} KiB | "
            f"{runtime['python_gc_delta']['collections']} | "
            f"{item['solver_work']['task_pseudoinverse_calls']['mean']:.2f} |"
        )

    lines += [
        "",
        "The optional margin policy is scalar and allocation-free, but these full "
        "tails are deadline-red because contingency solves dominate. Timing does not "
        "justify promotion while the behavior gate fails.",
        "",
        "## Decision",
        "",
        "Keep the pure support-margin phase-rate primitive and explicit root-horizontal "
        "priority as opt-in research controls; keep both disabled in production "
        "defaults. Do not relax contact thresholds, do not call a pre-edge hold a "
        "success, and do not promote the CMU/G1 controller. The next controller work "
        "must make the root/CoM/contact reference dynamically feasible—likely through "
        "a robot-native footstep/DCM planner or a reference implementation that emits "
        "consistent pelvis, CoM, and contact-wrench intent—before phase policy is "
        "revisited.",
        "",
        "## Next evaluation boundary",
        "",
        "Reference generators are evaluated open-loop before another controller policy "
        "is tried: identical initial robot state, footsteps, contact timing, sole "
        "geometry, and command; no tracked state, WBC status, solver output, or physics "
        "integration. The first contract checks contact continuity, reach, positive "
        "normal specific force, friction demand, and zero-angular-momentum CoP/DCM "
        "support margins. The current authored CMU reference already fails this layer, "
        "including 85.37 m/s² peak CoM acceleration and an 8.276 peak friction ratio. "
        "See `benchmarks/results/g1-reference-contract-r40/"
        "OPEN_LOOP_REFERENCE_CONTRACT.md`.",
        "",
        f"The source corpus still reports maximum retargeted endpoint speed "
        f"`{motion['r39']['maximum_floating_target_speed_m_s']:.3f} m/s`. Independent "
        "PlaCo, Pinocchio, and upstream Upkie comparisons remain in "
        "`benchmarks/results/reference-r38/REFERENCE_COMPARISON.md`; none exposes an "
        "equivalent measured capture-margin cadence law, so no unsupported reference-"
        "parity claim is made here.",
        "",
    ]

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines))
    print(output)


if __name__ == "__main__":
    main()
