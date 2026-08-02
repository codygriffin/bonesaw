#!/usr/bin/env python3
"""Score authored G1 walking references without a policy or physics rollout."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


DT = 0.005
GRAVITY = 9.81
DEFAULT_CASES = {
    "synthetic step": "g1-synthetic-step-r39-retiming-disabled-smoke",
    "CMU 0.35x + DCM preview": "g1-transfer-r39-retiming-release100-guard020",
    "CMU 0.35x + static support preview": "g1-transfer-r40-support-preview-retiming",
    "CMU 0.10x + DCM preview": "g1-transfer-r40-scale010",
}
STANDALONE_CASES = {
    "PlaCo WPG · matched first step": "g1-placo-wpg-r41",
    "Bonesaw LIPM · matched hard boundary": "g1-bonesaw-lipm-r42",
    "Bonesaw LIPM · robot-reachable": "g1-bonesaw-lipm-r43",
    "Bonesaw LIPM · morphology-rooted": "g1-bonesaw-lipm-r44",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", default="benchmarks/results")
    parser.add_argument(
        "--output", default="benchmarks/results/g1-reference-contract-r44"
    )
    return parser.parse_args()


def convex_hull(points: np.ndarray) -> np.ndarray:
    unique = sorted({(float(point[0]), float(point[1])) for point in points})
    if len(unique) <= 1:
        return np.asarray(unique, dtype=np.float64)

    def cross(origin: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1], dtype=np.float64)


def signed_polygon_margin(hull: np.ndarray, point: np.ndarray) -> float:
    if len(hull) < 3:
        return float("-inf")
    margins = []
    for index, start in enumerate(hull):
        edge = hull[(index + 1) % len(hull)] - start
        offset = point - start
        cross_z = edge[0] * offset[1] - edge[1] * offset[0]
        margins.append(cross_z / np.linalg.norm(edge))
    return float(min(margins))


def sole_vertices(
    centers: np.ndarray, half_length: float, half_width: float
) -> np.ndarray:
    offsets = np.asarray(
        [
            [-half_length, -half_width],
            [-half_length, half_width],
            [half_length, -half_width],
            [half_length, half_width],
        ]
    )
    return (centers[:, None, :] + offsets[None, :, :]).reshape(-1, 2)


def quantile(values: np.ndarray, probability: float) -> float:
    return float(np.quantile(values, probability)) if len(values) else 0.0


def touchdown_metrics(
    positions: np.ndarray, velocities: np.ndarray, stance: np.ndarray
) -> dict[str, float | int]:
    tangential: list[float] = []
    normal: list[float] = []
    discontinuity: list[float] = []
    for foot in range(2):
        edges = np.flatnonzero(stance[1:, foot] & ~stance[:-1, foot]) + 1
        for edge in edges:
            tangential.append(float(np.linalg.norm(velocities[edge - 1, foot, :2])))
            normal.append(float(abs(velocities[edge - 1, foot, 2])))
            discontinuity.append(
                float(np.linalg.norm(positions[edge, foot] - positions[edge - 1, foot]))
            )
    return {
        "count": len(tangential),
        "preedge_tangential_speed_maximum_mps": max(tangential, default=0.0),
        "preedge_normal_speed_maximum_mps": max(normal, default=0.0),
        "position_discontinuity_maximum_m": max(discontinuity, default=0.0),
    }


def score_reference(directory: Path) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    standalone = directory / "reference-inputs.npz"
    if standalone.is_file():
        payload = json.loads((directory / "reference-metadata.json").read_text())
        motion = payload["motion"]
        raw = np.load(standalone)
    else:
        payload = json.loads((directory / "floating-walk-metrics.json").read_text())
        motion = payload["motion"]
        raw = np.load(directory / "floating-walk-raw.npz")

    # Deliberately authored-only: do not read effective/tracked state, controller
    # status, forces, residuals, solver work, or timing from the artifact.
    root = raw["root_targets"]
    com = raw["center_of_mass_targets"]
    com_velocity = raw["center_of_mass_target_velocities"]
    com_acceleration = raw["center_of_mass_target_accelerations"]
    feet = raw["target_positions"][:, :2]
    foot_velocity = raw["target_velocities"][:, :2]
    foot_acceleration = raw["target_accelerations"][:, :2]
    stance = raw["reference_stance"].astype(bool)

    center_x = float(motion["contact_patch_center_x_m"])
    half_length = float(motion["contact_patch_half_length_m"])
    half_width = float(motion["contact_patch_half_width_m"])
    patch_z = float(motion["contact_patch_z_m"])
    support_margin = float(motion.get("support_margin_m", 0.01))
    eroded_length = half_length - support_margin
    eroded_width = half_width - support_margin
    if eroded_length <= 0.0 or eroded_width <= 0.0:
        raise ValueError("support margin erases the sole patch")

    ticks = len(root)
    cop = np.full((ticks, 2), np.nan)
    dcm = np.full((ticks, 2), np.nan)
    cop_margin = np.full(ticks, -np.inf)
    dcm_margin = np.full(ticks, -np.inf)
    friction_ratio = np.full(ticks, np.inf)
    normal_specific_force = GRAVITY + com_acceleration[:, 2]
    support_height = np.full(ticks, np.nan)
    for tick in range(ticks):
        active = stance[tick]
        if not np.any(active):
            continue
        plane_z = float(np.mean(feet[tick, active, 2] + patch_z))
        height = max(float(com[tick, 2] - plane_z), 1.0e-6)
        support_height[tick] = height
        centers = feet[tick, active, :2].copy()
        centers[:, 0] += center_x
        hull = convex_hull(sole_vertices(centers, eroded_length, eroded_width))
        omega = np.sqrt(GRAVITY / height)
        dcm[tick] = com[tick, :2] + com_velocity[tick, :2] / omega
        dcm_margin[tick] = signed_polygon_margin(hull, dcm[tick])
        if normal_specific_force[tick] > 1.0e-9:
            cop[tick] = com[tick, :2] - (
                height * com_acceleration[tick, :2] / normal_specific_force[tick]
            )
            cop_margin[tick] = signed_polygon_margin(hull, cop[tick])
            friction_ratio[tick] = (
                np.linalg.norm(com_acceleration[tick, :2])
                / normal_specific_force[tick]
            )

    stance_mask = np.broadcast_to(stance[:, :, None], foot_velocity.shape)
    stance_velocity = np.where(stance_mask, foot_velocity, 0.0)
    stance_acceleration = np.where(stance_mask, foot_acceleration, 0.0)
    com_jerk = np.diff(com_acceleration, axis=0) / DT
    root_velocity = np.gradient(root, DT, axis=0, edge_order=2)
    root_acceleration = np.gradient(root_velocity, DT, axis=0, edge_order=2)
    root_jerk = np.diff(root_acceleration, axis=0) / DT
    reach = np.linalg.norm(feet - root[:, None, :], axis=2)
    touchdown = touchdown_metrics(feet, foot_velocity, stance)

    finite_cop = np.isfinite(cop_margin)
    finite_dcm = np.isfinite(dcm_margin)
    metrics: dict[str, Any] = {
        "ticks": ticks,
        "inputs": [
            "root_targets",
            "center_of_mass_targets/velocities/accelerations",
            "target_positions/velocities/accelerations",
            "reference_stance",
            "sole geometry",
        ],
        "forbidden_rollout_inputs": [
            "effective references",
            "tracked state",
            "controller status",
            "contact forces",
            "solver residual/work",
            "latency",
        ],
        "flight_ticks": int(np.sum(~np.any(stance, axis=1))),
        "double_support_fraction": float(np.mean(np.all(stance, axis=1))),
        "stance_foot_speed_maximum_mps": float(np.max(np.abs(stance_velocity))),
        "stance_foot_acceleration_maximum_mps2": float(
            np.max(np.abs(stance_acceleration))
        ),
        "com_acceleration_norm_p95_mps2": quantile(
            np.linalg.norm(com_acceleration, axis=1), 0.95
        ),
        "com_acceleration_norm_maximum_mps2": float(
            np.max(np.linalg.norm(com_acceleration, axis=1))
        ),
        "com_jerk_norm_p95_mps3": quantile(np.linalg.norm(com_jerk, axis=1), 0.95),
        "com_jerk_norm_maximum_mps3": float(
            np.max(np.linalg.norm(com_jerk, axis=1))
        ),
        "root_jerk_norm_maximum_mps3": float(
            np.max(np.linalg.norm(root_jerk, axis=1))
        ),
        "nonpositive_normal_specific_force_ticks": int(
            np.sum(normal_specific_force <= 1.0e-9)
        ),
        "friction_ratio_p95": quantile(friction_ratio[np.isfinite(friction_ratio)], 0.95),
        "friction_ratio_maximum": float(
            np.max(friction_ratio[np.isfinite(friction_ratio)])
        ),
        "cop_support_margin_minimum_m": float(np.min(cop_margin[finite_cop])),
        "cop_support_margin_minimum_tick": int(np.nanargmin(cop_margin)),
        "cop_outside_support_ticks": np.flatnonzero(
            cop_margin < -1.0e-9
        ).astype(int).tolist(),
        "cop_outside_support_fraction": float(np.mean(cop_margin[finite_cop] < -1.0e-9)),
        "dcm_support_margin_minimum_m": float(np.min(dcm_margin[finite_dcm])),
        "dcm_support_margin_minimum_tick": int(np.nanargmin(dcm_margin)),
        "dcm_outside_support_fraction": float(np.mean(dcm_margin[finite_dcm] < -1.0e-9)),
        "root_to_foot_reach_maximum_m": float(np.max(reach)),
        "com_root_offset_maximum_m": float(np.max(np.linalg.norm(com - root, axis=1))),
        "touchdown": touchdown,
    }
    checks = {
        "no_flight": metrics["flight_ticks"] == 0,
        "positive_normal_specific_force": metrics["nonpositive_normal_specific_force_ticks"] == 0,
        "zero_momentum_cop_inside_eroded_support": metrics["cop_support_margin_minimum_m"] >= -1.0e-6,
        "friction_ratio_le_1": metrics["friction_ratio_maximum"] <= 1.0,
        "com_acceleration_le_25_mps2": metrics["com_acceleration_norm_maximum_mps2"] <= 25.0,
        "root_to_foot_reach_le_0_9m": metrics["root_to_foot_reach_maximum_m"] <= 0.9,
        "touchdown_tangential_speed_le_0_2mps": touchdown["preedge_tangential_speed_maximum_mps"] <= 0.2,
        "touchdown_normal_speed_le_0_2mps": touchdown["preedge_normal_speed_maximum_mps"] <= 0.2,
        "touchdown_position_continuity_le_0_025m": touchdown["position_discontinuity_maximum_m"] <= 0.025,
    }
    metrics["checks"] = checks
    metrics["pass"] = all(checks.values())
    if standalone.is_file():
        metrics["generator"] = {
            key: payload[key]
            for key in (
                "implementation",
                "placo_version",
                "placo_source_revision",
                "placo_license",
                "generator_family",
                "bonesaw_version",
                "bonesaw_license",
                "planner_source",
                "policy_or_physics_rollout",
                "ik_or_wbc_solve",
                "exact_repeat",
                "runtime",
                "plan",
            )
            if key in payload
        }
    traces = {
        "cop_xy": cop,
        "dcm_xy": dcm,
        "cop_support_margin_m": cop_margin,
        "dcm_support_margin_m": dcm_margin,
        "friction_ratio": friction_ratio,
        "normal_specific_force_mps2": normal_specific_force,
        "support_height_m": support_height,
    }
    return metrics, traces


def render_report(results: dict[str, dict[str, Any]]) -> str:
    lines = [
        "# Bonesaw R44 open-loop G1 reference contract",
        "",
        "This eval intentionally runs **no controller policy and no physics rollout**. "
        "It scores only authored root, CoM, foot, and contact-schedule arrays. Tracked "
        "state, effective/retimed references, solve status, contact forces, residuals, "
        "solver work, and latency are forbidden inputs.",
        "",
        "The centroidal check is a conditional necessary—not sufficient—certificate. "
        "Under zero angular-momentum rate it asks whether the authored CoM acceleration can "
        "produce a CoP inside the 1 cm-eroded finite sole hull with positive normal force "
        "and friction ratio at most 1.0.",
        "",
        "| reference | gate | CoM a max | CoM jerk max | CoP min margin | CoP outside | friction max | DCM min margin | reach max | touchdown vxy/vz |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for label, item in results.items():
        touchdown = item["touchdown"]
        lines.append(
            f"| {label} | {'PASS' if item['pass'] else 'FAIL'} | "
            f"{item['com_acceleration_norm_maximum_mps2']:.2f} m/s² | "
            f"{item['com_jerk_norm_maximum_mps3']:.1f} m/s³ | "
            f"{item['cop_support_margin_minimum_m'] * 100:.2f} cm | "
            f"{item['cop_outside_support_fraction'] * 100:.2f}% | "
            f"{item['friction_ratio_maximum']:.3f} | "
            f"{item['dcm_support_margin_minimum_m'] * 100:.2f} cm | "
            f"{item['root_to_foot_reach_maximum_m']:.3f} m | "
            f"{touchdown['preedge_tangential_speed_maximum_mps']:.3f} / "
            f"{touchdown['preedge_normal_speed_maximum_mps']:.3f} m/s |"
        )
    lines += [
        "",
        "## Interpretation",
        "",
        "The synthetic step is a deliberately simple control and also fails the open-loop "
        "balance contract even though its closed-loop tracking regression is green; it "
        "holds the CoM between the feet during single support. The CMU-derived references fail before "
        "Bonesaw chooses a task priority or integrates one state tick: finite-difference "
        "contact edges create extreme CoM acceleration/jerk, negative normal-force "
        "requests, and CoP/friction demands outside the support contract. The static "
        "support-preview variant is especially discontinuous. Scaling horizontal motion "
        "to 0.10× reduces the demand but does not change the failure class.",
        "",
        "This locates the next architectural boundary: compare reference generators "
        "open-loop under the same initial state, footsteps, timing, sole geometry, and "
        "command. Only references that pass this contract should enter the WBC tracking "
        "and closed-loop physics suites.",
        "",
    ]
    placo_label = "PlaCo WPG · matched first step"
    if placo_label in results:
        placo_result = results[placo_label]
        cmu = results["CMU 0.35x + DCM preview"]
        outside = placo_result["cop_outside_support_ticks"]
        generator = placo_result["generator"]
        runtime = generator["runtime"]
        lines += [
            "## Independent PlaCo WPG result",
            "",
            "PlaCo 0.9.23 plans the same initial CoM, initial sole poses, first "
            "touchdown pose, 199-tick starting double support, and 229-tick first "
            "swing. Only its `WalkPatternGenerator` runs: no `WalkTasks`, IK, WBC, "
            "state integration, or simulator is instantiated.",
            "",
            f"Relative to the CMU-derived reference, PlaCo reduces maximum CoM "
            f"acceleration by `{cmu['com_acceleration_norm_maximum_mps2'] / placo_result['com_acceleration_norm_maximum_mps2']:.1f}×`, "
            f"maximum jerk by `{cmu['com_jerk_norm_maximum_mps3'] / placo_result['com_jerk_norm_maximum_mps3']:.1f}×`, "
            f"and maximum friction ratio by `{cmu['friction_ratio_maximum'] / placo_result['friction_ratio_maximum']:.1f}×`. "
            "Its touchdown position and velocity gates pass with wide margin.",
            "",
            f"The strict reference gate still rejects it: the zero-angular-momentum "
            f"CoP is outside the newly active right sole on ticks "
            f"`{outside}`. Tick 199 misses by "
            f"`{-placo_result['cop_support_margin_minimum_m'] * 100:.3f} cm`; tick "
            "200 is only 0.014 mm outside, then all remaining CoP samples are inside. "
            "This is a support-boundary defect, not a WBC failure and not a reason to "
            "weaken the gate.",
            "",
            "PlaCo's public WPG provides CoM and trunk orientation but not pelvis "
            "translation. The comparison derives pelvis translation from PlaCo CoM "
            "minus the identical initial G1 CoM-to-root offset; that adapter affects "
            "only the reach check, not the centroidal certificate.",
            "",
            f"The standalone generator is bitwise repeatable: "
            f"`{generator['exact_repeat']}`. One isolated run records "
            f"`{runtime['plan_ns'] / 1e6:.1f} ms` planning and "
            f"`{runtime['sample_ns'] / 1e6:.1f} ms` to sample 600 ticks. Peak "
            f"process RSS is `{runtime['maximum_rss_kib'] / 1024:.1f} MiB`; it "
            "includes Python, PlaCo, Pinocchio, model import, and planner setup and "
            "is not an incremental planner allocation claim.",
            "",
            "Sources: https://placo.readthedocs.io/en/stable/placo/"
            "placo__humanoid.html and https://github.com/Rhoban/placo at pinned "
            "revision e6c288604639d67b979a16cb2ad26913413c8e3a.",
            "",
        ]
    bonesaw_label = (
        "Bonesaw LIPM · robot-reachable"
        if "Bonesaw LIPM · robot-reachable" in results
        else "Bonesaw LIPM · matched hard boundary"
    )
    if bonesaw_label in results:
        bonesaw_result = results[bonesaw_label]
        generator = bonesaw_result["generator"]
        plan = generator["plan"]
        runtime = generator["runtime"]
        placo_result = results.get(placo_label)
        checks_passed = sum(bonesaw_result["checks"].values())
        lines += [
            "## Rust-native support-boundary result",
            "",
            "Bonesaw solves two exact constant-CoP LIPM arcs over the matched "
            "opening double-support interval. It searches a fixed switch-time grid "
            "without heap storage, solves both CoP positions analytically, and admits "
            "a candidate only when the first CoP is inside the opening support hull "
            "and the second and terminal CoPs are inside the future right-sole hull.",
            "",
            f"The selected split is `{plan['switch_ratio']:.3f}` of the "
            f"`{plan['duration_seconds']:.3f} s` boundary interval. Its declared CoP "
            f"margins are `{plan['first_cop_margin_m'] * 100:.3f} cm` then "
            f"`{plan['second_cop_margin_m'] * 100:.3f} cm`, with "
            f"`{plan['terminal_cop_margin_m'] * 100:.3f} cm` at the equilibrium hold.",
            "",
            *(
                [
                    f"The official-G1 landing reach is reduced from "
                    f"`{plan['authored_root_to_touchdown_reach_m']:.3f}` to "
                    f"`{plan['applied_root_to_touchdown_reach_m']:.3f} m` without "
                    "changing landing height; the retarget is authored before any "
                    "controller executes.",
                    "",
                ]
                if "applied_root_to_touchdown_reach_m" in plan
                else []
            ),
            f"The standalone authored trace {'passes' if bonesaw_result['pass'] else 'fails'} "
            f"`{checks_passed}/9` strict gates. Planning records "
            f"`{runtime['plan_ns'] / 1e3:.1f} µs`; 600 closed-form samples record "
            f"`{runtime['sample_ns'] / 1e3:.1f} µs`. The measured planner and sample "
            f"regions make `{runtime['plan_allocation_calls']}` and "
            f"`{runtime['sample_allocation_calls']}` allocation calls respectively. "
            "JSON transport and Python NPZ packaging are outside those regions.",
            "",
            f"The generator is bitwise repeatable: `{generator['exact_repeat']}`. "
            "It invokes no WBC, IK, policy, state integration, or physics rollout.",
            "",
        ]
        if placo_result is not None:
            lines += [
                f"Against PlaCo, the Rust reference changes maximum CoM acceleration "
                f"from `{placo_result['com_acceleration_norm_maximum_mps2']:.3f}` to "
                f"`{bonesaw_result['com_acceleration_norm_maximum_mps2']:.3f} m/s²` and "
                f"the minimum CoP margin from "
                f"`{placo_result['cop_support_margin_minimum_m'] * 100:.3f}` to "
                f"`{bonesaw_result['cop_support_margin_minimum_m'] * 100:.3f} cm`. "
                "This is an intentionally conservative first-step boundary primitive, "
                "not yet a multi-step walking-pattern replacement.",
                "",
            ]
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    root = Path(args.results_root)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    results: dict[str, dict[str, Any]] = {}
    traces: dict[str, np.ndarray] = {}
    for label, directory_name in DEFAULT_CASES.items():
        metrics, case_traces = score_reference(root / directory_name)
        results[label] = metrics
        slug = directory_name.replace("g1-", "")
        traces.update({f"{slug}_{key}": value for key, value in case_traces.items()})
    for label, directory_name in STANDALONE_CASES.items():
        directory = root / directory_name
        if not (directory / "reference-inputs.npz").is_file():
            continue
        metrics, case_traces = score_reference(directory)
        results[label] = metrics
        slug = directory_name.replace("g1-", "")
        traces.update({f"{slug}_{key}": value for key, value in case_traces.items()})
    (output / "reference-contract-metrics.json").write_text(
        json.dumps({"schema": 1, "results": results}, indent=2) + "\n"
    )
    np.savez_compressed(output / "reference-contract-raw.npz", **traces)
    (output / "OPEN_LOOP_REFERENCE_CONTRACT.md").write_text(render_report(results))
    print(output / "OPEN_LOOP_REFERENCE_CONTRACT.md")


if __name__ == "__main__":
    main()
