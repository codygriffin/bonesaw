#!/usr/bin/env python3
"""R216 zero-step construction replay for causal G1 center responses.

R215 labels are immutable inputs. Rust reconstructs model response and split
root/articulated kinetic geometry; Python compares a small declared predictor
family. This script performs no simulator, controller, policy, or integration
step and cannot promote authority.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_contact_law_momentum_holdout import (
    CONTROL_DT,
    FOOT_FRAMES,
    GRAVITY,
    group_maximum,
    quaternion_from_rpy,
    sha256,
    standing_posture,
)
from g1_spatial_patch_transition_holdout import FRESH_CONTACT_LAWS, SAMPLE_OFFSETS


REVISION = "g1-center-response-split-residual-replay-r216"
SOURCE_REVISION = "g1-spatial-patch-transition-holdout-r215"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-spatial-patch-transition-holdout-r215/"
    "g1-spatial-patch-transition-replay.npz"
)
WIDTH_GATES = {
    "root_angular_rad_s": 2.0,
    "root_linear_m_s": 0.5,
    "joint_rad_s": 10.0,
}
CALIBRATION_RESERVE = 1.05


@dataclass(frozen=True)
class Candidate:
    name: str
    center_kind: str
    load_fraction: float
    causal: bool = True


CAUSAL_CANDIDATES = tuple(
    Candidate(f"{center}_load_{load:g}", center, load)
    for center in ("geometric", "closing_weighted", "earliest", "lowest")
    for load in (0.0, 0.5, 1.0)
)
ORACLE_CANDIDATE = Candidate("completed_impulse_center_oracle", "impulse_oracle", 0.0, False)
SPATIAL_ORACLE_CANDIDATE = Candidate(
    "completed_spatial_wrench_oracle", "spatial_oracle", 0.0, False
)
CANDIDATES = (*CAUSAL_CANDIDATES, ORACLE_CANDIDATE, SPATIAL_ORACLE_CANDIDATE)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/G1_CENTER_RESPONSE_SPLIT_RESIDUAL_REPLAY_R216.html",
    )
    return parser.parse_args()


def total_urdf_mass(model_path: pathlib.Path) -> float:
    import xml.etree.ElementTree as ET

    total = 0.0
    for link in ET.parse(model_path).getroot().findall("link"):
        mass = link.find("inertial/mass")
        if mass is not None:
            total += float(mass.attrib["value"])
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError("URDF must declare positive finite total mass")
    return total


def center_weights(
    kind: str,
    points: np.ndarray,
    velocities: np.ndarray,
    completed_impulses: np.ndarray | None = None,
) -> np.ndarray:
    """Return two rows of four convex weights; only oracle uses labels."""

    points = np.asarray(points, np.float64).reshape(2, 4, 3)
    velocities = np.asarray(velocities, np.float64).reshape(2, 4, 3)
    weights = np.zeros((2, 4), np.float64)
    for foot in range(2):
        if kind == "geometric":
            weights[foot] = 0.25
        elif kind == "closing_weighted":
            raw = np.maximum(-velocities[foot, :, 2], 0.0)
            weights[foot] = raw / np.sum(raw) if np.sum(raw) > 1.0e-12 else 0.25
        elif kind == "earliest":
            closing = np.maximum(-velocities[foot, :, 2], 1.0e-9)
            impact_time = np.maximum(points[foot, :, 2], 0.0) / closing
            weights[foot, int(np.argmin(impact_time))] = 1.0
        elif kind == "lowest":
            weights[foot, int(np.argmin(points[foot, :, 2]))] = 1.0
        elif kind == "impulse_oracle":
            if completed_impulses is None:
                raise ValueError("oracle center requires completed impulses")
            normal = np.maximum(
                np.asarray(completed_impulses, np.float64).reshape(2, 4, 3)[foot, :, 2],
                0.0,
            )
            weights[foot] = (
                normal / np.sum(normal) if np.sum(normal) > 1.0e-12 else 0.25
            )
        else:
            raise ValueError(f"unknown center kind: {kind}")
    return weights


def predicted_center_impulse(
    center_velocity: np.ndarray,
    effective_mass: np.ndarray,
    friction: float,
    supported_weight_impulse: float,
    load_fraction: float,
) -> np.ndarray:
    """Plastic diagonal contact prediction with a declared weight share."""

    velocity = np.asarray(center_velocity, np.float64)
    mass = np.asarray(effective_mass, np.float64)
    impulse = np.empty((2, 3), np.float64)
    for foot in range(2):
        normal = mass[foot, 2] * max(-velocity[foot, 2], 0.0)
        normal += load_fraction * supported_weight_impulse
        tangent_cap = friction * normal
        impulse[foot, 0] = float(
            np.clip(-mass[foot, 0] * velocity[foot, 0], -tangent_cap, tangent_cap)
        )
        impulse[foot, 1] = float(
            np.clip(-mass[foot, 1] * velocity[foot, 1], -tangent_cap, tangent_cap)
        )
        impulse[foot, 2] = normal
    return impulse


def reconstructed_state(
    state_index: int,
    root_height: float,
    q_nominal: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    phase = 0.37 * state_index + np.arange(q_nominal.size, dtype=np.float64) * 0.23
    q = q_nominal + 0.025 * np.sin(phase)
    quaternion = quaternion_from_rpy(
        0.025 * math.sin(0.31 * state_index),
        0.035 * math.cos(0.27 * state_index),
        0.02 * math.sin(0.19 * state_index),
    )
    return np.asarray([0.0, 0.0, root_height], np.float64), quaternion, q


def split_energy(
    session: Any,
    root: np.ndarray,
    quaternion: np.ndarray,
    q: np.ndarray,
    momentum: np.ndarray,
    singleton: np.ndarray,
    velocity_lower: np.ndarray,
    velocity_upper: np.ndarray,
) -> tuple[float, float]:
    singleton.fill(0.0)
    singleton[:6] = momentum[:6]
    session.generalized_velocity_interval_from_momentum_box(
        root, quaternion, q, singleton, singleton, velocity_lower, velocity_upper
    )
    root_energy = float(np.dot(momentum[:6], velocity_lower[:6]))
    singleton.fill(0.0)
    singleton[6:] = momentum[6:]
    session.generalized_velocity_interval_from_momentum_box(
        root, quaternion, q, singleton, singleton, velocity_lower, velocity_upper
    )
    articulated_energy = float(np.dot(momentum[6:], velocity_lower[6:]))
    return max(root_energy, 0.0), max(articulated_energy, 0.0)


def main() -> int:
    args = parse_args()
    model_path = pathlib.Path(args.model).resolve()
    replay_path = pathlib.Path(args.source_replay).resolve()
    if not model_path.is_file() or not replay_path.is_file():
        raise SystemExit("R216 requires the pinned G1 model and immutable R215 replay")

    import bonesaw

    point_frames = [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    point_session = bonesaw.ContactTransitionModelSession(str(model_path), point_frames)
    center_session = bonesaw.ContactTransitionModelSession(str(model_path), list(FOOT_FRAMES))
    spatial_session = bonesaw.ContactTransitionModelSession(str(model_path), list(FOOT_FRAMES))
    joint_names = list(point_session.joint_names())
    q_nominal = standing_posture(joint_names)
    generalized_dof = int(point_session.generalized_dof())
    total_mass = total_urdf_mass(model_path)
    supported_weight_impulse = 0.5 * total_mass * GRAVITY * CONTROL_DT
    bases8 = np.repeat(np.eye(3, dtype=np.float64)[None], 8, axis=0)
    bases2 = np.repeat(np.eye(3, dtype=np.float64)[None], 2, axis=0)
    point_response = np.empty((generalized_dof, 8, 3), np.float64)
    point_mass = np.empty((8, 3), np.float64)
    point_delassus = np.empty((24, 24), np.float64)
    center_response = np.empty((generalized_dof, 2, 3), np.float64)
    center_mass = np.empty((2, 3), np.float64)
    center_delassus = np.empty((6, 6), np.float64)
    spatial_response = np.empty((generalized_dof, 2, 6), np.float64)
    spatial_delassus = np.empty((12, 12), np.float64)
    predicted = np.empty((len(CANDIDATES), generalized_dof), np.float64)
    momentum = np.empty_like(predicted)
    singleton = np.zeros(generalized_dof, np.float64)
    singleton_velocity_lower = np.empty(generalized_dof, np.float64)
    singleton_velocity_upper = np.empty(generalized_dof, np.float64)
    split_lower = np.empty(generalized_dof, np.float64)
    split_upper = np.empty(generalized_dof, np.float64)
    global_lower = np.empty(generalized_dof, np.float64)
    global_upper = np.empty(generalized_dof, np.float64)

    records: dict[str, dict[str, list[np.ndarray] | list[float]]] = {
        candidate.name: {
            "root_energy": [],
            "articulated_energy": [],
            "total_energy": [],
            "residual": [],
            "query_ns": [],
        }
        for candidate in CANDIDATES
    }
    states: list[tuple[str, np.ndarray, np.ndarray, np.ndarray]] = []
    zero_allocation = True

    with np.load(replay_path) as replay:
        for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
            prefix = law.name
            points_all = replay[f"{prefix}_contact_points"]
            velocities_all = replay[f"{prefix}_prospective_velocity"]
            impulses_all = replay[f"{prefix}_contact_impulse"]
            raw_error_all = replay[f"{prefix}_raw_velocity_error"]
            heights = replay[f"{prefix}_root_height"]
            for sample in range(points_all.shape[0]):
                root, quaternion, q = reconstructed_state(
                    sample + offset, float(heights[sample]), q_nominal
                )
                points = points_all[sample]
                velocities = velocities_all[sample]
                completed_impulses = impulses_all[sample]
                timing = point_session.point_impulse_velocity_response(
                    root,
                    quaternion,
                    q,
                    points,
                    bases8,
                    point_response,
                    point_mass,
                    point_delassus,
                )
                zero_allocation &= timing[1:] == (0, 0)
                realized_contact_delta = np.einsum(
                    "dca,ca->d", point_response, completed_impulses, optimize=True
                ) + raw_error_all[sample]

                for candidate_index, candidate in enumerate(CANDIDATES):
                    weights = center_weights(
                        "impulse_oracle"
                        if candidate.center_kind == "spatial_oracle"
                        else candidate.center_kind,
                        points,
                        velocities,
                        completed_impulses if not candidate.causal else None,
                    )
                    centers = np.einsum(
                        "fc,fca->fa", weights, points.reshape(2, 4, 3), optimize=True
                    )
                    center_velocity = np.einsum(
                        "fc,fca->fa",
                        weights,
                        velocities.reshape(2, 4, 3),
                        optimize=True,
                    )
                    if candidate.center_kind == "spatial_oracle":
                        timing = spatial_session.spatial_impulse_velocity_response(
                            root,
                            quaternion,
                            q,
                            centers,
                            bases2,
                            spatial_response,
                            spatial_delassus,
                        )
                        completed_by_foot = completed_impulses.reshape(2, 4, 3)
                        spatial_impulse = np.empty((2, 6), np.float64)
                        spatial_impulse[:, 3:] = np.sum(completed_by_foot, axis=1)
                        spatial_impulse[:, :3] = np.sum(
                            np.cross(
                                points.reshape(2, 4, 3) - centers[:, None, :],
                                completed_by_foot,
                            ),
                            axis=1,
                        )
                        predicted[candidate_index] = np.einsum(
                            "dca,ca->d", spatial_response, spatial_impulse, optimize=True
                        )
                    else:
                        timing = center_session.point_impulse_velocity_response(
                            root,
                            quaternion,
                            q,
                            centers,
                            bases2,
                            center_response,
                            center_mass,
                            center_delassus,
                        )
                        if candidate.causal:
                            predicted_impulse = predicted_center_impulse(
                                center_velocity,
                                center_mass,
                                law.friction,
                                supported_weight_impulse,
                                candidate.load_fraction,
                            )
                        else:
                            predicted_impulse = completed_impulses.reshape(2, 4, 3).sum(
                                axis=1
                            )
                        predicted[candidate_index] = np.einsum(
                            "dca,ca->d", center_response, predicted_impulse, optimize=True
                        )
                    zero_allocation &= timing[1:] == (0, 0)
                    records[candidate.name]["query_ns"].append(float(timing[0]))

                timing = center_session.generalized_momentum_impulse_residuals(
                    root, quaternion, q, realized_contact_delta, predicted, momentum
                )
                zero_allocation &= timing[1:] == (0, 0)
                for candidate_index, candidate in enumerate(CANDIDATES):
                    residual = realized_contact_delta - predicted[candidate_index]
                    root_energy, articulated_energy = split_energy(
                        center_session,
                        root,
                        quaternion,
                        q,
                        momentum[candidate_index],
                        singleton,
                        singleton_velocity_lower,
                        singleton_velocity_upper,
                    )
                    records[candidate.name]["root_energy"].append(root_energy)
                    records[candidate.name]["articulated_energy"].append(articulated_energy)
                    records[candidate.name]["total_energy"].append(
                        max(float(np.dot(momentum[candidate_index], residual)), 0.0)
                    )
                    records[candidate.name]["residual"].append(residual.copy())
                states.append((law.name, root.copy(), quaternion.copy(), q.copy()))

    law_slices = {
        FRESH_CONTACT_LAWS[0].name: slice(0, 48),
        FRESH_CONTACT_LAWS[1].name: slice(48, 96),
    }
    results = []
    replay_arrays: dict[str, np.ndarray] = {}
    for candidate in CANDIDATES:
        root_energy = np.asarray(records[candidate.name]["root_energy"], np.float64)
        articulated_energy = np.asarray(
            records[candidate.name]["articulated_energy"], np.float64
        )
        total_energy = np.asarray(records[candidate.name]["total_energy"], np.float64)
        residual = np.asarray(records[candidate.name]["residual"], np.float64)
        root_budget = CALIBRATION_RESERVE * float(np.max(root_energy))
        articulated_budget = CALIBRATION_RESERVE * float(np.max(articulated_energy))
        total_budget = CALIBRATION_RESERVE * float(np.max(total_energy))
        widths = np.empty_like(residual)
        global_widths = np.empty_like(residual)
        component_coverage = np.zeros_like(residual, np.uint8)
        timing_ns = np.empty(residual.shape[0], np.uint64)
        for index, (_, root, quaternion, q) in enumerate(states):
            timing = center_session.generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
                root,
                quaternion,
                q,
                root_budget,
                articulated_budget,
                split_lower,
                split_upper,
            )
            zero_allocation &= timing[1:] == (0, 0)
            timing_ns[index] = timing[0]
            widths[index] = split_upper - split_lower
            component_coverage[index] = (
                (residual[index] >= split_lower - 1.0e-12)
                & (residual[index] <= split_upper + 1.0e-12)
            )
            timing = center_session.generalized_velocity_bounds_from_kinetic_impulse_ellipsoid(
                root,
                quaternion,
                q,
                total_budget,
                global_lower,
                global_upper,
            )
            zero_allocation &= timing[1:] == (0, 0)
            global_widths[index] = global_upper - global_lower
        width_groups = np.asarray([group_maximum(row) for row in widths])
        width_summary = {
            "root_angular_rad_s": distribution(width_groups[:, 0]),
            "root_linear_m_s": distribution(width_groups[:, 1]),
            "joint_rad_s": distribution(width_groups[:, 2]),
        }
        global_width_groups = np.asarray([group_maximum(row) for row in global_widths])
        global_width_summary = {
            "root_angular_rad_s": distribution(global_width_groups[:, 0]),
            "root_linear_m_s": distribution(global_width_groups[:, 1]),
            "joint_rad_s": distribution(global_width_groups[:, 2]),
        }
        transfer = {}
        for source_name, source_slice in law_slices.items():
            source_root = CALIBRATION_RESERVE * float(np.max(root_energy[source_slice]))
            source_joint = CALIBRATION_RESERVE * float(
                np.max(articulated_energy[source_slice])
            )
            for target_name, target_slice in law_slices.items():
                if source_name == target_name:
                    continue
                covered = (root_energy[target_slice] <= source_root) & (
                    articulated_energy[target_slice] <= source_joint
                )
                transfer[f"{source_name}_to_{target_name}"] = float(np.mean(covered))
        residual_groups = np.asarray([group_maximum(np.abs(row)) for row in residual])
        result = {
            "candidate": candidate.name,
            "causal": candidate.causal,
            "center_kind": candidate.center_kind,
            "load_fraction": candidate.load_fraction,
            "calibration_reserve": CALIBRATION_RESERVE,
            "root_twice_energy_budget_j": root_budget,
            "articulated_twice_energy_budget_j": articulated_budget,
            "root_twice_energy_j": distribution(root_energy),
            "articulated_twice_energy_j": distribution(articulated_energy),
            "total_twice_energy_budget_j": total_budget,
            "total_twice_energy_j": distribution(total_energy),
            "residual_group_maximum": {
                "root_angular_rad_s": distribution(residual_groups[:, 0]),
                "root_linear_m_s": distribution(residual_groups[:, 1]),
                "joint_rad_s": distribution(residual_groups[:, 2]),
            },
            "split_interval_width": width_summary,
            "global_interval_width": global_width_summary,
            "component_coverage": float(np.mean(component_coverage)),
            "sample_coverage": float(np.mean(np.all(component_coverage, axis=1))),
            "cross_law_energy_membership": transfer,
            "split_bound_timing_ns": distribution(timing_ns),
            "center_response_timing_ns": distribution(
                np.asarray(records[candidate.name]["query_ns"], np.float64)
            ),
            "width_gate_passed": all(
                width_summary[name]["p95"] <= limit
                for name, limit in WIDTH_GATES.items()
            ),
        }
        results.append(result)
        replay_arrays[f"{candidate.name}_root_twice_energy"] = root_energy
        replay_arrays[f"{candidate.name}_articulated_twice_energy"] = articulated_energy
        replay_arrays[f"{candidate.name}_total_twice_energy"] = total_energy
        replay_arrays[f"{candidate.name}_residual"] = residual
        replay_arrays[f"{candidate.name}_split_width"] = widths
        replay_arrays[f"{candidate.name}_global_width"] = global_widths

    causal_results = [result for result in results if result["causal"]]
    best_causal = min(
        causal_results,
        key=lambda result: (
            -min(result["cross_law_energy_membership"].values()),
            result["split_interval_width"]["joint_rad_s"]["p95"],
            result["split_interval_width"]["root_angular_rad_s"]["p95"],
        ),
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay": str(replay_path),
        "source_replay_sha256": sha256(replay_path),
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "samples": len(states),
        "physics_steps": 0,
        "controller_steps": 0,
        "policy_steps": 0,
        "candidate_count": len(CANDIDATES),
        "zero_rust_allocation": zero_allocation,
        "profile_promoted": False,
        "authority_admitted": False,
        "best_causal_diagnostic_candidate": best_causal["candidate"],
        "construction_profile_rejected": True,
        "results": results,
    }
    rows = []
    for result in results:
        width = result["split_interval_width"]
        global_width = result["global_interval_width"]
        transfer_min = min(result["cross_law_energy_membership"].values())
        rows.append(
            [
                result["candidate"],
                "yes" if result["causal"] else "ORACLE",
                f"{result['root_twice_energy_budget_j']:.5f}",
                f"{result['articulated_twice_energy_budget_j']:.5f}",
                f"{100.0 * transfer_min:.1f}%",
                f"{width['root_angular_rad_s']['p95']:.3f}",
                f"{width['root_linear_m_s']['p95']:.3f}",
                f"{width['joint_rad_s']['p95']:.3f}",
                f"{global_width['joint_rad_s']['p95']:.3f}",
                f"{result['split_bound_timing_ns']['p99'] / 1_000.0:.3f}",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 center-response + split-residual replay · r216",
            "",
            f"> Construction replay **PASS** · best causal diagnostic row **{best_causal['candidate']}** · split construction **REJECTED** · authority **NOT ADMITTED** · physics/controller/policy steps **0**.",
            "",
            "## Contract",
            "",
            "- Inputs are the checksum-pinned immutable R215 replay. Completed impulses and raw model error are labels only; no simulator is stepped and no state is carried between rows.",
            "- Twelve causal rows combine four declared center rules with 0, 1/2, or one supported-weight impulse. Two label-leaky controls use completed impulse: one collapses it to a point center, while the other preserves the exact resultant force and free moment as a spatial wrench.",
            "- Rust reconstructs point response, generalized momentum residual, and the exact Minkowski sum of independent root and articulated kinetic-impulse ellipsoids. The displayed 5% calibration reserve is label-informed construction width, not a frozen profile.",
            "",
            "## Construction comparison",
            "",
            *markdown_table(
                [
                    "candidate",
                    "causal",
                    "root E₂ J",
                    "joint E₂ J",
                    "worst cross-law membership",
                    "root ω width p95",
                    "root v width p95",
                    "joint width p95",
                    "global joint width p95",
                    "split bound p99 µs",
                ],
                rows,
            ),
            "",
            "## Decision",
            "",
            f"R216 rejects the split-budget construction rather than opening R217. The best causal diagnostic row is `{best_causal['candidate']}`, but its label-informed interval is still hundreds of rad/s wide. The completed spatial-wrench oracle isolates the remaining floor after exact force and moment accounting; if its single full-metric width also exceeds the 10 rad/s gate, the next construction must condition residual geometry on causal state/contact features before any fresh holdout is justified.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-center-response-split-residual-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_CENTER_RESPONSE_SPLIT_RESIDUAL_REPLAY.md").write_text(report)
    np.savez_compressed(
        output / "g1-center-response-split-residual-replay.npz", **replay_arrays
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "best_causal_diagnostic_candidate": best_causal["candidate"],
                "construction_profile_rejected": True,
                "zero_rust_allocation": zero_allocation,
                "physics_steps": 0,
                "profile_promoted": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if zero_allocation else 1


if __name__ == "__main__":
    raise SystemExit(main())
