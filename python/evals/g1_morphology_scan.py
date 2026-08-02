#!/usr/bin/env python3
"""Scan authored references through the Rust morphology certificate only."""

from __future__ import annotations

import argparse
import json
import pathlib

import numpy as np

from reference_comparison import standing_posture


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("references", nargs="+")
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument(
        "--output", default="benchmarks/results/g1-morphology-scan-r44.json"
    )
    args = parser.parse_args()
    import bonesaw

    session = bonesaw.KinematicWitnessSession(args.model, target_capacity=2)
    frame_names = list(session.frame_names)
    frame_ids = np.asarray(
        [
            frame_names.index("left_ankle_roll_link"),
            frame_names.index("right_ankle_roll_link"),
        ],
        dtype=np.int64,
    )
    initial = standing_posture(list(session.joint_names))
    cases = []
    for reference_name in args.references:
        reference_path = pathlib.Path(reference_name)
        metadata = json.loads(
            (reference_path.parent / "reference-metadata.json").read_text()
        )
        with np.load(reference_path) as source:
            root = np.asarray(source["root_targets"])
            com = np.asarray(source["center_of_mass_targets"])
            targets = np.asarray(source["target_positions"])
        ticks = len(root)
        q = np.empty((ticks, session.dof), dtype=np.float64)
        v = np.empty_like(q)
        acceleration = np.empty_like(q)
        point_error = np.empty(ticks, dtype=np.float64)
        orientation_error = np.empty(ticks, dtype=np.float64)
        com_error = np.empty(ticks, dtype=np.float64)
        iterations = np.empty(ticks, dtype=np.uint16)
        converged = np.empty(ticks, dtype=np.uint8)
        solve_ns = np.empty(ticks, dtype=np.uint64)
        allocations = np.empty(ticks, dtype=np.uint64)
        allocated_bytes = np.empty(ticks, dtype=np.uint64)
        session.run_trace(
            0.005,
            root,
            com,
            frame_ids,
            targets,
            np.full(2, 100.0, dtype=np.float64),
            initial,
            q,
            v,
            acceleration,
            point_error,
            orientation_error,
            com_error,
            iterations,
            converged,
            solve_ns,
            allocations,
            allocated_bytes,
            maximum_iterations=96,
            damping=1e-5,
            posture_weight=1e-5,
            center_of_mass_weight=0.25,
            orientation_weight=1.0,
            maximum_step_rad=0.04,
            point_tolerance_m=1e-4,
            center_of_mass_tolerance_m=2e-3,
            orientation_tolerance_rad=1e-3,
        )
        cases.append(
            {
                "reference": str(reference_path),
                "root_horizontal_follow_ratio": metadata["plan"].get(
                    "root_horizontal_follow_ratio", 1.0
                ),
                "converged_ticks": int(np.count_nonzero(converged)),
                "maximum_point_error_m": float(np.max(point_error)),
                "maximum_orientation_error_rad": float(np.max(orientation_error)),
                "maximum_center_of_mass_error_m": float(np.max(com_error)),
                "maximum_joint_step_rad": float(np.max(np.abs(np.diff(q, axis=0)))),
                "p99_solve_us": float(np.percentile(solve_ns, 99) / 1_000.0),
                "allocation_calls": int(np.sum(allocations, dtype=np.uint64)),
            }
        )
    output = pathlib.Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({"schema": 1, "cases": cases}, indent=2) + "\n")
    print(json.dumps(cases, indent=2))


if __name__ == "__main__":
    main()
