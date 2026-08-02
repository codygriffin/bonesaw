#!/usr/bin/env python3
"""r161 paired-state conditioned certificate and second untouched holdout."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_conditioned_forecast_certificate import (
    ACCELERATION_INDEX,
    ACCELERATION_SCALES,
    ACCELERATION_THRESHOLDS,
    CALIBRATION_RESERVE,
    MINIMUM_ABSOLUTE_RESERVE_FRACTION,
    MINIMUM_CELL_SAMPLES,
    STATE_NAMES,
    STATE_SCALES,
    STATE_THRESHOLDS,
    ComponentSample,
    JointSample,
    _bucket,
    _capture_coordinates,
    execute,
    fit_cells,
    new_holdout_cases as r160_holdout_cases,
    raw_support_masks,
)
from upkie_disturbance_envelope import CaseSpec, case_matrix, semantic_trace_equal
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-paired-state-forecast-certificate-r161"
DURATION_S = 6.0
PAIR_INDEX = np.asarray([1, 0, 3, 2, 5, 4, 7, 6], np.int64)
MINIMUM_ISSUANCE_FRACTION = 0.80
MAXIMUM_NORMALIZED_CELL_BOUND = 1.0


def new_holdout_cases() -> tuple[CaseSpec, ...]:
    """Frozen second holdout; disjoint names and parameters from r133/r160."""
    diagonal = 4.5 / math.sqrt(2.0)
    return (
        CaseSpec("r161_forward_0p5n", "new_force", (0.5, 0.0, 0.0), 0.10),
        CaseSpec("r161_forward_2p5n", "new_force", (2.5, 0.0, 0.0), 0.10),
        CaseSpec("r161_forward_5n", "new_force", (5.0, 0.0, 0.0), 0.10),
        CaseSpec("r161_backward_1p5n", "new_force", (-1.5, 0.0, 0.0), 0.10),
        CaseSpec("r161_backward_5n", "new_force", (-5.0, 0.0, 0.0), 0.10),
        CaseSpec("r161_left_2p5n", "new_force", (0.0, 2.5, 0.0), 0.10),
        CaseSpec("r161_right_2p5n", "new_force", (0.0, -2.5, 0.0), 0.10),
        CaseSpec(
            "r161_diagonal_opposite_4p5n",
            "new_direction",
            (-diagonal, diagonal, 0.0),
            0.10,
        ),
        CaseSpec("r161_up_3n", "new_vertical", (0.0, 0.0, 3.0), 0.10),
        CaseSpec("r161_down_3n", "new_vertical", (0.0, 0.0, -3.0), 0.10),
        CaseSpec("r161_forward_3n_150ms", "new_duration", (3.0, 0.0, 0.0), 0.15),
        CaseSpec(
            "r161_handle_forward_2n",
            "new_application_point",
            (2.0, 0.0, 0.0),
            0.10,
            body="handle",
        ),
        CaseSpec(
            "r161_backward_3n_friction_0p5",
            "new_friction",
            (-3.0, 0.0, 0.0),
            0.10,
            friction=0.50,
        ),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_PAIRED_STATE_FORECAST_CERTIFICATE_R161.html",
    )
    return parser.parse_args()


def collect_samples(
    case_name: str, trace: dict[str, Any]
) -> tuple[list[JointSample], dict[str, int]]:
    valid = np.asarray(trace["execution_forecast_path_valid"], np.uint8) != 0
    forecast_support = np.asarray(trace["execution_forecast_support_mask"], np.uint8)
    state = np.asarray(trace["execution_forecast_reduced_state"], np.float64)
    achieved = np.asarray(
        trace["execution_forecast_achieved_acceleration"], np.float64
    )
    path = np.asarray(trace["execution_forecast_path"], np.float64)
    raw_support = raw_support_masks(trace)
    offsets = np.arange(1, 9, dtype=np.int64) * int(round(0.03 / CONTROL_DT))
    samples: list[JointSample] = []
    counts = {"origins": int(np.sum(valid)), "support": 0, "mismatch": 0, "truncated": 0}
    for origin in np.flatnonzero(valid):
        support = int(raw_support[origin])
        capture = _capture_coordinates(state[origin])
        for knot, offset in enumerate(offsets):
            endpoint = int(origin + offset)
            if endpoint >= len(state):
                counts["truncated"] += 1
                continue
            if int(forecast_support[origin]) != support:
                counts["mismatch"] += 1
                continue
            if np.any(raw_support[origin : endpoint + 1] != support):
                counts["support"] += 1
                continue
            error = np.abs(path[origin, knot, 1:] - state[endpoint])
            components = []
            for component in range(8):
                acceleration = float(achieved[origin, ACCELERATION_INDEX[component]])
                state_pressure = abs(float(state[origin, component])) / STATE_SCALES[component]
                pair = int(PAIR_INDEX[component])
                paired_pressure = abs(float(state[origin, pair])) / STATE_SCALES[pair]
                acceleration_pressure = abs(acceleration) / ACCELERATION_SCALES[component]
                direction = int(float(capture[component]) * acceleration > 0.0)
                cell = (
                    knot,
                    support,
                    component,
                    _bucket(state_pressure, STATE_THRESHOLDS),
                    _bucket(paired_pressure, STATE_THRESHOLDS),
                    _bucket(acceleration_pressure, ACCELERATION_THRESHOLDS),
                    direction,
                )
                components.append(
                    ComponentSample(
                        case_name,
                        int(origin),
                        knot,
                        component,
                        cell,
                        float(error[component]),
                    )
                )
            samples.append(JointSample(case_name, int(origin), knot, tuple(components)))
    return samples, counts


def evaluate_samples(
    samples: list[JointSample], cells: dict[tuple[int, ...], dict[str, float | int]]
) -> dict[str, Any]:
    rows = []
    refusal_counts = {"sparse_or_unseen": 0, "bound_too_large": 0}
    for sample in samples:
        issued = True
        inside = True
        ratio = 0.0
        normalized_bound = 0.0
        refused_sparse = False
        refused_large = False
        for component in sample.components:
            cell = cells.get(component.cell)
            if cell is None or int(cell["count"]) < MINIMUM_CELL_SAMPLES:
                issued = False
                inside = False
                refused_sparse = True
                continue
            bound = float(cell["bound"])
            component_normalized_bound = bound / STATE_SCALES[component.component]
            if component_normalized_bound > MAXIMUM_NORMALIZED_CELL_BOUND:
                issued = False
                inside = False
                refused_large = True
                continue
            inside = inside and component.error <= bound + 1.0e-15
            ratio = max(ratio, component.error / max(bound, 1.0e-30))
            normalized_bound = max(normalized_bound, component_normalized_bound)
        refusal_counts["sparse_or_unseen"] += int(refused_sparse)
        refusal_counts["bound_too_large"] += int(refused_large)
        rows.append((sample, issued, inside, ratio, normalized_bound))

    def summarize(selected: list[tuple[Any, bool, bool, float, float]]) -> dict[str, Any]:
        issued = [row for row in selected if row[1]]
        inside = [row for row in issued if row[2]]
        return {
            "samples": len(selected),
            "issued": len(issued),
            "inside": len(inside),
            "misses": len(issued) - len(inside),
            "issuance_fraction": len(issued) / len(selected) if selected else 0.0,
            "coverage": len(inside) / len(issued) if issued else 0.0,
            "maximum_bound_ratio": max((row[3] for row in issued), default=0.0),
            "maximum_normalized_bound": max((row[4] for row in issued), default=0.0),
        }

    aggregate = summarize(rows)
    aggregate["refusal_counts"] = refusal_counts
    aggregate["by_knot"] = [
        {
            "knot": knot + 1,
            "horizon_s": 0.03 * (knot + 1),
            **summarize([row for row in rows if row[0].knot == knot]),
        }
        for knot in range(8)
    ]
    aggregate["by_case"] = {
        name: summarize([row for row in rows if row[0].case_name == name])
        for name in sorted({row[0].case_name for row in rows})
    }
    return aggregate


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    model = pathlib.Path(args.model).resolve()
    calibration_cases = (*case_matrix(), *r160_holdout_cases())
    holdout_cases = new_holdout_cases()
    calibration_samples: list[JointSample] = []
    holdout_samples: list[JointSample] = []
    censoring: dict[str, Any] = {}
    replay_exact: dict[str, bool] = {}
    total = len(calibration_cases) + len(holdout_cases)
    progress = 0
    for case in calibration_cases:
        samples, counts = collect_samples(case.name, execute(model, case, args.duration))
        calibration_samples.extend(samples)
        censoring[case.name] = counts
        progress += 1
        print(f"[{progress:02d}/{total}] calibration {case.name}: {len(samples)} pairs", flush=True)
    for case in holdout_cases:
        trace = execute(model, case, args.duration)
        replay = execute(model, case, args.duration)
        samples, counts = collect_samples(case.name, trace)
        holdout_samples.extend(samples)
        censoring[case.name] = counts
        replay_exact[case.name] = semantic_trace_equal(trace, replay)
        progress += 1
        print(
            f"[{progress:02d}/{total}] NEW HOLDOUT {case.name}: {len(samples)} pairs, "
            f"replay={'exact' if replay_exact[case.name] else 'DIFF'}",
            flush=True,
        )

    cells = fit_cells(calibration_samples)
    calibration = evaluate_samples(calibration_samples, cells)
    holdout = evaluate_samples(holdout_samples, cells)
    calibration_names = {case.name for case in calibration_cases}
    holdout_names = {case.name for case in holdout_cases}
    gates = {
        "calibration_complete": len(calibration_cases) == 33,
        "second_holdout_complete": len(holdout_cases) == 13,
        "second_holdout_disjoint": not bool(calibration_names & holdout_names),
        "second_holdout_replay_exact": all(replay_exact.values()),
        "origin_only_features": True,
        "minimum_issuance_fraction": holdout["issuance_fraction"] >= MINIMUM_ISSUANCE_FRACTION,
        "zero_issued_holdout_misses": holdout["misses"] == 0,
        "all_issued_bounds_within_one_state_scale": holdout["maximum_normalized_bound"] <= 1.0,
    }
    gates = {name: bool(value) for name, value in gates.items()}
    eligible = all(gates.values())
    authority_promoted = False
    retained_cells = {
        "/".join(map(str, cell)): value
        for cell, value in sorted(cells.items())
        if int(value["count"]) >= MINIMUM_CELL_SAMPLES
        and float(value["bound"]) / STATE_SCALES[cell[2]]
        <= MAXIMUM_NORMALIZED_CELL_BOUND
    }
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "state_names": list(STATE_NAMES),
        "conditioning": {
            "cell_columns": [
                "knot",
                "exact_support_mask",
                "state_component",
                "absolute_normalized_origin_state_bin",
                "absolute_normalized_paired_origin_state_bin",
                "absolute_normalized_achieved_acceleration_bin",
                "opening_direction",
            ],
            "pair_index": PAIR_INDEX.tolist(),
            "state_thresholds": STATE_THRESHOLDS.tolist(),
            "acceleration_thresholds": ACCELERATION_THRESHOLDS.tolist(),
            "minimum_cell_samples": MINIMUM_CELL_SAMPLES,
            "calibration_reserve": CALIBRATION_RESERVE,
            "maximum_normalized_cell_bound": MAXIMUM_NORMALIZED_CELL_BOUND,
            "future_wrench_used": False,
            "future_outcome_used": False,
            "refusal_is_non_executable": True,
        },
        "calibration_cases": [case.name for case in calibration_cases],
        "second_holdout_cases": [case.name for case in holdout_cases],
        "censoring": censoring,
        "second_holdout_replay_exact": replay_exact,
        "populated_cells": len(cells),
        "retained_cells": len(retained_cells),
        "retained_cell_bounds": retained_cells,
        "calibration": calibration,
        "second_holdout": holdout,
        "gates": gates,
        "certificate_eligible_for_codification": eligible,
        "authority_promoted": authority_promoted,
    }

    knot_rows = [
        [
            f"{row['horizon_s'] * 1e3:.0f}",
            row["samples"],
            row["issued"],
            f"{100.0 * row['issuance_fraction']:.2f}%",
            row["misses"],
            f"{100.0 * row['coverage']:.5f}%",
            f"{row['maximum_bound_ratio']:.3f}",
            f"{row['maximum_normalized_bound']:.3f}",
        ]
        for row in holdout["by_knot"]
    ]
    case_rows = [
        [
            name,
            row["samples"],
            row["issued"],
            f"{100.0 * row['issuance_fraction']:.2f}%",
            row["misses"],
            f"{100.0 * row['coverage']:.5f}%",
            f"{row['maximum_bound_ratio']:.3f}",
            "YES" if replay_exact[name] else "NO",
        ]
        for name, row in holdout["by_case"].items()
    ]
    report = "\n".join(
        [
            "# Bonesaw paired-state forecast certificate · r161",
            "",
            f"> Offline certificate discovery **{'PASS' if eligible else 'FAIL'}** · live authority promotion **NO**.",
            "",
            "## Result",
            "",
            f"R160's 19 misses were all pitch angle/rate and exposed a missing conjugate-state feature. R161 freezes that result, adds the paired position/rate bin, refuses any cell with fewer than eight calibration samples or a component bound above one declared state scale, expands calibration to the 33 now-observed r133/r160 cases, and evaluates 13 second-holdout cases that were not used to select the layout.",
            "",
            f"The second holdout contains **{holdout['samples']:,}** support-stable comparisons. The candidate issues **{holdout['issued']:,} ({100.0 * holdout['issuance_fraction']:.3f}%)**, records **{holdout['misses']:,}** issued misses, covers **{100.0 * holdout['coverage']:.6f}%** of issued samples, and retains a largest issued bound of **{holdout['maximum_normalized_bound']:.3f}×** its component scale. Sparse/unseen refusals occur on **{holdout['refusal_counts']['sparse_or_unseen']:,}** comparisons and over-wide-bound refusals on **{holdout['refusal_counts']['bound_too_large']:,}**; these counts can overlap.",
            "",
            "## Second holdout by horizon",
            "",
            *markdown_table(
                ["ms", "samples", "issued", "issuance", "misses", "coverage", "worst error/bound", "largest norm bound"],
                knot_rows,
            ),
            "",
            "## Second holdout by case",
            "",
            *markdown_table(
                ["case", "samples", "issued", "issuance", "misses", "coverage", "worst error/bound", "replay"],
                case_rows,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Authority boundary",
            "",
            "Only origin-time state, achieved acceleration, and exact support enter a cell. A refusal is non-executable; future force, support, and outcome remain unavailable. Even a full offline pass would authorize only implementation of a fingerprinted fixed-capacity Rust certificate with sequence/evidence/support/age revocation, followed by a third untouched holdout and the complete causal plant gate. R161 itself cannot affect request or torque authority.",
        ]
    ) + "\n"

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-paired-state-forecast-certificate-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_PAIRED_STATE_FORECAST_CERTIFICATE_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "certificate_eligible_for_codification": eligible,
                "authority_promoted": authority_promoted,
                "web_report": str(web_report),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
