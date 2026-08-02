#!/usr/bin/env python3
"""r160 conditioned forecast-error certificate discovery and new holdout.

This is an offline calibration experiment, not executable authority.  Cells use
only origin-time state, final exact-WBC achieved acceleration, and exact support.
The evaluator refuses sparse/unseen cells and never conditions on a future test
wrench, fall outcome, or realized error.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import CaseSpec, case_matrix, semantic_trace_equal
from upkie_forecast_realization_contract import execute, raw_support_masks
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-conditioned-forecast-certificate-r160"
DURATION_S = 6.0
STATE_NAMES = (
    "roll_rad",
    "roll_rate_rad_s",
    "pitch_rad",
    "pitch_rate_rad_s",
    "lateral_position_m",
    "lateral_velocity_m_s",
    "yaw_rad",
    "yaw_rate_rad_s",
)
STATE_SCALES = np.asarray(
    [math.pi / 4.0, 4.0, math.pi / 4.0, 4.0, 0.10, 1.0, math.pi / 4.0, 4.0],
    np.float64,
)
ACCELERATION_INDEX = np.asarray([0, 0, 3, 3, 1, 1, 2, 2], np.int64)
ACCELERATION_SCALES = np.asarray([40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 20.0, 20.0])
STATE_THRESHOLDS = np.asarray([0.10, 0.25, 0.50, 1.00], np.float64)
ACCELERATION_THRESHOLDS = np.asarray([0.10, 0.25, 0.50, 1.00, 2.00], np.float64)
MINIMUM_CELL_SAMPLES = 8
CALIBRATION_RESERVE = 2.0
MINIMUM_ABSOLUTE_RESERVE_FRACTION = 1.0e-4
MINIMUM_ISSUANCE_FRACTION = 0.80
MAXIMUM_NORMALIZED_BOUND = 2.0


def new_holdout_cases() -> tuple[CaseSpec, ...]:
    """Frozen before executing r160; none is a retained r133/r159 case."""
    diagonal = 3.0 / math.sqrt(2.0)
    return (
        CaseSpec("r160_forward_1n", "new_force", (1.0, 0.0, 0.0), 0.10),
        CaseSpec("r160_forward_3n", "new_force", (3.0, 0.0, 0.0), 0.10),
        CaseSpec("r160_backward_3n", "new_force", (-3.0, 0.0, 0.0), 0.10),
        CaseSpec("r160_left_3n", "new_force", (0.0, 3.0, 0.0), 0.10),
        CaseSpec("r160_right_3n", "new_force", (0.0, -3.0, 0.0), 0.10),
        CaseSpec(
            "r160_diagonal_opposite_3n",
            "new_direction",
            (-diagonal, diagonal, 0.0),
            0.10,
        ),
        CaseSpec("r160_up_2n", "new_vertical", (0.0, 0.0, 2.0), 0.10),
        CaseSpec("r160_down_2n", "new_vertical", (0.0, 0.0, -2.0), 0.10),
        CaseSpec("r160_forward_4n_75ms", "new_duration", (4.0, 0.0, 0.0), 0.075),
        CaseSpec("r160_forward_1p5n_250ms", "new_duration", (1.5, 0.0, 0.0), 0.25),
        CaseSpec(
            "r160_handle_backward_3n",
            "new_application_point",
            (-3.0, 0.0, 0.0),
            0.10,
            body="handle",
        ),
        CaseSpec(
            "r160_forward_3n_friction_0p2",
            "new_friction",
            (3.0, 0.0, 0.0),
            0.10,
            friction=0.20,
        ),
        CaseSpec(
            "r160_left_1p5n_two_pulses",
            "new_repeated",
            (0.0, 1.5, 0.0),
            0.10,
            repetitions=2,
            repeat_interval_s=0.75,
        ),
    )


@dataclass(frozen=True)
class ComponentSample:
    case_name: str
    origin_tick: int
    knot: int
    component: int
    cell: tuple[int, int, int, int, int, int]
    error: float


@dataclass(frozen=True)
class JointSample:
    case_name: str
    origin_tick: int
    knot: int
    components: tuple[ComponentSample, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_CONDITIONED_FORECAST_CERTIFICATE_R160.html"
    )
    parser.add_argument(
        "--calibration-cases",
        help="debug-only retained-case subset; full certificate requires all 20",
    )
    parser.add_argument(
        "--holdout-cases",
        help="debug-only new-holdout subset; full certificate requires all 13",
    )
    return parser.parse_args()


def _bucket(value: float, thresholds: np.ndarray) -> int:
    return int(np.searchsorted(thresholds, value, side="right"))


def _capture_coordinates(state: np.ndarray) -> np.ndarray:
    return np.asarray(
        [
            state[0] + 0.12 * state[1],
            state[0] + 0.12 * state[1],
            state[2] + 0.12 * state[3],
            state[2] + 0.12 * state[3],
            state[4] + 0.12 * state[5],
            state[4] + 0.12 * state[5],
            state[6] + 0.12 * state[7],
            state[6] + 0.12 * state[7],
        ],
        np.float64,
    )


def collect_samples(case_name: str, trace: dict[str, Any]) -> tuple[list[JointSample], dict[str, int]]:
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
            component_samples = []
            for component in range(8):
                acceleration = float(achieved[origin, ACCELERATION_INDEX[component]])
                state_pressure = abs(float(state[origin, component])) / STATE_SCALES[component]
                acceleration_pressure = abs(acceleration) / ACCELERATION_SCALES[component]
                direction = int(float(capture[component]) * acceleration > 0.0)
                cell = (
                    knot,
                    support,
                    component,
                    _bucket(state_pressure, STATE_THRESHOLDS),
                    _bucket(acceleration_pressure, ACCELERATION_THRESHOLDS),
                    direction,
                )
                component_samples.append(
                    ComponentSample(
                        case_name,
                        int(origin),
                        knot,
                        component,
                        cell,
                        float(error[component]),
                    )
                )
            samples.append(
                JointSample(case_name, int(origin), knot, tuple(component_samples))
            )
    return samples, counts


def fit_cells(samples: list[JointSample]) -> dict[tuple[int, ...], dict[str, float | int]]:
    values: dict[tuple[int, ...], list[float]] = defaultdict(list)
    for sample in samples:
        for component in sample.components:
            values[component.cell].append(component.error)
    cells = {}
    for cell, errors in values.items():
        component = cell[2]
        maximum = max(errors)
        cells[cell] = {
            "count": len(errors),
            "maximum_error": maximum,
            "bound": max(
                CALIBRATION_RESERVE * maximum,
                MINIMUM_ABSOLUTE_RESERVE_FRACTION * STATE_SCALES[component],
            ),
        }
    return cells


def evaluate_samples(
    samples: list[JointSample], cells: dict[tuple[int, ...], dict[str, float | int]]
) -> dict[str, Any]:
    rows = []
    per_case: dict[str, dict[str, int | float]] = {}
    for sample in samples:
        issued = True
        inside = True
        maximum_ratio = 0.0
        maximum_normalized_bound = 0.0
        for component in sample.components:
            cell = cells.get(component.cell)
            if cell is None or int(cell["count"]) < MINIMUM_CELL_SAMPLES:
                issued = False
                inside = False
                continue
            bound = float(cell["bound"])
            inside = inside and component.error <= bound + 1.0e-15
            maximum_ratio = max(maximum_ratio, component.error / max(bound, 1.0e-30))
            maximum_normalized_bound = max(
                maximum_normalized_bound,
                bound / STATE_SCALES[component.component],
            )
        rows.append((sample, issued, inside, maximum_ratio, maximum_normalized_bound))
    for case_name in sorted({sample.case_name for sample in samples}):
        selected = [row for row in rows if row[0].case_name == case_name]
        issued = [row for row in selected if row[1]]
        inside = [row for row in issued if row[2]]
        per_case[case_name] = {
            "samples": len(selected),
            "issued": len(issued),
            "inside": len(inside),
            "misses": len(issued) - len(inside),
            "issuance_fraction": len(issued) / len(selected) if selected else 0.0,
            "coverage": len(inside) / len(issued) if issued else 0.0,
            "maximum_bound_ratio": max((row[3] for row in issued), default=0.0),
            "maximum_normalized_bound": max((row[4] for row in issued), default=0.0),
        }
    by_knot = []
    for knot in range(8):
        selected = [row for row in rows if row[0].knot == knot]
        issued = [row for row in selected if row[1]]
        inside = [row for row in issued if row[2]]
        by_knot.append(
            {
                "knot": knot + 1,
                "horizon_s": 0.03 * (knot + 1),
                "samples": len(selected),
                "issued": len(issued),
                "inside": len(inside),
                "misses": len(issued) - len(inside),
                "issuance_fraction": len(issued) / len(selected) if selected else 0.0,
                "coverage": len(inside) / len(issued) if issued else 0.0,
                "maximum_bound_ratio": max((row[3] for row in issued), default=0.0),
                "maximum_normalized_bound": max((row[4] for row in issued), default=0.0),
            }
        )
    issued = [row for row in rows if row[1]]
    inside = [row for row in issued if row[2]]
    return {
        "samples": len(rows),
        "issued": len(issued),
        "inside": len(inside),
        "misses": len(issued) - len(inside),
        "issuance_fraction": len(issued) / len(rows) if rows else 0.0,
        "coverage": len(inside) / len(issued) if issued else 0.0,
        "maximum_bound_ratio": max((row[3] for row in issued), default=0.0),
        "maximum_normalized_bound": max((row[4] for row in issued), default=0.0),
        "by_knot": by_knot,
        "by_case": per_case,
    }


def select_cases(cases: tuple[CaseSpec, ...], value: str | None) -> tuple[CaseSpec, ...]:
    if value is None:
        return cases
    names = set(value.split(","))
    selected = tuple(case for case in cases if case.name in names)
    missing = names - {case.name for case in selected}
    if missing:
        raise SystemExit(f"unknown cases: {sorted(missing)}")
    return selected


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    model = pathlib.Path(args.model).resolve()
    calibration_cases = select_cases(case_matrix(), args.calibration_cases)
    holdout_cases = select_cases(new_holdout_cases(), args.holdout_cases)
    calibration_samples: list[JointSample] = []
    holdout_samples: list[JointSample] = []
    case_censoring: dict[str, Any] = {}
    replay_exact: dict[str, bool] = {}

    total = len(calibration_cases) + len(holdout_cases)
    progress = 0
    for case in calibration_cases:
        trace = execute(model, case, args.duration)
        samples, censoring = collect_samples(case.name, trace)
        calibration_samples.extend(samples)
        case_censoring[case.name] = censoring
        progress += 1
        print(f"[{progress:02d}/{total}] calibration {case.name}: {len(samples)} pairs", flush=True)
    for case in holdout_cases:
        trace = execute(model, case, args.duration)
        replay = execute(model, case, args.duration)
        samples, censoring = collect_samples(case.name, trace)
        holdout_samples.extend(samples)
        case_censoring[case.name] = censoring
        replay_exact[case.name] = semantic_trace_equal(trace, replay)
        progress += 1
        print(
            f"[{progress:02d}/{total}] NEW HOLDOUT {case.name}: {len(samples)} pairs, "
            f"replay={'exact' if replay_exact[case.name] else 'DIFF'}",
            flush=True,
        )

    cells = fit_cells(calibration_samples)
    calibration_evaluation = evaluate_samples(calibration_samples, cells)
    holdout_evaluation = evaluate_samples(holdout_samples, cells)
    full_calibration = len(calibration_cases) == len(case_matrix())
    full_holdout = len(holdout_cases) == len(new_holdout_cases())
    gates = {
        "retained_calibration_complete": full_calibration,
        "new_holdout_complete": full_holdout,
        "new_holdout_names_disjoint": not (
            {case.name for case in calibration_cases}
            & {case.name for case in holdout_cases}
        ),
        "new_holdout_replay_exact": all(replay_exact.values()),
        "calibration_samples_exercised": bool(calibration_samples),
        "holdout_samples_exercised": bool(holdout_samples),
        "origin_only_features": True,
        "minimum_issuance_fraction": (
            holdout_evaluation["issuance_fraction"] >= MINIMUM_ISSUANCE_FRACTION
        ),
        "zero_issued_holdout_misses": holdout_evaluation["misses"] == 0,
        "useful_maximum_bound": (
            holdout_evaluation["maximum_normalized_bound"]
            <= MAXIMUM_NORMALIZED_BOUND
        ),
    }
    gates = {name: bool(value) for name, value in gates.items()}
    certificate_eligible = all(gates.values())
    authority_promoted = False
    retained_cells = {
        "/".join(map(str, cell)): value
        for cell, value in sorted(cells.items())
        if int(value["count"]) >= MINIMUM_CELL_SAMPLES
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
                "absolute_normalized_achieved_acceleration_bin",
                "opening_direction",
            ],
            "state_thresholds": STATE_THRESHOLDS.tolist(),
            "acceleration_thresholds": ACCELERATION_THRESHOLDS.tolist(),
            "minimum_cell_samples": MINIMUM_CELL_SAMPLES,
            "calibration_reserve": CALIBRATION_RESERVE,
            "minimum_absolute_reserve_fraction": MINIMUM_ABSOLUTE_RESERVE_FRACTION,
            "future_wrench_used": False,
            "future_outcome_used": False,
            "sparse_cell_policy": "refuse certificate",
        },
        "calibration_cases": [case.name for case in calibration_cases],
        "new_holdout_cases": [case.name for case in holdout_cases],
        "case_censoring": case_censoring,
        "new_holdout_replay_exact": replay_exact,
        "populated_cells": len(cells),
        "retained_cells": len(retained_cells),
        "retained_cell_bounds": retained_cells,
        "calibration": calibration_evaluation,
        "new_holdout": holdout_evaluation,
        "gates": gates,
        "certificate_eligible_for_codification": certificate_eligible,
        "authority_promoted": authority_promoted,
    }

    knot_rows = [
        [
            f"{row['horizon_s'] * 1e3:.0f}",
            row["samples"],
            row["issued"],
            f"{100.0 * row['issuance_fraction']:.2f}%",
            row["misses"],
            f"{100.0 * row['coverage']:.4f}%",
            f"{row['maximum_bound_ratio']:.3f}",
            f"{row['maximum_normalized_bound']:.3f}",
        ]
        for row in holdout_evaluation["by_knot"]
    ]
    case_rows = [
        [
            name,
            row["samples"],
            row["issued"],
            f"{100.0 * row['issuance_fraction']:.2f}%",
            row["misses"],
            f"{100.0 * row['coverage']:.4f}%",
            f"{row['maximum_bound_ratio']:.3f}",
            "YES" if replay_exact[name] else "NO",
        ]
        for name, row in holdout_evaluation["by_case"].items()
    ]
    report = "\n".join(
        [
            "# Bonesaw conditioned forecast certificate discovery · r160",
            "",
            f"> Discovery gates **{'PASS' if certificate_eligible else 'FAIL'}** · live authority promotion **NO**.",
            "",
            "## Result",
            "",
            f"The retained 20-case calibration contributes **{len(calibration_samples):,}** support-stable joint samples and **{len(cells):,}** populated component cells; **{len(retained_cells):,}** meet the predeclared eight-sample minimum. The 13 genuinely new cases contribute **{len(holdout_samples):,}** samples. The certificate issues for **{holdout_evaluation['issued']:,}/{holdout_evaluation['samples']:,} ({100.0 * holdout_evaluation['issuance_fraction']:.3f}%)**, misses **{holdout_evaluation['misses']:,}** issued samples, and covers **{100.0 * holdout_evaluation['coverage']:.5f}%** of issued samples. Its largest issued component bound is **{holdout_evaluation['maximum_normalized_bound']:.3f}×** that state's declared scale.",
            "",
            "Cells are keyed only by forecast knot, exact origin support, state component, absolute normalized origin state, absolute normalized final-WBC achieved acceleration, and whether that acceleration opens the component's short capture coordinate. Calibration uses a 2× observed-maximum reserve. Future wrench, future support, fall outcome, and realized error are unavailable to issuance. Sparse or unseen cells refuse issuance.",
            "",
            "## New holdout by horizon",
            "",
            *markdown_table(
                [
                    "horizon ms",
                    "samples",
                    "issued",
                    "issuance",
                    "misses",
                    "issued coverage",
                    "worst error/bound",
                    "largest norm bound",
                ],
                knot_rows,
            ),
            "",
            "## New holdout by case",
            "",
            *markdown_table(
                [
                    "case",
                    "samples",
                    "issued",
                    "issuance",
                    "misses",
                    "issued coverage",
                    "worst error/bound",
                    "replay",
                ],
                case_rows,
            ),
            "",
            "## Admission gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Authority boundary",
            "",
            "This is model discovery, not a safety guarantee and not an executable request. Passing would only make the fixed cell layout eligible for a typed Rust implementation with sequence, evidence, support, age, and calibration-fingerprint revocation. That implementation would still require an additional untouched holdout and the complete causal plant A/B. A failed gate is retained as evidence and cannot be repaired by silently widening or dropping the offending new cases.",
        ]
    ) + "\n"

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-conditioned-forecast-certificate-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_CONDITIONED_FORECAST_CERTIFICATE_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "certificate_eligible_for_codification": certificate_eligible,
                "authority_promoted": authority_promoted,
                "web_report": str(web_report),
            },
            sort_keys=True,
        )
    )
    return 0 if all(gates[name] for name in gates if name not in ("zero_issued_holdout_misses", "minimum_issuance_fraction", "useful_maximum_bound")) else 1


if __name__ == "__main__":
    raise SystemExit(main())
