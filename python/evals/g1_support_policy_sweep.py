#!/usr/bin/env python3
"""Run and summarize the predeclared G1 support-transfer policy matrix."""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import subprocess
import sys
from typing import Any

import numpy as np


CASES: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("baseline", ()),
    (
        "centroidal-intent-0p1",
        ("--centroidal-angular-momentum-weight", "0.1"),
    ),
    (
        "centroidal-intent-1",
        ("--centroidal-angular-momentum-weight", "1.0"),
    ),
    (
        "support-preview-0p1",
        (
            "--center-of-mass-reference",
            "support-preview",
            "--center-of-mass-task-weight",
            "0.1",
            "--center-of-mass-task-priority",
            "1",
        ),
    ),
    (
        "support-preview-0p1-centroidal-1",
        (
            "--center-of-mass-reference",
            "support-preview",
            "--center-of-mass-task-weight",
            "0.1",
            "--center-of-mass-task-priority",
            "1",
            "--centroidal-angular-momentum-weight",
            "1.0",
        ),
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ticks", type=int, default=600)
    parser.add_argument(
        "--output", default="benchmarks/results/g1-support-policy-sweep"
    )
    parser.add_argument(
        "--reuse", action="store_true", help="aggregate existing case results"
    )
    return parser.parse_args()


def run_case(name: str, arguments: tuple[str, ...], output: pathlib.Path, ticks: int) -> None:
    case_output = output / name
    if (case_output / "floating-walk-metrics.json").is_file():
        return
    environment = dict(os.environ)
    environment.setdefault("OPENBLAS_NUM_THREADS", "1")
    environment.setdefault("OMP_NUM_THREADS", "1")
    subprocess.run(
        [
            sys.executable,
            str(pathlib.Path(__file__).with_name("floating_walk_corpus.py")),
            "--ticks",
            str(ticks),
            "--output",
            str(case_output),
            *arguments,
        ],
        check=True,
        env=environment,
    )


def load_case(name: str, output: pathlib.Path) -> dict[str, Any]:
    document = json.loads(
        (output / name / "floating-walk-metrics.json").read_text()
    )
    metrics = document["metrics"]
    status = metrics["status_counts"]
    with np.load(output / name / "floating-walk-raw.npz") as trace:
        centroidal_residual = np.asarray(trace["task_rms"][:, 6], dtype=np.float64)
    return {
        "case": name,
        "passed": metrics["acceptance"]["passed"],
        "nominal_prefix_ticks": metrics["nominal_prefix"]["ticks"],
        "maximum_touchdown_transition_ticks": metrics[
            "maximum_touchdown_transition_ticks"
        ],
        "normal_contact_contingency_ticks": status[
            "normal_contact_contingency"
        ],
        "contact_release_contingency_ticks": status[
            "contact_release_contingency"
        ],
        "primal_infeasible_ticks": status["primal_infeasible"],
        "root_tracking_rms_m": metrics["root_tracking_rms_m"],
        "stance_foot_tracking_rms_m": metrics["stance_foot_tracking_rms_m"],
        "swing_foot_tracking_rms_m": metrics["swing_foot_tracking_rms_m"],
        "maximum_root_rotation_rad": metrics["maximum_root_rotation_rad"],
        "maximum_joint_velocity_rad_s": metrics[
            "maximum_joint_velocity_rad_s"
        ],
        "latency_p50_us": metrics["latency_us"]["p50"],
        "latency_p99_us": metrics["latency_us"]["p99"],
        "centroidal_rate_residual_rms_nm": float(
            np.sqrt(np.mean(np.square(centroidal_residual)))
        ),
        "centroidal_rate_residual_max_nm": float(np.max(centroidal_residual)),
        "failed_checks": [
            check
            for check, passed in metrics["acceptance"]["checks"].items()
            if not passed
        ],
    }


def markdown(rows: list[dict[str, Any]], ticks: int) -> str:
    lines = [
        "# G1 support-policy sweep",
        "",
        f"Predeclared `{ticks}`-tick strict floating-WBC comparison. Every case "
        "uses the same official Unitree G1 model and CMU-derived target trace; "
        "only the named support policy changes.",
        "",
        "A case is green only if the unchanged floating-walk acceptance gate "
        "passes. Avoiding infeasibility while remaining in normal-only touchdown "
        "or drifting away from the reference is not counted as success.",
        "",
        "| case | gate | nominal ticks | touchdown max | normal | release | infeasible | root RMS cm | stance RMS cm | swing RMS cm | rotation deg | centroidal residual RMS/max N·m | p50 ms | p99 ms |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['case']} | {'PASS' if row['passed'] else 'FAIL'} | "
            f"{row['nominal_prefix_ticks']} | "
            f"{row['maximum_touchdown_transition_ticks']} | "
            f"{row['normal_contact_contingency_ticks']} | "
            f"{row['contact_release_contingency_ticks']} | "
            f"{row['primal_infeasible_ticks']} | "
            f"{100 * row['root_tracking_rms_m']:.3f} | "
            f"{100 * row['stance_foot_tracking_rms_m']:.3f} | "
            f"{100 * row['swing_foot_tracking_rms_m']:.3f} | "
            f"{row['maximum_root_rotation_rad'] * 57.29577951308232:.3f} | "
            f"{row['centroidal_rate_residual_rms_nm']:.3f}/"
            f"{row['centroidal_rate_residual_max_nm']:.3f} | "
            f"{row['latency_p50_us'] / 1_000:.3f} | "
            f"{row['latency_p99_us'] / 1_000:.3f} |"
        )
    lines += [
        "",
        "## Interpretation",
        "",
        "The sweep separates three claims: exact external-wrench task support, "
        "a longer nominal window, and a completed walking transfer. The first "
        "can pass unit and oracle checks while the latter two remain red. The "
        "canonical policy is not changed unless a case passes the whole gate.",
        "",
        "Each case directory retains its full Markdown report, metrics JSON, and "
        "compressed per-tick NPZ trace.",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    if args.ticks <= 0:
        raise SystemExit("--ticks must be positive")
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    if not args.reuse:
        for name, arguments in CASES:
            run_case(name, arguments, output, args.ticks)
    rows = [load_case(name, output) for name, _ in CASES]
    document = {"schema": 1, "ticks": args.ticks, "cases": rows}
    (output / "g1-support-policy-sweep.json").write_text(
        json.dumps(document, indent=2) + "\n"
    )
    report = output / "G1_SUPPORT_POLICY_SWEEP.md"
    report.write_text(markdown(rows, args.ticks))
    print(report)


if __name__ == "__main__":
    main()
