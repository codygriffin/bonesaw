#!/usr/bin/env python3
"""Validate the isolated guarded row-Gram pseudoinverse A/B for R291."""

from __future__ import annotations

import argparse
import json
import pathlib
from typing import Any

from cpu_reference_report import render_report_html


ROOT = pathlib.Path(__file__).resolve().parents[2]
REVISION = "g1-row-gram-task-r291"
RESULT_DIR = ROOT / "benchmarks/results" / REVISION
RESULT = RESULT_DIR / f"{REVISION}-metrics.json"
REPORT = RESULT_DIR / "G1_ROW_GRAM_TASK_R291.md"
WEB_REPORT = ROOT / "web/G1_ROW_GRAM_TASK_R291.html"


def load_result(path: pathlib.Path = RESULT) -> dict[str, Any]:
    return json.loads(path.read_text())


def validate_result(result: dict[str, Any]) -> None:
    if result.get("revision") != REVISION:
        raise ValueError(f"unexpected revision: {result.get('revision')!r}")
    execution = result["execution"]
    if execution["policy_steps"] != 0 or execution["physics_steps"] != 0:
        raise ValueError("R291 must remain simulator-free")
    verdict = result["verdict"]
    if verdict["semantic_exact"]:
        raise ValueError("row-Gram candidate unexpectedly became semantic exact")
    if result["comparison"]["first_divergence_tick"] != 0:
        raise ValueError("row-Gram divergence witness moved")
    if not verdict["hard_residuals_passed"]:
        raise ValueError("row-Gram candidate violated hard residuals")
    if verdict["tracking_gate_passed"]:
        raise ValueError("row-Gram candidate unexpectedly passed tracking")
    if verdict["promoted"] or verdict["authority_admitted"]:
        raise ValueError("R291 cannot promote authority")


def build_report(result: dict[str, Any]) -> str:
    control = result["profiles"]["control"]
    candidate = result["profiles"]["row_gram"]
    return "\n".join(
        [
            "# G1 guarded row-Gram pseudoinverse · R291",
            "",
            "> Arithmetic candidate **REJECTED** · hard residuals **PASS** · tracking **FAIL** · authority **CLOSED**.",
            "",
            "R291 enables the existing `row-gram-task-pseudoinverse-experiment` feature without changing the production default. It is a guarded factorization shortcut for well-conditioned wide task projections; the frozen integrated G1 replay is the required semantic holdout.",
            "",
            "## Frozen policy/physics-free replay",
            "",
            "| profile | p50 µs | p99 µs | max µs | root RMS m | CoM RMS m | stance-foot RMS m | max root rotation |",
            "|---|---:|---:|---:|---:|---:|---:|---:|",
            f"| control | {control['p50_us']:.1f} | {control['p99_us']:.1f} | {control['maximum_us']:.1f} | {control['root_rms_m']:.3f} | {control['com_rms_m']:.3f} | {control['foot_rms_m']:.3f} | {control['maximum_root_rotation_rad']:.3f} |",
            f"| row-Gram | {candidate['p50_us']:.1f} | {candidate['p99_us']:.1f} | {candidate['maximum_us']:.1f} | {candidate['root_rms_m']:.3f} | {candidate['com_rms_m']:.3f} | {candidate['foot_rms_m']:.3f} | {candidate['maximum_root_rotation_rad']:.3f} |",
            "",
            f"Only {result['comparison']['semantic_exact_arrays']}/{result['comparison']['total_arrays']} retained raw arrays match; the first divergence is tick {result['comparison']['first_divergence_tick']} (root/q state). The row-Gram path lowers one-run p99 to {candidate['p99_us']:.1f} µs, but root/CoM/foot RMS rises by {100*(candidate['root_rms_m']/control['root_rms_m']-1):.1f}%/{100*(candidate['com_rms_m']/control['com_rms_m']-1):.1f}%/{100*(candidate['foot_rms_m']/control['foot_rms_m']-1):.1f}% and maximum root rotation reaches {candidate['maximum_root_rotation_rad']:.3f} rad.",
            "",
            f"Hard dynamics/contact residuals remain {candidate['maximum_dynamics_residual']:.3e}/{candidate['maximum_contact_residual']:.3e}, so this is a semantic/tracking rejection rather than a hard-feasibility failure. The candidate remains available only for feature-level kernel tests; R284 remains the production Jacobi path.",
            "",
            "## Decision",
            "",
            "Do not promote the row-Gram factorization. A faster local pseudoinverse is not an acceptable WBC optimization when it changes the floating trace at tick zero. The next CPU slice must preserve arithmetic/semantic state before timing is considered.",
            "",
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    result = load_result()
    validate_result(result)
    if not args.check_only:
        RESULT_DIR.mkdir(parents=True, exist_ok=True)
        REPORT.write_text(build_report(result))
        WEB_REPORT.write_text(render_report_html(REPORT.read_text(), title="G1 row-Gram pseudoinverse · R291"))
    print(f"validated {RESULT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
