#!/usr/bin/env python3
"""Admit the Rust Upkie fall-safe supervisor without claiming recovery."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_mujoco_plant_report import run_case, summarize_case


REVISION = "upkie-fall-safe-contingency-r136"
CASES = (
    ("nominal", (0.0, 0.0, 0.0)),
    ("forward_4n", (4.0, 0.0, 0.0)),
    ("left_1n", (0.0, 1.0, 0.0)),
    ("left_2n", (0.0, 2.0, 0.0)),
    ("right_2n", (0.0, -2.0, 0.0)),
    ("forward_6n_overload", (6.0, 0.0, 0.0)),
)
GREEN = {"nominal", "forward_4n"}
SEMANTIC_FIELDS = (
    "root_position",
    "root_quaternion",
    "root_twist",
    "q",
    "v",
    "torque",
    "status",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_FALL_SAFE_CONTINGENCY_R136.html"
    )
    return parser.parse_args()


def execute(
    model: pathlib.Path,
    force: tuple[float, float, float],
    enabled: bool,
    *,
    primary_blend: bool = True,
) -> dict[str, Any]:
    trace = run_case(
        model,
        duration=6.0,
        push_start=1.0,
        push_duration=0.1,
        push_force=0.0,
        push_force_world=force,
        contact_model="soft",
        balance_mode="capture",
        capture_velocity_fraction=0.2,
        terminate_on_fall=True,
        fall_safe_enabled=enabled,
        fall_safe_primary_blend=primary_blend,
    )
    summary = summarize_case(trace, 1.1)
    terminal = len(np.asarray(trace["time_s"])) - 1
    return {
        "trace": trace,
        "metrics": {
            "fell": summary["fell"],
            "fall_time_s": summary["fall_time_s"],
            "terminal_kinetic_energy_j": summary["terminal_kinetic_energy_j"],
            "maximum_kinetic_energy_j": summary["maximum_kinetic_energy_j"],
            "terminal_root_angular_speed_rad_s": float(
                np.linalg.norm(np.asarray(trace["root_twist"])[terminal, :3])
            ),
            "terminal_root_linear_speed_m_s": float(
                np.linalg.norm(np.asarray(trace["root_twist"])[terminal, 3:])
            ),
            "terminal_joint_speed_l2_rad_s": float(
                np.linalg.norm(np.asarray(trace["v"])[terminal])
            ),
            "terminal_torque_l2_nm": float(
                np.linalg.norm(np.asarray(trace["torque"])[terminal])
            ),
            "post_startup_nonadmitted_steps": summary[
                "post_startup_nonadmitted_steps"
            ],
            "maximum_command_age_steps": summary["maximum_command_age_steps"],
            "maximum_hard_constraint_violation": summary[
                "maximum_hard_constraint_violation"
            ],
            "fall_safe_mode_counts": summary["fall_safe_mode_counts"],
            "minimum_primary_authority": summary[
                "minimum_fall_safe_primary_authority"
            ],
            "minimum_fresh_command_authority": summary[
                "minimum_fall_safe_fresh_command_authority"
            ],
            "maximum_risk": summary["maximum_fall_safe_risk"],
            "controller_allocation_calls": summary["controller_allocation_calls"],
            "controller_allocated_bytes": summary["controller_allocated_bytes"],
        },
    }


def semantic_equal(left: dict[str, Any], right: dict[str, Any]) -> bool:
    return all(
        np.array_equal(np.asarray(left[field]), np.asarray(right[field]))
        for field in SEMANTIC_FIELDS
    )


def main() -> None:
    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rows: dict[str, Any] = {}
    gates: dict[str, bool] = {}
    for name, force in CASES:
        baseline = execute(model, force, False)
        candidate = execute(model, force, True)
        replay = execute(model, force, True)
        baseline_metrics = baseline["metrics"]
        candidate_metrics = candidate["metrics"]
        row = {
            "force_world_n": list(force),
            "baseline": baseline_metrics,
            "candidate": candidate_metrics,
            "candidate_replay_exact": semantic_equal(candidate["trace"], replay["trace"]),
            "baseline_candidate_exact": semantic_equal(
                baseline["trace"], candidate["trace"]
            ),
            "terminal_kinetic_energy_ratio": (
                1.0
                if baseline_metrics["terminal_kinetic_energy_j"] == 0.0
                else candidate_metrics["terminal_kinetic_energy_j"]
                / baseline_metrics["terminal_kinetic_energy_j"]
            ),
        }
        rows[name] = row
    adverse = [name for name, _ in CASES if name not in GREEN]
    gates["matrix_complete"] = set(rows) == {name for name, _ in CASES}
    gates["green_recovery_preserved"] = all(
        not rows[name]["candidate"]["fell"] for name in GREEN
    )
    gates["green_semantics_exact"] = all(
        rows[name]["baseline_candidate_exact"] for name in GREEN
    )
    gates["adverse_supervisor_engages"] = all(
        any(int(mode) > 0 for mode in rows[name]["candidate"]["fall_safe_mode_counts"])
        for name in adverse
    )
    gates["adverse_terminal_energy_reduced"] = all(
        rows[name]["terminal_kinetic_energy_ratio"] < 1.0 for name in adverse
    )
    gates["adverse_terminal_root_rotation_reduced"] = all(
        rows[name]["candidate"]["terminal_root_angular_speed_rad_s"]
        < rows[name]["baseline"]["terminal_root_angular_speed_rad_s"]
        for name in adverse
    )
    gates["stale_command_age_reduced"] = all(
        rows[name]["candidate"]["maximum_command_age_steps"]
        < rows[name]["baseline"]["maximum_command_age_steps"]
        for name in adverse
    )
    gates["candidate_replay_exact"] = all(
        row["candidate_replay_exact"] for row in rows.values()
    )
    gates["zero_rust_allocation"] = all(
        row[side]["controller_allocation_calls"] == 0
        and row[side]["controller_allocated_bytes"] == 0
        for row in rows.values()
        for side in ("baseline", "candidate")
    )
    gates["finite_metrics"] = all(
        np.isfinite(
            [
                row[side]["terminal_kinetic_energy_j"],
                row[side]["maximum_kinetic_energy_j"],
                row[side]["terminal_root_angular_speed_rad_s"],
                row[side]["terminal_joint_speed_l2_rad_s"],
            ]
        ).all()
        for row in rows.values()
        for side in ("baseline", "candidate")
    )
    passed = all(gates.values())
    deployment_gates = {
        "no_earlier_adverse_boundary": all(
            float(rows[name]["candidate"]["fall_time_s"])
            >= float(rows[name]["baseline"]["fall_time_s"])
            for name in adverse
        )
    }
    deployment_passed = all(deployment_gates.values())
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "passed": passed,
        "deployment_passed": deployment_passed,
        "gates": gates,
        "deployment_gates": deployment_gates,
        "rows": rows,
        "interpretation": {
            "promotion": (
                "contingency mechanism admitted; live deployment rejected"
                if passed and not deployment_passed
                else "live deployment admitted"
                if passed
                else "mechanism rejected"
            ),
            "recovery_claim": False,
            "command_contract": "five-tick startup lease, continuous risk blend, damped contingency, stale-command authority reaches zero by the twelfth consecutive nonadmission",
        },
    }
    report_rows = []
    for name, row in rows.items():
        baseline = row["baseline"]
        candidate = row["candidate"]
        report_rows.append(
            [
                name,
                "RECOVERED" if not baseline["fell"] else f'FALL {baseline["fall_time_s"]:.3f}s',
                "RECOVERED" if not candidate["fell"] else f'FALL {candidate["fall_time_s"]:.3f}s',
                f'{baseline["terminal_kinetic_energy_j"]:.4f} → {candidate["terminal_kinetic_energy_j"]:.4f}',
                f'{baseline["terminal_root_angular_speed_rad_s"]:.3f} → {candidate["terminal_root_angular_speed_rad_s"]:.3f}',
                f'{baseline["maximum_command_age_steps"]} → {candidate["maximum_command_age_steps"]}',
                "YES" if row["baseline_candidate_exact"] else "NO",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw Upkie explicit contingency transition · r136",
            "",
            f"> Evaluation **{'PASS' if passed else 'FAIL'}**. Live deployment **{'PASS' if deployment_passed else 'REJECTED'}**. This admits a bounded mechanism, not lateral or overload recovery.",
            "",
            "## Outcome",
            "",
            "The Rust supervisor leaves nominal and the qualified 4 N sagittal recovery byte-exact. Once tilt, planar angular rate, height, or consecutive solver rejection consumes authority, it continuously blends the primary acceleration request into bounded Rust-authored velocity damping. A five-tick startup lease is explicit; stale admitted torque is faded by the twelfth consecutive rejection. Every adverse row still falls, but every declared terminal kinetic-energy and root-rotation witness is lower than baseline.",
            "",
            *markdown_table(
                ["case", "baseline", "candidate", "terminal KE J", "terminal root ω rad/s", "max stale age ticks", "baseline exact"],
                report_rows,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Deployment gate",
            "",
            *markdown_table(
                ["gate", "result"],
                [
                    [name, "PASS" if value else "REJECTED"]
                    for name, value in deployment_gates.items()
                ],
            ),
            "",
            "## Contract",
            "",
            "- The terminal boundary remains root height below 350 mm or tilt above 45°. Earlier entry into a low-energy fall is not recovery and is not scored as one.",
            "- `MaxIterations` candidates remain non-executable. The supervisor uses status only as a continuous lease-pressure input; it never relaxes hard admission.",
            "- Damping targets are written in allocation-free Rust and then pass through the ordinary floating WBC. Python owns MuJoCo, A/B orchestration, kinetic-energy scoring, and artifacts.",
            "- Candidate replay excludes clocks but includes full root/joint/torque/status semantics. Green baseline/candidate exactness includes those same fields.",
        ]
    ) + "\n"
    (output / "upkie-fall-safe-contingency-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_FALL_SAFE_CONTINGENCY_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "gates": gates, "web_report": str(web_report)}, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
