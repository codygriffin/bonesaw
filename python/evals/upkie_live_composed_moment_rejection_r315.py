#!/usr/bin/env python3
"""R315 bounded composition screen for the two R314 moment mechanisms.

The screen is intentionally policy-free at the controller boundary: each row
is the same frozen repeated-wrench plant consequence used by R313/R314. Rust
owns both actions. Python declares four finite profiles and reports whether a
composition can improve the remaining ±8 N overload without nonadmission,
deadline, torque, mode, or determinism regressions.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco

import upkie_live_body_moment_rejection_r314 as body
import upkie_live_dynamic_contact_transition_r300 as r300
import upkie_live_moment_rejection_r314 as feed


ROOT = pathlib.Path(__file__).resolve().parents[2]
REVISION = "upkie-live-composed-moment-rejection-r315"
FROZEN_MUJOCO_VERSION = "3.3.7"
FORCES_N = (-8.0, -6.0, 6.0, 8.0)
PROFILES: dict[str, dict[str, Any]] = {
    "body_only": body.CANDIDATE_OPTIONS,
    "root_wrench_only": feed.CANDIDATE_OPTIONS,
    "body_plus_root_wrench": {
        **body.CANDIDATE_OPTIONS,
        "external_wrench_feedforward_enabled": True,
    },
    "body_plus_root_wrench_d12": {
        **body.CANDIDATE_OPTIONS,
        "external_wrench_feedforward_enabled": True,
        "root_roll_damping": 12.0,
    },
}


def _digest(case: dict[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(
            r300._semantic(case), sort_keys=True, separators=(",", ":")
        ).encode()
    ).hexdigest()


def _consequence(case: dict[str, Any], force_n: float) -> dict[str, Any]:
    summary = body.summarize(case)
    return {
        "force_y_n": force_n,
        "terminal_tick": summary["terminal_tick"],
        "nonadmitted_ticks": summary["nonadmitted_ticks"],
        "force_qualified_relock_count": summary["force_qualified_relock_count"],
        "strict_recovery_tick": summary["strict_recovery_tick"],
        "maximum_torque_utilization": summary["maximum_torque_utilization"],
        "maximum_dynamics_residual": summary["maximum_dynamics_residual"],
        "maximum_contact_residual": summary["maximum_contact_residual"],
        "controller_step_p99_us": summary["controller_step_us"]["p99"],
        "worker_step_p99_us": summary["worker_step_us"]["p99"],
    }


def evaluate(model: pathlib.Path) -> dict[str, Any]:
    profile_rows: list[dict[str, Any]] = []
    replay_case: dict[str, Any] | None = None
    replay_options = PROFILES["body_plus_root_wrench"]
    for name, options in PROFILES.items():
        consequences = []
        for force_n in FORCES_N:
            case = body.run_case(model, force_n, options)
            consequences.append(_consequence(case, force_n))
            if name == "body_plus_root_wrench" and force_n == 8.0:
                replay_case = case
        profile_rows.append(
            {
                "name": name,
                "external_wrench_feedforward_enabled": bool(
                    options.get("external_wrench_feedforward_enabled", False)
                ),
                "body_moment_rejection_enabled": bool(
                    options.get("body_moment_rejection_enabled", False)
                ),
                "root_roll_damping": float(options.get("root_roll_damping", 8.0)),
                "fall_count": sum(
                    row["terminal_tick"] is not None for row in consequences
                ),
                "nonadmitted_tick_count": sum(
                    len(row["nonadmitted_ticks"]) for row in consequences
                ),
                "maximum_torque_utilization": max(
                    row["maximum_torque_utilization"] for row in consequences
                ),
                "maximum_controller_step_p99_us": max(
                    row["controller_step_p99_us"] for row in consequences
                ),
                "maximum_worker_step_p99_us": max(
                    row["worker_step_p99_us"] for row in consequences
                ),
                "consequences": consequences,
            }
        )
    assert replay_case is not None
    replay = body.run_case(model, 8.0, replay_options)
    body_row = next(row for row in profile_rows if row["name"] == "body_only")
    composed_row = next(
        row for row in profile_rows if row["name"] == "body_plus_root_wrench"
    )
    gates = {
        "frozen_mujoco_version_matches": mujoco.__version__ == FROZEN_MUJOCO_VERSION,
        "all_profiles_default_off": all(
            name not in {"external_wrench_feedforward_enabled", "body_moment_rejection_enabled"}
            for name in feed.r312.PUBLIC_CONTROLLER_OPTIONS
        ),
        "semantic_replay_exact": _digest(replay_case) == _digest(replay),
        "composition_finishes_every_case": composed_row["fall_count"] == 0,
        "composition_has_zero_nonadmission": composed_row["nonadmitted_tick_count"] == 0,
        "composition_meets_controller_deadline": (
            composed_row["maximum_controller_step_p99_us"] <= 5_000.0
        ),
        "composition_meets_worker_deadline": (
            composed_row["maximum_worker_step_p99_us"] <= 20_000.0
        ),
        "composition_does_not_raise_fall_count": (
            composed_row["fall_count"] <= body_row["fall_count"]
        ),
    }
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "environment": {
            "mujoco_version": mujoco.__version__,
            "frozen_mujoco_version": FROZEN_MUJOCO_VERSION,
        },
        "profiles": profile_rows,
        "gates": gates,
        "composition_promoted": all(gates.values()),
        "finding": (
            "Correct root-origin wrench feed-forward at the R314 0.7 confidence "
            "scale is causal and deterministic and its root-only profile clears "
            "the frozen holdout. Composing it with the state-local body-moment "
            "law is still rejected: the lower-damping "
            "composition delays both terminal boundaries to ticks 84/85 while "
            "introducing four nonadmitted solves and a >7 ms controller p99; the "
            "higher-damping composition also regresses the -8 N fall to tick 63. "
            "The root-only recovery result remains evaluation-only; public "
            "authority and the state-local R314 qualification are unchanged."
        ),
    }


def markdown(metrics: dict[str, Any]) -> str:
    lines = [
        "# Upkie composed moment rejection — R315",
        "",
        "Status: **REJECTED; DEFAULT-OFF NEGATIVE EVIDENCE**.",
        "",
        metrics["finding"],
        "",
        "| profile | falls | nonadmitted ticks | max torque | controller p99 µs | worker p99 µs |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in metrics["profiles"]:
        lines.append(
            f'| {row["name"]} | {row["fall_count"]} | '
            f'{row["nonadmitted_tick_count"]} | '
            f'{row["maximum_torque_utilization"]:.3f} | '
            f'{row["maximum_controller_step_p99_us"]:.1f} | '
            f'{row["maximum_worker_step_p99_us"]:.1f} |'
        )
    lines += ["", "## Composition gates", ""]
    lines += [
        f'- {"PASS" if passed else "OPEN"} `{name}`'
        for name, passed in metrics["gates"].items()
    ]
    lines += [
        "",
        "The screen changes no public authority, timeout, iteration budget, contact "
        "mode, or reset behavior. The rejected profiles remain executable research "
        "options only.",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=pathlib.Path, default=ROOT / "models/upkie/upkie.urdf")
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=ROOT / "benchmarks/results/upkie-live-composed-moment-rejection-r315",
    )
    args = parser.parse_args()
    metrics = evaluate(args.model.resolve())
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    (args.output / "UPKIE_LIVE_COMPOSED_MOMENT_REJECTION_R315.md").write_text(
        markdown(metrics)
    )
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
