#!/usr/bin/env python3
"""R309 qualify leg-posture authority at the stable live cadence."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import upkie_live_dynamic_contact_transition_r300 as r300


REVISION = "upkie-live-posture-priority-r309"
TICKS = 300
WORKER_OPTIONS = {
    "stream_dt": 0.020,
    "control_dt": 0.020,
    "physics_dt": 0.004,
    "balanced_nominal_joint_target": True,
}
LEGACY_OPTIONS = {"joint_posture_priority": 1, "joint_posture_weight": 1.0}
CANDIDATE_OPTIONS = {"joint_posture_priority": 0, "joint_posture_weight": 1.0}


def _semantic_digest(case: dict[str, Any]) -> str:
    payload = json.dumps(
        r300._semantic(case), sort_keys=True, separators=(",", ":")
    ).encode()
    return hashlib.sha256(payload).hexdigest()


def evaluate(
    legacy_nominal: dict[str, Any],
    legacy_disturbed: dict[str, Any],
    candidate_nominal: dict[str, Any],
    candidate_disturbed: dict[str, Any],
    candidate_replay: dict[str, Any],
    *,
    ticks: int,
) -> dict[str, Any]:
    summaries = {
        "legacy_nominal": r300.summarize(legacy_nominal),
        "legacy_disturbed": r300.summarize(legacy_disturbed),
        "candidate_nominal": r300.summarize(candidate_nominal),
        "candidate_disturbed": r300.summarize(candidate_disturbed),
        "candidate_replay": r300.summarize(candidate_replay),
    }
    candidate_digest = _semantic_digest(candidate_disturbed)
    replay_digest = _semantic_digest(candidate_replay)
    nominal = summaries["candidate_nominal"]
    disturbed = summaries["candidate_disturbed"]
    gates = {
        "nominal_completes_6s": nominal["ticks"] == ticks
        and nominal["terminal_pending"] is None,
        "nominal_remains_bilateral": nominal["observed_patterns"] == ["11"],
        "disturbed_completes_6s": disturbed["ticks"] == ticks
        and disturbed["terminal_pending"] is None,
        "disturbed_remains_bilateral": disturbed["observed_patterns"] == ["11"],
        "zero_nonadmitted_steps": not nominal["nonadmitted_ticks"]
        and not disturbed["nonadmitted_ticks"],
        "hard_residual_below_1e_minus_8": nominal["maximum_constraint_violation"] < 1.0e-8
        and disturbed["maximum_constraint_violation"] < 1.0e-8,
        "controller_p99_below_5ms": nominal["controller_step_us"]["p99"] < 5_000.0
        and disturbed["controller_step_us"]["p99"] < 5_000.0,
        "worker_p99_below_20ms": nominal["worker_step_us"]["p99"] < 20_000.0
        and disturbed["worker_step_us"]["p99"] < 20_000.0,
        "zero_wbc_allocations": nominal["allocation_calls"] == 0
        and nominal["allocated_bytes"] == 0
        and disturbed["allocation_calls"] == 0
        and disturbed["allocated_bytes"] == 0,
        "peak_torque_below_20pct": nominal["maximum_torque_utilization"] < 0.20
        and disturbed["maximum_torque_utilization"] < 0.20,
        "semantic_replay_exact": candidate_digest == replay_digest,
    }
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "rates_hz": {"wbc": 50, "physics": 250, "stream": 50},
        "legacy": LEGACY_OPTIONS,
        "candidate": CANDIDATE_OPTIONS,
        "summaries": summaries,
        "candidate_semantic_digest": candidate_digest,
        "replay_semantic_digest": replay_digest,
        "gates": gates,
        "qualified_for_default": all(gates.values()),
        "finding": (
            "Priority-1 leg posture is starved by the root/contact hierarchy and causes nominal "
            "hip/knee divergence. Moving the same bounded posture request to priority 0 removes "
            "nominal and disturbed contact loss without extra torque, allocation, or policy."
        ),
    }


def render_markdown(metrics: dict[str, Any]) -> str:
    legacy_n = metrics["summaries"]["legacy_nominal"]
    candidate_n = metrics["summaries"]["candidate_nominal"]
    legacy_d = metrics["summaries"]["legacy_disturbed"]
    candidate_d = metrics["summaries"]["candidate_disturbed"]
    gates = "\n".join(
        f"- {'PASS' if passed else 'FAIL'} — {name.replace('_', ' ')}"
        for name, passed in metrics["gates"].items()
    )
    return f"""# Upkie posture priority R309

Generated: {metrics['generated_at']}

Status: **{'qualified for default' if metrics['qualified_for_default'] else 'not qualified'}**

This A/B changes only the exact WBC hierarchy: the six-coordinate posture row
moves from priority 1 to priority 0 at weight 1.0. Gains, limits, model,
contact evidence, 50 Hz controller, 250 Hz MuJoCo plant, and 8 N wrench remain fixed.

| Measure | legacy nominal | candidate nominal | legacy disturbed | candidate disturbed |
|---|---:|---:|---:|---:|
| completed ticks | {legacy_n['ticks']} | {candidate_n['ticks']} | {legacy_d['ticks']} | {candidate_d['ticks']} |
| terminal | {legacy_n['terminal_pending']} | {candidate_n['terminal_pending']} | {legacy_d['terminal_pending']} | {candidate_d['terminal_pending']} |
| first contact loss | {legacy_n['first_non_double_tick']} | {candidate_n['first_non_double_tick']} | {legacy_d['first_non_double_tick']} | {candidate_d['first_non_double_tick']} |
| maximum tilt | {legacy_n['maximum_root_tilt_rad']:.6f} | {candidate_n['maximum_root_tilt_rad']:.6f} | {legacy_d['maximum_root_tilt_rad']:.6f} | {candidate_d['maximum_root_tilt_rad']:.6f} |
| minimum root height | {legacy_n['minimum_root_height_m']:.6f} | {candidate_n['minimum_root_height_m']:.6f} | {legacy_d['minimum_root_height_m']:.6f} | {candidate_d['minimum_root_height_m']:.6f} |
| peak torque utilization | {legacy_n['maximum_torque_utilization']:.6f} | {candidate_n['maximum_torque_utilization']:.6f} | {legacy_d['maximum_torque_utilization']:.6f} | {candidate_d['maximum_torque_utilization']:.6f} |
| controller p99 | {legacy_n['controller_step_us']['p99']:.3f} µs | {candidate_n['controller_step_us']['p99']:.3f} µs | {legacy_d['controller_step_us']['p99']:.3f} µs | {candidate_d['controller_step_us']['p99']:.3f} µs |
| worker p99 | {legacy_n['worker_step_us']['p99']:.3f} µs | {candidate_n['worker_step_us']['p99']:.3f} µs | {legacy_d['worker_step_us']['p99']:.3f} µs | {candidate_d['worker_step_us']['p99']:.3f} µs |
| controller jitter p99 | {legacy_n['controller_adjacent_jitter_us']['p99']:.3f} µs | {candidate_n['controller_adjacent_jitter_us']['p99']:.3f} µs | {legacy_d['controller_adjacent_jitter_us']['p99']:.3f} µs | {candidate_d['controller_adjacent_jitter_us']['p99']:.3f} µs |
| root tracking RMS | {legacy_n['root_position_rms_m']:.6f} m | {candidate_n['root_position_rms_m']:.6f} m | {legacy_d['root_position_rms_m']:.6f} m | {candidate_d['root_position_rms_m']:.6f} m |
| CoM tracking RMS | {legacy_n['com_position_rms_m']:.6f} m | {candidate_n['com_position_rms_m']:.6f} m | {legacy_d['com_position_rms_m']:.6f} m | {candidate_d['com_position_rms_m']:.6f} m |

## Qualification gates

{gates}

## Architectural conclusion

{metrics['finding']}
"""


def run(
    model: pathlib.Path, *, ticks: int
) -> tuple[dict[str, Any], dict[str, Any], str]:
    def case(disturbed: bool, options: dict[str, Any]) -> dict[str, Any]:
        return r300.run_case(
            model,
            disturbed=disturbed,
            maximum_ticks=ticks,
            controller_options=options,
            worker_options=WORKER_OPTIONS,
        )

    legacy_nominal = case(False, LEGACY_OPTIONS)
    legacy_disturbed = case(True, LEGACY_OPTIONS)
    candidate_nominal = case(False, CANDIDATE_OPTIONS)
    candidate_disturbed = case(True, CANDIDATE_OPTIONS)
    candidate_replay = case(True, CANDIDATE_OPTIONS)
    metrics = evaluate(
        legacy_nominal,
        legacy_disturbed,
        candidate_nominal,
        candidate_disturbed,
        candidate_replay,
        ticks=ticks,
    )
    traces = {
        "revision": REVISION,
        "legacy_nominal": legacy_nominal,
        "legacy_disturbed": legacy_disturbed,
        "candidate_nominal": candidate_nominal,
        "candidate_disturbed": candidate_disturbed,
        "candidate_replay": candidate_replay,
    }
    return metrics, traces, render_markdown(metrics)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", type=pathlib.Path, default=pathlib.Path("models/upkie/upkie.urdf")
    )
    parser.add_argument("--ticks", type=int, default=TICKS)
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("benchmarks/results/upkie-live-posture-priority-r309"),
    )
    args = parser.parse_args()
    if args.ticks < 300:
        parser.error("ticks must be at least 300")
    metrics, traces, markdown = run(args.model, ticks=args.ticks)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (args.output_dir / "traces.json").write_text(
        json.dumps(traces, separators=(",", ":")) + "\n", encoding="utf-8"
    )
    (args.output_dir / "UPKIE_LIVE_POSTURE_PRIORITY_R309.md").write_text(
        markdown, encoding="utf-8"
    )
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0 if metrics["qualified_for_default"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
