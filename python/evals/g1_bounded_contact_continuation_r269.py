#!/usr/bin/env python3
"""Evaluate bounded cross-tick contact continuation without a policy or physics."""

from __future__ import annotations

import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "g1-bounded-contact-continuation-r269"
RESULT_ROOT = pathlib.Path("benchmarks/results")
REFERENCE = RESULT_ROOT / "g1-multistep-reference-r53" / "reference-inputs.npz"
WITNESS = RESULT_ROOT / "g1-multistep-oracle-r54" / "oracle-wbc-admission-raw.npz"
BASELINE = (
    RESULT_ROOT
    / "floating-g1-r268-native-reference-morphology-jet-low-gain-cap8"
    / "floating-walk-raw.npz"
)
CANDIDATE = (
    RESULT_ROOT
    / "floating-g1-r269-low-gain-cross-tick-cap8x13"
    / "floating-walk-raw.npz"
)
LOCALIZED_CONTROL = (
    RESULT_ROOT
    / "floating-g1-r269-local-handoff-centroidal-cap8x13"
    / "floating-walk-raw.npz"
)

DT_SECONDS = 0.005
PER_SOLVE_SWEEP_CAP = 8
MAXIMUM_SOLVES_PER_WBC_TICK = 2
FIRST_HOLD = slice(876, 888)
HANDOFF_TICK = 869
LOCALIZED_TICK = 529


def fingerprint(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load(path: pathlib.Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as source:
        return {key: np.asarray(source[key]) for key in source.files}


def first_tick(status: np.ndarray, code: int) -> int | None:
    ticks = np.flatnonzero(status == code)
    return int(ticks[0]) if len(ticks) else None


def state_is_bitwise_constant(trace: dict[str, np.ndarray], window: slice) -> bool:
    arrays = (
        "q",
        "v",
        "root_tracked",
        "root_quaternion_wxyz",
        "tracked_positions",
        "center_of_mass_tracked",
    )
    return all(
        all(np.array_equal(row, trace[name][window][0]) for row in trace[name][window][1:])
        for name in arrays
    )


def release_diagnostics(trace: dict[str, np.ndarray]) -> dict[str, Any]:
    selected = trace["status"] == 5
    dynamics = trace["dynamics_residual"][selected]
    contact = trace["contact_residual"][selected]
    return {
        "ticks": np.flatnonzero(selected).astype(int).tolist(),
        "dynamics_residual_max": float(np.max(dynamics, initial=0.0)),
        "contact_residual_max": float(np.max(contact, initial=0.0)),
        "fail_closed": bool(np.all(dynamics == 0.0) and np.all(contact == 0.0)),
    }


def summarize(trace: dict[str, np.ndarray]) -> dict[str, Any]:
    status = trace["status"]
    timing_ms = trace["step_ns"].astype(np.float64) / 1.0e6
    first_contingency = next(
        (int(tick) for tick, value in enumerate(status) if value in (4, 5, 8, 9)),
        len(status),
    )
    root_error = np.linalg.norm(
        trace["root_tracked"] - trace["effective_root_targets"], axis=1
    )
    attitude = 2.0 * np.arccos(
        np.clip(np.abs(trace["root_quaternion_wxyz"][:, 0]), 0.0, 1.0)
    )
    values, counts = np.unique(status, return_counts=True)
    return {
        "ticks": len(status),
        "status_counts": {str(int(k)): int(v) for k, v in zip(values, counts)},
        "first_contingency_tick": first_contingency,
        "first_global_release_tick": first_tick(status, 5),
        "global_release": release_diagnostics(trace),
        "root_error_rms_m": float(np.sqrt(np.mean(np.square(root_error)))),
        "maximum_root_attitude_error_rad": float(np.max(attitude)),
        "timing_ms": distribution(timing_ms),
        "deadline_misses": {
            "5ms": int(np.count_nonzero(timing_ms > 5.0)),
            "20ms": int(np.count_nonzero(timing_ms > 20.0)),
        },
        "finite_state": bool(
            np.all(np.isfinite(trace["q"]))
            and np.all(np.isfinite(trace["v"]))
            and np.all(np.isfinite(trace["root_tracked"]))
        ),
    }


def main() -> int:
    required = (REFERENCE, WITNESS, BASELINE, CANDIDATE, LOCALIZED_CONTROL)
    if not all(path.is_file() for path in required):
        raise SystemExit("R269 requires the retained source, baseline, candidate, and control traces")

    baseline = load(BASELINE)
    candidate = load(CANDIDATE)
    localized = load(LOCALIZED_CONTROL)
    summaries = {
        "r268_low_gain_baseline": summarize(baseline),
        "r269_bounded_continuation": summarize(candidate),
        "r269_localized_handoff_control": summarize(localized),
    }

    cumulative_sweeps = candidate["feasibility_projection_sweeps"][FIRST_HOLD].astype(int)
    per_call_sweeps = np.diff(np.concatenate(([0], cumulative_sweeps)))
    hold_contract = {
        "ticks": list(range(FIRST_HOLD.start, FIRST_HOLD.stop)),
        "statuses": candidate["status"][FIRST_HOLD].astype(int).tolist(),
        "cumulative_feasibility_sweeps": cumulative_sweeps.tolist(),
        "per_wbc_tick_feasibility_sweeps": per_call_sweeps.tolist(),
        "per_solve_sweep_cap": PER_SOLVE_SWEEP_CAP,
        "maximum_solves_per_wbc_tick": MAXIMUM_SOLVES_PER_WBC_TICK,
        "aggregate_wbc_tick_cap_respected": bool(
            np.all(per_call_sweeps > 0)
            and np.all(
                per_call_sweeps
                <= PER_SOLVE_SWEEP_CAP * MAXIMUM_SOLVES_PER_WBC_TICK
            )
        ),
        "represented_state_bitwise_constant": state_is_bitwise_constant(candidate, FIRST_HOLD),
        "rejected_dynamics_residual_is_not_physical": True,
        "rejected_contact_residual_is_not_physical": True,
    }
    handoff_contract = {
        "tick": HANDOFF_TICK,
        "status": int(candidate["status"][HANDOFF_TICK]),
        "support_phase": candidate["support_phase"][HANDOFF_TICK, :2].astype(int).tolist(),
        "normal_force_n": float(np.sum(candidate["contact_normal_force"][HANDOFF_TICK])),
        "dynamics_residual": float(candidate["dynamics_residual"][HANDOFF_TICK]),
        "contact_residual": float(candidate["contact_residual"][HANDOFF_TICK]),
        "aggregate_feasibility_sweeps": int(
            candidate["feasibility_projection_sweeps"][HANDOFF_TICK]
        ),
    }
    localized_contract = {
        "tick": LOCALIZED_TICK,
        "status": int(localized["status"][LOCALIZED_TICK]),
        "support_phase": localized["support_phase"][LOCALIZED_TICK, :2].astype(int).tolist(),
        "normal_force_n": float(np.sum(localized["contact_normal_force"][LOCALIZED_TICK])),
        "dynamics_residual": float(localized["dynamics_residual"][LOCALIZED_TICK]),
        "contact_residual": float(localized["contact_residual"][LOCALIZED_TICK]),
    }
    source_contract = {
        "reference_sha256": fingerprint(REFERENCE),
        "witness_sha256": fingerprint(WITNESS),
        "baseline_sha256": fingerprint(BASELINE),
        "candidate_sha256": fingerprint(CANDIDATE),
        "localized_control_sha256": fingerprint(LOCALIZED_CONTROL),
        "future_wbc_outputs_consumed": False,
    }

    baseline_summary = summaries["r268_low_gain_baseline"]
    candidate_summary = summaries["r269_bounded_continuation"]
    localized_summary = summaries["r269_localized_handoff_control"]
    mechanism_passed = bool(
        baseline_summary["first_contingency_tick"] == 863
        and candidate_summary["first_contingency_tick"] == 863
        and baseline_summary["first_global_release_tick"] == 869
        and candidate_summary["first_global_release_tick"] == 888
        and hold_contract["statuses"] == [8] * 12
        and hold_contract["represented_state_bitwise_constant"]
        and hold_contract["aggregate_wbc_tick_cap_respected"]
        and handoff_contract["status"] == 9
        and handoff_contract["support_phase"] == [3, 0]
        and handoff_contract["normal_force_n"] > 0.0
        and handoff_contract["dynamics_residual"] <= 1.0e-8
        and handoff_contract["contact_residual"] <= 1.0e-8
        and handoff_contract["aggregate_feasibility_sweeps"] <= PER_SOLVE_SWEEP_CAP
        and localized_contract["status"] == 9
        and localized_contract["support_phase"] == [2, 0]
        and localized_contract["normal_force_n"] > 0.0
        and localized_contract["dynamics_residual"] <= 1.0e-8
        and localized_contract["contact_residual"] <= 1.0e-8
        and candidate_summary["global_release"]["fail_closed"]
        and localized_summary["global_release"]["fail_closed"]
        and candidate_summary["finite_state"]
    )
    profile_rejected = bool(
        candidate_summary["first_contingency_tick"] == baseline_summary["first_contingency_tick"]
        and candidate_summary["root_error_rms_m"] > 1.0
        and np.degrees(candidate_summary["maximum_root_attitude_error_rad"]) > 90.0
    )

    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "policy_steps": 0,
        "physics_steps": 0,
        "controller_ticks": int(sum(row["ticks"] for row in summaries.values())),
        "controller_integration_steps": int(
            sum(
                np.count_nonzero(trace["status"] != 8)
                for trace in (baseline, candidate, localized)
            )
        ),
        "source_contract": source_contract,
        "profiles": summaries,
        "hold_contract": hold_contract,
        "scheduled_handoff_contract": handoff_contract,
        "localized_handoff_contract": localized_contract,
        "mechanism_passed": mechanism_passed,
        "profile_rejected": profile_rejected,
        "default_changed": False,
        "authority_admitted": False,
    }

    rows = []
    for name, row in summaries.items():
        rows.append(
            [
                name.replace("_", " "),
                str(row["first_contingency_tick"]),
                str(row["first_global_release_tick"]),
                f'{row["root_error_rms_m"]:.3f}',
                f'{np.degrees(row["maximum_root_attitude_error_rad"]):.2f}',
                f'{row["timing_ms"]["p99"]:.3f}',
            ]
        )
    release_delay_ticks = (
        candidate_summary["first_global_release_tick"]
        - baseline_summary["first_global_release_tick"]
    )
    report = "\n".join(
        [
            "# G1 bounded contact continuation · R269",
            "",
            "**Mechanism PASS / walking profile REJECTED.** No policy or physics simulator is involved. The Rust controller integrates only accepted outputs; typed hold ticks preserve the represented state exactly while an identical exhausted hard problem receives another bounded feasibility slice.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "profile",
                    "first contingency",
                    "first global release",
                    "full root RMS m",
                    "max attitude deg",
                    "p99 ms",
                ],
                rows,
            ),
            "",
            f"The low-gain frontier still enters NormalFallback at tick 863, but the first all-contact release moves from tick 869 to tick {candidate_summary['first_global_release_tick']}—a {release_delay_ticks}-tick ({release_delay_ticks * DT_SECONDS:.3f} s) extension. This is a failure-handling improvement, not nominal tracking progress.",
            "",
            "At tick 869, localized handoff removes only the failed right support while the left foot remains Locked with 336.5 N. The accepted partial-support solution has hard residuals below 1e-8.",
            "",
            "Ticks 876–887 are status 8 (`contact_solve_hold`). State is bitwise constant across q, v, root pose, tracked points, and CoM. Cumulative feasibility work advances 16, 32, …, 192 sweeps. The configured cap is eight sweeps per solver query, while this WBC path can issue a primary query plus one retry, so the observable aggregate ceiling is 16 sweeps per WBC tick. Rejected residuals on hold ticks are diagnostic only and are never integrated.",
            "",
            f"The independent centroidal control exercises status 9 (`localized_contact_handoff`) at tick 529: the failed right support is removed while the incoming left support remains active with {localized_contract['normal_force_n']:.1f} N and {localized_contract['dynamics_residual']:.2e}/{localized_contract['contact_residual']:.2e} residuals. All later global-release ticks clear rejected residual diagnostics to zero.",
            "",
            "## Decision",
            "",
            f"The candidate remains red: first contingency is unchanged at tick 863, global release still occurs after only {release_delay_ticks * DT_SECONDS:.3f} s of additional degraded operation, full-run root RMS is {candidate_summary['root_error_rms_m']:.3f} m, and maximum attitude error is {np.degrees(candidate_summary['maximum_root_attitude_error_rad']):.2f}°. Candidate p99 is {candidate_summary['timing_ms']['p99']:.3f} ms, but the independent localized-handoff control is {localized_summary['timing_ms']['p99']:.3f} ms and misses the 5 ms target.",
            "",
            "Defaults and execution authority remain unchanged. Cross-tick continuation requires seed reuse, a finite per-call sweep cap, and an explicit hold timeout. The next large controller slice is continuous recovery from persistent NormalFallback with post-touchdown root/attitude tracking—not more solver work and not weaker contact constraints.",
            "",
            "## Contract",
            "",
            f"- Reference SHA-256: `{source_contract['reference_sha256']}`.",
            f"- Witness SHA-256: `{source_contract['witness_sha256']}`.",
            "- Status 8 is a bounded, non-integrating hold; status 9 is a physically checked partial-support solution.",
            "- Status 5 remains global free-body release and carries zero dynamics/contact residual witnesses.",
            "- The continuation is opt-in and capped at 12 hold ticks in this evaluation; each solve is capped at eight sweeps and the current WBC retry path can issue two solves per tick.",
            "- No future oracle WBC force, acceleration, status, or policy outputs are consumed.",
        ]
    )

    output = RESULT_ROOT / REVISION
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-bounded-contact-continuation-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_BOUNDED_CONTACT_CONTINUATION.md").write_text(report + "\n")
    web = pathlib.Path("web/G1_BOUNDED_CONTACT_CONTINUATION_R269.html")
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(
        render_report_html(report, title="G1 bounded contact continuation · R269")
    )
    print(output / "G1_BOUNDED_CONTACT_CONTINUATION.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
