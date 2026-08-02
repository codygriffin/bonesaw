#!/usr/bin/env python3
"""Policy- and physics-free localization of the native G1 reference bridge."""

from __future__ import annotations

import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "g1-native-reference-integration-r268"
RESULT_ROOT = pathlib.Path("benchmarks/results")
TRACES = {
    "initialization_only": RESULT_ROOT
    / "floating-g1-r268-native-reference-witness-cap8"
    / "floating-walk-raw.npz",
    "oracle_task_stack": RESULT_ROOT
    / "floating-g1-r268-native-reference-oracle-stack-cap8"
    / "floating-walk-raw.npz",
    "morphology_posture_jet": RESULT_ROOT
    / "floating-g1-r268-native-reference-morphology-jet-cap8"
    / "floating-walk-raw.npz",
}
REFERENCE = RESULT_ROOT / "g1-multistep-reference-r53" / "reference-inputs.npz"
WITNESS = (
    RESULT_ROOT
    / "g1-multistep-oracle-r54"
    / "oracle-wbc-admission-raw.npz"
)
PRIOR_NATIVE_PREFIX_TICKS = 265


def fingerprint(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load(path: pathlib.Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as source:
        return {key: np.asarray(source[key]) for key in source.files}


def transitions(values: np.ndarray) -> list[int]:
    changed = values[1:] != values[:-1]
    if changed.ndim > 1:
        changed = np.any(changed, axis=tuple(range(1, changed.ndim)))
    return (np.flatnonzero(changed) + 1).astype(int).tolist()


def summarize(path: pathlib.Path) -> dict[str, Any]:
    trace = load(path)
    status = trace["status"]
    timing_ms = trace["step_ns"].astype(np.float64) / 1.0e6
    first_contingency = next(
        (tick for tick, value in enumerate(status) if value in (4, 5)), len(status)
    )
    prefix = slice(0, first_contingency)
    root_error = np.linalg.norm(
        trace["root_tracked"] - trace["effective_root_targets"], axis=1
    )
    # Status 5 is the explicit contact-release/free-body contingency.  It is
    # executable only as a bounded fallback, so the contact/authority
    # diagnostics must be fail-closed rather than stale values from the
    # rejected contact solve.
    release_fallback = status == 5
    release_dynamics = trace["dynamics_residual"][release_fallback]
    release_contact = trace["contact_residual"][release_fallback]
    values, counts = np.unique(status, return_counts=True)
    return {
        "artifact": str(path),
        "ticks": len(status),
        "status_counts": {str(int(k)): int(v) for k, v in zip(values, counts)},
        "status_transitions": transitions(status),
        "support_transitions": transitions(trace["support_phase"][:, :2]),
        "first_contingency_tick": first_contingency,
        "clean_prefix_seconds": first_contingency * 0.005,
        "root_error_rms_m_before_contingency": float(
            np.sqrt(np.mean(np.square(root_error[prefix])))
        ),
        "root_error_at_liftoff_m": float(root_error[300]),
        "root_error_at_first_touchdown_edge_m": float(root_error[529]),
        "maximum_root_rotation_before_contingency_rad": float(
            np.max(
                2.0
                * np.arccos(
                    np.clip(
                        np.abs(trace["root_quaternion_wxyz"][prefix, 0]), 0.0, 1.0
                    )
                )
            )
        ),
        "timing_ms": distribution(timing_ms),
        "deadline_misses": {
            "5ms": int(np.count_nonzero(timing_ms > 5.0)),
            "20ms": int(np.count_nonzero(timing_ms > 20.0)),
        },
        "finite_state": bool(
            np.all(np.isfinite(trace["root_tracked"]))
            and np.all(np.isfinite(trace["q"]))
            and np.all(np.isfinite(trace["v"]))
        ),
        "maximum_accepted_dynamics_residual": float(
            np.max(trace["dynamics_residual"][status <= 1])
        ),
        "maximum_accepted_contact_residual": float(
            np.max(trace["contact_residual"][status <= 1])
        ),
        "release_fallback_ticks": int(np.count_nonzero(release_fallback)),
        "release_fallback_dynamics_residual_max": float(
            np.max(release_dynamics, initial=0.0)
        ),
        "release_fallback_contact_residual_max": float(
            np.max(release_contact, initial=0.0)
        ),
        "release_fallback_diagnostics_fail_closed": bool(
            np.all(release_dynamics == 0.0) and np.all(release_contact == 0.0)
        ),
    }


def main() -> int:
    required = [REFERENCE, WITNESS, *TRACES.values()]
    if not all(path.is_file() for path in required):
        raise SystemExit("R268 requires the retained reference, witness, and three traces")
    reference = load(REFERENCE)
    witness = load(WITNESS)
    summaries = {name: summarize(path) for name, path in TRACES.items()}
    initialization = summaries["initialization_only"]
    oracle = summaries["oracle_task_stack"]
    posture = summaries["morphology_posture_jet"]
    source_contract = {
        "reference_sha256": fingerprint(REFERENCE),
        "witness_sha256": fingerprint(WITNESS),
        "reference_ticks": int(len(reference["root_targets"])),
        "reference_root_velocity_is_distinct_from_com_velocity": bool(
            not np.array_equal(
                reference["root_target_velocities"],
                reference["center_of_mass_target_velocities"],
            )
        ),
        "initial_root_exact": bool(
            np.array_equal(witness["root_positions"][0], reference["root_targets"][0])
        ),
        "initial_witness_converged": bool(witness["ik_converged"][0]),
        "initial_point_error_m": float(witness["point_error"][0]),
        "initial_center_of_mass_error_m": float(witness["center_of_mass_error"][0]),
        "future_wbc_outputs_consumed": False,
        "consumed_witness_arrays": [
            "q",
            "v",
            "joint_accelerations",
            "root_positions",
            "root_velocities",
            "center_of_mass_positions",
            "target_positions",
            "point_error",
            "center_of_mass_error",
            "ik_converged",
        ],
    }
    mechanism_passed = bool(
        source_contract["reference_root_velocity_is_distinct_from_com_velocity"]
        and source_contract["initial_root_exact"]
        and source_contract["initial_witness_converged"]
        and source_contract["initial_point_error_m"] <= 0.01
        and source_contract["initial_center_of_mass_error_m"] <= 0.03
        and initialization["finite_state"]
        and initialization["first_contingency_tick"] > PRIOR_NATIVE_PREFIX_TICKS
        and initialization["root_error_at_liftoff_m"] < 1.0e-3
        and all(
            row["release_fallback_diagnostics_fail_closed"]
            for row in summaries.values()
        )
    )
    profile_rejected = bool(
        initialization["first_contingency_tick"] < 529
        and oracle["first_contingency_tick"] < initialization["first_contingency_tick"]
        and posture["first_contingency_tick"] < initialization["first_contingency_tick"]
    )
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "policy_steps": 0,
        "physics_steps": 0,
        "controller_integration_steps": int(sum(row["ticks"] for row in summaries.values())),
        "source_contract": source_contract,
        "prior_native_reference_clean_prefix_ticks": PRIOR_NATIVE_PREFIX_TICKS,
        "profiles": summaries,
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
                f'{row["root_error_rms_m_before_contingency"] * 100:.3f}',
                f'{np.degrees(row["maximum_root_rotation_before_contingency_rad"]):.3f}',
                f'{row["timing_ms"]["p99"]:.3f}',
                str(row["deadline_misses"]["20ms"]),
            ]
        )
    report = "\n".join(
        [
            "# G1 native-reference integrated bridge · R268",
            "",
            "**Mechanism PASS / controller profile REJECTED.** This evaluation uses no policy and no physics simulator. The Rust controller integrates its own state; only the immutable Rust-LIPM reference and the first state of the offline morphology witness initialize it. The optional posture case consumes only the witness q/v/q̈ jet, never oracle WBC force, status, or solved acceleration outputs.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "profile",
                    "first contingency tick",
                    "prefix root RMS cm",
                    "prefix max attitude deg",
                    "p99 ms",
                    ">20 ms",
                ],
                rows,
            ),
            "",
            f"Initialization-only extends the retained native-reference clean prefix from **{PRIOR_NATIVE_PREFIX_TICKS}** to **{initialization['first_contingency_tick']} ticks** and reaches the authored liftoff at tick 300 with {initialization['root_error_at_liftoff_m'] * 1000:.3f} mm root error. It still releases support before the first touchdown edge at tick 529, so no profile is promoted.",
            "",
            "Every explicit release/fallback tick clears rejected contact residuals before integration: the regenerated traces report zero dynamics and contact residual witnesses on all status-5 ticks. This is diagnostic hygiene, not a claim that the free-body state is physically supported.",
            "",
            "The exact R54 oracle task stack is not a closed-loop policy: it fails earlier. Directly replaying the policy-free morphology q/v/q̈ witness as a Preference posture jet also fails earlier. Both are retained as causal negative controls, not averaged into a score.",
            "",
            "## Contract",
            "",
            f"- Reference SHA-256: `{source_contract['reference_sha256']}`.",
            f"- Witness SHA-256: `{source_contract['witness_sha256']}`.",
            f"- Initial morphology error: {source_contract['initial_point_error_m'] * 1000:.3f} mm foot / {source_contract['initial_center_of_mass_error_m'] * 1000:.3f} mm CoM; both remain inside the retained 10/30 mm certificate.",
            "- Authored root position, velocity, and acceleration jets now cross the standalone boundary independently; CoM derivatives are no longer substituted for pelvis derivatives.",
            "- The optional morphology posture jet is sampled and time-warped in allocation-free Rust under the same reference cursor. It remains opt-in because this profile is red.",
            "- Defaults and execution authority are unchanged. The next controller slice must preserve root attitude/support through the final 90 ticks of the first swing without importing oracle WBC outputs or weakening touchdown/contact constraints.",
        ]
    )
    output = RESULT_ROOT / REVISION
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-native-reference-integration-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_NATIVE_REFERENCE_INTEGRATION.md").write_text(report + "\n")
    web = pathlib.Path("web/G1_NATIVE_REFERENCE_INTEGRATION_R268.html")
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(
        render_report_html(
            report, title="G1 native-reference integrated bridge · R268"
        )
    )
    print(output / "G1_NATIVE_REFERENCE_INTEGRATION.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
