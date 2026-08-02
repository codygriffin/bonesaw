#!/usr/bin/env python3
"""R252 spent-state passive damping action audit.

This is a policy-free design audit on the already-spent R248 states. It tests
an actuator-coordinate damping request behind Rust's bandwidth/slew response
and a zero-positive-power cap. No gain is promoted unless every terminal harm
component is nonregressing; the passivity mechanism alone is not authority.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_compliant_terminal_consequence_audit import joint_limits
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256, standing_posture
from g1_terminal_box_wbc_action_audit import make_wbc_session
from g1_terminal_box_wbc_plant_ab import (
    FRESH_PLANT_LAWS,
    HEADROOM_INDEX,
    NONREGRESSION_TOLERANCE,
    PHYSICS_DT,
    PRESSURE_INDICES,
    SAMPLE_OFFSETS,
    SUBSTEPS,
    build_plant,
    copy_state,
    plant_layout,
    prepare_initial_state,
    score_terminal_state,
    warning_count,
)


REVISION = "g1-passive-damping-action-audit-r252"
SOURCE_REVISION = "g1-actuator-bandwidth-action-freeze-r250"
SOURCE_METRICS = pathlib.Path(
    "benchmarks/results/g1-actuator-bandwidth-action-freeze-r250/"
    "g1-actuator-bandwidth-action-freeze-metrics.json"
)
REALIZATION_PROFILE = (25.0, 1_000.0)
DAMPING_GAINS_NM_PER_RAD_S = (1.0, 2.0, 5.0, 10.0)
MAXIMUM_POSITIVE_POWER_W = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-metrics", default=str(SOURCE_METRICS))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_PASSIVE_DAMPING_ACTION_AUDIT_R252.html"
    )
    return parser.parse_args()


def one_step_outputs(dof: int) -> dict[str, np.ndarray]:
    return {
        "limited": np.empty((1, dof), np.float64),
        "realized": np.empty((1, dof), np.float64),
        "error": np.empty((1, dof), np.float64),
        "power": np.empty((1, dof), np.float64),
        "availability": np.empty((1, dof), np.uint8),
        "slew": np.empty((1, dof), np.uint8),
        "passivity": np.empty((1, dof), np.uint8),
        "step_ns": np.empty(1, np.uint64),
        "allocation_calls": np.empty(1, np.uint64),
        "allocated_bytes": np.empty(1, np.uint64),
    }


def rollout_passive_damping(
    bonesaw: Any,
    model: mujoco.MjModel,
    initial: mujoco.MjData,
    joint_qvel: list[int],
    effort_limits: np.ndarray,
    gain: float,
) -> dict[str, Any]:
    dof = len(joint_qvel)
    session = bonesaw.ActuatorRealizationSession(
        np.tile(np.asarray(REALIZATION_PROFILE), (dof, 1)),
        np.zeros(dof, np.float64),
    )
    output = one_step_outputs(dof)
    data = copy_state(model, initial)
    effort_trace = np.empty((SUBSTEPS, dof), np.float64)
    power_trace = np.empty_like(effort_trace)
    passivity_trace = np.empty((SUBSTEPS, dof), np.uint8)
    step_ns = np.empty(SUBSTEPS, np.uint64)
    allocation_calls = np.empty(SUBSTEPS, np.uint64)
    allocated_bytes = np.empty(SUBSTEPS, np.uint64)
    warnings_before = warning_count(data)
    available = effort_limits[None, :]
    for step in range(SUBSTEPS):
        velocity = np.ascontiguousarray(data.qvel[joint_qvel])[None, :]
        requested = -gain * velocity
        session.run_passivity_limited_trace(
            requested,
            available,
            velocity,
            MAXIMUM_POSITIVE_POWER_W,
            PHYSICS_DT,
            output["limited"],
            output["realized"],
            output["error"],
            output["power"],
            output["availability"],
            output["slew"],
            output["passivity"],
            output["step_ns"],
            output["allocation_calls"],
            output["allocated_bytes"],
        )
        effort_trace[step] = output["realized"][0]
        power_trace[step] = output["power"][0]
        passivity_trace[step] = output["passivity"][0]
        step_ns[step] = output["step_ns"][0]
        allocation_calls[step] = output["allocation_calls"][0]
        allocated_bytes[step] = output["allocated_bytes"][0]
        data.qfrc_applied.fill(0.0)
        data.qfrc_applied[joint_qvel] = output["realized"][0]
        mujoco.mj_step(model, data)
    mujoco.mj_energyVel(model, data)
    return {
        "data": data,
        "effort_trace": effort_trace,
        "power_trace": power_trace,
        "passivity_trace": passivity_trace,
        "step_ns": step_ns,
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
        "warnings": warning_count(data) - warnings_before,
    }


def rollout_zero(
    model: mujoco.MjModel, initial: mujoco.MjData, joint_qvel: list[int]
) -> mujoco.MjData:
    data = copy_state(model, initial)
    for _ in range(SUBSTEPS):
        data.qfrc_applied.fill(0.0)
        mujoco.mj_step(model, data)
    mujoco.mj_energyVel(model, data)
    return data


def main() -> int:
    import bonesaw

    args = parse_args()
    model_path = pathlib.Path(args.model).resolve()
    source_path = pathlib.Path(args.source_metrics).resolve()
    if not model_path.is_file() or not source_path.is_file():
        raise SystemExit("R252 requires the pinned G1 model and immutable R250 metrics")
    source = json.loads(source_path.read_text())
    if source.get("revision") != SOURCE_REVISION:
        raise ValueError("R252 source revision mismatch")
    source_hash_before = sha256(source_path)
    wbc = make_wbc_session(bonesaw, model_path)
    joint_names = list(wbc.joint_names)
    q_nominal = standing_posture(joint_names)
    limits = joint_limits(model_path, joint_names)
    effort_limits = np.asarray(wbc.actuator_effort_limits, np.float64)
    selector = bonesaw.ContactTransitionModelSession(
        str(model_path), [FOOT_FRAMES[0]] * 2 + [FOOT_FRAMES[1]] * 2
    )
    samples = len(FRESH_PLANT_LAWS) * 48
    baseline_diagnostics = np.empty((samples, 17), np.float64)
    baseline_energy = np.empty(samples, np.float64)
    result_arrays: dict[str, np.ndarray] = {}
    gain_results: list[dict[str, Any]] = []

    for gain in DAMPING_GAINS_NM_PER_RAD_S:
        diagnostics = np.empty((samples, 17), np.float64)
        energy = np.empty(samples, np.float64)
        effort_trace = np.empty((samples, SUBSTEPS, len(joint_names)), np.float64)
        power_trace = np.empty_like(effort_trace)
        passivity_trace = np.empty_like(effort_trace, dtype=np.uint8)
        step_ns = np.empty((samples, SUBSTEPS), np.uint64)
        allocation_calls = np.empty_like(step_ns)
        allocated_bytes = np.empty_like(step_ns)
        warnings = np.empty(samples, np.uint16)
        exact_repeat = np.ones(samples, np.uint8)
        row = 0
        for law, offset in zip(FRESH_PLANT_LAWS, SAMPLE_OFFSETS, strict=True):
            model, initial = build_plant(model_path, law)
            root_qpos, joint_qpos, joint_qvel, _, foot_geoms, _ = plant_layout(
                model, joint_names
            )
            root_qvel = int(
                model.jnt_dofadr[
                    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
                ]
            )
            for sample in range(48):
                prepare_initial_state(
                    model,
                    initial,
                    root_qpos,
                    root_qvel,
                    joint_qpos,
                    joint_qvel,
                    foot_geoms,
                    q_nominal,
                    offset + sample,
                )
                if gain == DAMPING_GAINS_NM_PER_RAD_S[0]:
                    baseline = rollout_zero(model, initial, joint_qvel)
                    baseline_diagnostics[row], _ = score_terminal_state(
                        selector,
                        baseline,
                        root_qpos,
                        root_qvel,
                        joint_qpos,
                        joint_qvel,
                        limits,
                        0.0,
                    )
                    baseline_energy[row] = baseline.energy[1]
                first = rollout_passive_damping(
                    bonesaw, model, initial, joint_qvel, effort_limits, gain
                )
                second = rollout_passive_damping(
                    bonesaw, model, initial, joint_qvel, effort_limits, gain
                )
                utilization = float(
                    np.max(np.abs(first["effort_trace"]) / effort_limits[None, :])
                )
                diagnostics[row], _ = score_terminal_state(
                    selector,
                    first["data"],
                    root_qpos,
                    root_qvel,
                    joint_qpos,
                    joint_qvel,
                    limits,
                    utilization,
                )
                energy[row] = first["data"].energy[1]
                effort_trace[row] = first["effort_trace"]
                power_trace[row] = first["power_trace"]
                passivity_trace[row] = first["passivity_trace"]
                step_ns[row] = first["step_ns"]
                allocation_calls[row] = first["allocation_calls"]
                allocated_bytes[row] = first["allocated_bytes"]
                warnings[row] = first["warnings"] + second["warnings"]
                exact_repeat[row] = int(
                    np.array_equal(first["data"].qpos, second["data"].qpos)
                    and np.array_equal(first["data"].qvel, second["data"].qvel)
                    and np.array_equal(first["effort_trace"], second["effort_trace"])
                    and np.array_equal(first["power_trace"], second["power_trace"])
                    and np.array_equal(
                        first["passivity_trace"], second["passivity_trace"]
                    )
                )
                row += 1

        pressure_delta = diagnostics[:, list(PRESSURE_INDICES)] - baseline_diagnostics[
            :, list(PRESSURE_INDICES)
        ]
        headroom_delta = diagnostics[:, HEADROOM_INDEX] - baseline_diagnostics[
            :, HEADROOM_INDEX
        ]
        component_regression = np.maximum(np.max(pressure_delta, axis=1), -headroom_delta)
        aggregate_delta = diagnostics[:, 16] - baseline_diagnostics[:, 16]
        harm_delta = diagnostics[:, 15] - baseline_diagnostics[:, 15]
        energy_delta = energy - baseline_energy
        prefix = f"gain_{gain:g}".replace(".", "p")
        result_arrays.update(
            {
                f"{prefix}_diagnostics": diagnostics,
                f"{prefix}_effort_trace": effort_trace,
                f"{prefix}_power_trace": power_trace,
                f"{prefix}_passivity_trace": passivity_trace,
                f"{prefix}_component_regression": component_regression,
                f"{prefix}_aggregate_delta": aggregate_delta,
                f"{prefix}_harm_delta": harm_delta,
                f"{prefix}_energy_delta": energy_delta,
                f"{prefix}_step_ns": step_ns,
                f"{prefix}_exact_repeat": exact_repeat,
            }
        )
        gain_results.append(
            {
                "gain_nm_per_rad_s": gain,
                "strict_nonregression_samples": int(
                    np.count_nonzero(component_regression <= NONREGRESSION_TOLERANCE)
                ),
                "aggregate_improved_samples": int(
                    np.count_nonzero(aggregate_delta < -NONREGRESSION_TOLERANCE)
                ),
                "harm_improved_samples": int(
                    np.count_nonzero(harm_delta < -NONREGRESSION_TOLERANCE)
                ),
                "kinetic_energy_reduced_samples": int(
                    np.count_nonzero(energy_delta < -NONREGRESSION_TOLERANCE)
                ),
                "component_regression": distribution(component_regression),
                "aggregate_delta": distribution(aggregate_delta),
                "harm_delta": distribution(harm_delta),
                "kinetic_energy_delta_j": distribution(energy_delta),
                "maximum_positive_mechanical_power_w": float(np.max(power_trace)),
                "passivity_clipped_coordinate_steps": int(
                    np.count_nonzero(passivity_trace)
                ),
                "realization_step_ns": distribution(step_ns.reshape(-1)),
                "zero_rust_allocation": bool(
                    np.all(allocation_calls == 0) and np.all(allocated_bytes == 0)
                ),
                "semantic_repeat_samples": int(np.count_nonzero(exact_repeat)),
                "mujoco_warning_count": int(np.sum(warnings)),
                "strict_action_passed": bool(
                    np.all(component_regression <= NONREGRESSION_TOLERANCE)
                    and np.any(aggregate_delta < -NONREGRESSION_TOLERANCE)
                ),
            }
        )

    source_immutable = source_hash_before == sha256(source_path)
    mechanism_passed = bool(
        source_immutable
        and all(
            row["maximum_positive_mechanical_power_w"]
            <= MAXIMUM_POSITIVE_POWER_W + 1.0e-12
            and row["zero_rust_allocation"]
            and row["semantic_repeat_samples"] == samples
            and row["mujoco_warning_count"] == 0
            for row in gain_results
        )
    )
    passing = [row for row in gain_results if row["strict_action_passed"]]
    selected = min(passing, key=lambda row: row["gain_nm_per_rad_s"]) if passing else None
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_metrics_sha256": source_hash_before,
        "source_immutable": source_immutable,
        "design_audit_not_holdout": True,
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "samples": samples,
        "realization_profile": REALIZATION_PROFILE,
        "damping_gains_nm_per_rad_s": DAMPING_GAINS_NM_PER_RAD_S,
        "maximum_positive_mechanical_power_w": MAXIMUM_POSITIVE_POWER_W,
        "physics_steps": samples
        * (1 + 2 * len(DAMPING_GAINS_NM_PER_RAD_S))
        * SUBSTEPS,
        "policy_steps": 0,
        "mechanism_passed": mechanism_passed,
        "selected_gain_for_fresh_plant": None
        if selected is None
        else selected["gain_nm_per_rad_s"],
        "fresh_plant_action_selected": selected is not None,
        "authority_admitted": False,
        "gains": gain_results,
    }
    rows = [
        [
            f"{row['gain_nm_per_rad_s']:.1f}",
            f"{row['strict_nonregression_samples']} / {samples}",
            f"{row['aggregate_improved_samples']} / {samples}",
            f"{row['kinetic_energy_reduced_samples']} / {samples}",
            str(row["passivity_clipped_coordinate_steps"]),
            f"{row['realization_step_ns']['p99'] / 1e3:.2f}",
            "PASS" if row["strict_action_passed"] else "REJECT",
        ]
        for row in gain_results
    ]
    report = "\n".join(
        [
            "# Bonesaw passive damping action audit · r252",
            "",
            f"> Passivity mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · useful strict action **{'FROZEN' if selected else 'NOT FOUND'}** · authority **NOT ADMITTED**.",
            "",
            "R252 evaluates actuator-coordinate `tau_request = -gain * velocity` at every 4 ms plant step behind the R249 25 Hz / 1,000 N·m/s realization and a Rust zero-positive-power cap. The 96 R248 states are spent design evidence; every nonzero branch runs twice for bitwise semantic replay. No policy runs.",
            "",
            *markdown_table(
                [
                    "gain Nm/(rad/s)",
                    "strict nonregression",
                    "aggregate improved",
                    "kinetic energy reduced",
                    "power clamps",
                    "p99 µs",
                    "action decision",
                ],
                rows,
            ),
            "",
            "The mechanism passes only if every applied actuator coordinate has non-positive observed mechanical power, every repeated plant trace is bitwise exact, timed Rust allocation is zero, and MuJoCo emits no warnings. An action would additionally require zero regression in every terminal component on all 96 states. Passivity is therefore visible evidence, not a substitute for the terminal consequence gate.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-passive-damping-action-audit-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-passive-damping-action-audit.npz",
        baseline_diagnostics=baseline_diagnostics,
        baseline_energy=baseline_energy,
        **result_arrays,
    )
    (output / "G1_PASSIVE_DAMPING_ACTION_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw passive damping · r252"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "selected_gain_for_fresh_plant": metrics[
                    "selected_gain_for_fresh_plant"
                ],
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
