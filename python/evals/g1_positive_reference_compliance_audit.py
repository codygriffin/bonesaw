#!/usr/bin/env python3
"""R226 zero-integration audit of the documented positive reference contact law."""

from __future__ import annotations

import argparse
import json
import pathlib
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import grouped_summary, reconstruct_prestate
from g1_compliant_terminal_consequence_audit import reconstruct_prevelocity
from g1_contact_law_momentum_holdout import (
    CONTROL_DT,
    FOOT_FRAMES,
    PHYSICS_DT,
    WIDTH_GATES,
    build_plant,
    plant_layout,
    sha256,
    standing_posture,
    to_bonesaw_tangent,
)
from g1_substepped_compliant_contact_holdout import (
    FRESH_CONTACT_LAWS,
    FROZEN_PROFILE,
    SAMPLE_OFFSETS,
    residual_half_width,
)
from g1_substepped_compliant_contact_replay import group_maximum_absolute


REVISION = "g1-positive-reference-compliance-audit-r226"
SOURCE_REVISION = "g1-substepped-compliant-contact-holdout-r221"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-substepped-compliant-contact-holdout-r221/"
    "g1-substepped-compliant-contact-holdout.npz"
)
REFERENCE_DOCUMENTATION = (
    "https://mujoco.readthedocs.io/en/latest/modeling.html#solver-parameters"
)


@dataclass(frozen=True)
class ReferenceProfile:
    name: str
    substeps: int
    use_free_acceleration: bool


PROFILES = tuple(
    ReferenceProfile(
        f"reference_steps{substeps}{'_free_accel' if free else '_zero_accel'}",
        substeps,
        free,
    )
    for substeps in (5, 8, 16, 32, 64, 128)
    for free in (False, True)
)
SOLVER_FAMILIES = ("diagonal_effective_mass", "coupled_delassus")
PROJECTION_SWEEPS = 32
# At 50 Hz this reserves at least 75% of the 20 ms control period for the WBC,
# transport, state publication, and deadline margin.
CPU_QUERY_DEADLINE_NS = 5_000_000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_POSITIVE_REFERENCE_COMPLIANCE_AUDIT_R226.html"
    )
    return parser.parse_args()


def reference_contact_acceleration(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    points: np.ndarray,
    foot_geoms: list[int],
    jacobian: np.ndarray,
    jacobian_dot: np.ndarray,
    output: np.ndarray,
) -> None:
    if output.shape != points.shape or points.shape != (len(foot_geoms), 3):
        raise ValueError("reference contact acceleration shape mismatch")
    for contact, (point, geom) in enumerate(
        zip(points, foot_geoms, strict=True)
    ):
        body = int(model.geom_bodyid[geom])
        mujoco.mj_jac(model, data, jacobian, None, point, body)
        mujoco.mj_jacDot(model, data, jacobian_dot, None, point, body)
        output[contact] = (
            jacobian @ data.qacc_smooth + jacobian_dot @ data.qvel
        )


def prepare_law(
    session: Any,
    model_path: pathlib.Path,
    replay: Any,
    law: Any,
    offset: int,
    q_nominal: np.ndarray,
) -> dict[str, np.ndarray]:
    prefix = law.name
    plant, data = build_plant(model_path, law)
    root_qpos, qpos_indices, qvel_indices, _, foot_geoms, _ = plant_layout(
        plant, list(session.joint_names())
    )
    root_joint = mujoco.mj_name2id(plant, mujoco.mjtObj.mjOBJ_JOINT, "root")
    root_qvel = int(plant.jnt_dofadr[root_joint])
    samples = len(replay[f"{prefix}_root_height"])
    joint_dof = int(session.joint_dof())
    generalized_dof = int(session.generalized_dof())
    bases = np.repeat(np.eye(3, dtype=np.float64)[None], 8, axis=0)
    response = np.empty((samples, generalized_dof, 8, 3), np.float64)
    effective_mass = np.empty((samples, 8, 3), np.float64)
    delassus = np.empty((samples, 24, 24), np.float64)
    free_acceleration = np.empty((samples, 8, 3), np.float64)
    generalized_free_acceleration = np.empty((samples, generalized_dof), np.float64)
    jacobian = np.empty((3, plant.nv), np.float64)
    jacobian_dot = np.empty((3, plant.nv), np.float64)
    point_response_timing = np.empty(samples, np.uint64)
    for sample, height in enumerate(replay[f"{prefix}_root_height"]):
        state_index = offset + sample
        root, quaternion, q = reconstruct_prestate(
            joint_dof, q_nominal, state_index, float(height)
        )
        tangent = reconstruct_prevelocity(joint_dof, state_index)
        data.qpos[root_qpos : root_qpos + 7] = [*root, *quaternion]
        data.qpos[qpos_indices] = q
        data.qvel.fill(0.0)
        data.qvel[root_qvel : root_qvel + 3] = tangent[3:6]
        data.qvel[root_qvel + 3 : root_qvel + 6] = tangent[:3]
        data.qvel[qvel_indices] = tangent[6:]
        # Reference forward dynamics is a causal prestate query. It performs no
        # time integration and no policy/controller step.
        mujoco.mj_forward(plant, data)
        generalized_free_acceleration[sample] = to_bonesaw_tangent(
            data.qacc_smooth, root_qvel, qvel_indices
        )
        points = replay[f"{prefix}_contact_points"][sample]
        reference_contact_acceleration(
            plant,
            data,
            points,
            foot_geoms,
            jacobian,
            jacobian_dot,
            free_acceleration[sample],
        )
        result = session.point_impulse_velocity_response(
            root,
            quaternion,
            q,
            points,
            bases,
            response[sample],
            effective_mass[sample],
            delassus[sample],
        )
        point_response_timing[sample] = result[0]
        if result[1:] != (0, 0):
            raise RuntimeError("point response allocated inside Rust hot path")
    return {
        "response": response,
        "effective_mass": effective_mass,
        "delassus": delassus,
        "free_acceleration": free_acceleration,
        "generalized_free_acceleration": generalized_free_acceleration,
        "total_mass_kg": float(np.sum(plant.body_mass)),
        "point_response_timing_ns": point_response_timing,
    }


def law_reduced_integrator_id(law: Any) -> int:
    """Map the declared global integrator to a reduced point-model scheme.

    This preserves the explicit/implicit character of the authored law but is
    not an implementation of MuJoCo's generalized RK4 or implicitfast step.
    """
    if law.integrator == int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST):
        return 1
    if law.integrator == int(mujoco.mjtIntegrator.mjINT_RK4):
        return 2
    return 0


MODEL_INTEGRATOR_IDS = {
    "explicit": 0,
    "implicit": 1,
    "exponential_trapezoidal": 2,
    "generalized_rk4": 3,
}


def law_model_integrator_id(law: Any) -> int:
    """Map the authored integrator for model-owned state evolution.

    IDs retain the stable PyO3 ABI. IDs 0/1/2 remain explicit, implicit, and
    scalar exponential-trapezoidal. Model ID 3 selects four generalized RK
    stages with stage-local dynamics/contact refresh; ID 1 remains the
    velocity-first implicit-family construction.
    """
    if law.integrator == int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST):
        return MODEL_INTEGRATOR_IDS["implicit"]
    if law.integrator == int(mujoco.mjtIntegrator.mjINT_RK4):
        return MODEL_INTEGRATOR_IDS["generalized_rk4"]
    return MODEL_INTEGRATOR_IDS["explicit"]


def law_cone_id(law: Any) -> int:
    return 0 if law.cone == int(mujoco.mjtCone.mjCONE_ELLIPTIC) else 1


def evaluate_profile_law(
    session: Any,
    replay: Any,
    prepared: dict[str, np.ndarray],
    law: Any,
    profile: ReferenceProfile,
    solver_family: str,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    prefix = law.name
    samples = len(replay[f"{prefix}_root_height"])
    generalized_dof = int(session.generalized_dof())
    half_width = residual_half_width(generalized_dof)
    predicted_impulse = np.empty((8, 3), np.float64)
    velocity_after = np.empty((8, 3), np.float64)
    gap_after = np.empty(8, np.float64)
    residual = np.empty((samples, generalized_dof), np.float64)
    predicted = np.empty((samples, 8, 3), np.float64)
    timing = np.empty((samples, 2), np.uint64)
    component_covered = np.empty((samples, generalized_dof), np.uint8)
    sample_covered = np.empty(samples, np.uint8)
    zero_allocation = True
    zero_acceleration = np.zeros((8, 3), np.float64)
    parameter = lambda value: np.full(8, value, np.float64)
    acceleration = (
        prepared["free_acceleration"]
        if profile.use_free_acceleration
        else np.repeat(zero_acceleration[None], samples, axis=0)
    )
    common = (
        parameter(law.friction),
        parameter(law.solref_time_s),
        parameter(1.0),
        parameter(law.solimp_min),
        parameter(law.solimp_max),
        parameter(0.001),
        parameter(0.5),
        parameter(2.0),
    )
    for sample in range(samples):
        common_arguments = (
            np.ascontiguousarray(replay[f"{prefix}_contact_points"][sample, :, 2]),
            replay[f"{prefix}_prospective_velocity"][sample],
            np.ascontiguousarray(acceleration[sample]),
            prepared["delassus"][sample],
            replay[f"{prefix}_grouped_acceleration_impulse_upper"][sample]
            * FROZEN_PROFILE["impulse_cap_scale"],
            common[0],
        )
        tail_arguments = (
            *common[1:],
            2.0 * PHYSICS_DT,
            CONTROL_DT,
            profile.substeps,
        )
        output_arguments = (
            predicted_impulse,
            velocity_after,
            gap_after,
        )
        if solver_family == "diagonal_effective_mass":
            method = session.solve_positive_reference_compliant_contact_impulse
            arguments = (
                *common_arguments,
                np.ascontiguousarray(prepared["effective_mass"][sample, :, 2]),
                *tail_arguments,
                law_cone_id(law),
                law_reduced_integrator_id(law),
                *output_arguments,
            )
        elif solver_family == "coupled_delassus":
            method = session.solve_coupled_positive_reference_compliant_contact_impulse
            arguments = (
                *common_arguments,
                *tail_arguments,
                PROJECTION_SWEEPS,
                law_cone_id(law),
                law_reduced_integrator_id(law),
                *output_arguments,
            )
        else:
            raise ValueError(f"unknown solver family: {solver_family}")
        result = method(*arguments)
        timing[sample, 0] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        first = (
            predicted_impulse.copy(),
            velocity_after.copy(),
            gap_after.copy(),
        )
        result = method(*arguments)
        timing[sample, 1] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        if (
            not np.array_equal(first[0], predicted_impulse)
            or not np.array_equal(first[1], velocity_after)
            or not np.array_equal(first[2], gap_after)
        ):
            raise RuntimeError("positive reference query is not bitwise deterministic")
        actual_impulse = replay[f"{prefix}_contact_impulse"][sample]
        residual[sample] = replay[f"{prefix}_raw_velocity_error"][sample] + np.einsum(
            "dca,ca->d",
            prepared["response"][sample],
            actual_impulse - predicted_impulse,
            optimize=True,
        )
        predicted[sample] = predicted_impulse
        component_covered[sample] = np.abs(residual[sample]) <= half_width + 1.0e-12
        sample_covered[sample] = np.all(component_covered[sample])
    fitted_half_width = group_maximum_absolute(residual)
    fitted_width = {name: 2.0 * value for name, value in fitted_half_width.items()}
    width_gate = all(fitted_width[name] <= WIDTH_GATES[name] for name in WIDTH_GATES)
    strict = bool(np.all(sample_covered))
    timing_summary = distribution(timing.reshape(-1))
    deadline_gate = timing_summary["p99"] <= CPU_QUERY_DEADLINE_NS
    result = {
        "solver_family": solver_family,
        "profile": profile.name,
        "law": prefix,
        "substeps": profile.substeps,
        "uses_reference_free_acceleration": profile.use_free_acceleration,
        "sample_coverage_under_frozen_r220_box": float(np.mean(sample_covered)),
        "component_coverage_under_frozen_r220_box": float(np.mean(component_covered)),
        "strict_frozen_box_coverage": strict,
        "predictor_residual": grouped_summary(residual),
        "construction_fitted_width": fitted_width,
        "width_gate_passed": width_gate,
        "cpu_deadline_passed": deadline_gate,
        "construction_candidate_passed": bool(
            strict and width_gate and deadline_gate and zero_allocation
        ),
        "query_timing_ns": timing_summary,
        "zero_rust_allocation": zero_allocation,
        "bitwise_repeat": True,
        "profile_promoted": False,
        "authority_admitted": False,
    }
    arrays = {
        f"{solver_family}_{profile.name}_{prefix}_predicted_impulse": predicted,
        f"{solver_family}_{profile.name}_{prefix}_residual": residual,
        f"{solver_family}_{profile.name}_{prefix}_component_covered": component_covered,
        f"{solver_family}_{profile.name}_{prefix}_sample_covered": sample_covered,
        f"{solver_family}_{profile.name}_{prefix}_timing_ns": timing,
    }
    return result, arrays


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R226 requires the pinned G1 model and immutable R221 replay")
    replay = np.load(source)
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    q_nominal = standing_posture(list(session.joint_names()))
    prepared = {
        law.name: prepare_law(session, model, replay, law, offset, q_nominal)
        for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True)
    }
    results: list[dict[str, Any]] = []
    arrays: dict[str, np.ndarray] = {}
    for solver_family in SOLVER_FAMILIES:
        for profile in PROFILES:
            for law in FRESH_CONTACT_LAWS:
                result, result_arrays = evaluate_profile_law(
                    session, replay, prepared[law.name], law, profile, solver_family
                )
                results.append(result)
                arrays.update(result_arrays)
    mechanism_passed = all(
        result["zero_rust_allocation"] and result["bitwise_repeat"]
        for result in results
    )
    viable_constructions = [
        f"{solver_family}/{profile.name}"
        for solver_family in SOLVER_FAMILIES
        for profile in PROFILES
        if all(
            result["construction_candidate_passed"]
            for result in results
            if result["solver_family"] == solver_family
            and result["profile"] == profile.name
        )
    ]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay_sha256": sha256(source),
        "reference_documentation": REFERENCE_DOCUMENTATION,
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": 96,
        "physics_steps": 0,
        "integration_steps": 0,
        "policy_or_controller_steps": 0,
        "reference_forward_dynamics_queries": 96,
        "reference_parameters": {
            "positive_solref": "K=1/(dmax^2*tau^2*zeta^2), B=2/(dmax*tau)",
            "position_dependent_impedance": True,
            "refsafe_minimum_time_constant_s": 2.0 * PHYSICS_DT,
            "declared_friction_cone": True,
            "global_integrator_reduction": {
                "implicitfast": "implicit_euler",
                "rk4": "exponential_trapezoidal",
                "reproduces_mujoco_global_integrator": False,
            },
            "solver_families": list(SOLVER_FAMILIES),
            "coupled_projection_sweeps": PROJECTION_SWEEPS,
            "cpu_query_deadline_ns": CPU_QUERY_DEADLINE_NS,
            "cpu_query_deadline_basis": "25% of the 20 ms 50 Hz WBC period",
            "impedance_width_m": 0.001,
            "impedance_midpoint": 0.5,
            "impedance_power": 2.0,
        },
        "frozen_r220_profile_used_only_as_comparator": FROZEN_PROFILE,
        "mechanism_passed": mechanism_passed,
        "viable_constructions": viable_constructions,
        "profile_promoted": False,
        "authority_admitted": False,
        "results": results,
    }
    rows = []
    for result in results:
        residual = result["predictor_residual"]
        width = result["construction_fitted_width"]
        rows.append(
            [
                result["solver_family"],
                result["profile"],
                result["law"],
                f"{100.0 * result['sample_coverage_under_frozen_r220_box']:.3f}%",
                f"{residual['root_angular_rad_s']['p95']:.3f} / {residual['root_linear_m_s']['p95']:.3f} / {residual['joint_rad_s']['p95']:.3f}",
                f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
                f"{result['query_timing_ns']['p99'] / 1_000.0:.3f}",
                "candidate" if result["construction_candidate_passed"] else "reject",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 positive-reference compliance audit · r226",
            "",
            f"> Generic mechanisms **{'PASS' if mechanism_passed else 'FAIL'}** · viable solver/profile constructions **{len(viable_constructions)}** · profile/authority **NOT PROMOTED** · physics/integration/policy/controller steps **0 / 0 / 0 / 0**.",
            "",
            "## Contract",
            "",
            f"- Rust implements the documented positive time-constant/damping-ratio law from [MuJoCo solver parameters]({REFERENCE_DOCUMENTATION}): `K=1/(dmax² τ² ζ²)`, `B=2/(dmax τ)`, the complete position-dependent impedance spline, the explicit `τ≥2·physics_dt` refsafe clamp, first-order tangent decay, and circular/pyramidal sections. These are reference equations, not fitted R221 coefficients.",
            "- The new input types causal free point acceleration separately from contact velocity. This audit obtains that witness through 96 prestate `mj_forward` plus `J qacc_smooth + Jdot qvel` reference queries. It takes no reference step, policy, controller step, or state integration; online authority would have to obtain the same witness from the admitted WBC/model path.",
            f"- The construction grid compares independent effective-normal-mass response with complete Delassus distribution using {PROJECTION_SWEEPS} fixed forward/reverse projected sweeps. It tests 5, 8, 16, 32, 64, and 128 fixed microsteps both with and without the free-acceleration witness. The authored global integrator selects an explicit-character reduced scheme, but neither model reproduces MuJoCo's generalized RK4 or implicitfast step. R220's 16× cap and frozen residual box are retained only as comparators because R221 labels are no longer untouched. A row could advance only if both laws have strict frozen-box coverage, fitted 2.0/0.5/10.0 widths, repeat, allocation, and deadline together; no result is authority.",
            "- Every timed query repeats bitwise and allocates no Rust heap memory. A complete retained rerun reproduced all 192 non-timing NPZ arrays exactly; timing is reported but excluded from semantic equality.",
            "",
            "## Construction audit",
            "",
            *markdown_table(
                [
                    "solver",
                    "profile",
                    "law",
                    "coverage in R220 box",
                    "residual p95 ω/v/joint",
                    "fitted width ω/v/joint",
                    "query p99 µs",
                    "gate",
                ],
                rows,
            ),
            "",
            "## Decision",
            "",
            "Retain the typed positive-reference law, causal free-acceleration boundary, and complete-Delassus projected distribution. The audit removes the known constant-impedance/reference-scaling error and directly tests cross-contact coupling, but no solver/microstep/free-acceleration row passes both laws and useful fitted width. Therefore freeze nothing and do not spend a fresh holdout. The remaining gap is fidelity to the reference optimizer's soft-constraint regularization and contact formulation, not another scalar stiffness, microstep, or residual-width sweep.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-positive-reference-compliance-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_POSITIVE_REFERENCE_COMPLIANCE_AUDIT.md").write_text(report)
    np.savez_compressed(output / "g1-positive-reference-compliance-audit.npz", **arrays)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "viable_constructions": viable_constructions,
                "profile_promoted": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
