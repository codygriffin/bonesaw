#!/usr/bin/env python3
"""R215 fresh G1 holdout for the coupled finite-patch transition set."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone

import mujoco
import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_contact_law_momentum_holdout import (
    CONTACT_LAWS as R213_CONTACT_LAWS,
    SPATIAL_PATCH_PROFILE,
    SUBSTEPS,
    WIDTH_GATES,
    ContactLaw,
    run_law,
    sha256,
)
from g1_kinetic_impulse_ellipsoid_holdout import FRESH_CONTACT_LAWS as R214_CONTACT_LAWS


REVISION = "g1-spatial-patch-transition-holdout-r215"
FRESH_CONTACT_LAWS = (
    ContactLaw(
        "compliant_elliptic_rk4",
        friction=0.45,
        solref_time_s=0.030,
        solimp_min=0.75,
        solimp_max=0.92,
        cone=int(mujoco.mjtCone.mjCONE_ELLIPTIC),
        integrator=int(mujoco.mjtIntegrator.mjINT_RK4),
    ),
    ContactLaw(
        "rigid_elliptic_implicit",
        friction=1.10,
        solref_time_s=0.0015,
        solimp_min=0.997,
        solimp_max=0.9999,
        cone=int(mujoco.mjtCone.mjCONE_ELLIPTIC),
        integrator=int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
    ),
)
SAMPLE_OFFSETS = (30_000, 40_000)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--samples-per-law", type=int, default=48)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_SPATIAL_PATCH_TRANSITION_HOLDOUT_R215.html"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples_per_law <= 0:
        raise SystemExit("samples-per-law must be positive")
    model = pathlib.Path(args.model).resolve()
    if not model.is_file():
        raise SystemExit(f"missing model: {model}")
    old_names = {law.name for law in (*R213_CONTACT_LAWS, *R214_CONTACT_LAWS)}
    if {law.name for law in FRESH_CONTACT_LAWS} & old_names:
        raise SystemExit("R215 contact-law names must be fresh")
    results = []
    replay: dict[str, np.ndarray] = {}
    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        result, arrays = run_law(
            model,
            law,
            args.samples_per_law,
            sample_offset=offset,
            reserve_kind="kinetic_ellipsoid",
            reserve_fraction=0.50,
            evaluate_spatial_patch=True,
        )
        results.append(result)
        replay.update(arrays)
    mechanism_passed = all(
        result["zero_rust_allocation"]
        and result["spatial_patch_transition"] is not None
        for result in results
    )
    promoted = all(
        result["spatial_patch_transition"]["profile_promoted"] for result in results
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": len(results) * args.samples_per_law,
        "physics_steps": len(results) * args.samples_per_law * SUBSTEPS,
        "policy_or_controller_steps": 0,
        "sample_reset_every_transition": True,
        "profile": SPATIAL_PATCH_PROFILE,
        "width_gates": WIDTH_GATES,
        "mechanism_passed": mechanism_passed,
        "profile_promoted": promoted,
        "authority_admitted": False,
        "results": results,
    }
    rows = []
    for result in results:
        patch = result["spatial_patch_transition"]
        width = patch["interval_width"]
        rows.append(
            [
                result["law"],
                result["samples"],
                f"{100.0 * patch['sample_coverage']:.3f}%",
                f"{100.0 * patch['component_coverage']:.4f}%",
                f"{width['root_angular_rad_s']['p95']:.3f}",
                f"{width['root_linear_m_s']['p95']:.3f}",
                f"{width['joint_rad_s']['p95']:.3f}",
                f"{patch['normal_impulse_upper_ns']['p95']:.3f}",
                f"{patch['bound_timing_ns']['p99'] / 1_000.0:.3f}",
                "yes" if patch["zero_rust_allocation"] else "NO",
                "PASS" if patch["profile_promoted"] else "REJECT",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 spatial-patch transition holdout · r215",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen profile **{'PROMOTED' if promoted else 'REJECTED'}** · authority **NOT ADMITTED** · controller/policy steps **0**.",
            "",
            "## Contract",
            "",
            "- Rust replaces four independent point boxes per foot with one resultant spatial wrench. Tangential impulse is passive-slip/Coulomb limited; CoP moments and force share the same nonnegative normal impulse. The exact support of this finite-patch box-pyramid is evaluated per generalized coordinate without vertex allocation.",
            "- Before either fresh law ran, the profile was frozen at the r212 50/10/50 generalized acceleration reserve, 100 m/s² and 8 N tangential witnesses, a 10 m/s² normal speed reserve, one whole-body weight of sustained normal load, authored 85×30 mm sole half-extents, restitution upper 1, and zero unmodeled torsional friction.",
            "- Fresh state sequences begin at offsets 30,000 and 40,000. Compliant/elliptic/RK4 and rigid/elliptic/implicit laws differ from r213 and r214. Every sample resets; completed state is scoring-only.",
            "",
            "## Fresh result",
            "",
            *markdown_table(
                [
                    "contact law", "samples", "sample coverage", "component coverage",
                    "root ω width p95", "root v width p95", "joint width p95",
                    "normal upper p95 N·s", "bound p99 µs", "zero alloc", "profile",
                ],
                rows,
            ),
            "",
            "## Decision",
            "",
            (
                "The coupled patch crosses both fresh laws at useful width. It remains a model profile only until plant consequence, deadlines, typed provenance, and hardware gates pass."
                if promoted
                else "Finite-patch coupling is retained as model machinery but the frozen profile is rejected. It removes the eight-point Cartesian product yet full corner-of-patch moment uncertainty still amplifies through small ankle inertias. No holdout quantile is fed back. A useful construction needs a causal center contact prediction plus a state-conditioned kinetic residual, rather than the full patch reachable set as one tick-wide command authority."
            ),
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-spatial-patch-transition-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_SPATIAL_PATCH_TRANSITION_HOLDOUT.md").write_text(report)
    np.savez_compressed(output / "g1-spatial-patch-transition-replay.npz", **replay)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "profile_promoted": promoted,
                "sample_coverage": {
                    result["law"]: result["spatial_patch_transition"]["sample_coverage"]
                    for result in results
                },
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
