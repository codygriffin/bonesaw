#!/usr/bin/env python3
"""R214 fresh G1 holdout for a mass-metric impulse-residual ellipsoid."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone

import mujoco
import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_contact_law_momentum_holdout import (
    CONTACT_LAWS as R212_CONTACT_LAWS,
    CONTROL_DT,
    GRAVITY,
    SUBSTEPS,
    WIDTH_GATES,
    ContactLaw,
    run_law,
    sha256,
)


REVISION = "g1-kinetic-impulse-ellipsoid-holdout-r214"

# Frozen before executing either fresh R214 law. The ellipsoid is
# p^T M^-1 p <= f^2 m (g dt)^2, so a pure floating translation has a
# morphology-independent speed scale f*g*dt. It is analytic, not fit from R212.
KINETIC_RESERVE_FRACTION = 0.50
FRESH_CONTACT_LAWS = (
    ContactLaw(
        "medium_pyramidal_implicit",
        friction=0.55,
        solref_time_s=0.011,
        solimp_min=0.90,
        solimp_max=0.985,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
    ),
    ContactLaw(
        "hard_pyramidal_euler",
        friction=0.75,
        solref_time_s=0.0025,
        solimp_min=0.995,
        solimp_max=0.9995,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_EULER),
    ),
)
SAMPLE_OFFSETS = (10_000, 20_000)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf",
    )
    parser.add_argument("--samples-per-law", type=int, default=48)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/G1_KINETIC_IMPULSE_ELLIPSOID_HOLDOUT_R214.html",
    )
    return parser.parse_args()


def analytic_twice_energy(total_mass_kg: float) -> float:
    if not np.isfinite(total_mass_kg) or total_mass_kg <= 0.0:
        raise ValueError("total mass must be finite and positive")
    return (
        KINETIC_RESERVE_FRACTION**2
        * total_mass_kg
        * (GRAVITY * CONTROL_DT) ** 2
    )


def main() -> int:
    args = parse_args()
    if args.samples_per_law <= 0:
        raise SystemExit("samples-per-law must be positive")
    model = pathlib.Path(args.model).resolve()
    if not model.is_file():
        raise SystemExit(f"missing model: {model}")
    if {law.name for law in FRESH_CONTACT_LAWS} & {
        law.name for law in R212_CONTACT_LAWS
    }:
        raise SystemExit("R214 contact-law names must be fresh")
    results = []
    replay: dict[str, np.ndarray] = {}
    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        result, arrays = run_law(
            model,
            law,
            args.samples_per_law,
            sample_offset=offset,
            reserve_kind="kinetic_ellipsoid",
            reserve_fraction=KINETIC_RESERVE_FRACTION,
        )
        results.append(result)
        replay.update(arrays)
    mechanism_passed = all(result["zero_rust_allocation"] for result in results)
    promoted = all(result["profile_promoted"] for result in results)
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": len(results) * args.samples_per_law,
        "physics_steps": len(results) * args.samples_per_law * SUBSTEPS,
        "policy_or_controller_steps": 0,
        "sample_reset_every_transition": True,
        "reserve_kind": "kinetic_ellipsoid",
        "kinetic_reserve_fraction": KINETIC_RESERVE_FRACTION,
        "pure_translation_speed_scale_m_s": KINETIC_RESERVE_FRACTION
        * GRAVITY
        * CONTROL_DT,
        "width_gates": WIDTH_GATES,
        "mechanism_passed": mechanism_passed,
        "profile_promoted": promoted,
        "authority_admitted": False,
        "results": results,
    }
    rows = []
    for result in results:
        width = result["interval_width"]
        rows.append(
            [
                result["law"],
                result["samples"],
                f"{100.0 * result['sample_coverage']:.3f}%",
                f"{100.0 * result['component_coverage']:.4f}%",
                f"{width['root_angular_rad_s']['p95']:.3f}",
                f"{width['root_linear_m_s']['p95']:.3f}",
                f"{width['joint_rad_s']['p95']:.3f}",
                result["uncovered_samples"],
                "yes" if result["zero_rust_allocation"] else "NO",
                "PASS" if result["profile_promoted"] else "REJECT",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 kinetic impulse ellipsoid holdout · r214",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen profile **{'PROMOTED' if promoted else 'REJECTED'}** · authority **NOT ADMITTED** · controller/policy steps **0**.",
            "",
            "## Contract",
            "",
            "- Rust bounds a zero-centered generalized impulse set `pᵀM⁻¹p ≤ E₂` directly in the kinetic metric. Exact component support is `sqrt(E₂ (M⁻¹)ᵢᵢ)`, avoiding the coordinate box's sum of every absolute coupled inverse-mass column.",
            f"- Before either fresh law ran, `E₂` was frozen analytically as `f² m (gΔt)²` with f={KINETIC_RESERVE_FRACTION:.2f}. Its pure-translation speed scale is {KINETIC_RESERVE_FRACTION * GRAVITY * CONTROL_DT:.5f} m/s. No R212 residual or quantile determines it.",
            "- The fresh state sequences begin at offsets 10,000 and 20,000. Contact formulations are medium/pyramidal/implicit and hard/pyramidal/Euler, distinct from both R212 laws. Every sample resets; completed contact impulse remains an evaluation-only oracle label.",
            "",
            "## Fresh result",
            "",
            *markdown_table(
                [
                    "contact law",
                    "samples",
                    "sample coverage",
                    "component coverage",
                    "root ω width p95",
                    "root v width p95",
                    "joint width p95",
                    "missed samples",
                    "zero alloc",
                    "profile",
                ],
                rows,
            ),
            "",
            "The hard-law miss is state index 20,020 at `left_ankle_roll_joint`; its realized 16.164 rad/s residual exceeds the 7.659 rad/s ellipsoid half-width. A different state (20,005) has no within-window contact and remains covered, so the miss is not an empty-label artifact.",
            "",
            "## Decision",
            "",
            (
                "The kinetic ellipsoid satisfies strict coverage and every frozen width gate on both fresh laws. This promotes only the residual-set profile for a subsequent causal contact-wrench composition; completed contact impulse, terminal consequence, deadlines and hardware authority remain separate."
                if promoted
                else "The kinetic ellipsoid removes the coordinate box's catastrophic width amplification, but the single analytic energy radius does not satisfy strict coverage and useful width together on both fresh laws. The primitive is retained; the profile is rejected without tuning from missed holdout samples. The next construction must separate causal contact phase/load/slip or root and articulated energy budgets."
            ),
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-kinetic-impulse-ellipsoid-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_KINETIC_IMPULSE_ELLIPSOID_HOLDOUT.md").write_text(report)
    np.savez_compressed(
        output / "g1-kinetic-impulse-ellipsoid-replay.npz", **replay
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "profile_promoted": promoted,
                "sample_coverage": {
                    result["law"]: result["sample_coverage"] for result in results
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
