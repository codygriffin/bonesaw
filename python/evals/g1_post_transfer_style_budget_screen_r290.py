#!/usr/bin/env python3
"""Supplemental six-profile screen for the R290 post-transfer Style budget."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import statistics
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html


TIMING_ARRAYS = {"step_ns"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--baseline",
        default="benchmarks/results/floating-g1-r283-polish-budget7-repeat0",
    )
    parser.add_argument("--style1", nargs=5, required=True)
    parser.add_argument("--style2", nargs=5, required=True)
    for budget in (3, 4, 6, 8):
        parser.add_argument(f"--style{budget}", required=True)
    parser.add_argument(
        "--output", default="benchmarks/results/g1-post-transfer-style-budget-screen-r290"
    )
    parser.add_argument(
        "--web-report", default="web/G1_POST_TRANSFER_STYLE_BUDGET_SCREEN_R290.html"
    )
    return parser.parse_args()


def read_metrics(root: pathlib.Path) -> dict[str, Any]:
    return json.loads((root / "floating-walk-metrics.json").read_text())["metrics"]


def semantic_projection(root: pathlib.Path) -> tuple[str, dict[str, bytes]]:
    digest = hashlib.sha256()
    payloads: dict[str, bytes] = {}
    with np.load(root / "floating-walk-raw.npz", allow_pickle=False) as archive:
        for field in sorted(set(archive.files) - TIMING_ARRAYS):
            array = archive[field]
            payload = array.tobytes(order="C")
            payloads[field] = payload
            digest.update(field.encode())
            digest.update(array.dtype.str.encode())
            digest.update(str(array.shape).encode())
            digest.update(payload)
    return digest.hexdigest(), payloads


def first_nonzero(root: pathlib.Path, field: str) -> int | None:
    with np.load(root / "floating-walk-raw.npz", allow_pickle=False) as archive:
        indices = np.flatnonzero(archive[field])
    return int(indices[0]) if indices.size else None


def changed_common_fields(
    baseline: dict[str, bytes], candidate: dict[str, bytes]
) -> list[str]:
    return sorted(
        field
        for field in set(baseline) & set(candidate)
        if baseline[field] != candidate[field]
    )


def summarize_repeats(roots: list[pathlib.Path]) -> dict[str, Any]:
    digests = []
    latency = []
    for root in roots:
        digest, _ = semantic_projection(root)
        digests.append(digest)
        timing = read_metrics(root)["latency_us"]
        latency.append(
            {name: float(timing[name]) for name in ("p50", "p95", "p99", "max")}
        )
    return {
        "repeats": len(roots),
        "all_non_timing_digests_equal": len(set(digests)) == 1,
        "non_timing_sha256": digests[0],
        "latency_us": latency,
        "p99_min": min(row["p99"] for row in latency),
        "p99_mean": statistics.fmean(row["p99"] for row in latency),
        "p99_max": max(row["p99"] for row in latency),
    }


def main() -> None:
    args = parse_args()
    baseline_root = pathlib.Path(args.baseline)
    profile_roots = {
        budget: [pathlib.Path(path) for path in getattr(args, f"style{budget}")]
        if budget in (1, 2)
        else [pathlib.Path(getattr(args, f"style{budget}"))]
        for budget in (1, 2, 3, 4, 6, 8)
    }
    baseline_metrics = read_metrics(baseline_root)
    baseline_digest, baseline_payloads = semantic_projection(baseline_root)
    baseline_root_rms = float(baseline_metrics["root_tracking_rms_m"])
    baseline_release_ticks = [
        int(value) for value in baseline_metrics["release_tail_work"]["ticks"]
    ]

    profiles: dict[str, Any] = {}
    for budget, roots in profile_roots.items():
        first = roots[0]
        metrics = read_metrics(first)
        _, payloads = semantic_projection(first)
        root_rms = float(metrics["root_tracking_rms_m"])
        profiles[str(budget)] = {
            "budget": budget,
            "repeat_summary": summarize_repeats(roots),
            "first_clip_tick": first_nonzero(first, "support_trajectory_tube_clipped"),
            "first_budget_exhaustion_tick": first_nonzero(
                first, "cumulative_low_authority_budget_exhausted_mask"
            ),
            "budget_exhaustion_ticks": int(
                metrics["cumulative_solver_work"][
                    "low_authority_budget_exhausted_ticks"
                ]
            ),
            "changed_common_fields": changed_common_fields(
                baseline_payloads, payloads
            ),
            "root_tracking_rms_m": root_rms,
            "center_of_mass_tracking_rms_m": float(
                metrics["center_of_mass_tracking_rms_m"]
            ),
            "foot_tracking_rms_m": float(metrics["foot_tracking_rms_m"]),
            "root_rms_relative_delta": root_rms / baseline_root_rms - 1.0,
            "release_ticks": [
                int(value) for value in metrics["release_tail_work"]["ticks"]
            ],
            "status_counts": metrics["status_counts"],
            "nominal_prefix": metrics["nominal_prefix"],
        }

    active_one = profiles["1"]
    active_two = profiles["2"]
    inert = [profiles[str(budget)] for budget in (3, 4, 6, 8)]
    checks = {
        "style1_repeats_are_non_timing_exact": active_one["repeat_summary"][
            "all_non_timing_digests_equal"
        ],
        "style2_repeats_are_non_timing_exact": active_two["repeat_summary"][
            "all_non_timing_digests_equal"
        ],
        "budget_cannot_exhaust_on_clipping_tick": all(
            profile["first_budget_exhaustion_tick"] is None
            or profile["first_budget_exhaustion_tick"]
            > profile["first_clip_tick"]
            for profile in profiles.values()
        ),
        "style3_plus_are_inert_on_common_arrays": all(
            not profile["changed_common_fields"] for profile in inert
        ),
        "style3_plus_never_exhaust": all(
            profile["budget_exhaustion_ticks"] == 0 for profile in inert
        ),
        "active_profiles_have_no_hard_failure": all(
            profile["status_counts"]["primal_infeasible"] == 0
            and profile["status_counts"]["failed"] == 0
            for profile in (active_one, active_two)
        ),
        "style1_all_repeats_below_5ms": active_one["repeat_summary"][
            "p99_max"
        ]
        < 5_000.0,
        "style1_preserves_behavior": not active_one["changed_common_fields"],
        "style2_all_repeats_below_5ms": active_two["repeat_summary"][
            "p99_max"
        ]
        < 5_000.0,
        "style2_preserves_behavior": not active_two["changed_common_fields"],
    }
    result = {
        "schema": "bonesaw.g1-post-transfer-style-budget-screen-r290.v1",
        "revision": "g1-post-transfer-style-budget-screen-r290",
        "decision": "RETAIN_MECHANISM_REJECT_PROFILES",
        "execution": {
            "ticks": 2317,
            "timing_cpu": 4,
            "policy_steps": 0,
            "physics_steps": 0,
            "active_profile_repeats": 5,
        },
        "mechanism": {
            "default_off": True,
            "trigger": "first completed support-tube intent-clipping tick",
            "effect": "arm Style pseudoinverse ceiling on the following tick",
            "reset": "restore nominal budgets at every trace/reset boundary",
        },
        "baseline": {
            "non_timing_sha256": baseline_digest,
            "root_tracking_rms_m": baseline_root_rms,
            "center_of_mass_tracking_rms_m": float(
                baseline_metrics["center_of_mass_tracking_rms_m"]
            ),
            "foot_tracking_rms_m": float(baseline_metrics["foot_tracking_rms_m"]),
            "release_ticks": baseline_release_ticks,
            "latency_us": baseline_metrics["latency_us"],
        },
        "profiles": profiles,
        "checks": checks,
        "verdict": {
            "mechanism_retained_default_off": True,
            "finite_profiles_promoted": [],
            "p99_gate_passed_with_preserved_behavior": False,
            "authority_admitted": False,
        },
    }

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    metrics_output = output / "g1-post-transfer-style-budget-screen-r290-metrics.json"
    metrics_output.write_text(json.dumps(result, indent=2) + "\n")

    rows = []
    for budget in (1, 2, 3, 4, 6, 8):
        profile = profiles[str(budget)]
        repeat = profile["repeat_summary"]
        rows.append(
            [
                f"Style-{budget}",
                profile["first_budget_exhaustion_tick"],
                profile["budget_exhaustion_ticks"],
                ", ".join(str(value) for value in profile["release_ticks"]),
                f"{profile['root_tracking_rms_m']:.3f}",
                f"{100.0 * profile['root_rms_relative_delta']:+.2f}%",
                f"{repeat['p99_min']:.1f}–{repeat['p99_max']:.1f}",
                len(profile["changed_common_fields"]),
                "REJECT" if budget in (1, 2) else "INERT",
            ]
        )
    report = [
        "# G1 causal post-transfer Style budget profile screen · R290",
        "",
        "> Mechanism PASS · active finite profiles REJECTED · authority NOT ADMITTED.",
        "",
        "R290 adds a default-off continuous degradation boundary: a support-tube intent clip arms the already-typed Style projected-solve ceiling only after the clipping tick completes. Nominal Preference/Style work remains unbounded before that causal event, and every trace/reset restores the nominal configuration. Rust owns the hot-boundary update; Python owns immutable profile orchestration and reporting.",
        "",
        "## Profile screen",
        "",
    ]
    report += markdown_table(
        [
            "profile",
            "first exhaustion tick",
            "exhausted ticks",
            "release ticks",
            "root RMS m",
            "root Δ",
            "p99 range µs",
            "changed common arrays",
            "decision",
        ],
        rows,
    )
    report += [
        "",
        f"The first support-tube intent clip is tick {active_one['first_clip_tick']}. Style-1 first exhausts at tick {active_one['first_budget_exhaustion_tick']}—never on the clipping tick—and repeats one non-timing digest across all five runs. Four p99 values fall below 5 ms, but one reaches {active_one['repeat_summary']['p99_max']:.1f} µs (mean {active_one['repeat_summary']['p99_mean']:.1f} µs), so the repeatability gate fails. It also moves the first release {baseline_release_ticks[0]}→{active_one['release_ticks'][0]} and worsens root RMS {100.0 * active_one['root_rms_relative_delta']:+.2f}%.",
        "",
        f"Style-2 first exhausts at tick {active_two['first_budget_exhaustion_tick']} and is likewise bitwise repeatable. It still misses 5 ms ({active_two['repeat_summary']['p99_min']:.1f}–{active_two['repeat_summary']['p99_max']:.1f} µs), moves first release {baseline_release_ticks[0]}→{active_two['release_ticks'][0]}, and worsens root RMS {100.0 * active_two['root_rms_relative_delta']:+.2f}%. Style-3/4/6/8 never exhaust and preserve every common non-timing array, so they provide no work reduction.",
        "",
        "## Boundary and decision",
        "",
        "The mechanism is retained because it supplies the requested event-driven, reset-safe degradation surface without capping higher authority or changing dormant behavior. No measured finite profile is promoted: Style-1 misses the repeatable timing gate and destroys transfer, Style-2 changes behavior while remaining above 5 ms, and higher ceilings are inert. There are zero typed primal-infeasible/failed ticks in the active profiles, but hard feasibility alone is not tracking or command authority.",
        "",
        "The corpus executes no policy and no physics. Contact, actuator, thermal, plant, authenticated transport, browser frame-time, and hardware realization remain separate open gates.",
    ]
    report_text = "\n".join(report) + "\n"
    (output / "G1_POST_TRANSFER_STYLE_BUDGET_SCREEN_R290.md").write_text(report_text)
    web_output = pathlib.Path(args.web_report)
    web_output.parent.mkdir(parents=True, exist_ok=True)
    web_output.write_text(render_report_html(report_text))
    print(json.dumps({"decision": result["decision"], "checks": checks}, indent=2))


if __name__ == "__main__":
    main()
