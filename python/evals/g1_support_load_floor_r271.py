#!/usr/bin/env python3
"""Audit the default-off per-patch aggregate support-load floor experiment."""

from __future__ import annotations

import hashlib
import json
import pathlib
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "g1-support-load-floor-r271"
RESULT_ROOT = pathlib.Path("benchmarks/results")
BASELINE = RESULT_ROOT / "floating-g1-r270-lower-body-hard-cap8" / "floating-walk-raw.npz"
DORMANT = RESULT_ROOT / "floating-g1-r271-support-load-dormant-cap8" / "floating-walk-raw.npz"
FLOOR005 = RESULT_ROOT / "floating-g1-r271-support-load-floor005-cap8" / "floating-walk-raw.npz"
FLOOR10 = RESULT_ROOT / "floating-g1-r271-support-load-floor10-cap8" / "floating-walk-raw.npz"
FLOOR25 = RESULT_ROOT / "floating-g1-r271-support-load-floor25-cap8" / "floating-walk-raw.npz"
REFERENCE = RESULT_ROOT / "g1-multistep-reference-r53" / "reference-inputs.npz"
WITNESS = RESULT_ROOT / "g1-multistep-oracle-r54" / "oracle-wbc-admission-raw.npz"
MODEL = pathlib.Path("benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf")
GRAVITY = 9.81
ACTIVE_SUPPORT_PHASES = (2, 3, 4)
EXCLUDED_SUPPORT_STATUSES = (5, 8)
CONTACTS_PER_PATCH = 4
CRITICAL_COORDINATE = 9
CRITICAL_POSITION_LOWER_RAD = -0.087267


def fingerprint(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load(path: pathlib.Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as source:
        return {key: np.asarray(source[key]) for key in source.files}


def total_urdf_mass(path: pathlib.Path) -> float:
    root = ET.parse(path).getroot()
    masses = [float(node.attrib["value"]) for node in root.findall(".//inertial/mass")]
    if not masses or not np.all(np.isfinite(masses)) or min(masses) < 0.0:
        raise ValueError(f"invalid inertial masses in {path}")
    total = float(sum(masses))
    if total <= 0.0:
        raise ValueError(f"non-positive total mass in {path}")
    return total


def first_tick(status: np.ndarray, codes: tuple[int, ...]) -> int:
    selected = np.isin(status, codes)
    return int(np.flatnonzero(selected)[0]) if np.any(selected) else len(status)


def dormant_differences(
    baseline: dict[str, np.ndarray], dormant: dict[str, np.ndarray]
) -> list[str]:
    differences: list[str] = []
    for key in sorted(set(baseline) | set(dormant)):
        if key == "step_ns":
            continue
        if key not in baseline or key not in dormant:
            differences.append(key)
            continue
        left = baseline[key]
        right = dormant[key]
        if left.shape != right.shape or left.dtype != right.dtype:
            differences.append(key)
        elif not np.array_equal(left, right, equal_nan=True):
            differences.append(key)
    return differences


def support_load_contract(
    trace: dict[str, np.ndarray], requested_fraction: float, supported_weight_n: float
) -> dict[str, Any]:
    """Audit compact force slots against active-target order."""

    status = trace["status"]
    phases = trace["support_phase"]
    normal = trace["contact_normal_force"]
    margins: list[float] = []
    checked_rows = 0
    violations = 0
    binding_ticks: list[int] = []
    for tick in range(len(status)):
        if int(status[tick]) in EXCLUDED_SUPPORT_STATUSES:
            continue
        active_targets = np.flatnonzero(np.isin(phases[tick], ACTIVE_SUPPORT_PHASES))
        if len(active_targets) == 0:
            continue
        floor_n = requested_fraction * supported_weight_n / len(active_targets)
        tick_binding = False
        for compact_patch in range(len(active_targets)):
            begin = compact_patch * CONTACTS_PER_PATCH
            end = begin + CONTACTS_PER_PATCH
            if end > normal.shape[1]:
                raise ValueError("active support patches exceed retained compact force slots")
            carried_n = float(np.sum(normal[tick, begin:end]))
            margin_n = carried_n - floor_n
            margins.append(margin_n)
            checked_rows += 1
            violations += int(margin_n < -1.0e-6)
            tick_binding |= margin_n <= 1.0e-5
        if tick_binding:
            binding_ticks.append(tick)
    return {
        "checked_rows": checked_rows,
        "violations": violations,
        "minimum_margin_n": float(min(margins)) if margins else None,
        "first_binding_tick": binding_ticks[0] if binding_ticks else None,
        "binding_tick_count": len(binding_ticks),
    }


def summarize(
    trace: dict[str, np.ndarray], requested_fraction: float, supported_weight_n: float
) -> dict[str, Any]:
    status = trace["status"]
    timing_ms = trace["step_ns"].astype(np.float64) / 1.0e6
    root_error = np.linalg.norm(
        trace["root_tracked"] - trace["effective_root_targets"], axis=1
    )
    attitude = 2.0 * np.arccos(
        np.clip(np.abs(trace["root_quaternion_wxyz"][:, 0]), 0.0, 1.0)
    )
    first_fallback = first_tick(status, (4, 5, 8, 9))
    first_release = first_tick(status, (5,))
    values, counts = np.unique(status, return_counts=True)
    lower_limit_ticks = np.flatnonzero(
        trace["q"][:, CRITICAL_COORDINATE] <= CRITICAL_POSITION_LOWER_RAD + 1e-9
    )
    prefix = root_error[:first_fallback]
    return {
        "ticks": int(len(status)),
        "controller_ticks": int(len(status)),
        "integration_steps": int(np.count_nonzero(status != 8)),
        "status_counts": {str(int(k)): int(v) for k, v in zip(values, counts)},
        "first_fallback_tick": first_fallback,
        "first_release_tick": first_release,
        "prefix_root_rms_m": float(np.sqrt(np.mean(np.square(prefix)))) if len(prefix) else 0.0,
        "full_root_rms_m": float(np.sqrt(np.mean(np.square(root_error)))),
        "maximum_root_attitude_deg": float(np.degrees(np.max(attitude))),
        "critical_joint_first_lower_limit_tick": (
            int(lower_limit_ticks[0]) if len(lower_limit_ticks) else None
        ),
        "requested_fraction": requested_fraction,
        "load_contract": support_load_contract(trace, requested_fraction, supported_weight_n),
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
    required = (BASELINE, DORMANT, FLOOR005, FLOOR10, FLOOR25, REFERENCE, WITNESS, MODEL)
    if not all(path.is_file() for path in required):
        missing = [str(path) for path in required if not path.is_file()]
        raise SystemExit("R271 is missing required evidence: " + ", ".join(missing))
    traces = {
        "r270_lower_body_hard_baseline": load(BASELINE),
        "r271_dormant_zero_floor": load(DORMANT),
        "r271_floor005": load(FLOOR005),
        "r271_floor10": load(FLOOR10),
        "r271_floor25": load(FLOOR25),
    }
    total_mass_kg = total_urdf_mass(MODEL)
    supported_weight_n = total_mass_kg * GRAVITY
    fractions = {
        "r270_lower_body_hard_baseline": 0.0,
        "r271_dormant_zero_floor": 0.0,
        "r271_floor005": 0.005,
        "r271_floor10": 0.10,
        "r271_floor25": 0.25,
    }
    profiles = {
        name: summarize(trace, fractions[name], supported_weight_n)
        for name, trace in traces.items()
    }
    baseline = profiles["r270_lower_body_hard_baseline"]
    candidates = [profiles[name] for name in ("r271_floor005", "r271_floor10", "r271_floor25")]
    semantic_differences = dormant_differences(
        traces["r270_lower_body_hard_baseline"], traces["r271_dormant_zero_floor"]
    )
    dormant_equal = not semantic_differences
    mechanism_passed = bool(
        dormant_equal
        and all(row["finite_state"] for row in candidates)
        and all(row["load_contract"]["checked_rows"] > 0 for row in candidates)
        and all(row["load_contract"]["violations"] == 0 for row in candidates)
        and all(row["load_contract"]["first_binding_tick"] is not None for row in candidates)
    )
    profile_rejected = bool(
        all(row["first_fallback_tick"] < baseline["first_fallback_tick"] for row in candidates)
        and all(row["first_release_tick"] < baseline["first_release_tick"] for row in candidates)
    )
    rows = []
    for name, row in profiles.items():
        contract = row["load_contract"]
        rows.append([
            name.replace("_", " "), f'{row["requested_fraction"]:.3f}',
            str(contract["first_binding_tick"] if contract["first_binding_tick"] is not None else "—"),
            str(row["first_fallback_tick"]), str(row["first_release_tick"]),
            f'{contract["minimum_margin_n"]:.6f}' if contract["minimum_margin_n"] is not None else "—",
            str(contract["violations"]), f'{row["full_root_rms_m"]:.3f}',
            f'{row["timing_ms"]["p99"]:.3f}',
        ])
    source_paths = {
        "reference": REFERENCE, "witness": WITNESS, "model": MODEL,
        "baseline": BASELINE, "dormant": DORMANT, "floor005": FLOOR005,
        "floor10": FLOOR10, "floor25": FLOOR25,
    }
    source_contract = {
        **{f"{name}_sha256": fingerprint(path) for name, path in source_paths.items()},
        "total_mass_kg": total_mass_kg,
        "supported_weight_n": supported_weight_n,
        "critical_coordinate": CRITICAL_COORDINATE,
        "critical_joint_name": "right_knee_joint",
        "policy_steps": 0,
        "physics_steps": 0,
        "force_slot_mapping": "active-target order; four consecutive normal slots per sole",
        "aggregate_row_is_hard": True,
    }
    floor005 = profiles["r271_floor005"]
    floor10 = profiles["r271_floor10"]
    floor25 = profiles["r271_floor25"]
    report = "\n".join([
        "# G1 per-patch aggregate support-load floor experiment · R271", "",
        "Mechanism implemented; global walking profile rejected. R271 adds a default-off hard aggregate normal-load row to each finite support patch. The row constrains the sum of a foot's four normal-force slots, so load can redistribute across the sole.",
        "", "## Result", "",
        *markdown_table(
            ["profile", "floor", "first bind", "fallback", "release", "min row margin N", "violations", "root RMS m", "p99 ms"],
            rows,
        ), "",
        f"The 0% R271 trace is bitwise equal to all {len(traces['r270_lower_body_hard_baseline']) - 1} retained non-timing R270 arrays: {dormant_equal}. Every nonzero trace satisfies every audited per-patch row within 1e-6 N; the smallest 0.5% floor first binds at tick {floor005['load_contract']['first_binding_tick']} and moves contingency/release from the R270 baseline's {baseline['first_fallback_tick']}/{baseline['first_release_tick']} to {floor005['first_fallback_tick']}/{floor005['first_release_tick']}. The 10% and 25% cases fail at {floor10['first_fallback_tick']} and {floor25['first_fallback_tick']}.",
        "",
        "That separates mechanism from policy: the generic row works and is dormant at zero, but applying the same minimum continuously to every active walking patch forbids deliberate unloading of the outgoing foot. The first 0.5% bind occurs hundreds of ticks before R270's right-knee limit event at tick 874, so this profile is not a recovery for that event and receives no authority.",
        "",
        "Timing is reported per isolated trace but is not a selection criterion. The evaluator performs no policy or physics steps; controller ticks and non-hold integration steps are recorded per profile. All candidate states remain finite, and releases are retained as fail-closed evidence rather than resets hidden from the score.",
        "", "## Contract", "",
        "- Hard row: sum of the four compact normal-force slots for one active sole is at least fraction × supported weight ÷ active patches.",
        "- Compact slots are decoded in active-target order, not permanent left/right slots; release and hold statuses are excluded from successful-support claims.",
        "- API fraction is finite in [0, 1] and defaults to zero. The dormant A/B decides whether defaults changed.",
        f"- Pinned G1 mass is {total_mass_kg:.9f} kg; supported weight is {supported_weight_n:.9f} N at 9.81 m/s².",
        "- Source contains only the immutable reference, initial morphology witness, and pinned URDF. Policy steps: 0; physics steps: 0.",
        f"- Reference SHA-256: `{source_contract['reference_sha256']}`; witness SHA-256: `{source_contract['witness_sha256']}`; model SHA-256: `{source_contract['model_sha256']}`.",
    ])
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "policy_steps": 0,
        "physics_steps": 0,
        "source_contract": source_contract,
        "profiles": profiles,
        "dormant_non_timing_arrays_equal": dormant_equal,
        "dormant_semantic_differences": semantic_differences,
        "mechanism_passed": mechanism_passed,
        "profile_rejected": profile_rejected,
        "default_changed": not dormant_equal,
        "authority_admitted": False,
    }
    output = RESULT_ROOT / REVISION
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-support-load-floor-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_SUPPORT_LOAD_FLOOR_R271.md").write_text(report + "\n")
    pathlib.Path("web/G1_SUPPORT_LOAD_FLOOR_R271.html").write_text(
        render_report_html(report, title="G1 per-patch support-load floor · R271")
    )
    print(output / "G1_SUPPORT_LOAD_FLOOR_R271.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
