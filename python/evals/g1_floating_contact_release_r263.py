#!/usr/bin/env python3
"""Measure the floating WBC contact-release recovery slice.

This is an artifact-only regression.  It compares the r262 capped trace,
which could leave the integrated state unchanged after the first unsolved
contact, with the r263 trace.  Rust owns the retry/latch/fallback behavior;
Python only loads retained arrays and reports timing and state witnesses.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from typing import Any

import numpy as np


REVISION = "g1-floating-contact-release-r263"
BASELINE = pathlib.Path(
    "benchmarks/results/floating-g1-transfer-r262-cap8/floating-walk-raw.npz"
)
CANDIDATE = pathlib.Path(
    "benchmarks/results/floating-g1-transfer-r263-release-cap8/floating-walk-raw.npz"
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", default=str(BASELINE))
    parser.add_argument("--candidate", default=str(CANDIDATE))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    return parser.parse_args(argv)


def longest_true_run(mask: np.ndarray) -> int:
    longest = current = 0
    for value in np.asarray(mask, dtype=bool):
        if value:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest


def load_raw(path: pathlib.Path) -> dict[str, np.ndarray]:
    if not path.is_file():
        raise FileNotFoundError(path)
    with np.load(path, allow_pickle=False) as raw:
        required = {"status", "step_ns", "root_tracked", "q"}
        missing = sorted(required - set(raw.files))
        if missing:
            raise ValueError(f"{path}: missing fields {missing}")
        return {name: raw[name].copy() for name in raw.files}


def summarize(path: pathlib.Path, label: str) -> dict[str, Any]:
    raw = load_raw(path)
    status = raw["status"].astype(np.uint8, copy=False)
    step_ns = raw["step_ns"].astype(np.uint64, copy=False)
    root = raw["root_tracked"]
    q = raw["q"]
    if not all(np.isfinite(raw[name]).all() for name in ("root_tracked", "q")):
        raise ValueError(f"{path}: state arrays contain non-finite values")
    state_stalled = np.all(np.diff(root, axis=0) == 0.0, axis=1) & np.all(
        np.diff(q, axis=0) == 0.0, axis=1
    )
    status_changes = np.flatnonzero(np.r_[True, status[1:] != status[:-1]])
    status_counts = {
        str(code): int(np.count_nonzero(status == code))
        for code in sorted(int(code) for code in np.unique(status))
    }
    first_contingency = np.flatnonzero(~np.isin(status, (0, 1, 6, 7)))
    first = int(first_contingency[0]) if len(first_contingency) else len(status)
    post_contingency_motion = bool(
        np.any(np.linalg.norm(np.diff(root[first:], axis=0), axis=1) > 0.0)
        if first < len(root) - 1
        else False
    )
    return {
        "label": label,
        "artifact": str(path),
        "ticks": int(len(status)),
        "status_counts": status_counts,
        "status_change_ticks": [int(value) for value in status_changes],
        "first_contingency_tick": first,
        "longest_exact_state_stall_ticks": int(longest_true_run(state_stalled)),
        "post_contingency_root_motion": post_contingency_motion,
        "latency_us": {
            "p50": float(np.percentile(step_ns, 50) / 1_000.0),
            "p99": float(np.percentile(step_ns, 99) / 1_000.0),
            "max": float(np.max(step_ns) / 1_000.0),
        },
        "deadline_misses": {
            "5ms": int(np.count_nonzero(step_ns > 5_000_000)),
            "20ms": int(np.count_nonzero(step_ns > 20_000_000)),
        },
        "finite_state": bool(
            np.isfinite(root).all() and np.isfinite(q).all()
        ),
    }


def build_result(args: argparse.Namespace) -> tuple[dict[str, Any], str]:
    baseline = summarize(pathlib.Path(args.baseline), "r262-cap8")
    candidate = summarize(pathlib.Path(args.candidate), "r263-release-cap8")
    candidate_status = candidate["status_counts"]
    gates = {
        "baseline_exhibits_state_stall": baseline[
            "longest_exact_state_stall_ticks"
        ]
        > 0,
        "candidate_has_no_state_stall": candidate[
            "longest_exact_state_stall_ticks"
        ]
        == 0,
        "candidate_moves_after_contingency": candidate[
            "post_contingency_root_motion"
        ],
        "candidate_no_failed_or_infeasible": not any(
            int(candidate_status.get(str(code), 0)) > 0 for code in (2, 3)
        ),
        "candidate_zero_20ms_misses": candidate["deadline_misses"]["20ms"] == 0,
        "candidate_p99_under_5ms": candidate["latency_us"]["p99"] < 5_000.0,
        "candidate_release_is_bounded": candidate_status.get("5", 0) <= 2,
    }
    result = {
        "revision": REVISION,
        "baseline": baseline,
        "candidate": candidate,
        "gates": gates,
        "functional_admission": False,
        "note": (
            "The release latch and free-body fallback close the freeze/reset "
            "failure mode. They do not admit the floating walking transfer: "
            "contact/tracking/residual behavior remains a separate open gate."
        ),
    }
    report = render_report(result)
    return result, report


def render_report(result: dict[str, Any]) -> str:
    baseline = result["baseline"]
    candidate = result["candidate"]
    gates = result["gates"]
    rows = "\n".join(
        f"| `{name}` | {'PASS' if value else 'FAIL'} |"
        for name, value in gates.items()
    )
    return f"""# G1 floating contact-release recovery r263

This report compares the fixed-cap8 r262 trace with the current r263 Rust
session. It is a CPU, policy-free, physics-free artifact comparison: Python
loads retained Rust outputs and does not reconstruct the controller.

## Result

The r263 latch prevents a failed contact from being rebuilt every tick, and
the bounded free-body fallback keeps the state advancing after an unsolved
contact. Functional walking admission remains **closed**.

| Gate | Result |
|---|---:|
{rows}

## Runtime/state comparison

| profile | first contingency | longest exact state stall | post-contingency motion | p50 µs | p99 µs | max µs | >5 ms | >20 ms | release ticks |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| r262 cap8 | {baseline['first_contingency_tick']} | {baseline['longest_exact_state_stall_ticks']} | {baseline['post_contingency_root_motion']} | {baseline['latency_us']['p50']:.1f} | {baseline['latency_us']['p99']:.1f} | {baseline['latency_us']['max']:.1f} | {baseline['deadline_misses']['5ms']} | {baseline['deadline_misses']['20ms']} | {baseline['status_counts'].get('5', 0)} |
| r263 release cap8 | {candidate['first_contingency_tick']} | {candidate['longest_exact_state_stall_ticks']} | {candidate['post_contingency_root_motion']} | {candidate['latency_us']['p50']:.1f} | {candidate['latency_us']['p99']:.1f} | {candidate['latency_us']['max']:.1f} | {candidate['deadline_misses']['5ms']} | {candidate['deadline_misses']['20ms']} | {candidate['status_counts'].get('5', 0)} |

Candidate status counts: `{json.dumps(candidate['status_counts'], sort_keys=True)}`.

The fallback is intentionally fail-closed with respect to contact authority:
once a contact solve cannot be admitted, hard contact rows are suppressed
until reset. The state may continue under bounded free-body damping/gravity,
but this is not a claim that the commanded walk is physically realized.
"""


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result, report = build_result(args)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-floating-contact-release-r263-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_FLOATING_CONTACT_RELEASE_R263.md").write_text(report)
    print(output / "G1_FLOATING_CONTACT_RELEASE_R263.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
