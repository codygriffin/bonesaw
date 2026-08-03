#!/usr/bin/env python3
"""Qualify the caller-owned historical frame-query batch surface (R286)."""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
from typing import Any


ROOT = pathlib.Path(__file__).resolve().parents[2]
REVISION = "frame-query-batch-r286"
RESULT_DIR = ROOT / "benchmarks/results" / REVISION
RESULT = RESULT_DIR / f"{REVISION}-metrics.json"
REPORT = RESULT_DIR / "FRAME_QUERY_BATCH_R286.md"


def run_audit(binary: pathlib.Path, model: pathlib.Path) -> dict[str, Any]:
    completed = subprocess.run(
        [str(binary), str(model)],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise ValueError("frame-query audit produced no JSON result")
    return json.loads(lines[-1])


def qualify(audit: dict[str, Any]) -> dict[str, Any]:
    if audit.get("schema") != "bonesaw.frame-query-batch-r286.v1":
        raise ValueError(f"unexpected audit schema: {audit.get('schema')}")
    if audit["query_count"] < 32 or audit["measured_batches"] < 8:
        raise ValueError("R286 requires a bounded, nontrivial batch replay")
    if not audit["repeated_results_bitwise_equal"]:
        raise ValueError("repeated frame-query results are not bitwise stable")
    if audit["output_external_capacity_before"] != audit[
        "output_external_capacity_after"
    ]:
        raise ValueError("caller-owned external provenance capacity grew in the hot loop")
    return {
        "schema": "bonesaw.frame-query-batch-r286.v1",
        "revision": REVISION,
        "execution": {
            "model": audit["model"],
            "query_count": audit["query_count"],
            "warmup_batches": audit["warmup_batches"],
            "measured_batches": audit["measured_batches"],
        },
        "performance": {
            "elapsed_ns": audit["elapsed_ns"],
            "nanoseconds_per_query": audit["nanoseconds_per_query"],
        },
        "semantic_contract": {
            "repeated_results_bitwise_equal": audit[
                "repeated_results_bitwise_equal"
            ],
            "output_external_capacity_before": audit[
                "output_external_capacity_before"
            ],
            "output_external_capacity_after": audit[
                "output_external_capacity_after"
            ],
            "allocation_claim": audit["allocation_claim"],
        },
        "verdict": {
            "batch_surface_qualified": True,
            "wbc_authority_admitted": False,
            "physics_authority_admitted": False,
        },
    }


def build_report(result: dict[str, Any]) -> str:
    execution = result["execution"]
    performance = result["performance"]
    contract = result["semantic_contract"]
    return f"""# Historical frame-query batch surface · R286

> Batch semantics **PASS** · caller-owned output capacity **PASS** · authority **UNCHANGED**.

R286 adds `CompiledFrameAtlas::query_history_into` and
`query_history_batch`. They preserve the scalar historical-query semantics,
reuse the external-provenance output storage supplied by the caller, and
return an indexed error when one query in a batch fails. This is a query/data
flow improvement only; it does not change WBC priorities, contact policy,
physics, or command authority.

## Qualification

| field | value |
|---|---:|
| model | `{execution['model']}` |
| queries per batch | {execution['query_count']} |
| warmup batches | {execution['warmup_batches']} |
| measured batches | {execution['measured_batches']} |
| total elapsed | {performance['elapsed_ns'] / 1e6:.3f} ms |
| mean query time | {performance['nanoseconds_per_query'] / 1e3:.3f} µs |
| repeated results bitwise equal | `{contract['repeated_results_bitwise_equal']}` |
| external provenance capacity | {contract['output_external_capacity_before']} → {contract['output_external_capacity_after']} |

The benchmark intentionally does **not** claim that the legacy `RobotHistory`
reconstruction path is allocation-free; that is a separate follow-up slice.
The claim here is limited to the new caller-owned batch output surface and its
semantic repeatability.

No policy step, physics step, actuator command, plant observation, or authority
admission is part of this result.
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--binary",
        type=pathlib.Path,
        default=ROOT / "target/release/frame_query_batch_audit",
    )
    parser.add_argument(
        "--model", type=pathlib.Path, default=ROOT / "models/upkie/upkie.urdf"
    )
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    result = qualify(run_audit(args.binary, args.model))
    if not args.check_only:
        RESULT_DIR.mkdir(parents=True, exist_ok=True)
        RESULT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        REPORT.write_text(build_report(result))
    print(f"validated {RESULT}")


if __name__ == "__main__":
    main()
