#!/usr/bin/env python3
"""R305 Rust-only measured-load contact reacquisition witness.

The sequence is synthetic by design: it exercises the Rust authority boundary
without changing the live worker's public 250/50 Hz defaults or emitting an
actuator command.  A future physical recovery candidate can consume the
qualified witness only after its own WBC admission and plant gates pass.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from typing import Any

import numpy as np


REVISION = "contact-reacquisition-observer-r305"


def run_sequence(model_path: pathlib.Path) -> dict[str, Any]:
    import bonesaw

    session = bonesaw.UpkieBalanceSession(str(model_path))
    session.configure_contact_reacquisition(3, 1.0, 0.5, 0.05)
    names = list(session.contact_reacquisition_diagnostic_names)
    rows: list[dict[str, Any]] = []
    sequence = (
        # Exact startup support is a baseline, never reacquisition authority.
        ([1, 1], [25.0, 25.0]),
        # A valid observed loss arms the future re-entry witness.
        ([0, 0], [0.0, 0.0]),
        # Three consecutive bilateral, force-backed samples qualify.
        ([1, 1], [20.0, 20.0]),
        ([1, 1], [20.0, 20.0]),
        ([1, 1], [20.0, 20.0]),
    )
    target = np.asarray([1, 1], np.uint8)
    for tick, (mask, loads) in enumerate(sequence, 1):
        diagnostics = np.empty(13, np.float64)
        mask_array = np.asarray(mask, np.uint8)
        session.step_contact_reacquisition_from_loads(
            tick,
            True,
            target,
            mask_array,
            mask_array,
            mask_array,
            np.asarray(loads, np.float64),
            diagnostics,
        )
        rows.append(dict(zip(names, diagnostics.tolist(), strict=True)))

    # A rejected sample must not publish the previous qualification bit.
    rejected = np.empty(13, np.float64)
    session.step_contact_reacquisition_from_loads(
        6,
        False,
        target,
        target,
        target,
        target,
        np.asarray([20.0, 20.0], np.float64),
        rejected,
    )
    rows.append(dict(zip(names, rejected.tolist(), strict=True)))
    return {
        "revision": REVISION,
        "diagnostic_names": names,
        "rows": rows,
        "baseline_status": int(rows[0]["status"]),
        "armed_status": int(rows[1]["status"]),
        "qualified_status": int(rows[4]["status"]),
        "qualified": bool(rows[4]["qualified"]),
        "rejected_status": int(rows[5]["status"]),
        "rejected_qualified": bool(rows[5]["qualified"]),
        "actuator_authority_emitted": False,
        "public_worker_defaults_changed": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    args = parser.parse_args()
    result = run_sequence(pathlib.Path(args.model))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
