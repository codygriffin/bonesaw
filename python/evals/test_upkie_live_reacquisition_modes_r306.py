from __future__ import annotations

import pathlib
import sys
import unittest

EVALS = pathlib.Path(__file__).resolve().parent
ROOT = EVALS.parents[1]
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

try:
    import bonesaw  # noqa: F401

    HAS_RUNTIME = True
except ImportError:
    HAS_RUNTIME = False

from upkie_live_reacquisition_modes_r306 import (  # noqa: E402
    MAX_TICKS,
    evaluate,
    run_modes,
)


@unittest.skipUnless(HAS_RUNTIME, "the Rust extension is not installed")
class UpkieLiveReacquisitionModesR306Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.metrics = run_modes(ROOT / "models/upkie/upkie.urdf", MAX_TICKS)
        cls.gates = evaluate(cls.metrics)

    def test_causal_and_safety_invariants_pass(self) -> None:
        for name in (
            "all_modes_use_250hz_physics_50hz_wbc",
            "all_modes_preserve_causal_prior_window",
            "all_outputs_finite",
            "hard_rows_subset_measured_contact",
            "max_iterations_never_admitted",
            "zero_timed_rust_allocations",
            "worker_p99_under_20ms",
            "public_capture_profile_is_unchanged",
        ):
            self.assertTrue(self.gates[name], name)

    def test_existing_modes_do_not_claim_reacquisition(self) -> None:
        self.assertTrue(self.gates["no_mode_reacquires_wheels_for_ten_ticks"])
        for summary in self.metrics["summaries"].values():
            self.assertFalse(summary["strict_recovery"])


if __name__ == "__main__":
    unittest.main()
