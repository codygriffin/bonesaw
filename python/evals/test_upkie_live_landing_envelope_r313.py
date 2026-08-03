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
    import mujoco  # noqa: F401

    HAS_RUNTIME = True
except ImportError:
    HAS_RUNTIME = False

import upkie_live_landing_envelope_r313 as r313  # noqa: E402


@unittest.skipUnless(HAS_RUNTIME, "the live MuJoCo/PyO3 runtime is not installed")
class UpkieLiveLandingEnvelopeR313Tests(unittest.TestCase):
    def test_envelope_is_causal_allocation_free_and_not_promoted(self) -> None:
        metrics, _, _ = r313.run(
            ROOT / "models/upkie/upkie.urdf",
            ticks=140,
        )
        self.assertTrue(metrics["harness_valid"], metrics)
        self.assertFalse(metrics["candidate_default_enabled"])
        self.assertTrue(metrics["harness_gates"]["causal_measured_window"], metrics)
        self.assertTrue(metrics["harness_gates"]["mode_firewall"], metrics)
        self.assertTrue(metrics["harness_gates"]["finite_outputs"], metrics)
        self.assertTrue(metrics["harness_gates"]["zero_landing_allocations"], metrics)
        self.assertTrue(metrics["harness_gates"]["landing_deadline"], metrics)
        self.assertTrue(
            metrics["harness_gates"]["candidate_replay_exact"], metrics
        )
        # R313 is intentionally a negative physical result on the measured
        # upper-base holdout.  This assertion prevents a later refactor from
        # silently turning a mechanism-only experiment into public authority.
        self.assertFalse(metrics["qualified_for_public_default"], metrics)
        self.assertFalse(
            metrics["promotion_gates"]["candidate_completes_every_case"], metrics
        )


if __name__ == "__main__":
    unittest.main()
