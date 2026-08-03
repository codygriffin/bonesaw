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

import upkie_live_measured_landing_r312 as r312  # noqa: E402


@unittest.skipUnless(HAS_RUNTIME, "the live MuJoCo/PyO3 runtime is not installed")
class UpkieLiveMeasuredLandingR312Tests(unittest.TestCase):
    def test_phase_boundary_is_default_off_causal_and_allocation_free(self) -> None:
        metrics, _, _ = r312.run(
            ROOT / "models/upkie/upkie.urdf",
            nominal_ticks=120,
            disturbed_ticks=250,
            composition_forces=(-8.0, 8.0),
        )
        self.assertTrue(
            metrics["qualified_as_bounded_default_off_experiment"], metrics
        )
        self.assertFalse(metrics["candidate_default_enabled"])
        self.assertTrue(
            metrics["qualification_gates"]["nominal_measured_landing_remains_dormant"],
            metrics,
        )
        self.assertTrue(
            metrics["qualification_gates"]["measured_phase_activates_after_physics_loss"],
            metrics,
        )
        self.assertTrue(
            metrics["qualification_gates"]["mode_firewall_never_promotes_unobserved_leg"],
            metrics,
        )
        self.assertTrue(
            metrics["qualification_gates"]["causal_physics_window_and_exact_snapshot"],
            metrics,
        )
        self.assertTrue(
            metrics["qualification_gates"][
                "rust_measured_landing_hot_path_zero_allocations"
            ],
            metrics,
        )
        self.assertTrue(
            metrics["qualification_gates"]["finite_measured_landing_outputs"], metrics
        )
        self.assertTrue(
            metrics["qualification_gates"]["mechanism_replay_exact"], metrics
        )
        self.assertTrue(
            metrics["r311_public_composition"]["harness_valid"], metrics
        )
        self.assertFalse(
            metrics["r311_public_composition"]["recovery_promoted"], metrics
        )
        # The current 8 N trace is intentionally a negative recovery witness;
        # this assertion prevents a reset or a missing telemetry field from
        # silently turning it into a promotion.
        self.assertTrue(
            metrics["negative_evidence"][
                "candidate_disturbed_negative_recovery_is_explicit"
            ],
            metrics,
        )
        self.assertFalse(metrics["recovery_promoted"])


if __name__ == "__main__":
    unittest.main()
