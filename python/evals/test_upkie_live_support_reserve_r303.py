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

import upkie_live_dynamic_contact_transition_r300 as r300  # noqa: E402
import upkie_live_support_reserve_r303 as r303  # noqa: E402


@unittest.skipUnless(HAS_RUNTIME, "the live MuJoCo/PyO3 runtime is not installed")
class UpkieLiveSupportReserveR303Tests(unittest.TestCase):
    def test_measured_reserve_crossing_precedes_contact_loss(self) -> None:
        model = ROOT / "models/upkie/upkie.urdf"
        disturbed = r300.run_case(model, disturbed=True, maximum_ticks=80)
        control = r300.run_case(model, disturbed=False, maximum_ticks=80)
        metrics = r303.evaluate(control, disturbed)
        self.assertTrue(all(metrics["gates"].values()), metrics)
        # Keep this test causal rather than freezing one controller tuning's
        # exact 50 Hz crossing tick.  Nominal posture/gain changes can move
        # the crossing while the safety contract remains the same.
        self.assertIsNotNone(metrics["disturbed_trigger_tick"])
        self.assertIsNotNone(metrics["disturbed_physics_loss_tick"])
        self.assertIsNotNone(metrics["disturbed_observed_loss_tick"])
        self.assertGreaterEqual(metrics["pre_loss_lead_ticks"], 1)
        self.assertLess(
            metrics["disturbed_trigger_tick"],
            metrics["disturbed_physics_loss_tick"],
        )
        self.assertLess(
            metrics["disturbed_trigger_tick"],
            metrics["disturbed_observed_loss_tick"],
        )
        self.assertFalse(metrics["actuator_authority_emitted"])

    def test_gate_is_fail_closed_for_invalid_or_nonbilateral_evidence(self) -> None:
        state = {
            "observed": [1, 1],
            "observed_wheel_normal_force_n": [float("nan"), 1.0],
        }
        previous = {
            "observed": [1, 1],
            "observed_wheel_normal_force_n": [2.0, 2.0],
        }
        self.assertFalse(r303.reserve_gate(state, previous))
        state["observed"] = [1, 0]
        state["observed_wheel_normal_force_n"] = [0.1, 0.1]
        self.assertFalse(r303.reserve_gate(state, previous))

    def test_default_off_load_guard_stays_an_unpromoted_override(self) -> None:
        model = ROOT / "models/upkie/upkie.urdf"
        candidate = r300.run_case(
            model,
            disturbed=True,
            maximum_ticks=80,
            controller_options={"support_load_guard_enabled": True},
        )
        summary = r303.guard_probe_summary(candidate)
        self.assertTrue(summary["evaluation_override"])
        self.assertFalse(summary["promoted"])
        self.assertEqual(summary["allocation_calls"], 0)
        self.assertEqual(summary["allocated_bytes"], 0)
        self.assertIsNone(summary["strict_recovery_tick"])
        self.assertIsNotNone(summary["terminal_tick"])


if __name__ == "__main__":
    unittest.main()
