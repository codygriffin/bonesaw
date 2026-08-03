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
import upkie_live_support_recovery_r302 as r302  # noqa: E402


@unittest.skipUnless(HAS_RUNTIME, "the live MuJoCo/PyO3 runtime is not installed")
class UpkieLiveSupportRecoveryR302Tests(unittest.TestCase):
    def test_strict_recovery_rejects_a_long_body_ground_stall(self) -> None:
        model = ROOT / "models/upkie/upkie.urdf"
        baseline = r300.run_case(model, disturbed=True, maximum_ticks=700)
        default = r300.run_case(
            model,
            disturbed=True,
            maximum_ticks=700,
            controller_options=r302.controller_options(None),
        )
        candidate = r300.run_case(
            model,
            disturbed=True,
            maximum_ticks=700,
            controller_options=r302.controller_options(r302.SELECTED_CONFIG),
        )
        replay = r300.run_case(
            model,
            disturbed=True,
            maximum_ticks=700,
            controller_options=r302.controller_options(r302.SELECTED_CONFIG),
        )
        dormant = r300.run_case(model, disturbed=False, maximum_ticks=60)
        dormant_candidate = r300.run_case(
            model,
            disturbed=False,
            maximum_ticks=60,
            controller_options=r302.controller_options(r302.SELECTED_CONFIG),
        )
        gates = r302.evaluate(
            baseline,
            default,
            candidate,
            replay,
            dormant,
            dormant_candidate,
        )
        behavior = {
            "candidate_has_no_fall_boundary",
            "candidate_recovers_supported_upright",
            "candidate_never_stalls_on_body_without_wheel_support",
        }
        self.assertTrue(
            all(value for name, value in gates.items() if name not in behavior),
            gates,
        )
        self.assertTrue(all(not gates[name] for name in behavior), gates)
        summary = r300.summarize(candidate)
        self.assertEqual(summary["terminal_tick"], 644)
        self.assertIsNone(summary["supported_upright_recovery_tick"])
        self.assertGreaterEqual(summary["maximum_body_ground_stall_ticks"], 400)

    def test_calibration_grid_and_frozen_selection_are_explicit(self) -> None:
        self.assertEqual(len(r302.calibration_profiles()), 160)
        self.assertEqual(len(r302.SELECTED_CONFIG), 11)
        self.assertIn(r302.SELECTED_CONFIG, r302.calibration_profiles())


if __name__ == "__main__":
    unittest.main()
