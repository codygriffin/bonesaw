from __future__ import annotations

import pathlib
import unittest

import numpy as np

from g1_compliant_terminal_consequence_audit import (
    ROOT_IMPACT_PLANE_M,
    SOURCE_REPLAY,
    joint_limits,
    reconstruct_prevelocity,
)
from g1_contact_law_momentum_holdout import FOOT_FRAMES


ROOT = pathlib.Path(__file__).resolve().parents[2]
MODEL = ROOT / "benchmarks" / "cache" / "unitree-g1" / "g1_23dof_mode_10.urdf"


class G1CompliantTerminalConsequenceAuditTests(unittest.TestCase):
    def test_reconstructed_velocity_is_finite_and_state_local(self) -> None:
        first = reconstruct_prevelocity(23, 70_000)
        repeat = reconstruct_prevelocity(23, 70_000)
        other = reconstruct_prevelocity(23, 70_001)
        self.assertEqual(first.shape, (29,))
        np.testing.assert_array_equal(first, repeat)
        self.assertTrue(np.all(np.isfinite(first)))
        self.assertFalse(np.array_equal(first, other))

    def test_declared_impact_plane_and_model_limits_are_valid(self) -> None:
        import bonesaw

        replay = np.load(ROOT / SOURCE_REPLAY)
        self.assertTrue(
            all(
                np.all(replay[f"{name}_root_height"] > ROOT_IMPACT_PLANE_M)
                for name in ("mid_elliptic_implicitfast", "hard_pyramidal_rk4")
            )
        )
        session = bonesaw.ContactTransitionModelSession(
            str(MODEL), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
        )
        lower, upper, velocity = joint_limits(MODEL, list(session.joint_names()))
        self.assertEqual(lower.shape, (23,))
        self.assertTrue(np.all(lower < upper))
        self.assertTrue(np.all(velocity > 0.0))


if __name__ == "__main__":
    unittest.main()
