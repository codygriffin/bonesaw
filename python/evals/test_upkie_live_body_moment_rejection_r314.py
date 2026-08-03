from __future__ import annotations

import pathlib
import tempfile
import unittest

import upkie_live_body_moment_rejection_r314 as r314


class UpkieBodyMomentRejectionR314Test(unittest.TestCase):
    def test_bounded_behavior_qualifies_without_full_recovery_promotion(self) -> None:
        model = pathlib.Path("models/upkie/upkie.urdf")
        with tempfile.TemporaryDirectory() as directory:
            metrics = r314.run(
                model,
                pathlib.Path(directory),
                screen_profiles=((36.0, 14.0, 10.5),),
            )
        self.assertTrue(metrics["bounded_behavior_qualified"])
        self.assertFalse(metrics["recovery_promoted"])
        self.assertTrue(all(metrics["mechanism_gates"].values()))
        self.assertTrue(all(metrics["behavior_gates"].values()))
        self.assertFalse(all(metrics["promotion_gates"].values()))
        by_force = {row["force_y_n"]: row for row in metrics["rows"]}
        for force in (-6.0, 6.0):
            self.assertIsNone(by_force[force]["candidate"]["terminal_tick"])
            self.assertTrue(
                by_force[force]["candidate"]["final_100_bilateral_upright"]
            )
        self.assertEqual(
            sum(
                row["candidate"]["terminal_pending"] is not None
                for row in metrics["rows"]
            ),
            2,
        )


if __name__ == "__main__":
    unittest.main()
