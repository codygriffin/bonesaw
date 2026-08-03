from __future__ import annotations

import pathlib
import tempfile
import unittest

import upkie_live_force_backed_relock_r313 as r313


class UpkieForceBackedRelockR313Test(unittest.TestCase):
    def test_bounded_relock_qualifies_without_promoting_recovery(self) -> None:
        model = pathlib.Path("models/upkie/upkie.urdf")
        with tempfile.TemporaryDirectory() as directory:
            metrics = r313.run(
                model,
                pathlib.Path(directory),
                forces=r313.TERMINAL_FORCES_N,
            )
        self.assertTrue(metrics["bounded_relock_qualified"])
        self.assertFalse(metrics["recovery_promoted"])
        self.assertTrue(all(metrics["mechanism_gates"].values()))
        self.assertFalse(all(metrics["promotion_gates"].values()))
        selected = [row for row in metrics["preload_screen"] if row["selected"]]
        self.assertEqual(len(selected), 1)
        self.assertAlmostEqual(
            selected[0]["target_wheel_height_m"],
            r313.TARGET_WHEEL_HEIGHT_M,
        )
        by_force = {row["force_y_n"]: row for row in metrics["rows"]}
        for force in (-6.0, 6.0):
            self.assertGreaterEqual(
                by_force[force]["candidate"]["force_qualified_relock_count"], 3
            )
            self.assertGreaterEqual(by_force[force]["terminal_delta_vs_r312"], 0)
        self.assertEqual(
            sum(
                row["candidate"]["terminal_pending"] is not None
                for row in metrics["rows"]
            ),
            4,
        )


if __name__ == "__main__":
    unittest.main()
