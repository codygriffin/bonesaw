from __future__ import annotations

import json
import pathlib
import unittest


METRICS = pathlib.Path(
    "benchmarks/results/g1-floating-reacquisition-localization-r267/"
    "g1-floating-reacquisition-localization-metrics.json"
)


class FloatingReacquisitionLocalizationR267Test(unittest.TestCase):
    def test_per_target_edge_passes_but_controller_profile_is_rejected(self) -> None:
        result = json.loads(METRICS.read_text())
        self.assertEqual(
            result["revision"], "g1-floating-reacquisition-localization-r267"
        )
        self.assertTrue(result["artifact_only"])
        self.assertTrue(result["mechanism_passed"])
        self.assertTrue(result["per_target_reacquisition_exercised"])
        self.assertTrue(result["target_release_after_attempt"])
        self.assertFalse(result["target_locked_after_reacquisition"])
        self.assertTrue(result["profile_rejected"])
        self.assertFalse(result["default_changed"])
        self.assertFalse(result["authority_admitted"])
        self.assertEqual(result["physics_steps"], 0)
        self.assertEqual(result["policy_steps"], 0)
        self.assertEqual(result["plant_actions"], 0)

        cap_rows = {row["cap"]: row for row in result["projection_cap_sweep"]}
        self.assertEqual(cap_rows[512]["status_at_tick_300"], 4)
        self.assertEqual(cap_rows[768]["status_at_tick_300"], 1)
        self.assertGreater(cap_rows[768]["maximum_ms"], 20.0)

        iteration_rows = {
            row["maximum_iterations"]: row
            for row in result["equality_repair_iteration_sweep"]
        }
        self.assertEqual(iteration_rows[8]["first_contingency_tick"], 302)
        self.assertEqual(iteration_rows[16]["first_contingency_tick"], 330)
        self.assertEqual(result["native_contingency_ticks"], 270)
        self.assertEqual(
            result["native_equality_repair"]["deadline_misses"]["20ms"], 51
        )
        self.assertEqual(
            result["known_r165_physical_gate"],
            "rejected_11_earlier_fall_boundaries",
        )


if __name__ == "__main__":
    unittest.main()
