from __future__ import annotations

import unittest

import numpy as np

import upkie_directional_contact_transition_audit as audit


class DirectionalContactTransitionAuditTests(unittest.TestCase):
    def test_directional_witness_is_axis_ordered_and_nonnegative(self) -> None:
        config = audit.DIRECTIONAL_PROFILES[audit.PRIMARY_PROFILE]
        witness = audit.build_directional_witnesses(
            np.asarray([[-2.0, 3.0, -4.0]], np.float64),
            np.asarray([[1.0, 2.0, 3.0]], np.float64),
            5.0,
            0.4,
            config,
        )
        np.testing.assert_allclose(witness[0, :3], [2.5, 3.5, 4.5])
        np.testing.assert_allclose(witness[0, 3:6], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(witness[0, 6:9], [8.0, 8.0, 98.1])
        self.assertEqual(witness[0, 9], 0.4)

    def test_structured_reserve_is_grouped_by_generalized_coordinate(self) -> None:
        np.testing.assert_array_equal(
            audit.STRUCTURED_ACCELERATION_RESERVE,
            np.asarray([5.0] * 6 + [50.0] * 6),
        )

    def test_impulse_score_separates_sample_and_component_coverage(self) -> None:
        base = {
            "covered": True,
            "component_covered": [True] * 12,
            "component_exceedance": [0.0] * 12,
            "interval_width": [2.0] * 12,
            "interval_utilization": [0.5] * 12,
            "impulse_covered": True,
            "actual_impulse_world_ns": [[0.5, 0.0, 1.0]],
            "impulse_upper_ns": [[1.0, 0.0, 2.0]],
            "model_timing_ns": 100,
            "model_allocation_calls": 0,
            "model_allocated_bytes": 0,
            "bound_timing_ns": 200,
            "bound_allocation_calls": 0,
            "bound_allocated_bytes": 0,
            "case": "case",
            "plant_profile": "plant",
            "tick": 1,
            "selected_action": 0,
        }
        result = audit.score_samples([base])
        self.assertEqual(result["impulse_component_coverage"], 1.0)
        self.assertEqual(result["maximum_impulse_utilization"]["p95"], 0.5)
        self.assertEqual(result["maximum_impulse_exceedance_ns"], 0.0)


if __name__ == "__main__":
    unittest.main()
