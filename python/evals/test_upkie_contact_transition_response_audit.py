from __future__ import annotations

import unittest

import numpy as np

import upkie_contact_transition_response_audit as audit


class ContactTransitionResponseAuditTests(unittest.TestCase):
    def test_witness_uses_model_normal_mass_with_declared_total_mass_floor(self) -> None:
        witness = audit.build_contact_witnesses(
            np.asarray([[0.0, 0.0, -2.0], [0.0, 0.0, 1.0]], np.float64),
            np.asarray([[1.0, 2.0, 3.0], [4.0, 5.0, 8.0]], np.float64),
            5.0,
            0.4,
            {
                "normal_acceleration_upper_m_s2": 100.0,
                "effective_mass_scale": 2.0,
                "total_mass_floor_scale": 1.0,
                "normal_load_scale": 2.0,
                "restitution_upper": 1.0,
            },
        )
        np.testing.assert_allclose(witness[:, 0], np.asarray([2.5, 0.5]))
        np.testing.assert_allclose(witness[:, 1], np.asarray([6.0, 16.0]))
        np.testing.assert_allclose(witness[:, 2], np.asarray([98.1, 98.1]))
        np.testing.assert_allclose(witness[:, 3], np.asarray([0.4, 0.4]))

    def test_score_requires_every_velocity_component_per_sample(self) -> None:
        base = {
            "interval_width": [2.0] * 12,
            "interval_utilization": [0.5] * 12,
            "impulse_covered": True,
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
        samples = [
            {
                **base,
                "covered": True,
                "component_covered": [True] * 12,
                "component_exceedance": [0.0] * 12,
            },
            {
                **base,
                "covered": False,
                "component_covered": [True] * 11 + [False],
                "component_exceedance": [0.0] * 11 + [3.0],
            },
        ]
        result = audit.score_profile(samples)
        self.assertEqual(result["sample_coverage"], 0.5)
        self.assertEqual(result["component_coverage"], 23.0 / 24.0)
        self.assertEqual(result["maximum_component_exceedance_per_s"], 3.0)
        self.assertEqual(result["uncovered_samples"][0]["coordinates"], ["right_wheel"])
        self.assertTrue(result["zero_rust_allocation"])


if __name__ == "__main__":
    unittest.main()
