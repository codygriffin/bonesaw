from __future__ import annotations

import pathlib
import unittest

import numpy as np

import upkie_contact_transition_interval_audit as audit


class ContactTransitionIntervalAuditTests(unittest.TestCase):
    def test_upkie_mass_is_finite_and_positive(self) -> None:
        mass = audit.urdf_total_mass_kg(
            pathlib.Path("models/upkie/upkie.urdf").resolve()
        )
        self.assertGreater(mass, 0.0)
        self.assertTrue(np.isfinite(mass))

    def test_witness_uses_only_pre_step_closing_velocity_and_authored_bounds(self) -> None:
        witness = audit.build_contact_witnesses(
            np.asarray([[1.0, 2.0, -3.0], [0.0, 0.0, 4.0]]),
            5.0,
            0.4,
            {
                "normal_acceleration_upper_m_s2": 100.0,
                "effective_mass_scale": 1.0,
                "normal_load_scale": 2.0,
                "restitution_upper": 1.0,
            },
        )
        np.testing.assert_allclose(witness[:, 0], np.asarray([3.5, 0.5]))
        np.testing.assert_allclose(witness[:, 1], np.asarray([5.0, 5.0]))
        np.testing.assert_allclose(witness[:, 2], np.asarray([98.1, 98.1]))
        np.testing.assert_allclose(witness[:, 3], np.asarray([0.4, 0.4]))

    def test_sample_requires_every_contact_component(self) -> None:
        samples = [
            {
                "covered": True,
                "component_covered": [True, True, True, True],
                "normal_exceedance": 0.0,
                "tangential_exceedance": 0.0,
                "maximum_normal_upper": 1.0,
                "maximum_impulse_utilization": 0.5,
                "timing_ns": 100,
                "allocation_calls": 0,
                "allocated_bytes": 0,
                "case": "a",
                "plant_profile": "p",
                "tick": 0,
            },
            {
                "covered": False,
                "component_covered": [True, True, True, False],
                "normal_exceedance": 0.0,
                "tangential_exceedance": 0.2,
                "maximum_normal_upper": 2.0,
                "maximum_impulse_utilization": 1.1,
                "timing_ns": 200,
                "allocation_calls": 0,
                "allocated_bytes": 0,
                "case": "b",
                "plant_profile": "p",
                "tick": 1,
            },
        ]
        result = audit.score_profile(samples)
        self.assertEqual(result["sample_coverage"], 0.5)
        self.assertEqual(result["component_coverage"], 0.875)
        self.assertEqual(result["maximum_tangential_exceedance_ns"], 0.2)
        self.assertTrue(result["zero_rust_allocation"])

    def test_covered_worst_sample_is_highest_utilization(self) -> None:
        samples = [
            {
                "covered": True,
                "component_covered": [True, True, True, True],
                "normal_exceedance": 0.0,
                "tangential_exceedance": 0.0,
                "maximum_normal_upper": 2.0,
                "maximum_impulse_utilization": utilization,
                "timing_ns": 100,
                "allocation_calls": 0,
                "allocated_bytes": 0,
                "case": case,
                "plant_profile": "p",
                "tick": tick,
            }
            for case, tick, utilization in (("low", 0, 0.1), ("high", 1, 0.9))
        ]

        result = audit.score_profile(samples)

        self.assertEqual(result["worst_sample"]["case"], "high")


if __name__ == "__main__":
    unittest.main()
