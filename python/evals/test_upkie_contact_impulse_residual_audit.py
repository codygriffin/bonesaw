from __future__ import annotations

import unittest

import numpy as np

import upkie_contact_impulse_residual_audit as audit


class ContactImpulseResidualAuditTests(unittest.TestCase):
    def test_summary_keeps_support_qdd_pressure_and_impulse_distinct(self) -> None:
        samples = {
            "action": np.asarray([0, 2], np.uint8),
            "support_mask": np.asarray([3, 1], np.uint8),
            "best_support_mask": np.asarray([3, 2], np.uint8),
            "physical_support_error_norm": np.asarray([1.0, 2.0]),
            "best_support_error_norm": np.asarray([1.0, 1.0]),
            "qdd_envelope_exceedance": np.asarray([0.0, 2.0]),
            "pressure_envelope_exceedance": np.asarray([0.0, 0.0]),
            "normal_impulse_ns": np.asarray([0.1, 0.2]),
            "tangential_impulse_ns": np.asarray([0.01, 0.20]),
            "constraint_impulse_norm": np.asarray([0.2, 0.5]),
            "external_impulse_ns": np.asarray([0.0, 0.1]),
            "score_allocation_calls": np.zeros(2, np.uint64),
            "score_allocated_bytes": np.zeros(2, np.uint64),
        }

        summary = audit.summarize(samples)

        self.assertEqual(summary["sample_count"], 2)
        self.assertEqual(summary["physical_support_is_best_fraction"], 0.5)
        self.assertEqual(summary["qdd_componentwise_coverage"], 0.5)
        self.assertEqual(summary["pressure_componentwise_coverage"], 1.0)
        self.assertEqual(summary["action_counts"], {"0": 1, "1": 0, "2": 1})
        self.assertTrue(summary["zero_rust_rescore_allocation"])


if __name__ == "__main__":
    unittest.main()
