from __future__ import annotations

import json
import pathlib
import unittest

from g1_actuator_realization_profile_audit import (
    CONTROL_DT_SECONDS,
    PROFILE_NAMES,
    PROFILE_PARAMETERS,
    REVISION,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / "benchmarks/results/g1-actuator-realization-profile-audit-r249/g1-actuator-realization-profile-audit-metrics.json"


class G1ActuatorRealizationProfileEvidenceTests(unittest.TestCase):
    def test_profile_family_is_fixed_at_the_50_hz_boundary(self) -> None:
        self.assertEqual(REVISION, "g1-actuator-realization-profile-audit-r249")
        self.assertEqual(CONTROL_DT_SECONDS, 0.020)
        self.assertEqual(len(PROFILE_NAMES), 4)
        self.assertEqual(PROFILE_PARAMETERS.shape, (4, 2))
        self.assertEqual(PROFILE_PARAMETERS[0].tolist(), [float("inf"), float("inf")])

    def test_archived_audit_is_mechanism_only(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(metrics["samples"], 96)
        self.assertEqual(metrics["control_dt_seconds"], 0.020)
        self.assertEqual(metrics["physics_steps"], 0)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["plant_actions"], 0)
        self.assertTrue(metrics["mechanism_passed"])
        self.assertIsNone(metrics["selected_profile_for_fresh_plant"])
        self.assertFalse(metrics["fresh_plant_profile_selected"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(
            metrics["diagnostic_reference_profile"],
            "bandwidth_25hz_slew_1000_nm_s",
        )
        self.assertEqual(len(metrics["profiles"]), 4)
        self.assertTrue(all(row["repeat_passed"] for row in metrics["profiles"]))
        self.assertTrue(all(row["zero_rust_allocation"] for row in metrics["profiles"]))


if __name__ == "__main__":
    unittest.main()
