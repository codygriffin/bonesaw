from __future__ import annotations

import json
import pathlib
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULTS = ROOT / "benchmarks/results"
METRICS = RESULTS / (
    "g1-dcm-transition-gate-r277/g1-dcm-transition-gate-metrics.json"
)


class DcmTransitionGateR277Test(unittest.TestCase):
    def test_dormant_replay_is_exact_and_not_authoritative(self) -> None:
        result = json.loads(METRICS.read_text())
        self.assertTrue(result["mechanism_passed"])
        self.assertTrue(result["profile_rejected"])
        self.assertFalse(result["authority_admitted"])
        self.assertEqual(result["policy_steps"], 0)
        self.assertEqual(result["physics_steps"], 0)
        self.assertEqual(result["source_contract"]["dormant_semantic_differences"], [])
        self.assertEqual(result["source_contract"]["shared_non_timing_array_count"], 82)
        self.assertEqual(result["source_contract"]["new_arrays"], ["dcm_pre_liftoff_active"])

    def test_gate_is_binary_and_covers_only_transition_or_single_support(self) -> None:
        raw = RESULTS / (
            "floating-g1-r277-dcm-gated-w0p00005-h10-cap8/"
            "floating-walk-raw.npz"
        )
        with np.load(raw, allow_pickle=False) as trace:
            active = trace["dcm_pre_liftoff_active"]
            contacts = trace["contact_active"][:, :2]
            self.assertEqual(active.dtype, np.uint8)
            self.assertTrue(np.all(np.isin(active, (0, 1))))
            self.assertGreater(int(np.count_nonzero(active)), 0)
            for tick in np.flatnonzero(active):
                current = int(np.count_nonzero(contacts[tick]))
                if current == 1:
                    continue
                self.assertGreater(current, 1)
                stop = min(len(active), int(tick) + 11)
                future = np.count_nonzero(contacts[int(tick) + 1 : stop], axis=1)
                self.assertTrue(np.any(future < current))

    def test_zero_horizon_preserves_continuous_dcm_semantics(self) -> None:
        raw = RESULTS / (
            "floating-g1-r277-dcm-continuous-w0p025-cap8/"
            "floating-walk-raw.npz"
        )
        with np.load(raw, allow_pickle=False) as trace:
            np.testing.assert_array_equal(
                trace["dcm_pre_liftoff_active"],
                np.ones(2317, dtype=np.uint8),
            )


if __name__ == "__main__":
    unittest.main()
