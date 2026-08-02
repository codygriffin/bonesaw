#!/usr/bin/env python3
"""Unit tests for native WBC counter report semantics."""

import unittest

from native_wbc_counter import semantic_projection, summarize


class NativeWbcCounterTests(unittest.TestCase):
    def test_semantic_projection_removes_only_timing_fields(self) -> None:
        report = {
            "model": "g1",
            "program_fingerprint": "abc",
            "scope": "native",
            "floating_dynamic_wbc": {
                "ticks": 20,
                "infeasible_ticks": 0,
                "mean_tick_us": 1.0,
                "p50_tick_us": 2.0,
                "p99_tick_us": 3.0,
                "max_tick_us": 4.0,
            },
        }

        projected = semantic_projection(report)["floating_dynamic_wbc"]

        self.assertEqual(projected, {"ticks": 20, "infeasible_ticks": 0})

    def test_summary_retains_distribution_and_relative_span(self) -> None:
        result = summarize([8.0, 10.0, 12.0])

        self.assertEqual(result["median"], 10.0)
        self.assertAlmostEqual(result["relative_span"], 0.4)

    def test_zero_summary_is_finite(self) -> None:
        result = summarize([0.0, 0.0, 0.0])

        self.assertEqual(result["relative_span"], 0.0)
        self.assertEqual(result["coefficient_of_variation"], 0.0)


if __name__ == "__main__":
    unittest.main()
