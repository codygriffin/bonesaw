#!/usr/bin/env python3
"""Tests for parsing stable perf-stat counter artifacts."""

import pathlib
import tempfile
import unittest

from cpu_counter_stability import read_perf, relative_span


class CpuCounterStabilityTests(unittest.TestCase):
    def test_reads_user_space_perf_csv(self) -> None:
        contents = """# started now
100,,task-clock:u,100,100.00
200,,cycles:u,100,100.00
300,,instructions:u,100,100.00
40,,branches:u,100,100.00
5,,branch-misses:u,100,100.00
6,,cache-misses:u,100,100.00
"""
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "perf.csv"
            path.write_text(contents)
            counters = read_perf(path)

        self.assertEqual(counters["instructions"], 300)
        self.assertEqual(counters["task-clock"], 100)

    def test_relative_span_uses_median_scale(self) -> None:
        import numpy as np

        self.assertAlmostEqual(relative_span(np.array([8.0, 10.0, 12.0])), 0.4)


if __name__ == "__main__":
    unittest.main()
