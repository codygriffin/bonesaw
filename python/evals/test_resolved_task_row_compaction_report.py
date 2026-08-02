#!/usr/bin/env python3
"""Unit tests for r66 delta summaries."""

import unittest

import numpy as np

from resolved_task_row_compaction_report import delta


class ResolvedTaskRowCompactionReportTests(unittest.TestCase):
    def test_delta_counts_values_and_ticks_separately(self) -> None:
        left = np.zeros((3, 2))
        right = left.copy()
        right[1] = [3.0, 4.0]

        result = delta(left, right)

        self.assertEqual(result["changed_values"], 2)
        self.assertEqual(result["changed_ticks"], 1)
        self.assertEqual(result["maximum"], 4.0)


if __name__ == "__main__":
    unittest.main()
