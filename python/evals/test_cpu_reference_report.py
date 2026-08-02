#!/usr/bin/env python3
"""Regression tests for the dependency-free report renderer."""

import unittest

from cpu_reference_report import render_report_html


class RenderReportHtmlTests(unittest.TestCase):
    def test_uses_first_level_one_heading_as_document_title(self) -> None:
        rendered = render_report_html("# CPU tail <audit>\n\nResult")

        self.assertIn("<title>CPU tail &lt;audit&gt;</title>", rendered)

    def test_accepts_explicit_title_without_heading(self) -> None:
        rendered = render_report_html("Result", title="Bonesaw & report")

        self.assertIn("<title>Bonesaw &amp; report</title>", rendered)


if __name__ == "__main__":
    unittest.main()
