from __future__ import annotations

import unittest

from external_load_provenance_audit import DECLARED_CLASS, command, provenance


class ExternalLoadProvenanceAuditTests(unittest.TestCase):
    def test_command_keeps_source_class_and_frames_explicit(self) -> None:
        value = command(4, [0.0, 0.0, 0.5], source="interactive_operator")
        self.assertEqual(value["provenance"], provenance("interactive_operator"))
        self.assertEqual(value["provenance"]["load_class"], DECLARED_CLASS)
        self.assertEqual(value["provenance"]["force_frame"], "world")
        self.assertEqual(value["provenance"]["application_point_frame"], "world")

    def test_missing_provenance_is_a_distinct_negative_fixture(self) -> None:
        value = command(5, [0.0, 0.0, 0.5], include_provenance=False)
        self.assertNotIn("provenance", value)

    def test_evidence_only_classes_do_not_change_the_command_shape(self) -> None:
        for load_class in ("measured_impact_impulse", "unobserved_model_reserve"):
            value = command(6, [0.0, 0.0, 0.5], load_class=load_class)
            self.assertEqual(value["provenance"]["load_class"], load_class)
            self.assertEqual(value["type"], "plant_push")


if __name__ == "__main__":
    unittest.main()
