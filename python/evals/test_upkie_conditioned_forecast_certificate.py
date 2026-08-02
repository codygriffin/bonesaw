import unittest

import numpy as np

import upkie_conditioned_forecast_certificate as certificate


class ConditionedForecastCertificateTests(unittest.TestCase):
    def test_new_holdout_is_named_and_parameter_disjoint(self) -> None:
        retained = {case.name for case in certificate.case_matrix()}
        holdout = certificate.new_holdout_cases()
        self.assertEqual(len(holdout), 13)
        self.assertFalse(retained & {case.name for case in holdout})
        self.assertEqual(len({case.name for case in holdout}), len(holdout))

    def test_sparse_cell_refuses_joint_certificate(self) -> None:
        component = certificate.ComponentSample(
            "case", 0, 0, 0, (0, 3, 0, 0, 0, 0), 0.1
        )
        joint = certificate.JointSample("case", 0, 0, (component,) * 8)
        result = certificate.evaluate_samples(
            [joint],
            {component.cell: {"count": 7, "maximum_error": 0.1, "bound": 0.2}},
        )
        self.assertEqual(result["issued"], 0)
        self.assertEqual(result["misses"], 0)

    def test_fit_uses_reserve_and_holdout_does_not_mutate_cells(self) -> None:
        calibration = [
            certificate.JointSample(
                "cal",
                index,
                0,
                tuple(
                    certificate.ComponentSample(
                        "cal",
                        index,
                        0,
                        component,
                        (0, 3, component, 0, 0, 0),
                        0.01,
                    )
                    for component in range(8)
                ),
            )
            for index in range(certificate.MINIMUM_CELL_SAMPLES)
        ]
        cells = certificate.fit_cells(calibration)
        snapshot = {key: dict(value) for key, value in cells.items()}
        result = certificate.evaluate_samples(calibration[:1], cells)
        self.assertEqual(result["issued"], 1)
        self.assertEqual(result["misses"], 0)
        self.assertEqual(cells, snapshot)
        self.assertTrue(
            all(
                np.isclose(value["bound"], 0.02)
                for value in cells.values()
            )
        )


if __name__ == "__main__":
    unittest.main()
