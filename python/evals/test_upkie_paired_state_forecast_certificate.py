import unittest

import upkie_paired_state_forecast_certificate as certificate


class PairedStateForecastCertificateTests(unittest.TestCase):
    def test_second_holdout_is_disjoint_from_both_prior_corpora(self) -> None:
        prior = {case.name for case in certificate.case_matrix()}
        prior |= {case.name for case in certificate.r160_holdout_cases()}
        holdout = certificate.new_holdout_cases()
        self.assertEqual(len(holdout), 13)
        self.assertFalse(prior & {case.name for case in holdout})

    def test_overwide_component_bound_refuses_joint_certificate(self) -> None:
        components = tuple(
            certificate.ComponentSample(
                "case",
                0,
                0,
                component,
                (0, 3, component, 0, 0, 0, 0),
                0.0,
            )
            for component in range(8)
        )
        cells = {
            component.cell: {
                "count": certificate.MINIMUM_CELL_SAMPLES,
                "maximum_error": 0.0,
                "bound": certificate.STATE_SCALES[component.component] * 0.5,
            }
            for component in components
        }
        cells[components[2].cell]["bound"] = certificate.STATE_SCALES[2] * 1.01
        result = certificate.evaluate_samples(
            [certificate.JointSample("case", 0, 0, components)], cells
        )
        self.assertEqual(result["issued"], 0)
        self.assertEqual(result["refusal_counts"]["bound_too_large"], 1)


if __name__ == "__main__":
    unittest.main()
