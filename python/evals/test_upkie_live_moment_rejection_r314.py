from __future__ import annotations

import pathlib
import sys
import unittest

import numpy as np

EVALS = pathlib.Path(__file__).resolve().parent
ROOT = EVALS.parents[1]
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

import upkie_live_moment_rejection_r314 as r314  # noqa: E402


EVALUATION_PROVENANCE = {
    "source": "evaluation_harness",
    "load_class": "declared_continuous_wrench",
    "force_frame": "world",
    "application_point_frame": "world",
}


class UpkieLiveMomentRejectionR314Tests(unittest.TestCase):
    def test_candidate_is_explicit_and_default_off(self) -> None:
        self.assertEqual(r314.CANDIDATE_OPTIONS["centroidal_angular_momentum_weight"], 0.0)
        self.assertTrue(r314.CANDIDATE_OPTIONS["external_wrench_feedforward_enabled"])
        self.assertAlmostEqual(
            r314.CANDIDATE_OPTIONS["external_wrench_feedforward_scale"], 0.7
        )
        self.assertEqual(r314.CANDIDATE_OPTIONS["root_roll_damping"], 12.0)
        self.assertNotIn(
            "centroidal_angular_momentum_weight",
            r314.r312.PUBLIC_CONTROLLER_OPTIONS,
        )

    def test_short_holdout_reports_applied_moment_and_finite_outputs(self) -> None:
        case = r314.run_case(
            ROOT / "models" / "upkie" / "upkie.urdf",
            8.0,
            r314.CANDIDATE_OPTIONS,
            ticks=80,
        )
        summary = r314.summarize(case)
        self.assertGreater(summary["maximum_external_moment_nm"], 1.9)
        self.assertTrue(summary["external_moment_finite"])
        self.assertTrue(all(isinstance(tick, int) for tick in summary["nonadmitted_ticks"]))
        self.assertFalse(summary["numeric_reset"])

    def test_full_wrench_feedforward_consumes_the_delayed_root_wrench(self) -> None:
        from upkie_live_plant_worker import LiveUpkiePlant

        worker = LiveUpkiePlant(
            ROOT / "models" / "upkie" / "upkie.urdf",
            controller_options=r314.CANDIDATE_OPTIONS,
        )
        body_id = worker.body_by_name["base"]
        force = np.asarray([0.0, 2.0, 0.0], np.float64)
        point = worker.data.xipos[body_id] + np.asarray([0.0, 0.0, 0.2])
        root_body_id = int(worker.model.body_rootid[body_id])
        expected_moment = np.cross(point - worker.data.xpos[root_body_id], force)
        worker.step(
            {
                "type": "step",
                "command_id": 11,
                "external_load": {
                    "active": True,
                    "body": "base",
                    "force_world": force.tolist(),
                    "application_point_world": point.tolist(),
                    "provenance": EVALUATION_PROVENANCE,
                    "request_id": 11,
                },
            }
        )
        self.assertFalse(worker.controller.external_wrench_observation_valid)
        next_root_origin = np.asarray(
            worker.data.xpos[root_body_id], dtype=np.float64
        ).copy()
        worker.step({"type": "step", "command_id": 12})
        self.assertTrue(worker.controller.external_wrench_observation_valid)
        np.testing.assert_allclose(
            worker.controller.external_wrench_world[0],
            np.r_[np.cross(point - next_root_origin, force), force],
            atol=1.0e-12,
        )

    def test_feedforward_scale_is_bounded_at_the_adapter_boundary(self) -> None:
        from upkie_live_plant_worker import LiveUpkiePlant

        with self.assertRaises(ValueError):
            LiveUpkiePlant(
                ROOT / "models" / "upkie" / "upkie.urdf",
                controller_options={
                    "external_wrench_feedforward_enabled": True,
                    "external_wrench_feedforward_scale": 1.01,
                },
            )

    def test_axis_scales_are_explicit_and_rust_owned(self) -> None:
        from upkie_live_plant_worker import LiveUpkiePlant

        axis_scales = (0.68, 0.68, 0.68, 0.52, 0.52, 0.52)
        worker = LiveUpkiePlant(
            ROOT / "models" / "upkie" / "upkie.urdf",
            controller_options={
                **r314.CANDIDATE_OPTIONS,
                "external_wrench_feedforward_axis_scales": axis_scales,
            },
        )
        np.testing.assert_allclose(
            worker.controller.external_wrench_feedforward_axis_scales,
            axis_scales,
        )
        contract = worker.hello()["external_load_contract"][
            "wbc_external_wrench_feedforward"
        ]
        self.assertEqual(contract["axis_order"], "root_moment_xyz_then_root_force_xyz")
        self.assertEqual(contract["axis_scales"], list(axis_scales))

        with self.assertRaises(ValueError):
            LiveUpkiePlant(
                ROOT / "models" / "upkie" / "upkie.urdf",
                controller_options={
                    "external_wrench_feedforward_axis_scales": (0.5,) * 5,
                },
            )

    def test_axis_scale_changes_feedforward_consequence_without_changing_public_default(self) -> None:
        from upkie_live_plant_worker import LiveUpkiePlant

        def delayed_effort(options: dict[str, object]) -> np.ndarray:
            worker = LiveUpkiePlant(
                ROOT / "models" / "upkie" / "upkie.urdf",
                controller_options=options,
            )
            body_id = worker.body_by_name["base"]
            point = worker.data.xipos[body_id] + np.asarray([0.0, 0.0, 0.2])
            command = {
                "type": "step",
                "command_id": 21,
                "external_load": {
                    "active": True,
                    "body": "base",
                    "force_world": [0.0, 2.0, 0.0],
                    "application_point_world": point.tolist(),
                    "provenance": EVALUATION_PROVENANCE,
                    "request_id": 21,
                },
            }
            worker.step(command)
            state = worker.step({"type": "step", "command_id": 22})
            return np.asarray(state["actuator_effort_nm"], np.float64)

        scalar = delayed_effort(dict(r314.CANDIDATE_OPTIONS))
        split = delayed_effort(
            {
                **r314.CANDIDATE_OPTIONS,
                "external_wrench_feedforward_axis_scales": (0.70, 0.70, 0.70, 0.20, 0.20, 0.20),
            }
        )
        self.assertGreater(float(np.max(np.abs(scalar - split))), 1.0e-7)
        with self.assertRaises(ValueError):
            LiveUpkiePlant(
                ROOT / "models" / "upkie" / "upkie.urdf",
                controller_options={
                    "external_wrench_feedforward_axis_scales": (0.5,) * 5 + (1.1,),
                },
            )


if __name__ == "__main__":
    unittest.main()
