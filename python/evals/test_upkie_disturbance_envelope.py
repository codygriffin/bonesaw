from __future__ import annotations

import math
import pathlib
import sys
import unittest

import numpy as np


EVALS = pathlib.Path(__file__).resolve().parent
ROOT = EVALS.parents[1]
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

import upkie_disturbance_envelope as envelope  # noqa: E402
import upkie_mujoco_plant_report as plant  # noqa: E402


MODEL = ROOT / "models/upkie/upkie.urdf"


class UpkieDisturbanceEnvelopeTests(unittest.TestCase):
    def test_support_contingency_configuration_is_rust_validated(self) -> None:
        import bonesaw

        session = bonesaw.UpkieBalanceSession(str(MODEL))
        configured = (
            9.81,
            6.0,
            12.0,
            8.0,
            60.0,
            14.0,
            10.0,
            10.0,
            24.0,
            90.0,
            140.0,
        )
        session.configure_support_contingency(*configured)
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            session.configure_support_contingency(
                *configured[:2], 0.0, *configured[3:]
            )
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            session.configure_support_contingency(
                math.nan, *configured[1:]
            )

    def test_adapter_requires_complete_support_contingency_profile(self) -> None:
        import bonesaw

        balance = bonesaw.UpkieBalanceSession(str(MODEL))
        with self.assertRaisesRegex(ValueError, "exactly 11"):
            plant.RustWbcAdapter(
                MODEL,
                np.asarray([0.0, 0.0, 0.58]),
                0.0,
                balance,
                "capture",
                0.2,
                support_contingency_config=(9.81,),
            )

    def run_contact_observation_profile(
        self, **profile: int | float
    ) -> dict[str, object]:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        return envelope.run_case(
            MODEL,
            case,
            0.15,
            balance_mode="capture",
            fall_safe_enabled=True,
            fall_safe_primary_blend=False,
            measured_contact_admission=True,
            execute_reduced_support=True,
            support_contingency_enabled=True,
            support_contingency_execute=True,
            support_contingency_realize_primary_torque=True,
            contact_program_authority_ticks=0,
            contact_observation_prestart_samples=3,
            use_feasibility_row_spans=True,
            **profile,
        )

    def test_delayed_contact_observation_age_reaches_rust_authority(self) -> None:
        trace = self.run_contact_observation_profile(
            contact_observation_delay_ticks=4
        )

        self.assertTrue(np.all(trace["contact_observation_available"] == 1))
        expected_age_ns = np.minimum(
            (np.arange(len(trace["time_s"]), dtype=np.int64) + 1) * 5_000_000,
            20_000_000,
        )
        np.testing.assert_array_equal(
            trace["contact_observation_age_ns"],
            expected_age_ns,
        )

    def test_steady_prestarted_delay_does_not_invent_timestamp_faults(self) -> None:
        trace = self.run_contact_observation_profile(
            contact_observation_prestart_age_ticks=4,
            contact_observation_delay_ticks=4,
        )

        np.testing.assert_array_equal(
            trace["contact_observation_age_ns"],
            np.full(len(trace["time_s"]), 20_000_000, np.int64),
        )
        np.testing.assert_array_equal(
            trace["contact_observation_status"],
            np.zeros(len(trace["time_s"]), np.uint8),
        )
        np.testing.assert_array_equal(
            trace["contact_observation_provenance"],
            np.zeros(len(trace["time_s"]), np.uint8),
        )
        np.testing.assert_array_equal(
            trace["contact_program_authority_selection"],
            np.ones(len(trace["time_s"]), np.uint8),
        )
        self.assertTrue(np.all(np.linalg.norm(trace["torque"], axis=1) > 0.0))

    def test_contact_observation_dropout_fails_closed_end_to_end(self) -> None:
        trace = self.run_contact_observation_profile(
            contact_observation_dropout_period_ticks=10,
            contact_observation_dropout_burst_ticks=1,
        )
        unavailable = np.flatnonzero(trace["contact_observation_available"] == 0)

        np.testing.assert_array_equal(unavailable, np.asarray([0, 10, 20]))
        np.testing.assert_array_equal(
            trace["contact_program_authority_selection"][unavailable],
            np.zeros(len(unavailable), np.uint8),
        )
        np.testing.assert_array_equal(
            trace["contact_program_authority_executable"][unavailable],
            np.zeros(len(unavailable), np.uint8),
        )
        np.testing.assert_array_equal(
            trace["torque"][unavailable],
            np.zeros((len(unavailable), 6), np.float64),
        )

    def test_physical_contact_impulse_trace_is_opt_in_and_replay_exact(self) -> None:
        dormant = self.run_contact_observation_profile()
        np.testing.assert_array_equal(
            dormant["physical_wheel_contact_impulse_ns"],
            np.zeros((len(dormant["time_s"]), 2, 2), np.float64),
        )
        np.testing.assert_array_equal(
            dormant["physical_wheel_contact_impulse_world_ns"],
            np.zeros((len(dormant["time_s"]), 2, 3), np.float64),
        )
        np.testing.assert_array_equal(
            dormant["physical_wheel_contact_position_m_ns"],
            np.zeros((len(dormant["time_s"]), 2, 3), np.float64),
        )
        np.testing.assert_array_equal(
            dormant["physical_wheel_contact_moment_world_origin_nms"],
            np.zeros((len(dormant["time_s"]), 2, 3), np.float64),
        )
        trace = self.run_contact_observation_profile(
            record_physical_contact_impulses=True
        )
        replay = self.run_contact_observation_profile(
            record_physical_contact_impulses=True
        )
        wheel_impulse = np.asarray(trace["physical_wheel_contact_impulse_ns"])
        generalized_impulse = np.asarray(
            trace["physical_constraint_generalized_impulse_ns"]
        )
        world_impulse = np.asarray(
            trace["physical_wheel_contact_impulse_world_ns"]
        )
        position_m_ns = np.asarray(
            trace["physical_wheel_contact_position_m_ns"]
        )
        moment_world_origin = np.asarray(
            trace["physical_wheel_contact_moment_world_origin_nms"]
        )
        self.assertTrue(np.all(np.isfinite(wheel_impulse)))
        self.assertTrue(np.all(wheel_impulse >= 0.0))
        self.assertGreater(float(np.sum(wheel_impulse[:, :, 0])), 0.0)
        self.assertTrue(np.all(np.isfinite(world_impulse)))
        self.assertGreater(float(np.max(np.abs(world_impulse))), 0.0)
        self.assertTrue(np.all(np.isfinite(position_m_ns)))
        self.assertTrue(np.all(np.isfinite(moment_world_origin)))
        self.assertGreater(float(np.max(np.abs(moment_world_origin))), 0.0)
        active = wheel_impulse[:, :, 0] > 0.0
        centroid = np.divide(
            position_m_ns,
            wheel_impulse[:, :, :1],
            out=np.zeros_like(position_m_ns),
            where=wheel_impulse[:, :, :1] > 0.0,
        )
        self.assertTrue(np.all(np.isfinite(centroid[active])))
        self.assertTrue(np.all(np.abs(centroid[active][:, 2]) < 0.01))
        self.assertTrue(
            np.all(
                np.linalg.norm(world_impulse[:, :, :2], axis=2)
                <= wheel_impulse[:, :, 1] + 1.0e-12
            )
        )
        self.assertTrue(
            np.all(
                np.abs(world_impulse[:, :, 2])
                <= wheel_impulse[:, :, 0] + 1.0e-12
            )
        )
        self.assertTrue(np.all(np.isfinite(generalized_impulse)))
        self.assertGreater(float(np.max(np.abs(generalized_impulse))), 0.0)
        self.assertTrue(envelope.semantic_trace_equal(trace, replay))

    def test_physical_contact_prestate_is_explicit_and_replay_exact(self) -> None:
        trace = self.run_contact_observation_profile(
            record_physical_contact_prestate=True
        )
        replay = self.run_contact_observation_profile(
            record_physical_contact_prestate=True
        )
        available = np.asarray(
            trace["physical_wheel_contact_prestate_available"], np.bool_
        )
        distance = np.asarray(trace["physical_wheel_contact_distance_m"])
        velocity = np.asarray(
            trace["physical_wheel_contact_relative_velocity_m_s"]
        )
        self.assertTrue(np.any(available))
        self.assertTrue(np.all(np.isfinite(distance[available])))
        self.assertTrue(np.all(np.isinf(distance[~available])))
        self.assertTrue(np.all(np.isfinite(velocity)))
        self.assertGreater(float(np.max(np.abs(velocity[available]))), 0.0)
        self.assertTrue(envelope.semantic_trace_equal(trace, replay))

    def test_prospective_wheel_ground_state_exists_in_contact_and_flight(self) -> None:
        dormant = self.run_contact_observation_profile()
        np.testing.assert_array_equal(
            dormant["physical_wheel_prospective_contact_distance_m"],
            np.zeros((len(dormant["time_s"]), 2), np.float64),
        )
        np.testing.assert_array_equal(
            dormant["physical_wheel_prospective_contact_point_world_m"],
            np.zeros((len(dormant["time_s"]), 2, 3), np.float64),
        )
        trace = self.run_contact_observation_profile(
            record_physical_prospective_contact_state=True
        )
        replay = self.run_contact_observation_profile(
            record_physical_prospective_contact_state=True
        )
        distance = np.asarray(
            trace["physical_wheel_prospective_contact_distance_m"]
        )
        velocity = np.asarray(
            trace["physical_wheel_prospective_contact_velocity_m_s"]
        )
        point = np.asarray(
            trace["physical_wheel_prospective_contact_point_world_m"]
        )
        quaternion = np.asarray(trace["root_quaternion_wxyz"])
        self.assertTrue(np.all(np.isfinite(distance)))
        self.assertTrue(np.all(np.isfinite(velocity)))
        self.assertTrue(np.all(np.isfinite(point)))
        np.testing.assert_allclose(np.linalg.norm(quaternion, axis=1), 1.0)
        np.testing.assert_allclose(point[:, :, 2], distance)
        self.assertGreater(float(np.max(distance) - np.min(distance)), 0.0)
        self.assertGreater(float(np.max(np.abs(velocity))), 0.0)
        self.assertTrue(envelope.semantic_trace_equal(trace, replay))

    def test_contact_observation_dropout_start_tick_is_explicit(self) -> None:
        trace = self.run_contact_observation_profile(
            contact_observation_dropout_period_ticks=10,
            contact_observation_dropout_burst_ticks=1,
            contact_observation_dropout_start_tick=5,
        )
        unavailable = np.flatnonzero(trace["contact_observation_available"] == 0)

        np.testing.assert_array_equal(unavailable, np.asarray([5, 15, 25]))
        np.testing.assert_array_equal(
            trace["contact_program_authority_executable"][unavailable],
            np.zeros(len(unavailable), np.uint8),
        )
        np.testing.assert_array_equal(
            trace["torque"][unavailable],
            np.zeros((len(unavailable), 6), np.float64),
        )

    def test_inexact_observation_hold_replays_only_after_prior_authority(self) -> None:
        trace = self.run_contact_observation_profile(
            contact_program_inexact_hold_ticks=1,
            contact_observation_dropout_period_ticks=10,
            contact_observation_dropout_burst_ticks=1,
        )
        unavailable = np.flatnonzero(trace["contact_observation_available"] == 0)

        np.testing.assert_array_equal(unavailable, np.asarray([0, 10, 20]))
        self.assertEqual(trace["contact_program_authority_selection"][0], 0)
        self.assertEqual(trace["contact_program_authority_executable"][0], 0)
        np.testing.assert_array_equal(trace["torque"][0], np.zeros(6))
        np.testing.assert_array_equal(
            trace["contact_program_authority_selection"][[10, 20]],
            np.full(2, 4, np.uint8),
        )
        np.testing.assert_array_equal(
            trace["contact_program_authority_executable"][[10, 20]],
            np.ones(2, np.uint8),
        )
        np.testing.assert_array_equal(trace["torque"][10], trace["torque"][9])
        np.testing.assert_array_equal(trace["torque"][20], trace["torque"][19])

    def test_inexact_observation_hold_authority_scales_without_compounding(self) -> None:
        trace = self.run_contact_observation_profile(
            contact_program_inexact_hold_ticks=2,
            contact_program_inexact_hold_authority=0.5,
            contact_observation_dropout_period_ticks=10,
            contact_observation_dropout_burst_ticks=2,
            contact_observation_dropout_start_tick=10,
        )

        np.testing.assert_array_equal(
            trace["contact_program_authority_selection"][[10, 11]],
            np.full(2, 4, np.uint8),
        )
        np.testing.assert_array_equal(trace["torque"][10], 0.5 * trace["torque"][9])
        np.testing.assert_array_equal(trace["torque"][11], trace["torque"][10])

    def test_inexact_observation_forecast_selector_is_causal_and_rust_owned(self) -> None:
        profile = {
            "contact_program_inexact_hold_ticks": 1,
            "contact_program_inexact_hold_forecast_selector": True,
            "contact_observation_dropout_period_ticks": 10,
            "contact_observation_dropout_burst_ticks": 1,
            "contact_observation_dropout_start_tick": 10,
        }
        trace = self.run_contact_observation_profile(**profile)
        replay = self.run_contact_observation_profile(**profile)
        queried = np.flatnonzero(
            trace["inexact_observation_authority_selector_queried"] != 0
        )

        np.testing.assert_array_equal(queried, np.asarray([10, 20]))
        scores = trace[
            "inexact_observation_authority_selector_candidate_scores"
        ][queried]
        selected_index = trace[
            "inexact_observation_authority_selector_selected_index"
        ][queried]
        np.testing.assert_array_equal(selected_index, np.argmin(scores, axis=1))
        levels = np.asarray([0, 8192, 16384, 24576, 32768], np.uint16)
        selected_q15 = trace[
            "inexact_observation_authority_selector_authority_q15"
        ][queried]
        np.testing.assert_array_equal(selected_q15, levels[selected_index])
        for tick, q15 in zip(queried, selected_q15, strict=True):
            authority = float(q15) / 32768.0
            if q15 == 0:
                self.assertEqual(trace["contact_program_authority_selection"][tick], 0)
                np.testing.assert_array_equal(trace["torque"][tick], np.zeros(6))
            else:
                self.assertEqual(trace["contact_program_authority_selection"][tick], 4)
                np.testing.assert_array_equal(
                    trace["torque"][tick], authority * trace["torque"][tick - 1]
                )
        self.assertTrue(
            np.all(
                trace[
                    "inexact_observation_authority_selector_allocation_calls"
                ][queried]
                == 0
            )
        )
        self.assertTrue(envelope.semantic_trace_equal(trace, replay))

    def test_inexact_observation_forecast_margin_withholds_exactly(self) -> None:
        trace = self.run_contact_observation_profile(
            contact_program_inexact_hold_ticks=1,
            contact_program_inexact_hold_forecast_selector=True,
            contact_program_inexact_hold_forecast_minimum_improvement=1.0e9,
            contact_observation_dropout_period_ticks=10,
            contact_observation_dropout_burst_ticks=1,
            contact_observation_dropout_start_tick=10,
        )
        queried = np.flatnonzero(
            trace["inexact_observation_authority_selector_queried"] != 0
        )

        np.testing.assert_array_equal(queried, np.asarray([10, 20]))
        np.testing.assert_array_equal(
            trace["inexact_observation_authority_selector_authority_q15"][queried],
            np.zeros(len(queried), np.uint16),
        )
        np.testing.assert_array_equal(
            trace["contact_program_authority_selection"][queried],
            np.zeros(len(queried), np.uint8),
        )
        np.testing.assert_array_equal(
            trace["torque"][queried], np.zeros((len(queried), 6), np.float64)
        )

    def test_inexact_observation_support_free_brake_is_fresh_and_wbc_admitted(self) -> None:
        trace = self.run_contact_observation_profile(
            contact_program_inexact_support_free_brake=True,
            contact_observation_dropout_period_ticks=10,
            contact_observation_dropout_burst_ticks=2,
            contact_observation_dropout_start_tick=10,
        )
        unavailable = np.flatnonzero(trace["contact_observation_available"] == 0)

        np.testing.assert_array_equal(unavailable, np.asarray([10, 11, 20, 21]))
        np.testing.assert_array_equal(
            trace["contact_program_authority_selection"][unavailable],
            np.full(len(unavailable), 5, np.uint8),
        )
        np.testing.assert_array_equal(
            trace["contact_program_authority_executable"][unavailable],
            np.ones(len(unavailable), np.uint8),
        )
        np.testing.assert_array_equal(
            trace["torque"][unavailable],
            trace["support_contingency_candidate_torque"][unavailable],
        )
        self.assertTrue(
            np.all(trace["support_contingency_status"][unavailable] <= 1)
        )
        self.assertTrue(
            np.all(
                trace["support_contingency_maximum_constraint_violation"][
                    unavailable
                ]
                < 1.0e-8
            )
        )

    def test_terminal_chooser_executes_its_rust_selected_typed_action(self) -> None:
        profile = {
            "contact_program_inexact_hold_ticks": 1,
            "contact_program_inexact_terminal_chooser": True,
            "contact_program_inexact_terminal_minimum_component_improvement": 0.0,
            "contact_observation_dropout_period_ticks": 10,
            "contact_observation_dropout_burst_ticks": 1,
            "contact_observation_dropout_start_tick": 10,
        }
        trace = self.run_contact_observation_profile(**profile)
        replay = self.run_contact_observation_profile(**profile)
        queried = np.flatnonzero(
            trace["inexact_observation_terminal_selector_queried"] != 0
        )

        np.testing.assert_array_equal(queried, np.asarray([10, 20]))
        actions = trace["inexact_observation_terminal_selector_action"][queried]
        self.assertTrue(np.all(np.isin(actions, (0, 1, 2))))
        expected_selection = np.choose(actions, (0, 4, 5))
        np.testing.assert_array_equal(
            trace["contact_program_authority_selection"][queried],
            expected_selection,
        )
        for tick, action in zip(queried, actions, strict=True):
            if action == 0:
                np.testing.assert_array_equal(
                    trace["torque"][tick], np.zeros(6, np.float64)
                )
            elif action == 2:
                np.testing.assert_array_equal(
                    trace["torque"][tick],
                    trace["support_contingency_candidate_torque"][tick],
                )
        self.assertTrue(
            np.all(
                np.isfinite(
                    trace[
                        "inexact_observation_terminal_selector_candidate_diagnostics"
                    ][queried]
                )
            )
        )
        self.assertTrue(
            np.all(
                trace[
                    "inexact_observation_terminal_selector_allocation_calls"
                ][queried]
                == 0
            )
        )
        self.assertTrue(envelope.semantic_trace_equal(trace, replay))

    def test_terminal_impact_audit_is_physical_separate_and_allocation_free(self) -> None:
        import bonesaw

        session = bonesaw.UpkieBalanceSession(str(MODEL))
        diagnostics = np.zeros((3, 17), np.float64)
        selection = np.zeros(6, np.float64)
        timing = session.score_terminal_impact_candidates(
            np.asarray([0.20, -0.5, 0.20, -0.10, 2.0, -1.0]),
            np.zeros(6, np.float64),
            np.zeros(6, np.float64),
            np.asarray([-1.26, -2.51, -np.inf, -1.26, -2.51, -np.inf]),
            np.asarray([1.26, 2.51, np.inf, 1.26, 2.51, np.inf]),
            np.asarray([28.8, 28.8, 111.0, 28.8, 28.8, 111.0]),
            np.ones(3, np.uint8),
            np.asarray([[0.0, 0.0], [-10.0, 5.0], [-20.0, 10.0]]),
            np.zeros((3, 6), np.float64),
            np.asarray([0.0, 0.4, 0.4]),
            0,
            0.0,
            0.01,
            diagnostics,
            selection,
        )

        self.assertGreater(timing[0], 0)
        self.assertEqual(timing[1:], (0, 0))
        np.testing.assert_array_equal(diagnostics[:, 0], np.ones(3))
        np.testing.assert_array_equal(
            diagnostics[:, 1], np.full(3, diagnostics[0, 1])
        )
        np.testing.assert_array_equal(
            diagnostics[:, 3], np.full(3, diagnostics[0, 3])
        )
        self.assertEqual(selection[0], 2.0)
        self.assertLess(diagnostics[2, 5], diagnostics[0, 5])

    def test_terminal_withhold_uses_admitted_zero_effort_dynamics_witness(self) -> None:
        profile = {
            "contact_program_inexact_hold_ticks": 1,
            "contact_program_inexact_terminal_chooser": True,
            "contact_program_inexact_terminal_zero_effort_baseline": True,
            "contact_observation_dropout_period_ticks": 10,
            "contact_observation_dropout_burst_ticks": 1,
            "contact_observation_dropout_start_tick": 10,
        }
        trace = self.run_contact_observation_profile(**profile)
        replay = self.run_contact_observation_profile(**profile)
        queried = np.flatnonzero(
            trace["inexact_observation_terminal_selector_queried"] != 0
        )

        np.testing.assert_array_equal(queried, np.asarray([10, 20]))
        np.testing.assert_array_equal(
            trace["inexact_observation_terminal_zero_effort_available"][queried],
            np.ones(len(queried), np.uint8),
        )
        baseline = trace[
            "inexact_observation_terminal_zero_effort_acceleration"
        ][queried]
        np.testing.assert_array_equal(
            trace["inexact_observation_terminal_selector_root_acceleration"][
                queried, 0
            ],
            baseline[:, :2],
        )
        np.testing.assert_array_equal(
            trace["inexact_observation_terminal_selector_joint_acceleration"][
                queried, 0
            ],
            baseline[:, 6:],
        )
        self.assertGreater(float(np.max(np.abs(baseline))), 1.0)
        self.assertTrue(
            np.all(
                trace[
                    "inexact_observation_terminal_zero_effort_allocation_calls"
                ][queried]
                == 0
            )
        )
        self.assertTrue(
            np.all(
                trace[
                    "inexact_observation_terminal_zero_effort_allocated_bytes"
                ][queried]
                == 0
            )
        )
        actions = trace["inexact_observation_terminal_selector_action"][queried]
        np.testing.assert_array_equal(
            trace["contact_program_authority_selection"][queried],
            np.choose(actions, (0, 4, 5)),
        )
        for tick, action in zip(queried, actions, strict=True):
            if action == 0:
                np.testing.assert_array_equal(
                    trace["torque"][tick], np.zeros(6, np.float64)
                )
        self.assertTrue(envelope.semantic_trace_equal(trace, replay))

    def test_terminal_support_hypotheses_aggregate_in_rust_without_allocation(self) -> None:
        import bonesaw

        session = bonesaw.UpkieBalanceSession(str(MODEL))
        hypotheses = np.empty((3, 4, 17), np.float64)
        state = np.asarray([0.20, -0.5, 0.20, -0.10, 2.0, -1.0])
        q = np.zeros(6, np.float64)
        v = np.zeros(6, np.float64)
        lower = np.asarray([-1.26, -2.51, -np.inf, -1.26, -2.51, -np.inf])
        upper = np.asarray([1.26, 2.51, np.inf, 1.26, 2.51, np.inf])
        velocity = np.asarray([28.8, 28.8, 111.0, 28.8, 28.8, 111.0])
        selection = np.empty(6, np.float64)
        for support in range(4):
            diagnostics = np.empty((3, 17), np.float64)
            scale = float(support + 1)
            session.score_terminal_impact_candidates(
                state,
                q,
                v,
                lower,
                upper,
                velocity,
                np.ones(3, np.uint8),
                np.asarray(
                    [
                        [0.0, 0.0],
                        [-2.0 * scale, scale],
                        [-4.0 * scale, 2.0 * scale],
                    ]
                ),
                np.zeros((3, 6), np.float64),
                np.asarray([0.0, 0.4, 0.4]),
                0,
                0.0,
                0.01,
                diagnostics,
                selection,
            )
            hypotheses[:, support] = diagnostics
        envelopes = np.empty((3, 17), np.float64)
        timing = session.select_terminal_impact_hypothesis_envelopes(
            hypotheses,
            0,
            0.0,
            0.01,
            envelopes,
            selection,
        )

        self.assertGreater(timing[0], 0)
        self.assertEqual(timing[1:], (0, 0))
        np.testing.assert_array_equal(envelopes[:, 0], np.ones(3))
        np.testing.assert_array_equal(
            envelopes[:, 9:16], np.max(hypotheses[:, :, 9:16], axis=1)
        )
        np.testing.assert_array_equal(
            envelopes[:, 6], np.min(hypotheses[:, :, 6], axis=1)
        )

        hypotheses[2, 3, 0] = 0.0
        hypotheses[2, 3, 14] = 1.0
        hypotheses[2, 3, 15] = max(hypotheses[2, 3, 15], 1.0)
        hypotheses[2, 3, 16] += 1.0
        session.select_terminal_impact_hypothesis_envelopes(
            hypotheses,
            0,
            0.0,
            0.01,
            envelopes,
            selection,
        )
        self.assertEqual(envelopes[2, 0], 0.0)
        self.assertNotEqual(selection[0], 2.0)

    def test_terminal_support_hypothesis_envelope_runs_end_to_end(self) -> None:
        profile = {
            "contact_program_inexact_hold_ticks": 1,
            "contact_program_inexact_terminal_chooser": True,
            "contact_program_inexact_terminal_support_hypothesis_envelope": True,
            "contact_observation_dropout_period_ticks": 10,
            "contact_observation_dropout_burst_ticks": 1,
            "contact_observation_dropout_start_tick": 10,
        }
        trace = self.run_contact_observation_profile(**profile)
        replay = self.run_contact_observation_profile(**profile)
        queried = np.flatnonzero(
            trace["inexact_observation_terminal_selector_queried"] != 0
        )

        np.testing.assert_array_equal(queried, np.asarray([10, 20]))
        self.assertTrue(
            np.all(
                trace["inexact_observation_terminal_hypothesis_available"][queried]
                == 1
            )
        )
        hypotheses = trace[
            "inexact_observation_terminal_hypothesis_diagnostics"
        ][queried]
        envelopes = trace[
            "inexact_observation_terminal_hypothesis_envelopes"
        ][queried]
        np.testing.assert_array_equal(
            envelopes[:, :, 9:16], np.max(hypotheses[:, :, :, 9:16], axis=2)
        )
        np.testing.assert_array_equal(
            envelopes[:, :, 6], np.min(hypotheses[:, :, :, 6], axis=2)
        )
        actions = trace["inexact_observation_terminal_selector_action"][queried]
        np.testing.assert_array_equal(
            trace["contact_program_authority_selection"][queried],
            np.choose(actions, (0, 4, 5)),
        )
        for tick, action in zip(queried, actions, strict=True):
            if action == 0:
                np.testing.assert_array_equal(
                    trace["torque"][tick], np.zeros(6, np.float64)
                )
            elif action == 2:
                np.testing.assert_array_equal(
                    trace["torque"][tick],
                    trace["inexact_observation_terminal_hypothesis_torque"][tick, 2],
                )
        for field in (
            "inexact_observation_terminal_hypothesis_query_allocation_calls",
            "inexact_observation_terminal_hypothesis_query_allocated_bytes",
            "inexact_observation_terminal_hypothesis_aggregate_allocation_calls",
            "inexact_observation_terminal_hypothesis_aggregate_allocated_bytes",
        ):
            self.assertTrue(np.all(trace[field][queried] == 0), field)
        self.assertTrue(envelope.semantic_trace_equal(trace, replay))

    def test_left_and_right_contact_bit_chatter_are_deterministic(self) -> None:
        expected_ticks = np.asarray([0, 10, 20])
        for contact_index in (0, 1):
            with self.subTest(contact_index=contact_index):
                trace = self.run_contact_observation_profile(
                    contact_observation_flip_period_ticks=10,
                    contact_observation_flip_burst_ticks=1,
                    contact_observation_flip_contact=contact_index,
                )
                mismatch = np.flatnonzero(
                    np.any(
                        trace["physical_contact_active"]
                        != trace["observed_contact_active"],
                        axis=1,
                    )
                )
                np.testing.assert_array_equal(mismatch, expected_ticks)
                np.testing.assert_array_equal(
                    trace["physical_contact_active"][mismatch, contact_index]
                    ^ trace["observed_contact_active"][mismatch, contact_index],
                    np.ones(len(mismatch), np.uint8),
                )

    def test_frozen_matrix_covers_3d_repetition_and_friction(self) -> None:
        cases = envelope.case_matrix()
        envelope.validate_cases(cases)
        families = {case.family for case in cases}
        self.assertTrue({"axis_x", "axis_y", "axis_z", "repeated", "friction"} <= families)
        repeated = next(case for case in cases if case.family == "repeated")
        self.assertEqual(repeated.repetitions, 3)
        self.assertAlmostEqual(repeated.impulse_ns, 0.6)
        self.assertAlmostEqual(repeated.final_push_end_s, 3.1)

    def test_first_physical_fall_terminates_without_numeric_tail(self) -> None:
        case = next(
            case for case in envelope.case_matrix() if case.name == "left_4n"
        )
        trace = envelope.run_case(MODEL, case, 2.5)
        result = envelope.summarize(case, trace, 2.5)
        self.assertEqual(result["outcome"], "FALL")
        self.assertEqual(result["termination_reason"], "fall")
        self.assertLess(result["executed_ticks"], result["configured_ticks"])
        self.assertLess(result["terminal_time_s"], 2.5)
        self.assertEqual(result["mujoco_warning_count"], 0)
        self.assertTrue(result["finite"])

    def test_shared_plant_harness_accepts_world_vector_and_friction(self) -> None:
        trace = plant.run_case(
            MODEL,
            duration=1.2,
            push_start=1.0,
            push_duration=0.1,
            push_force=0.0,
            push_force_world=(0.0, 0.0, 1.0),
            sliding_friction=0.5,
            contact_model="soft",
            balance_mode="capture",
            capture_velocity_fraction=0.2,
        )
        force = np.asarray(trace["external_force_world"])
        np.testing.assert_array_equal(force[0], np.zeros(3))
        self.assertGreater(np.max(force[:, 2]), 0.99)
        self.assertEqual(np.max(np.abs(force[:, :2])), 0.0)

    def test_contact_command_lease_is_the_end_to_end_execution_authority(self) -> None:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        model, data, controller, wheel_bodies, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="planar_capture",
            execute_reduced_support=False,
            contact_command_lease_ticks=2,
        )
        state = plant.read_state(model, data)
        ground_position = float(np.mean(data.xpos[wheel_bodies, 0]))
        ground_height = float(np.mean(data.xpos[wheel_bodies, 2]))

        results = []
        for mask in ([1, 1], [1, 1], [1, 1], [1, 0], [1, 0], [1, 0]):
            result = controller.solve(
                *state,
                ground_position,
                ground_height,
                np.asarray(mask, np.uint8),
            )
            results.append(
                {
                    **result,
                    "torque": result["torque"].copy(),
                    "contact_force_basis": result["contact_force_basis"].copy(),
                }
            )

        fresh = results[2]
        self.assertTrue(fresh["contact_command_lease_executable"])
        self.assertGreater(np.linalg.norm(fresh["torque"]), 0.0)
        for leased in results[3:5]:
            self.assertEqual(leased["status"], 4)
            self.assertEqual(leased["contact_command_lease_status"], 2)
            self.assertTrue(leased["contact_command_lease_executable"])
            np.testing.assert_array_equal(leased["torque"], fresh["torque"])
            np.testing.assert_array_equal(
                leased["contact_force_basis"], np.zeros((2, 3))
            )
        expired = results[5]
        self.assertEqual(expired["contact_command_lease_status"], 3)
        self.assertFalse(expired["contact_command_lease_executable"])
        np.testing.assert_array_equal(expired["torque"], np.zeros(6))
        np.testing.assert_array_equal(
            expired["contact_force_basis"], np.zeros((2, 3))
        )

    def test_viability_planner_cannot_run_without_exact_contact_evidence(self) -> None:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        model, data, controller, wheel_bodies, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="capture",
            viability_planner_enabled=True,
        )
        state = plant.read_state(model, data)
        ground_position = float(np.mean(data.xpos[wheel_bodies, 0]))
        ground_height = float(np.mean(data.xpos[wheel_bodies, 2]))

        for _ in range(5):
            result = controller.solve(
                *state,
                ground_position,
                ground_height,
                None,
            )
            self.assertEqual(result["viability_request_status"], 5)
            self.assertFalse(result["viability_request_executable"])
            self.assertEqual(result["viability_planner_query_count"], 0)
            np.testing.assert_array_equal(result["viability_request"], np.zeros(3))

    def test_viability_planner_search_supervision_and_final_wbc_are_end_to_end(self) -> None:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        model, data, controller, wheel_bodies, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="capture",
            viability_planner_enabled=True,
        )
        state = list(plant.read_state(model, data))
        state[0] = state[0].copy()
        state[0][1] += 0.05
        ground_position = float(np.mean(data.xpos[wheel_bodies, 0]))
        ground_height = float(np.mean(data.xpos[wheel_bodies, 2]))
        exact_contact = np.asarray([1, 1], np.uint8)

        results = [
            controller.solve(
                *state,
                ground_position,
                ground_height,
                exact_contact,
            )
            for _ in range(5)
        ]
        fresh = results[0]
        held = results[1]
        self.assertEqual(fresh["viability_request_status"], 1)
        self.assertEqual(fresh["viability_planner_query_count"], 40)
        self.assertTrue(fresh["viability_request_executable"])
        self.assertGreater(np.linalg.norm(fresh["viability_request"]), 0.0)
        self.assertIn(fresh["status"], (0, 1))
        self.assertEqual(fresh["allocation_calls"], 0)
        self.assertEqual(held["viability_request_status"], 2)
        self.assertEqual(held["viability_planner_query_count"], 0)
        self.assertTrue(held["viability_request_executable"])
        self.assertTrue(
            np.all(
                np.abs(held["viability_request"] - fresh["viability_request"])
                <= np.asarray([40.0, 40.0, 20.0])
            )
        )

    def test_budgeted_multistep_planner_caps_queries_and_keeps_evidence_finite(self) -> None:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        model, data, controller, wheel_bodies, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="capture",
            viability_planner_enabled=True,
            viability_support_requires_active_request=True,
            viability_planner_strategy="multistep_budgeted",
        )
        state = list(plant.read_state(model, data))
        state[0] = state[0].copy()
        state[0][1] += 0.05
        ground_position = float(np.mean(data.xpos[wheel_bodies, 0]))
        ground_height = float(np.mean(data.xpos[wheel_bodies, 2]))
        results = [
            controller.solve(
                *state,
                ground_position,
                ground_height,
                np.asarray([1, 1], np.uint8),
            )
            for _ in range(3)
        ]

        self.assertTrue(
            any(result["viability_planner_query_count"] > 1 for result in results)
        )
        for result in results:
            self.assertGreaterEqual(result["viability_planner_query_count"], 1)
            self.assertLessEqual(result["viability_planner_query_count"], 4)
            self.assertEqual(result["allocation_calls"], 0)
            self.assertTrue(np.isfinite(result["viability_planner_zero_pressure"]))
            self.assertTrue(
                np.isfinite(result["viability_planner_candidate_pressure"])
            )

    def test_viability_confirmation_is_end_to_end_and_revokes_on_raw_support(self) -> None:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        model, data, controller, wheel_bodies, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="capture",
            viability_planner_enabled=True,
            viability_support_requires_active_request=True,
            viability_planner_strategy="multistep_budgeted",
            viability_confirmation_updates=2,
        )

        class FixedImprovingPlanner:
            ACTIVATION_PRESSURE = 0.10
            ROLL_BOUND_RAD = np.deg2rad(45.0)
            LATERAL_CAPTURE_BOUND_M = 0.10

            def __init__(self, forecast_size: int) -> None:
                self.request = np.asarray([10.0, 0.0, 0.0], np.float64)
                self.step_ns = 0
                self.allocation_calls = 0
                self.allocated_bytes = 0
                self.query_count = 2
                self.zero_pressure = 1.0
                self.candidate_pressure = 0.8
                self.candidate_forecast = np.zeros(forecast_size, np.float64)
                self.candidate_forecast_path = np.zeros((8, 9), np.float64)
                self.candidate_forecast_path_valid = False
                self.candidate_normal_force = np.asarray([20.0, 20.0], np.float64)

            def plan(self, *args: object) -> tuple[np.ndarray, float, bool]:
                return self.request, 0.2, True

        controller.viability_planner = FixedImprovingPlanner(
            len(controller.viability_forecast_diagnostics)
        )
        state = plant.read_state(model, data)
        ground = (
            float(np.mean(data.xpos[wheel_bodies, 0])),
            float(np.mean(data.xpos[wheel_bodies, 2])),
        )
        both = np.asarray([1, 1], np.uint8)
        first = controller.solve(*state, *ground, both)
        second = controller.solve(*state, *ground, both)
        second_request = second["viability_request"].copy()
        support_changed = controller.solve(
            *state, *ground, np.asarray([1, 0], np.uint8)
        )
        evidence_lost = controller.solve(*state, *ground, None)

        self.assertEqual(first["viability_confirmation_status"], 1)
        self.assertFalse(first["viability_request_executable"])
        self.assertEqual(second["viability_confirmation_status"], 2)
        self.assertTrue(second["viability_request_executable"])
        np.testing.assert_array_equal(
            second_request, np.asarray([10.0, 0.0, 0.0])
        )
        self.assertEqual(support_changed["viability_confirmation_status"], 4)
        self.assertFalse(support_changed["viability_request_executable"])
        self.assertEqual(evidence_lost["viability_confirmation_status"], 3)
        self.assertFalse(evidence_lost["viability_request_executable"])
        for result in (first, second, support_changed, evidence_lost):
            self.assertEqual(result["allocation_calls"], 0)

    def test_three_tick_planner_cadence_is_explicit_and_confirmation_safe(self) -> None:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        model, data, controller, wheel_bodies, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="capture",
            viability_planner_enabled=True,
            viability_planner_strategy="multistep_budgeted",
            viability_planner_update_period_ticks=3,
            viability_confirmation_updates=2,
        )

        class FixedImprovingPlanner:
            ACTIVATION_PRESSURE = 0.10
            ROLL_BOUND_RAD = np.deg2rad(45.0)
            LATERAL_CAPTURE_BOUND_M = 0.10

            def __init__(self, forecast_size: int) -> None:
                self.request = np.asarray([10.0, 0.0, 0.0], np.float64)
                self.step_ns = 0
                self.allocation_calls = 0
                self.allocated_bytes = 0
                self.query_count = 2
                self.zero_pressure = 1.0
                self.candidate_pressure = 0.8
                self.candidate_forecast = np.zeros(forecast_size, np.float64)
                self.candidate_forecast_path = np.zeros((8, 9), np.float64)
                self.candidate_forecast_path_valid = False
                self.candidate_normal_force = np.asarray([20.0, 20.0], np.float64)

            def plan(self, *args: object) -> tuple[np.ndarray, float, bool]:
                return self.request, 0.2, True

        controller.viability_planner = FixedImprovingPlanner(
            len(controller.viability_forecast_diagnostics)
        )
        state = plant.read_state(model, data)
        ground = (
            float(np.mean(data.xpos[wheel_bodies, 0])),
            float(np.mean(data.xpos[wheel_bodies, 2])),
        )
        exact = np.asarray([1, 1], np.uint8)
        results = [controller.solve(*state, *ground, exact) for _ in range(4)]

        self.assertEqual(
            [result["viability_planner_update"] for result in results],
            [True, False, False, True],
        )
        self.assertEqual(
            [result["viability_planner_query_count"] for result in results],
            [2, 0, 0, 2],
        )
        self.assertEqual(results[0]["viability_confirmation_status"], 1)
        self.assertTrue(results[1]["viability_confirmation_has_shadow"])
        self.assertTrue(results[2]["viability_confirmation_has_shadow"])
        self.assertEqual(results[3]["viability_confirmation_status"], 2)
        self.assertTrue(results[3]["viability_request_executable"])
        self.assertTrue(all(result["allocation_calls"] == 0 for result in results))

    def test_hybrid_guard_precedes_confirmation_and_rejects_opening_request(self) -> None:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        model, data, controller, wheel_bodies, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="capture",
            viability_planner_enabled=True,
            viability_planner_strategy="hybrid_confirmed_multistep_budgeted",
            viability_confirmation_updates=2,
        )

        class FixedPlanner:
            ACTIVATION_PRESSURE = 0.10
            ROLL_BOUND_RAD = np.deg2rad(45.0)
            LATERAL_CAPTURE_BOUND_M = 0.10

            def __init__(self, forecast_size: int) -> None:
                self.request = np.asarray([0.0, 0.0, -10.0], np.float64)
                self.step_ns = 0
                self.allocation_calls = 0
                self.allocated_bytes = 0
                self.query_count = 2
                self.zero_pressure = 1.0
                self.candidate_pressure = 0.8
                self.candidate_forecast = np.zeros(forecast_size, np.float64)
                self.candidate_forecast_path = np.zeros((8, 9), np.float64)
                self.candidate_forecast_path_valid = False
                self.candidate_normal_force = np.asarray([20.0, 0.0], np.float64)
                self.forecast_state = np.zeros(10, np.float64)
                self.forecast_state[8] = 0.54
                self.ROLL_BOUND_RAD = math.radians(45.0)

            def plan(self, *args: object) -> tuple[np.ndarray, float, bool]:
                return self.request, 0.2, True

        planner = FixedPlanner(len(controller.viability_forecast_diagnostics))
        controller.viability_planner = planner
        state = plant.read_state(model, data)
        ground = (
            float(np.mean(data.xpos[wheel_bodies, 0])),
            float(np.mean(data.xpos[wheel_bodies, 2])),
        )
        left_only = np.asarray([1, 0], np.uint8)
        results = [controller.solve(*state, *ground, left_only) for _ in range(4)]

        self.assertTrue(
            all(result["viability_hybrid_guard_status"] == 3 for result in results[:2])
        )
        self.assertEqual(results[2]["viability_hybrid_guard_status"], 1)
        self.assertFalse(results[2]["viability_request_executable"])
        self.assertTrue(results[3]["viability_hybrid_guard_executable"])
        self.assertTrue(results[3]["viability_request_executable"])

        planner.request[:] = (-10.0, 0.0, 0.0)
        rejected = controller.solve(*state, *ground, left_only)
        self.assertEqual(rejected["viability_hybrid_guard_status"], 4)
        self.assertTrue(rejected["viability_hybrid_guard_shadow_admissible"])
        self.assertFalse(rejected["viability_request_executable"])
        repeated = controller.solve(*state, *ground, left_only)
        self.assertEqual(repeated["viability_hybrid_guard_status"], 4)
        self.assertTrue(repeated["viability_confirmation_executable"])
        self.assertFalse(repeated["viability_request_executable"])
        self.assertEqual(rejected["allocation_calls"], 0)

    def test_lateral_budgeted_activation_does_not_wake_on_sagittal_pressure(self) -> None:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        full_model, full_data, full, full_wheels, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="capture",
            viability_planner_enabled=True,
            viability_support_requires_active_request=True,
            viability_planner_strategy="multistep_budgeted",
        )
        lateral_model, lateral_data, lateral, lateral_wheels, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="capture",
            viability_planner_enabled=True,
            viability_support_requires_active_request=True,
            viability_planner_strategy="lateral_multistep_budgeted",
        )
        full_state = list(plant.read_state(full_model, full_data))
        lateral_state = list(plant.read_state(lateral_model, lateral_data))
        full_state[2] = full_state[2].copy()
        lateral_state[2] = lateral_state[2].copy()
        full_state[2][1] = 1.0
        lateral_state[2][1] = 1.0
        full_ground = (
            float(np.mean(full_data.xpos[full_wheels, 0])),
            float(np.mean(full_data.xpos[full_wheels, 2])),
        )
        lateral_ground = (
            float(np.mean(lateral_data.xpos[lateral_wheels, 0])),
            float(np.mean(lateral_data.xpos[lateral_wheels, 2])),
        )
        exact = np.asarray([1, 1], np.uint8)

        full_result = full.solve(*full_state, *full_ground, exact)
        lateral_result = lateral.solve(*lateral_state, *lateral_ground, exact)

        self.assertGreater(full_result["viability_planner_query_count"], 1)
        self.assertEqual(lateral_result["viability_planner_query_count"], 1)
        self.assertFalse(lateral_result["viability_request_executable"])

    def test_paired_lateral_planner_uses_two_same_state_queries_at_most(self) -> None:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        model, data, controller, wheel_bodies, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="capture",
            viability_planner_enabled=True,
            viability_support_requires_active_request=True,
            viability_planner_strategy="lateral_paired_multistep_budgeted",
        )
        state = list(plant.read_state(model, data))
        state[0] = state[0].copy()
        state[0][1] += 0.05
        ground = (
            float(np.mean(data.xpos[wheel_bodies, 0])),
            float(np.mean(data.xpos[wheel_bodies, 2])),
        )
        exact = np.asarray([1, 1], np.uint8)

        results = [controller.solve(*state, *ground, exact) for _ in range(6)]

        self.assertTrue(
            any(result["viability_planner_query_count"] == 2 for result in results)
        )
        for result in results:
            self.assertGreaterEqual(result["viability_planner_query_count"], 1)
            self.assertLessEqual(result["viability_planner_query_count"], 2)
            self.assertEqual(result["allocation_calls"], 0)
            self.assertTrue(np.isfinite(result["viability_planner_zero_pressure"]))
            self.assertTrue(
                np.isfinite(result["viability_planner_candidate_pressure"])
            )

    def test_inactive_viability_planner_cannot_replace_measured_support(self) -> None:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        control_model, control_data, control, control_wheels, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="capture",
        )
        candidate_model, candidate_data, candidate, candidate_wheels, _ = (
            envelope.prepare_case(
                MODEL,
                case,
                balance_mode="capture",
                viability_planner_enabled=True,
            )
        )
        control_state = plant.read_state(control_model, control_data)
        candidate_state = plant.read_state(candidate_model, candidate_data)
        np.testing.assert_array_equal(control_state[0], candidate_state[0])
        control_ground = (
            float(np.mean(control_data.xpos[control_wheels, 0])),
            float(np.mean(control_data.xpos[control_wheels, 2])),
        )
        candidate_ground = (
            float(np.mean(candidate_data.xpos[candidate_wheels, 0])),
            float(np.mean(candidate_data.xpos[candidate_wheels, 2])),
        )

        masks = ([1, 1], [1, 1], [1, 1], [1, 0], [1, 0], [1, 0], [1, 0])
        compared_inactive_steps = 0
        observed_reduced_support_while_inactive = False
        for mask in masks:
            observed = np.asarray(mask, np.uint8)
            control_result = control.solve(
                *control_state,
                *control_ground,
                observed,
            )
            candidate_result = candidate.solve(
                *candidate_state,
                *candidate_ground,
                observed,
            )
            if candidate_result["viability_request_executable"]:
                continue
            compared_inactive_steps += 1
            self.assertEqual(
                (
                    control_result["support_active_left"],
                    control_result["support_active_right"],
                ),
                (
                    candidate_result["support_active_left"],
                    candidate_result["support_active_right"],
                ),
            )
            observed_reduced_support_while_inactive |= (
                candidate_result["support_active_left"]
                + candidate_result["support_active_right"]
                < 2
            )
            self.assertEqual(control_result["status"], candidate_result["status"])
            np.testing.assert_array_equal(
                control_result["torque"], candidate_result["torque"]
            )
        self.assertGreaterEqual(compared_inactive_steps, 4)
        self.assertTrue(observed_reduced_support_while_inactive)

    def test_request_gated_support_preserves_full_support_while_inactive(self) -> None:
        case = next(case for case in envelope.case_matrix() if case.name == "nominal")
        control_model, control_data, control, control_wheels, _ = envelope.prepare_case(
            MODEL,
            case,
            balance_mode="capture",
        )
        candidate_model, candidate_data, candidate, candidate_wheels, _ = (
            envelope.prepare_case(
                MODEL,
                case,
                balance_mode="capture",
                viability_planner_enabled=True,
                viability_support_requires_active_request=True,
            )
        )
        control_state = plant.read_state(control_model, control_data)
        candidate_state = plant.read_state(candidate_model, candidate_data)
        control_ground = (
            float(np.mean(control_data.xpos[control_wheels, 0])),
            float(np.mean(control_data.xpos[control_wheels, 2])),
        )
        candidate_ground = (
            float(np.mean(candidate_data.xpos[candidate_wheels, 0])),
            float(np.mean(candidate_data.xpos[candidate_wheels, 2])),
        )

        masks = ([1, 1], [1, 1], [1, 1], [1, 0], [1, 0], [1, 0], [1, 0])
        for mask in masks:
            control_result = control.solve(*control_state, *control_ground, None)
            candidate_result = candidate.solve(
                *candidate_state,
                *candidate_ground,
                np.asarray(mask, np.uint8),
            )
            self.assertFalse(candidate_result["viability_request_executable"])
            self.assertEqual(candidate_result["support_active_left"], 1)
            self.assertEqual(candidate_result["support_active_right"], 1)
            self.assertEqual(control_result["status"], candidate_result["status"])
            np.testing.assert_array_equal(
                control_result["torque"], candidate_result["torque"]
            )


if __name__ == "__main__":
    unittest.main()
