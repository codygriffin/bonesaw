from __future__ import annotations

import pathlib
import unittest
from types import SimpleNamespace

import numpy as np

import upkie_inexact_support_hypothesis_envelope_ab as envelope


class SupportHypothesisEnvelopeAuditTests(unittest.TestCase):
    def test_identical_realization_is_bracketed_exactly(self) -> None:
        import bonesaw

        model = pathlib.Path("models/upkie/upkie.urdf").resolve()
        balance = bonesaw.UpkieBalanceSession(str(model))
        limits = envelope.model_limits(model)
        lower, upper, velocity_limit, _effort_limit = limits
        state = np.asarray([0.20, -0.5, 0.20, -0.10, 2.0, -1.0])
        q = np.zeros(6, np.float64)
        v = np.zeros(6, np.float64)
        available = np.ones(3, np.uint8)
        root_acceleration = np.zeros((3, 2), np.float64)
        joint_acceleration = np.zeros((3, 6), np.float64)
        effort = np.zeros(3, np.float64)
        diagnostics = np.empty((3, 17), np.float64)
        selection = np.empty(6, np.float64)
        hypotheses = np.empty((3, 4, 17), np.float64)
        for support_mask in range(4):
            balance.score_terminal_impact_candidates(
                state,
                q,
                v,
                lower,
                upper,
                velocity_limit,
                available,
                root_acceleration,
                joint_acceleration,
                effort,
                0,
                0.0,
                0.01,
                diagnostics,
                selection,
            )
            hypotheses[:, support_mask] = diagnostics
        envelopes = np.empty((3, 17), np.float64)
        balance.select_terminal_impact_hypothesis_envelopes(
            hypotheses,
            0,
            0.0,
            0.01,
            envelopes,
            selection,
        )
        trace = {
            "inexact_observation_terminal_selector_queried": np.ones(1, np.uint8),
            "inexact_observation_terminal_selector_action": np.asarray(
                [int(selection[0])], np.uint8
            ),
            "physical_contact_active": np.ones((1, 2), np.uint8),
            "root_twist": np.zeros((1, 6), np.float64),
            "post_root_twist": np.zeros((1, 6), np.float64),
            "v": v[None],
            "post_v": v[None],
            "q": q[None],
            "inexact_observation_terminal_hypothesis_acceleration": np.zeros(
                (1, 4, 3, 12), np.float64
            ),
            "inexact_observation_terminal_hypothesis_available": np.ones(
                (1, 4, 3), np.uint8
            ),
            "inexact_observation_terminal_hypothesis_diagnostics": hypotheses[
                None
            ],
            "inexact_observation_terminal_hypothesis_envelopes": envelopes[None],
            "inexact_observation_terminal_hypothesis_effort_utilization": effort[
                None
            ],
            "inexact_observation_terminal_selector_state": state[None],
        }

        audit = envelope.realization_audit(trace, balance, limits)

        self.assertEqual(audit["sample_count"], 1)
        self.assertEqual(audit["componentwise_coverage"], 1.0)
        self.assertEqual(audit["maximum_envelope_exceedance"], 0.0)
        self.assertTrue(audit["zero_rust_allocation"])

    def test_attempt_contract_accepts_typed_fail_closed_invalid_baseline(self) -> None:
        ticks = 3
        trace = {
            "contact_observation_available": np.asarray([1, 0, 0], np.uint8),
            "inexact_observation_terminal_hypothesis_query_step_ns": np.asarray(
                [0, 100, 100], np.uint64
            ),
            "inexact_observation_terminal_selector_queried": np.asarray(
                [0, 1, 0], np.uint8
            ),
            "inexact_observation_terminal_hypothesis_available": np.ones(
                (ticks, 4, 3), np.uint8
            ),
            "contact_program_authority_selection": np.asarray(
                [1, 4, 0], np.uint8
            ),
            "contact_program_authority_executable": np.asarray(
                [1, 1, 0], np.uint8
            ),
            "torque": np.asarray(
                [[1.0] * 6, [2.0] * 6, [0.0] * 6], np.float64
            ),
        }
        trace["inexact_observation_terminal_hypothesis_available"][2, 3, 0] = 0

        contract = envelope.support_hypothesis_attempt_contract(trace)

        self.assertTrue(contract["attempted_every_unavailable_tick"])
        self.assertTrue(
            contract["queried_exactly_when_baseline_envelope_is_valid"]
        )
        self.assertTrue(contract["invalid_baseline_fails_closed"])
        self.assertEqual(contract["invalid_baseline_attempt_ticks"], 1)

    def test_attempt_prefix_includes_an_invalid_first_envelope(self) -> None:
        fields = {
            "time_s": np.zeros(2),
            "root_position": np.zeros((2, 3)),
            "root_twist": np.zeros((2, 6)),
            "rotation_vector": np.zeros((2, 3)),
            "q": np.zeros((2, 6)),
            "v": np.zeros((2, 6)),
            "torque": np.zeros((2, 6)),
            "inexact_observation_terminal_hypothesis_acceleration": np.zeros(
                (2, 4, 3, 12)
            ),
            "inexact_observation_terminal_hypothesis_available": np.zeros(
                (2, 4, 3), np.uint8
            ),
            "inexact_observation_terminal_hypothesis_diagnostics": np.zeros(
                (2, 3, 4, 17)
            ),
            "inexact_observation_terminal_hypothesis_envelopes": np.zeros(
                (2, 3, 17)
            ),
            "inexact_observation_terminal_selector_action": np.zeros(
                2, np.uint8
            ),
            "inexact_observation_terminal_hypothesis_query_step_ns": np.asarray(
                [100, 0], np.uint64
            ),
        }
        left = {name: value.copy() for name, value in fields.items()}
        right = {name: value.copy() for name, value in fields.items()}
        self.assertTrue(envelope.first_hypothesis_attempt_prefix_equal(left, right))
        right["torque"][0, 0] = 1.0
        self.assertFalse(envelope.first_hypothesis_attempt_prefix_equal(left, right))

    def test_no_query_trace_is_defined_and_fails_the_baseline_gate(self) -> None:
        ticks = 2
        trace = {
            "inexact_observation_terminal_selector_queried": np.zeros(ticks, np.uint8),
            "inexact_observation_terminal_hypothesis_diagnostics": np.zeros(
                (ticks, 3, 4, 17), np.float64
            ),
            "inexact_observation_terminal_hypothesis_envelopes": np.zeros(
                (ticks, 3, 17), np.float64
            ),
            "inexact_observation_terminal_selector_selection_diagnostics": np.zeros(
                (ticks, 6), np.float64
            ),
            "inexact_observation_terminal_hypothesis_available": np.zeros(
                (ticks, 4, 3), np.uint8
            ),
        }
        for name in (
            "query_step_ns",
            "query_allocation_calls",
            "query_allocated_bytes",
            "aggregate_allocation_calls",
            "aggregate_allocated_bytes",
        ):
            trace[f"inexact_observation_terminal_hypothesis_{name}"] = np.zeros(
                ticks, np.uint64
            )

        unused_limits = tuple(np.empty(0) for _ in range(4))
        balance = SimpleNamespace(terminal_impact_diagnostic_names=())
        audit = envelope.hypothesis_audit(trace, balance, unused_limits)

        self.assertEqual(audit["query_count"], 0)
        self.assertFalse(audit["baseline_available_for_every_hypothesis"])
        self.assertEqual(audit["fixed_effort_query_ns"]["maximum"], 0.0)
        self.assertTrue(audit["zero_allocation"])


if __name__ == "__main__":
    unittest.main()
