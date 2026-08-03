# Upkie measured body-moment rejection — R314

Status: **BOUNDED BEHAVIOR QUALIFIED; FULL RECOVERY OPEN; DEFAULT-OFF**.

A bounded Rust correction using only measured root roll and outward roll rate reduces the frozen R313 terminal count from four to two. Both mirrored ±6 N cases now finish 450 ticks with stable two-second bilateral tails; ±8 N remains outside recoverable authority and never terminates earlier. The layer stays default-off until the entire declared holdout finishes.

The action itself is a pure Rust state law with no policy or physics dependency. MuJoCo 3.3.7 is used only to measure plant consequence on the frozen holdout.

| force Y N | R313 terminal | R314 terminal | relocks | max torque | max action rad/s² |
|---:|---:|---:|---:|---:|---:|
| -8 | 79 | 79 | 0 | 0.219 | 10.500 |
| -6 | 236 | — | 5 | 0.165 | 5.643 |
| -4 | — | — | 0 | 0.152 | 0.911 |
| -2 | — | — | 0 | 0.152 | 0.558 |
| +2 | — | — | 0 | 0.152 | 0.578 |
| +4 | — | — | 0 | 0.152 | 0.909 |
| +6 | 236 | — | 5 | 0.167 | 5.659 |
| +8 | 69 | 72 | 0 | 0.256 | 10.500 |

## Mechanism and behavior gates

- PASS `frozen_mujoco_version_matches`
- PASS `candidate_is_default_off`
- PASS `declared_profile_is_present_once_in_finite_screen`
- PASS `nominal_action_is_exactly_dormant`
- PASS `nominal_physical_consequence_is_bit_exact`
- PASS `measured_mask_and_mode_firewall_hold`
- PASS `finite_zero_allocation_action`
- PASS `action_and_loop_deadlines_hold`
- PASS `bounded_command_and_torque`
- PASS `exact_replay`
- PASS `zero_new_nonadmission_or_numeric_reset`
- PASS `state_local_law_has_no_policy_forecast_or_contact_authority`
- PASS `terminal_fall_count_reduced_four_to_two`
- PASS `mirrored_six_newton_cases_finish`
- PASS `nonterminal_two_and_four_newton_cases_still_finish`
- PASS `eight_newton_guardrails_are_never_earlier`
- PASS `recovered_six_newton_cases_have_stable_two_second_tail`

## Full recovery promotion

- PASS `terminal_fall_count_reduced_four_to_two`
- PASS `mirrored_six_newton_cases_finish`
- PASS `nonterminal_two_and_four_newton_cases_still_finish`
- PASS `eight_newton_guardrails_are_never_earlier`
- PASS `recovered_six_newton_cases_have_stable_two_second_tail`
- OPEN `every_holdout_case_finishes`

## Architectural conclusion

R314 adds one bounded delta to the existing root-roll acceleration task after load-reserve composition. It cannot author contact, alter solver rows, promote a support mode, or bypass admission. Activation is continuous pressure from measured tilt and outward rate; damping is spent only while angular velocity moves farther from upright. The remaining largest behavior gap is the ±8 N overload class, not the relock timer or solver iteration budget.
