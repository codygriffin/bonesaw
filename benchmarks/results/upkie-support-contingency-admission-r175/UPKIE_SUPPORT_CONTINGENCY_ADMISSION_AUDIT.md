# Bonesaw observed-support contingency admission · r175

> Evaluation **PASS**. This is a policy- and physics-free corpus: Rust authors a support-conditioned low-energy action, then an independent floating WBC either admits or rejects it under the same exact observed support mask.

## Result

Double support brakes motion while pulling the CoM toward the midpoint; single support pulls toward the one observed wheel; zero support requests exact ballistic gravity and never invents a ground impulse. Angular and joint damping remain requests until the second WBC satisfies dynamics, contact, friction, joint, acceleration, and effort bounds.

| support | rows | admitted | root RMS | root max | max torque util | p99 µs |
|---|---|---|---|---|---|---|
| flight | 64 | 64 | 1.488e-01 | 5.417e-01 | 0.005 | 106.7 |
| left | 64 | 64 | 4.679e+00 | 1.337e+01 | 0.439 | 114.3 |
| right | 64 | 64 | 4.663e+00 | 1.337e+01 | 0.514 | 123.6 |
| double | 64 | 64 | 1.315e-01 | 7.644e-01 | 0.134 | 134.6 |

## Integrity gates

| gate | result |
|---|---|
| all_support_modes_authored | PASS |
| authoring_order_independent | PASS |
| mirrored_single_support_action | PASS |
| flight_linear_request_exactly_ballistic | PASS |
| supported_horizontal_pd_is_dissipative | PASS |
| every_independent_wbc_action_admitted | PASS |
| hard_constraints_feasible | PASS |
| author_replay_exact | PASS |
| candidate_replay_exact | PASS |
| zero_rust_allocation | PASS |
| zero_python_gc | PASS |
| finite_outputs | PASS |

## Architectural boundary

- No policy, plant integration, contact estimator, command lease, hidden clock, or reset participates.
- Reordering every query produces bit-identical authored requests, so prior support history and cache lifetime cannot select the action.
- The WBC session is separate from request authoring; a target is executable only when its typed solve status is `Solved` or `SolvedWithSlack`.
- Delay, noisy contact transitions, causal recovery, actuator bandwidth/thermal effects, and hardware calibration remain external gates.
