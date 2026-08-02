# Bonesaw contact-prestate residual conditioning · r200

> Candidate-sensor audit **PASS** · strict held-out bound **REJECTED** · authority **NOT ADMITTED**.

## Frozen contract

- Each query receives only the contact state already present before its scored 5 ms interval: per-wheel availability, signed contact distance, and three signed MuJoCo constraint-coordinate relative velocities. Unavailable payloads are explicitly zeroed behind the availability bits.
- The unchanged R196 state/model feature is the baseline. Linear and signed-log encodings use frozen 0.01 m and 1 m/s scales. Current-interval impulse, post-step state, next support, case identity, disturbance identity, and future outage duration are forbidden.
- These values come from exact pre-solve MuJoCo contacts. They are candidate-sensor/oracle evidence only; an online contract would require physically available sensing or estimation plus provenance, timestamp, age, uncertainty and fail-closed missing-evidence semantics.
- Samples: **266**; contact-present query rows **247**; flight rows **19**.

## Leave-one-named-case-out result

| features | method | sample coverage | component coverage | max exceedance | bound p95 max | worst holdout | strict |
|---|---|---|---|---|---|---|---|
| state_only_r196 | knn32_action_max | 85.34% | 97.154% | 246.232 | 153.097 | left_1n/drop5_matched/700 | NO |
| state_only_r196 | lipschitz_action | 91.35% | 97.207% | 15.284 | 37977.373 | left_1n/drop10/500 | NO |
| contact_prestate_linear | knn32_action_max | 84.96% | 97.100% | 246.232 | 153.097 | left_1n/drop5_matched/700 | NO |
| contact_prestate_linear | lipschitz_action | 91.35% | 97.207% | 15.086 | 39751.621 | left_1n/drop10/500 | NO |
| contact_prestate_signed_log | knn32_action_max | 84.96% | 97.100% | 246.232 | 153.097 | left_1n/drop5_matched/700 | NO |
| contact_prestate_signed_log | lipschitz_action | 91.35% | 97.207% | 15.166 | 38086.409 | left_1n/drop10/500 | NO |

Best contact row: **contact_prestate_linear / causal_lipschitz_action = 91.35% sample coverage**. Admission requires 100% coverage of every pressure component in every held-out named case.

## Decision

Contact prestate is useful only if it makes the discontinuous terminal residual conservatively bracketable under a fresh named-case holdout. Even a passing oracle row would not admit authority: the observation must first be realized as a typed, timed sensor contract and repeated on new disturbances and morphology perturbations. A failed row means current contact kinematics do not expose the future within-interval impact transition; the next model should bound transition time and momentum jump directly rather than tune another smoother.
