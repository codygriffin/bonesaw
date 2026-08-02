# Bonesaw lagged-impulse residual conditioning · r198

> Causal sensor-feature audit **PASS** · strict held-out bound **REJECTED** · authority **NOT ADMITTED**.

## Contract

- R198 appends only the immediately preceding completed 5 ms interval's wheel normal impulse, wheel tangential impulse, and generalized constraint-impulse norm to R196's current-state/model feature. The current interval, next support, case name, disturbance label, and future dropout duration are excluded.
- Exact simulator impulse is a candidate sensor contract, not an existing Bonesaw observation or hardware claim. A real adapter would need calibrated force/torque or contact estimation, timestamp/age/uncertainty, and the same missing-evidence behavior as every other authority input.
- Linear and log-scaled encodings are frozen before the named-case split. kNN32 and Lipschitz bounds train on six cases and test the seventh componentwise.
- The unmodified state-only feature reproduces R196 kNN/Lipschitz coverage exactly: **85.34%/91.35%**.

## Held-out result

| features | method | sample coverage | component coverage | max exceedance | bound p95 max | worst holdout | strict |
|---|---|---|---|---|---|---|---|
| state_only_r196 | knn32_action_max | 85.34% | 97.154% | 246.232 | 153.097 | left_1n/drop5_matched/700 | NO |
| state_only_r196 | lipschitz_action | 91.35% | 97.207% | 15.284 | 37977.373 | left_1n/drop10/500 | NO |
| lagged_impulse_linear | knn32_action_max | 85.34% | 97.154% | 246.232 | 153.097 | left_1n/drop5_matched/700 | NO |
| lagged_impulse_linear | lipschitz_action | 91.35% | 97.207% | 16.151 | 39264.363 | left_1n/drop10/500 | NO |
| lagged_impulse_log | knn32_action_max | 85.34% | 97.154% | 246.232 | 153.097 | left_1n/drop5_matched/700 | NO |
| lagged_impulse_log | lipschitz_action | 91.35% | 97.207% | 16.049 | 37967.279 | left_1n/drop10/500 | NO |

Best lagged row: **lagged_impulse_linear / causal_lipschitz_action = 91.35% sample coverage**. No row gains authority unless every held-out sample and component is covered.

## Decision

Lagged impulse is causal in time but does not automatically become an admissible observation or conservative transition tube. If strict holdout remains red, the next representation must expose pre-impact relative velocity, penetration/load and transition-time uncertainty, or solve a bounded momentum jump directly; another nearest-neighbour tuning pass is not authority.
