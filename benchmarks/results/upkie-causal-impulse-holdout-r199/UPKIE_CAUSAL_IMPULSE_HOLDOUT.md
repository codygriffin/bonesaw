# Bonesaw causal impulse holdout audit · r199

> Deployable strict holdout **FAIL** · authority **NOT ADMITTED**.

## Boundary

- Targets are completed-interval wheel impulses plus velocity-jump residuals. They are labels only. Features stop at the selection tick: root twist, joint state, selected action, and the center/spread of the four support-mode acceleration predictions.
- Exact physical support, post-step state/impulse, case identity, and future dropout duration are forbidden from deployable features. The physical-support/action row remains a labelled oracle diagnostic.
- Every bound is trained on six named cases and evaluated on the seventh. A sample passes only if all four targets are componentwise covered.

## Result

| method | n | sample coverage | component coverage | max tangential miss (N·s) | max Δv-error miss | strict |
|---|---|---|---|---|---|---|
| global_max | 266 | 99.624% | 99.624% | 0.010504 | 1.489 | NO |
| global_max_1p25 | 266 | 99.624% | 99.906% | 0.000000 | 0.000 | NO |
| action_max | 266 | 97.744% | 98.778% | 0.048077 | 6.884 | NO |
| causal_knn32_action_max | 266 | 75.564% | 90.602% | 0.222550 | 70.550 | NO |
| causal_lipschitz_action | 266 | 90.602% | 93.327% | 0.000318 | 0.180 | NO |
| oracle_support_action_max | 266 | 93.609% | 95.959% | 0.173232 | 52.811 | NO |

The strongest deployable row is **global_max** at **99.624%** complete-sample coverage. Its worst miss is **left_1n/drop5_matched/tick 700**, target **qdd_box_delta_velocity_exceedance**, value/bound/exceedance **73.527879/71.991978/1.535901**.

## Decision

No empirical pre-step bound becomes authority unless it covers every named holdout without hidden future or simulator-only inputs and remains useful rather than unbounded. This audit either rejects the tested current-state feature family or records only a diagnostic success; plant consequence, online Rust ownership, and the independent 5 ms timing gate remain separate.
