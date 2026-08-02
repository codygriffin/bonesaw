# Bonesaw G1 typed contact-estimator hypothesis replay · r219

> Mechanism **PASS** · viable construction profiles **0** · profile/authority **NOT PROMOTED** · replay physics/policy/controller/integration **0 / 0 / 0 / 0**.

## Contract

- Rust accepts a typed, explicitly enumerated finite contact-hypothesis set. Every scenario owns contact velocity, impulse caps, friction, restitution, and compliance while sharing the exact state-local Delassus and generalized response. All scenarios validate before outputs change; caller-owned scratch and timed execution allocate nothing.
- The 59-scenario family carries nominal, global signed gap/normal/tangent, per-point signed axis, and two law-error hypotheses. The 315-scenario family additionally enumerates all 2⁸ signed normal-velocity patterns. A finite scenario envelope does not certify unenumerated values between those scenarios.
- This diagnostic grid was selected after R218 labels and runs only on immutable replay. Completed transitions are scoring labels. Strict sample coverage and 2.0/0.5/10.0 p95 width gates are conjunctive; no row can be promoted from construction.

## Construction Pareto sweep

| profile | H | gap mm | normal/tangent m/s | cap | sample coverage | component coverage | width p95 ω/v/joint | query p99 µs | zero alloc |
|---|---|---|---|---|---|---|---|---|---|
| local59_small | 59 | 3.0 | 0.3 / 0.2 | 1× | 72.917% | 96.6595% | 0.864 / 0.129 / 7.185 | 1293.324 | yes |
| normal315_small | 315 | 3.0 | 0.3 / 0.2 | 1× | 81.250% | 97.7011% | 0.864 / 0.129 / 9.185 | 6613.936 | yes |
| normal315_medium | 315 | 5.0 | 0.5 / 0.3 | 1× | 89.583% | 98.6710% | 1.149 / 0.203 / 12.117 | 6475.094 | yes |
| normal315_large | 315 | 10.0 | 1.0 / 0.5 | 1× | 92.708% | 99.4253% | 1.923 / 0.339 / 23.306 | 6432.075 | yes |
| normal315_large_cap2 | 315 | 10.0 | 1.0 / 0.5 | 2× | 96.875% | 99.8204% | 2.625 / 0.444 / 22.785 | 6433.078 | yes |
| normal315_large_cap4 | 315 | 10.0 | 1.0 / 0.5 | 4× | 96.875% | 99.8204% | 2.627 / 0.452 / 22.521 | 8069.378 | yes |
| normal315_large_cap8 | 315 | 10.0 | 1.0 / 0.5 | 8× | 96.875% | 99.8204% | 2.697 / 0.466 / 22.673 | 6428.962 | yes |
| normal315_large_cap16 | 315 | 10.0 | 1.0 / 0.5 | 16× | 96.875% | 99.8204% | 2.706 / 0.467 / 22.673 | 7246.618 | yes |

## Decision

The generic finite-hypothesis mechanism is retained, but no construction row satisfies strict coverage and useful width together. The 315-scenario large profile reaches only 92.708% strict coverage at 23.306 rad/s joint p95 width. Raising impulse caps reaches 96.875% but remains near 22.5 rad/s and still misses. Because the construction gate already fails, spending a fresh holdout would not add authority evidence. Keep hypothesis provenance typed; move the predictor to a higher-order compliant law rather than expanding this discrete set until it memorizes labels.
