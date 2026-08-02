# Bonesaw pyramid-edge convergence audit · r243

> Prediction-only profile **FROZEN** · authority **NOT ADMITTED**.

Pyramidal contacts now have an opt-in model-only cone ABI 2 that solves nonnegative edge variables N±μTᵢ. Historical Cartesian diamond ABI 1 remains unchanged. Selection uses causal prediction refinement only.

| sweeps | 2× width · ang / lin / joint | % gate | p99 ms | decision |
|---|---|---|---|---|
| 1 | 0.4969 / 0.0861 / 5.5688 | 55.688% | 2.569 | — |
| 2 | 0.6950 / 0.1214 / 5.0710 | 50.710% | 2.680 | — |
| 4 | 0.5237 / 0.0943 / 2.5662 | 26.184% | 2.649 | — |
| 8 | 0.3213 / 0.0628 / 2.0874 | 20.874% | 2.526 | — |
| 16 | 0.1475 / 0.0308 / 1.9101 | 19.101% | 2.582 | — |
| 32 | 0.0622 / 0.0123 / 0.8132 | 8.132% | 2.747 | — |
| 64 | 0.0070 / 0.0026 / 0.1930 | 1.930% | 2.988 | FREEZE |
| 128 | 0.0150 / 0.0024 / 0.2254 | 2.254% | 3.501 | — |
| 256 | — | — | 4.585 | — |

The smallest stable profile is **64 sweeps**.

## Spent R241 diagnostic (ineligible for selection)

| law | cone ABI | coverage | exact active sets | width · ang / lin / joint |
|---|---|---|---|---|
| compliant_pyramidal_implicitfast_r241 | 2 | 100.000% | 48/48 | 0.063 / 0.014 / 2.745 |
| rigid_elliptic_rk4_constraint_rhs_r241 | 0 | 100.000% | 48/48 | 0.100 / 0.009 / 3.120 |

No new reference physics, policy, controller, selector, or plant step is used. A new untouched holdout is required before promotion.
