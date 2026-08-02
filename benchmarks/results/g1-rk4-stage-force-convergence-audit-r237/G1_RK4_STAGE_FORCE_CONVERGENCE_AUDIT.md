# Bonesaw current-stage-force RK4 convergence audit · r237

> Prediction-only profile **FROZEN** · authority **NOT ADMITTED**.

ABI id 4 evaluates the documented positive-reference contact law at each current generalized-RK4 stage. It does not advance contact gap by another complete state tick inside the derivative. Historical ABI id 3 and all R232/R233 evidence remain unchanged.

| sweeps | 2× refinement width · ang / lin / joint | % gate | p99 ms | decision |
|---|---|---|---|---|
| 1 | 0.7812 / 0.1204 / 6.1319 | 61.319% | 3.007 | — |
| 2 | 0.8083 / 0.1264 / 5.1513 | 51.513% | 3.058 | — |
| 4 | 0.5023 / 0.0815 / 2.4651 | 25.117% | 3.250 | — |
| 8 | 0.3223 / 0.0544 / 1.5872 | 16.116% | 3.800 | — |
| 16 | 0.1186 / 0.0209 / 0.6539 | 6.539% | 3.808 | — |
| 32 | 0.0191 / 0.0034 / 0.2193 | 2.193% | 3.943 | — |
| 64 | 0.0023 / 0.0006 / 0.0801 | 0.801% | 4.064 | FREEZE |
| 128 | — | — | 4.184 | — |

The smallest stable profile is **64 projected sweeps**.

Selection had access only to causal state and contact geometry. Completed R233 impulses and final-tangent residuals were reopened after the sweep count was fixed.

## Spent-label diagnostic (ineligible for selection)

| law | frozen coverage | exact active sets | fitted width · ang / lin / joint |
|---|---|---|---|
| medium_elliptic_rk4 | 91.667% | 47/48 | 0.287 / 0.049 / 11.184 |
| hard_pyramidal_rk4 | 97.917% | 43/48 | 0.792 / 0.126 / 9.912 |

This audit creates no new reference physics, policy, controller, selector, or plant steps. A new untouched cross-integrator holdout is required before any promotion decision.
An independent retained rerun reproduced all 122 non-timing NPZ arrays exactly; 16 measured timing arrays are intentionally excluded from semantic equality.
