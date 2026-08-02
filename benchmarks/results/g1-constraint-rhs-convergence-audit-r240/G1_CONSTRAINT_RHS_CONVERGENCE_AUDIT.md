# Bonesaw constraint-RHS convergence audit · r240

> Prediction-only profile **FROZEN** · authority **NOT ADMITTED**.

The mapping now follows the reference equations rather than an engine enum name: implicitfast smooth-force integration does not make contact constraint forces implicit. Non-RK contact RHS uses ABI 0; RK4 uses current-stage-force ABI 4. Historical mappers and evidence remain unchanged.

| sweeps | 2× refinement width · ang / lin / joint | % gate | p99 ms | decision |
|---|---|---|---|---|
| 1 | 0.3436 / 0.0674 / 8.1047 | 81.047% | 2.576 | — |
| 2 | 0.5189 / 0.0866 / 4.4721 | 44.721% | 2.721 | — |
| 4 | 0.6396 / 0.0990 / 3.0324 | 31.979% | 2.918 | — |
| 8 | 0.5361 / 0.0828 / 2.7570 | 27.570% | 2.780 | — |
| 16 | 0.2157 / 0.0334 / 1.0273 | 10.785% | 2.762 | — |
| 32 | 0.0176 / 0.0024 / 0.4011 | 4.011% | 2.697 | — |
| 64 | 0.0083 / 0.0015 / 0.3212 | 3.212% | 4.235 | — |
| 128 | 0.0090 / 0.0014 / 0.1980 | 1.980% | 3.337 | FREEZE |
| 256 | — | — | 4.041 | — |

The smallest stable profile is **128 projected sweeps**.

## Spent R238 diagnostic (ineligible for selection)

| law | ABI | coverage | exact active sets | fitted width · ang / lin / joint |
|---|---|---|---|---|
| balanced_elliptic_implicitfast_r238 | 0 | 97.917% | 48/48 | 0.137 / 0.039 / 5.135 |
| stiff_pyramidal_rk4_stage_force_r238 | 4 | 100.000% | 47/48 | 0.117 / 0.018 / 5.191 |

Selection had access only to causal arrays and created no new MuJoCo, policy, controller, selector, or plant steps. A new untouched mixed-integrator holdout is required before promotion.
An independent retained rerun reproduced all 136 non-timing arrays exactly; 18 measured timing arrays are excluded from semantic equality.
