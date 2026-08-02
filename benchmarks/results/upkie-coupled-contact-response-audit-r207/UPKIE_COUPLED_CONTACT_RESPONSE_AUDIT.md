# Bonesaw coupled contact-response audit · r207

> Coupled Rust mechanism **PASS** · strict raw coverage **FAIL** · authority **NOT ADMITTED**.

## Contract

- Rust now emits the complete symmetric two-wheel Delassus operator `W = J M⁻¹ Jᵀ`, not only six independent effective masses. A deterministic forward/reverse projected solve retains wheel-to-wheel coupling, nonnegative bounded normal impulse, directional passive caps, restitution, and a circular Coulomb section.
- The coupled output is a point prediction and convergence witness, not an outer bound. The leave-one-named-case-out residual below is label-calibrated diagnostic evidence and is never fed into command authority.
- MuJoCo is used once to freeze labels and all causal/model inputs. The saved NPZ supports later policy/physics-free sweep analysis through the Rust kernel.

## Fixed-work sweep

| sweeps | raw coverage | LOCO residual coverage | impulse RMSE N·s | raw max miss | LOCO root ω p95 | LOCO root v p95 | LOCO joint p95 | post-contact speed p95 | solve p99 µs |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 10.219% | 94.891% | 0.05087 | 62.11826 | 1.136 | 0.221 | 119.614 | 5.8487 | 0.444 |
| 2 | 10.219% | 95.620% | 0.05169 | 81.75126 | 1.185 | 0.227 | 108.523 | 5.8487 | 0.584 |
| 4 | 10.219% | 95.620% | 0.05293 | 120.92432 | 1.187 | 0.228 | 130.335 | 5.8487 | 0.965 |
| 8 | 10.219% | 95.985% | 0.05399 | 148.67299 | 1.187 | 0.228 | 158.084 | 5.8487 | 1.796 |
| 16 | 10.219% | 95.985% | 0.05445 | 156.49888 | 1.191 | 0.228 | 165.909 | 5.8487 | 3.478 |

The R204 directional outer tube covers **98.175%** of these complete samples at p95 width **9.687 / 0.991 / 58.020**. It remains the conservative comparison, not the coupled point solve.

Cross-wheel Delassus coupling ratio is **0.0323 p50 / 0.0325 p95**. Minimum eigenvalue is **1.96561e-12** and condition number is **4729253413.564 p95**.

## Decision

The full coupled response and fixed-work solve are admitted as model machinery only. Promotion requires a conservative coupled reachable set or independently calibrated root-momentum residual, strict fresh/second-morphology holdout, non-regressing terminal consequence, typed online contact/load witnesses, and the separate deadline/hardware gates.
