# Bonesaw spatial-conditioned momentum-residual audit · r210

> Mechanism **PASS** · coordinate LOCO strict coverage **FAIL** / R204 width **FAIL** · grouped LOCO strict coverage **PASS** / R204 width **FAIL** · profile **NOT PROMOTED** · authority **NOT ADMITTED**.

## Contract

- This audit replays immutable R206 arrays: **zero MuJoCo steps and zero policy/controller steps**. Unlike R209's unconditioned tube, R210 first subtracts the exact completed spatial-wrench response, then asks whether the remaining momentum residual can be both strict and useful. Rust computes the residuals and maps held-out boxes through the full model inverse mass.
- For each calibration sample, the lowest-Euclidean-norm residual among the already declared support hypotheses is retained. That candidate ranking uses completed labels and is disclosed; it is not an online selector.
- Every named case is evaluated with a box calibrated from all other named cases. The held-out fold is never read during calibration. The coordinate box retains signed per-coordinate extrema and zero; the grouped box uses one symmetric radius for root moment, root linear impulse and joint impulse respectively.
- The base response uses the exact completed R206 spatial wrench, so this is an optimistic conditional decomposition. Even strict residual coverage would validate only the residual mechanism—not causal contact authority. R203's independent structured acceleration reserve remains unchanged.

## Leave-one-named-case-out result

| variant | n | retained | fresh | max miss /s | root ω p95 | root v p95 | joint p95 | projection p99 µs |
|---|---|---|---|---|---|---|---|---|
| spatial_wrench_without_momentum_box | 274 | 81.579% | 62.500% | 5.317290 | 0.227 | 0.069 | 1.383 | 0.000 |
| loco_coordinate_momentum_box | 274 | 99.624% | 100.000% | 0.582821 | 28.683 | 3.370 | 50.823 | 9.202 |
| loco_group_symmetric_momentum_box | 274 | 100.000% | 100.000% | 0.000000 | 45.463 | 5.384 | 584.787 | 12.194 |

Usefulness is conjunctive: strict retained/fresh coverage plus p95 width no worse than the R204 directional diagnostic (**9.687 rad/s root angular, 0.991 m/s root linear, 58.020 rad/s joints**). The coordinate box fails strict coverage and both root-width ceilings; the grouped box closes coverage only by failing every width ceiling.

The label-ranked momentum residual norm is **0.066448 p95 / 0.733188 max**. Rust residual-query p99 is **6.625 µs**; coordinate/group box projection p99 is **9.202/12.194 µs**, all zero-allocation.

## Decision

Even after granting an oracle-quality completed spatial wrench, neither LOCO residual profile is useful: one is narrow enough only in joints and still misses; the other obtains strict coverage by becoming substantially wider than R204. No profile is promoted. The next calibration must condition on causal contact phase/load/slip without reading the evaluation label, freeze before a genuinely new morphology/contact law, and pair with a causal spatial-wrench set. Any exact wrench or label-ranked candidate remains evaluation-only.
