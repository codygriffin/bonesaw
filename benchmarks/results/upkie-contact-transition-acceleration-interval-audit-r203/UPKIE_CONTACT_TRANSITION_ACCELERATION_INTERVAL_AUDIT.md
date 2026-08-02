# Bonesaw continuous-acceleration interval audit · r203

> Rust mechanism **PASS** · retained strict coverage **FAIL** · fresh μ=0.02 holdout **PASS** · authority **NOT ADMITTED**.

## Contract

- R203 holds R202's tighter model-exact contact-impulse profile and model-owned `M⁻¹Jᵀ` fixed. It adds a distinct componentwise continuous generalized-acceleration interval around every support-hypothesis candidate, then considers all acceleration/time endpoint products before contact impulse is projected.
- The primary ±5 m/s² root-linear reserve was frozen from the R202 retained maximum miss (0.01665 m/s over 5 ms, rounded above 3.33 m/s²) before introducing the new μ=0.02 row. It adds exactly 0.05 m/s root-linear width over a complete 5 ms interval and adds no angular/joint width.
- Coverage and width stay separate. The fresh friction row tests construction transfer only; it is still the same Upkie morphology, MuJoCo contact law, controller, and observation-loss family.

## Reserve sweep

| acceleration profile | n | retained samples | fresh samples | component coverage | max miss /s | root ω width p95 | root v width p95 | joint width p95 | bound p99 µs |
|---|---|---|---|---|---|---|---|---|---|
| zero_reserve | 274 | 98.872% | 100.000% | 99.909% | 0.01861 | 29.729 | 2.205 | 1153.919 | 1.075 |
| root_linear_2mps2 | 274 | 98.872% | 100.000% | 99.909% | 0.01254 | 29.729 | 2.225 | 1153.919 | 0.953 |
| root_linear_5mps2 | 274 | 99.624% | 100.000% | 99.970% | 0.01254 | 29.729 | 2.255 | 1153.919 | 1.052 |
| root_linear_10mps2 | 274 | 99.624% | 100.000% | 99.970% | 0.01254 | 29.729 | 2.305 | 1153.919 | 1.007 |
| structured_5_5_50 | 274 | 100.000% | 100.000% | 100.000% | 0.00000 | 29.779 | 2.255 | 1154.415 | 0.920 |

Primary worst sample: **backward_4n/drop10/tick 201**, maximum miss **0.012536/s**.

## Decision

A passing reserve admits only the acceleration-interval mechanism. Authority remains off because the independent friction box still produces very large wheel/joint widths, the fresh row is not a morphology/contact-law holdout, external-wrench provenance is not yet typed into the online command, and terminal consequence plus the 5 ms end-to-end gate remain separate.
