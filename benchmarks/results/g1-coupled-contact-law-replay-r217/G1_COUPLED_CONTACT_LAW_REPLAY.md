# Bonesaw G1 coupled contact-law replay · r217

> Mechanism **PASS** · construction profile **REJECTED** · authority **NOT ADMITTED** · replay physics/policy/controller steps **0 / 0 / 0**.

## Contract

- Rust solves all eight prospective G1 foot points together through the full Delassus operator. The normal input is `v_n + max(gap,0)/dt`, so only end-of-step penetration pressure enters; tangential velocity stays physical. Projection keeps normal impulse nonnegative and enforces a circular Coulomb section.
- Declared contact relaxation maps continuously to restitution `exp(-τ/dt)` and diagonal regularization `τ/(6dt)`. The divisor 6 was selected after inspecting R215/R216 construction labels; it is calibration, not hardware physics and not an untouched holdout.
- Completed R215 impulses remain scoring-only. The all-label split kinetic envelope is again construction diagnosis. Any centered component interval covering the predictor has width at least twice its absolute residual.

## Replay result

| law | restitution / reg | CoP p95 mm | predictor residual p95 ω/v/joint | unavoidable joint width p95 | fitted coverage | fitted width p95 ω/v/joint | false + / − |
|---|---|---|---|---|---|---|---|
| compliant_elliptic_rk4 | 0.002 / 1.000 | 12.6 | 0.013 / 0.005 / 1.521 | 3.043 | 100.0% | 2.845 / 0.167 / 41.084 | 0 / 0 |
| rigid_elliptic_implicit | 0.741 / 0.050 | 88.6 | 0.194 / 0.034 / 11.277 | 22.554 | 100.0% | 12.707 / 0.753 / 168.399 | 2 / 0 |

The fitted envelope covers 96/96 construction samples at overall p95 width 6.854/0.404/92.886. The Rust coupled solve is 22.997 µs p99 with zero timed allocation.

## Decision

The coupled time-step/CoP mechanism is retained for a fresh-law holdout, but its R215-fitted predictor/profile is not authority. It materially improves compliant-law joint residual over R216, yet the rigid-law lower-bound width remains above the useful gate. Freeze the mapping before new laws; do not tune from that holdout. If the fresh rigid result remains wide, contact estimator uncertainty or a higher-order compliant law is required.
