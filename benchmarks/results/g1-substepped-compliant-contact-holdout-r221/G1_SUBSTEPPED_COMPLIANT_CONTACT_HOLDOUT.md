# Bonesaw G1 substepped compliant-contact fresh holdout · r221

> Mechanism **PASS** · frozen profile **REJECTED** · authority **NOT ADMITTED** · controller/policy steps **0**.

## Contract

- R220's 128 microsteps, effective-mass/relaxation/impedance mapping, 16× impulse-cap scale, and 1.748/0.299/9.802 group widths were frozen before these labels. The new mid/elliptic/implicit-fast and hard/pyramidal/RK4 laws begin at untouched offsets 70,000 and 80,000; every sample resets.
- MuJoCo supplies only the completed five-substep scoring label. No controller, policy, rollout selection, prior state, or R221 value enters the Rust predictor. Each query repeats bitwise and writes caller-owned buffers with zero timed allocation.
- Strict sample coverage, 2.0/0.5/10.0 widths, repeat, and allocation gates are conjunctive. Holdout misses are never tuned back into the frozen profile; passing promotes only this evaluation profile, never hardware or command authority.

## Fresh result

| law | sample coverage | component coverage | predictor residual p95 ω/v/joint | frozen width ω/v/joint | impulse error mean N·s | query p99 µs | profile |
|---|---|---|---|---|---|---|---|
| mid_elliptic_implicitfast | 91.667% | 99.7126% | 0.060 / 0.017 / 5.411 | 1.748 / 0.299 / 9.802 | 0.125 | 52.646 | REJECT |
| hard_pyramidal_rk4 | 64.583% | 98.1322% | 1.170 / 0.191 / 8.050 | 1.748 / 0.299 / 9.802 | 0.898 | 53.573 | REJECT |

## Decision

At least one untouched law violates strict coverage despite the useful frozen width. Retain the substepped compliant mechanism, reject the frozen profile, and do not tune the holdout miss back into it before another independently motivated construction.
