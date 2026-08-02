# Bonesaw model-owned contact response audit · r202

> Rust mechanism **PASS** · primary full-sample velocity-jump coverage **99.248%** · authority **NOT ADMITTED**.

## Contract

- Rust now owns FK, the floating mass matrix, one in-place Cholesky factorization, each prospective wheel-point Jacobian, all `M⁻¹Jᵀ` solves, and directional effective mass. Python supplies state and collision-query points, then owns corpus construction and scoring.
- The generalized tangent is `[root angular; root linear; six joints]`. For the selected terminal action, the interval is the componentwise union of every available left/right support-hypothesis acceleration plus the physical contact-impulse box. Completed-step MuJoCo velocity and impulse are labels only.
- Width is reported beside coverage: a trivially huge interval is not silently promoted. Model-derived normal effective mass is evaluated directly; the declared total-mass floor is a conservative construction parameter, not a learned residual.

## Named-case construction result

| bound profile | n | sample coverage | component coverage | impulse coverage | max miss /s | root ω width p95 | root v width p95 | joint width p95 | model p99 µs | bound p99 µs |
|---|---|---|---|---|---|---|---|---|---|---|
| model_exact_100mps2 | 266 | 98.872% | 99.906% | 100.000% | 0.019 | 29.729 | 2.211 | 1168.142 | 2297.794 | 16.761 |
| model_2x_100mps2 | 266 | 99.248% | 99.937% | 100.000% | 0.018 | 51.677 | 3.858 | 2026.187 | 2297.794 | 17.201 |
| total_mass_floor_100mps2 | 266 | 99.248% | 99.937% | 100.000% | 0.017 | 86.738 | 6.467 | 3294.670 | 2297.794 | 16.986 |
| total_mass_floor_250mps2 | 266 | 99.248% | 99.937% | 100.000% | 0.013 | 204.907 | 15.095 | 7789.161 | 2297.794 | 16.770 |

Primary worst sample: **forward_4n_friction_0p03/drop5/tick 200**, action **0**, support candidates **4**, maximum component miss **0.016646/s**.

Primary uncovered samples: **forward_4n_friction_0p03/drop5/tick 200** (root_vx, 0.016646/s); **forward_4n_friction_0p03/drop5_matched/tick 200** (root_vx, 0.016440/s).

## Decision

This revision can admit the model-owned response mechanism, but not command authority. Promotion still requires zero-miss coverage with useful width on a fresh morphology/friction/contact-timing holdout, explicit response uncertainty or a proven morphology envelope, plant-consequence non-regression, and the independent 5 ms timing gate.
