# Bonesaw G1 contact-law transition holdout · r213

> Generic mechanism **PASS** · frozen 50/10/50 grouped profile **REJECTED** · momentum sensitivity **REJECTED** · authority **NOT ADMITTED** · controller/policy steps **0**.

## Contract

- The pinned official 23-DOF G1 supplies authored mass, inertia, limits, and four primitive sphere contacts per foot. G1 remains an evaluation fixture; Upkie remains the interactive model.
- Every five-millisecond transition resets to an independently authored pre-impact state. MuJoCo alone generates the completed contact label. The two profiles predeclare different compliance, friction, cone, and integration laws; no sample is selected from a successful rollout.
- Rust evaluates the causal pre-impact eight-point `M⁻¹Jᵀ`. The primary query applies R212's frozen ±50 rad/s² root-angular, ±10 m/s² root-linear, and ±50 rad/s² joint continuous-acceleration reserve with the unchanged R204 passive directional contact profile. MuJoCo's completed velocity is scoring-only.
- A separate momentum sensitivity maps the completed impulse only for oracle decomposition, forms the generalized-momentum residual, and projects a reserve frozen at 25% of one tick of weight/effort through full `M⁻¹`. Usefulness gates remain 2.0 rad/s root angular, 0.5 m/s root linear, and 10.0 rad/s joints (p95 group maximum).

## Frozen grouped acceleration + directional contact tube

| contact law | samples | sample coverage | component coverage | root ω width p95 | root v width p95 | joint width p95 | bound p99 µs | zero alloc | profile |
|---|---|---|---|---|---|---|---|---|---|
| soft_pyramidal | 48 | 100.000% | 100.0000% | 6.660 | 1.043 | 1383.925 | 2.480 | yes | REJECT |
| stiff_elliptic | 48 | 100.000% | 100.0000% | 6.660 | 1.043 | 1383.925 | 1.747 | yes | REJECT |

The profile closes every transition but fails every useful-width gate. Its eight independent point-impulse boxes compound through the G1 leg Jacobians; strict coverage does not rescue a 1,383.9 rad/s p95 joint interval.

## Completed-impulse momentum sensitivity

| contact law | samples | sample coverage | component coverage | root ω width p95 | root v width p95 | joint width p95 | contact impulse p95 N·s | constraint recon p95 N·s | constraint recon p95 relative | zero alloc | profile |
|---|---|---|---|---|---|---|---|---|---|---|---|
| soft_pyramidal | 48 | 100.000% | 100.0000% | 34.881 | 0.917 | 336.739 | 0.7343 | 0.0045 | 0.91% | yes | REJECT |
| stiff_elliptic | 48 | 100.000% | 100.0000% | 34.881 | 0.917 | 336.739 | 1.8861 | 0.0170 | 1.19% | yes | REJECT |

The constraint-reconstruction column compares the point-response generalized impulse to MuJoCo's separately accumulated generalized constraint impulse. It is diagnostic, not a fitted correction.

## Decision

The R212 grouped construction is rejected out of sample: it obtains strict coverage only with unusably wide G1 joint intervals. The independently frozen momentum sensitivity is also rejected on width. No holdout quantile is fed back into either profile, and no command authority changes. The next construction must couple the finite foot patch (or carry a causal spatial-wrench set) instead of summing eight independent point boxes.

This is a second morphology and materially different parameterized MuJoCo contact formulation, not a second simulator engine or hardware contact calibration. That distinction remains explicit.
