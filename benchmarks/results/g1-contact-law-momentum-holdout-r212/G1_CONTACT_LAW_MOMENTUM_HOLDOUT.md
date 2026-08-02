# Bonesaw G1 contact-law momentum holdout · r212

> Generic mechanism **PASS** · frozen residual profile **REJECTED** · authority **NOT ADMITTED** · controller/policy steps **0**.

## Contract

- The pinned official 23-DOF G1 supplies authored mass, inertia, limits, and four primitive sphere contacts per foot. G1 remains an evaluation fixture; Upkie remains the interactive model.
- Every five-millisecond transition resets to an independently authored pre-impact state. MuJoCo alone generates the completed contact label. The two profiles predeclare different compliance, friction, cone, and integration laws; no sample is selected from a successful rollout.
- Rust evaluates the causal pre-impact eight-point `M⁻¹Jᵀ`, maps the completed impulse only for this oracle decomposition, forms the generalized-momentum residual, and projects the frozen reserve through full `M⁻¹` with zero timed allocation.
- Before label generation, the reserve was frozen at 25% of one tick of weight impulse for root linear momentum, the same fraction times root height for root angular momentum, and 25% of one tick of authored effort for each joint. Width gates were frozen at 2.0 rad/s root angular, 0.5 m/s root linear, and 10.0 rad/s joints (p95 group maximum).

## Held-out transition result

| contact law | samples | sample coverage | component coverage | root ω width p95 | root v width p95 | joint width p95 | contact impulse p95 N·s | constraint recon p95 N·s | constraint recon p95 relative | zero alloc | profile |
|---|---|---|---|---|---|---|---|---|---|---|---|
| soft_pyramidal | 48 | 100.000% | 100.0000% | 34.881 | 0.917 | 336.739 | 0.7343 | 0.0045 | 0.91% | yes | REJECT |
| stiff_elliptic | 48 | 100.000% | 100.0000% | 34.881 | 0.917 | 336.739 | 1.8861 | 0.0170 | 1.19% | yes | REJECT |

The constraint-reconstruction column compares the point-response generalized impulse to MuJoCo's separately accumulated generalized constraint impulse. It is diagnostic, not a fitted correction.

## Decision

The generic machinery is retained, but the frozen physical reserve is rejected. It obtains strict coverage only by mapping through full `M⁻¹` to widths beyond every declared usefulness gate: roughly 34.9 rad/s root angular, 0.917 m/s root linear, and 336.7 rad/s at the joints. Coverage and useful width are conjunctive, so this is a failure. No holdout quantile is fed back into the profile; completed impulse remains an evaluation-only label, and no command authority changes.

This is a second morphology and materially different parameterized MuJoCo contact formulation, not a second simulator engine or hardware contact calibration. That distinction remains explicit.
