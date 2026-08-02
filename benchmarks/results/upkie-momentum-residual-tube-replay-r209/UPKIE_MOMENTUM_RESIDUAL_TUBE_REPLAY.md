# Bonesaw generalized-momentum residual tube replay · r209

> Rust covector/tangent mechanism **PASS** · physics steps **0** · policy steps **0** · strict retained/fresh coverage **FAIL** · authority **NOT ADMITTED**.

## Contract

- The immutable R206 replay supplies state, support-hypothesis accelerations and completed velocity labels. This run performs no MuJoCo integration and no policy/controller step.
- Rust first computes generalized-momentum residual covectors `M(q)(Δv_observed−Δv_predicted)`. Python fits signed leave-one-named-case-out boxes; Rust maps each box back through the exact full inverse mass to a velocity interval.
- Nearest-candidate fitting is an optimistic support-oracle diagnostic. All-candidate fitting encloses every declared support hypothesis. Both are empirical construction bounds, not calibrated online authority.

## Momentum-space LOCO sweep

| fit mode | scale | retained coverage | fresh coverage | component coverage | max miss | root ω p95 | root v p95 | joint p95 | projection p99 µs |
|---|---|---|---|---|---|---|---|---|---|
| nearest_candidate | 1.00× | 98.872% | 100.000% | 99.909% | 60.17962 | 49.056 | 5.691 | 179.104 | 8.423 |
| nearest_candidate | 1.25× | 98.872% | 100.000% | 99.909% | 56.84255 | 61.317 | 7.113 | 223.738 | 9.289 |
| all_candidates | 1.00× | 98.872% | 100.000% | 99.909% | 59.21494 | 51.503 | 5.946 | 183.099 | 9.768 |
| all_candidates | 1.25× | 98.872% | 100.000% | 99.909% | 55.63671 | 64.376 | 7.431 | 228.732 | 9.065 |

Highest complete-sample coverage is **nearest_candidate_scale_1.00** at **98.905%**. Narrowest joint tube is **nearest_candidate_scale_1.00** at **179.104 rad/s p95**.

Rust residual p99 is **5.228 µs**; inverse-mass box projection remains allocation-free. R204's causal directional comparison is 98.120% retained / 100% fresh at 9.687 / 0.991 / 58.020 p95 width.

## Decision

The covector→tangent mechanism is admitted as deterministic model machinery. These empirical boxes are rejected for command authority unless strict retained/fresh and second-morphology/contact-law holdouts close at useful width, with causal feature provenance, consequence non-regression, deadline evidence and hardware calibration kept separate.
