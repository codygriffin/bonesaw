# Bonesaw constraint-RHS fresh holdout · r241

> Mechanism **PASS** · frozen profile **REJECTED** · authority **NOT ADMITTED**.

This untouched corpus crosses the R238 pairing: a new pyramidal implicitfast law uses explicit constraint-RHS ABI 0, while a new elliptic RK4 law uses current-stage-force ABI 4. Laws, offsets 170,000/180,000, 128 sweeps, deadline, and residual box were fixed before generation.

| law | ABI | frozen coverage | exact active sets | pred-only / missed | fitted width · ang / lin / joint | p99 ms | decision |
|---|---|---|---|---|---|---|---|
| compliant_pyramidal_implicitfast_r241 | 0 | 97.917% | 45/48 | 3 / 0 | 0.124 / 0.025 / 17.138 | 0.754 | REJECT |
| rigid_elliptic_rk4_constraint_rhs_r241 | 4 | 100.000% | 48/48 | 0 / 0 | 0.100 / 0.009 / 3.120 | 3.559 | PROMOTE |

The predictor received causal arrays only. Scoring compares returned and reference final generalized tangents directly. Strict coverage, deadline, repeat, and zero allocation are conjunctive across both rows; authority remains separately disabled regardless of this result.
An independent retained rerun reproduced all 62 non-timing arrays exactly; 10 measured timing arrays are excluded from semantic equality.
