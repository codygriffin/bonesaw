# Bonesaw implicitfast constraint-RHS localization · r239

> SPENT-LABEL DIAGNOSTIC ONLY · construction/selection/authority **INELIGIBLE** · new MuJoCo/policy/controller/selector/plant steps **0 / 0 / 0 / 0 / 0**.

MuJoCo documents that implicit and implicitfast exclude constraint forces Jᵀf(v) from the force-velocity Jacobian. Contact friction remains a constraint-space reference acceleration. Mapping the reference engine's implicitfast setting onto Bonesaw's local implicit contact damping therefore conflates smooth-force integration with constraint-RHS evaluation.

| diagnostic ABI | coverage | exact active sets | pred-only / missed | fitted width · ang / lin / joint | p99 ms |
|---|---|---|---|---|---|
| 0 | 97.917% | 48/48 | 0 / 0 | 0.145 / 0.039 / 5.077 | 0.906 |
| 1 | 95.833% | 47/48 | 1 / 0 | 0.213 / 0.045 / 21.255 | 0.666 |
| 2 | 95.833% | 47/48 | 1 / 0 | 0.204 / 0.043 / 21.414 | 0.668 |

ABI 0 removes the predicted-only point in sample 26 and reduces fitted joint width from 21.255 to 5.077 rad/s. Sample 10 remains uncovered with an exact active set, localizing the residual tail to within-foot wrench distribution rather than activation.

- ABI 0: s10 right_ankle_roll_joint -2.539 rad/s (exact), s38 right_ankle_roll_joint +2.045 rad/s (exact), s40 left_ankle_roll_joint +2.001 rad/s (exact).
- ABI 1: s26 left_ankle_roll_joint +10.628 rad/s (set mismatch), s10 right_ankle_roll_joint -2.782 rad/s (exact), s27 left_ankle_roll_joint -2.577 rad/s (exact).
- ABI 2: s26 left_ankle_roll_joint +10.707 rad/s (set mismatch), s10 right_ankle_roll_joint -2.646 rad/s (exact), s27 left_ankle_roll_joint -2.295 rad/s (exact).

This comparison read completed R238 labels and cannot choose a production mapping or numerical profile. The documented constraint semantics supply the independent equation-level rationale; a causal-only convergence freeze and a new untouched holdout are still required.
An independent retained rerun reproduced all 33 non-timing arrays exactly; three measured timing arrays are excluded from semantic equality.
