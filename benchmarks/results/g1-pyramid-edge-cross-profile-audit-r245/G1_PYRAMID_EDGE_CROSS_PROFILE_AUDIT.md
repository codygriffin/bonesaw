# Bonesaw pyramid-edge cross-profile audit · r245

> Per-profile work **FROZEN** · authority **NOT ADMITTED**.

R245 corrects the R243 profiling gap by forcing both the non-RK and RK4 rows through model-only pyramid-edge cone ABI 2. Each integrator freezes its own smallest causal refinement profile; completed labels remain inaccessible until afterward.

| causal law | integrator ABI | selected sweeps | 2× refinement | p99 ms |
|---|---|---|---|---|
| compliant_pyramidal_implicitfast_r241 | 0 | 64 | 1.033% | 1.188 |
| rigid_elliptic_rk4_constraint_rhs_r241 | 4 | 32 | 1.780% | 3.826 |

The model now evaluates friction velocity at the instantaneous sphere surface material point, retaining centre-minus-radius gap geometry while including ω×r in tangent motion.

## Spent R241 diagnostic (ineligible for selection)

| law | coverage | exact active | width · ang / lin / joint |
|---|---|---|---|
| compliant_pyramidal_implicitfast_r241 | 100.000% | 48/48 | 0.116 / 0.014 / 1.551 |
| rigid_elliptic_rk4_constraint_rhs_r241 | 100.000% | 48/48 | 0.093 / 0.009 / 2.933 |

No new reference physics, policy, controller, selector, or plant step is used. A new untouched crossed holdout is required.
