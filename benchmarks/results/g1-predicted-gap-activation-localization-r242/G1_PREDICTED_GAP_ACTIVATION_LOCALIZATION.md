# Bonesaw predicted-gap activation localization · r242

> SPENT-LABEL DIAGNOSTIC ONLY · construction/selection/authority **INELIGIBLE**.

This diagnostic removes the historical non-RK boundary-only activation guard only for opt-in ABI 6. A positive-gap point is admitted when its one-state-step causal prediction crosses the plane while its current normal velocity is closing. Historical ABI 0 and the proven RK4 ABI 4 are unchanged.

| law | ABI | coverage | exact active sets | pred-only / missed | fitted width · ang / lin / joint | p99 ms |
|---|---|---|---|---|---|---|
| compliant_pyramidal_implicitfast_r241 | 6 | 77.083% | 32/48 | 19 / 0 | 1.478 / 0.275 / 22.719 | 0.748 |
| rigid_elliptic_rk4_constraint_rhs_r241 | 4 | 100.000% | 48/48 | 0 / 0 | 0.100 / 0.009 / 3.120 | 3.515 |

ABI 6 is rejected for promotion: the implicitfast row predicts extra unloaded points and widens the joint residual tail. No new MuJoCo, policy, controller, selector, or plant steps were used; the R241 labels were opened only after the causal diagnostic completed.
An independent retained rerun is required before any future construction uses this path.
