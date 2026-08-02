# Bonesaw terminal realization residual conditioning · r196

> Deployable leave-one-named-case-out bracketing **REJECTED**.

## Contract

- The target is the positive componentwise pressure residual between the following measured 5 ms plant interval and r195's selected four-support envelope. The plant remains offline evaluation only.
- Deployable features are current root clearance/vertical velocity/tilt/angular rate, four bounded hip/knee coordinates, six normalized joint velocities, the selected envelope pressures, per-pressure spread across support hypotheses, action class, and effort use. No physical contact mask, named case, future dropout duration, external-force label, or post-step state enters those methods.
- Every result is leave-one-named-case-out. `oracle_support_action_max` uses measured physical support only as a diagnostic ceiling; `global_max_1p25` is a declared sensitivity row, not a fitted authority.
- Samples: **266**; actions 0/1/2 **252/1/13**; physical masks 0/1/2/3 **19/5/9/233**.

## Aggregate holdout

| method | samples | sample coverage | component coverage | max exceedance | bound p95 max component | strict |
|---|---|---|---|---|---|---|
| global_max | 266 | 99.25% | 99.785% | 114.425 | 267.523 | NO |
| global_max_1p25 | 266 | 99.62% | 99.839% | 76.151 | 334.403 | NO |
| action_max | 266 | 98.12% | 99.517% | 114.425 | 267.523 | NO |
| causal_knn32_action_max | 266 | 85.34% | 97.154% | 246.232 | 153.097 | NO |
| causal_lipschitz_action | 266 | 91.35% | 97.207% | 15.284 | 37977.373 | NO |
| oracle_support_action_max | 266 | 94.36% | 98.443% | 114.425 | 267.523 | NO |

## Worst held-out samples

| method | case/profile/tick | action/support transition | pressure | residual | bound | exceedance | qdd error max |
|---|---|---|---|---|---|---|---|
| global_max | left_1n/drop5_matched/700 | 0/[0, 3, 2] | joint_velocity_pressure | 267.523 | 153.097 | 114.425 | 14732.8 |
| global_max_1p25 | left_1n/drop5_matched/700 | 0/[0, 3, 2] | joint_velocity_pressure | 267.523 | 191.371 | 76.151 | 14732.8 |
| action_max | left_1n/drop5_matched/700 | 0/[0, 3, 2] | joint_velocity_pressure | 267.523 | 153.097 | 114.425 | 14732.8 |
| causal_knn32_action_max | left_1n/drop5_matched/700 | 0/[0, 3, 2] | joint_velocity_pressure | 267.523 | 21.291 | 246.232 | 14732.8 |
| causal_lipschitz_action | left_1n/drop10/500 | 2/[3, 1, 0] | joint_velocity_pressure | 21.064 | 5.780 | 15.284 | 1717.4 |
| oracle_support_action_max | left_1n/drop5_matched/700 | 0/[0, 3, 2] | joint_velocity_pressure | 267.523 | 153.097 | 114.425 | 14732.8 |

## Named-case folds

| method | held-out case | samples | coverage | max exceedance |
|---|---|---|---|---|
| global_max | nominal | 56 | 100.0% | 0.000 |
| global_max | forward_4n_reference | 56 | 100.0% | 0.000 |
| global_max | backward_4n | 56 | 100.0% | 0.000 |
| global_max | left_1n | 26 | 96.2% | 114.425 |
| global_max | right_1n_mirror | 21 | 100.0% | 0.000 |
| global_max | handle_forward_4n | 40 | 97.5% | 0.838 |
| global_max | forward_4n_friction_0p03 | 11 | 100.0% | 0.000 |
| global_max_1p25 | nominal | 56 | 100.0% | 0.000 |
| global_max_1p25 | forward_4n_reference | 56 | 100.0% | 0.000 |
| global_max_1p25 | backward_4n | 56 | 100.0% | 0.000 |
| global_max_1p25 | left_1n | 26 | 96.2% | 76.151 |
| global_max_1p25 | right_1n_mirror | 21 | 100.0% | 0.000 |
| global_max_1p25 | handle_forward_4n | 40 | 100.0% | 0.000 |
| global_max_1p25 | forward_4n_friction_0p03 | 11 | 100.0% | 0.000 |
| action_max | nominal | 56 | 100.0% | 0.000 |
| action_max | forward_4n_reference | 56 | 100.0% | 0.000 |
| action_max | backward_4n | 56 | 100.0% | 0.000 |
| action_max | left_1n | 26 | 88.5% | 114.425 |
| action_max | right_1n_mirror | 21 | 95.2% | 0.504 |
| action_max | handle_forward_4n | 40 | 97.5% | 0.838 |
| action_max | forward_4n_friction_0p03 | 11 | 100.0% | 0.000 |
| causal_knn32_action_max | nominal | 56 | 100.0% | 0.000 |
| causal_knn32_action_max | forward_4n_reference | 56 | 58.9% | 0.826 |
| causal_knn32_action_max | backward_4n | 56 | 98.2% | 0.586 |
| causal_knn32_action_max | left_1n | 26 | 88.5% | 246.232 |
| causal_knn32_action_max | right_1n_mirror | 21 | 71.4% | 7.251 |
| causal_knn32_action_max | handle_forward_4n | 40 | 85.0% | 9.659 |
| causal_knn32_action_max | forward_4n_friction_0p03 | 11 | 100.0% | 0.000 |
| causal_lipschitz_action | nominal | 56 | 94.6% | 0.085 |
| causal_lipschitz_action | forward_4n_reference | 56 | 94.6% | 1.230 |
| causal_lipschitz_action | backward_4n | 56 | 94.6% | 0.392 |
| causal_lipschitz_action | left_1n | 26 | 80.8% | 15.284 |
| causal_lipschitz_action | right_1n_mirror | 21 | 81.0% | 0.448 |
| causal_lipschitz_action | handle_forward_4n | 40 | 92.5% | 1.904 |
| causal_lipschitz_action | forward_4n_friction_0p03 | 11 | 81.8% | 0.440 |
| oracle_support_action_max | nominal | 56 | 100.0% | 0.000 |
| oracle_support_action_max | forward_4n_reference | 56 | 100.0% | 0.000 |
| oracle_support_action_max | backward_4n | 56 | 96.4% | 0.223 |
| oracle_support_action_max | left_1n | 26 | 80.8% | 114.425 |
| oracle_support_action_max | right_1n_mirror | 21 | 90.5% | 18.464 |
| oracle_support_action_max | handle_forward_4n | 40 | 90.0% | 18.839 |
| oracle_support_action_max | forward_4n_friction_0p03 | 11 | 81.8% | 2.784 |

## Runtime and interpretation

- Independent Rust rescoring maximum: **0.912 µs**; zero Rust allocation: **True**.
- The global worst holdout is **left_1n/drop5_matched tick 700**, action **0**, support transition **[0, 3, 2]**, with **14732.8 rad-or-m/s²** maximum acceleration error. This is a contact-impact discontinuity; a smooth fixed-contact acceleration residual is the wrong state representation for that tail.
- This audit can admit a residual representation for another consequence A/B; it cannot promote terminal authority by itself. Any selected model must next be frozen, independently replayed, tested on new disturbances/morphology perturbations, and composed with the 5 ms deadline.
