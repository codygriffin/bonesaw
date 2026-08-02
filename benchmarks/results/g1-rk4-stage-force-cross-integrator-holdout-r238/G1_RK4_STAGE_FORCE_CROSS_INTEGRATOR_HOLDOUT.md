# Bonesaw stage-force cross-integrator holdout · r238

> Mechanism **PASS** · frozen profile **REJECTED** · authority **NOT ADMITTED**.

This untouched corpus pairs a new implicitfast law (model ABI id 1) with a new RK4 law (current-stage-force ABI id 4). Both law parameters and disjoint state offsets were declared after R237 froze 64 sweeps and before any trajectory was generated.

| law | ABI | frozen coverage | exact active sets | pred-only / missed | fitted width · ang / lin / joint | p99 ms | decision |
|---|---|---|---|---|---|---|---|
| balanced_elliptic_implicitfast_r238 | 1 | 95.833% | 47/48 | 1 / 0 | 0.213 / 0.045 / 21.255 | 0.657 | REJECT |
| stiff_pyramidal_rk4_stage_force_r238 | 4 | 100.000% | 47/48 | 1 / 0 | 0.122 / 0.018 / 5.191 | 2.817 | PROMOTE |

The prediction call received only causal state/contact arrays. Scoring then compared Rust's returned final generalized tangent directly with the reference final tangent. No policy, controller, selector, or plant action participated.

Promotion requires strict coverage, deadline, bitwise-repeat, and zero-allocation success for both integrators. A failure remains useful mechanism evidence but grants no authority.
An independent retained rerun reproduced all 62 non-timing NPZ arrays exactly; 10 measured timing arrays are intentionally excluded from semantic equality.
