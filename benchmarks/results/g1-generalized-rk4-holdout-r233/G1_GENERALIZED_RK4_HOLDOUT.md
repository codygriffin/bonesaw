# Bonesaw generalized RK4 fresh holdout · r233

> Mechanism **PASS** · frozen profile **REJECTED** · authority **NOT ADMITTED**.

Two previously unused RK4 contact laws and disjoint state offsets were generated only after R232 froze 64 projected sweeps. There are no policy, controller, selector, or plant actions in the prediction path.
The model-coupled PyO3 ABI uses generalized-rk4 id 3; ids 0/1/2 remain explicit, implicit, and scalar exponential-trapezoidal.

Tracking error is the reference final generalized tangent minus Rust's returned evolved tangent. The legacy impulse-through-initial-response proxy is retained under an explicit diagnostic name but cannot score a state-evolving integrator.

| law | frozen coverage | exact active sets | pred-only / missed | fitted width · ang / lin / joint | p99 ms | decision |
|---|---|---|---|---|---|---|
| medium_elliptic_rk4 | 95.833% | 47/48 | 0 / 1 | 0.285 / 0.043 / 10.446 | 3.382 | REJECT |
| hard_pyramidal_rk4 | 91.667% | 44/48 | 4 / 0 | 0.699 / 0.110 / 11.248 | 3.320 | REJECT |

Promotion requires strict per-sample coverage, the frozen 5 ms query deadline, bitwise repeat, and zero timed Rust allocation for both laws. Failure retains the RK4 mechanism as a diagnostic predictor but grants no command authority.

An independent retained rerun reproduces all 62 non-timing NPZ arrays exactly. Timing arrays remain measured evidence and are excluded from semantic equality.
