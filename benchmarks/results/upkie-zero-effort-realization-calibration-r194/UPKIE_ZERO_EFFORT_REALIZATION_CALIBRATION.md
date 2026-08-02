# Bonesaw zero-effort realization calibration · r194

> Leave-one-case-out empirical bound **REJECTED**.

## What is measured

- Only ticks where the Rust chooser selected typed withhold are audited. The model prediction is the no-contact fixed-zero-effort generalized acceleration; realization is the finite difference of MuJoCo root/joint velocity over the following 5 ms while the same zero torque executes.
- The realized acceleration is rescored from the same current state. Signed terminal-pressure error is `realized − predicted`; positive error means the online model underestimated harm.
- This is calibration evidence, not online physics. The online path remains a fixed Rust model query and terminal scorer.

| held-out case | samples | pressure error p95 | pressure error max | qdd error max | physical masks | LOCO coverage | max exceedance |
|---|---|---|---|---|---|---|---|
| nominal | 45 | 2.511 | 2.518 | 311.0 | {'3': 45} | 100.0% | 0.000 |
| forward_4n_reference | 39 | 3.315 | 3.841 | 336.6 | {'3': 39} | 100.0% | 0.000 |
| backward_4n | 37 | 4.225 | 4.417 | 317.2 | {'3': 37} | 100.0% | 0.000 |
| left_1n | 10 | 2.533 | 2.563 | 311.0 | {'0': 1, '1': 1, '3': 8} | 100.0% | 0.000 |
| right_1n_mirror | 15 | 11.290 | 28.200 | 1643.7 | {'0': 2, '2': 2, '3': 11} | 93.3% | 20.831 |
| handle_forward_4n | 22 | 4.597 | 5.393 | 349.5 | {'1': 1, '3': 21} | 86.4% | 0.172 |
| forward_4n_friction_0p03 | 11 | 3.572 | 3.847 | 336.5 | {'0': 3, '3': 8} | 100.0% | 0.000 |

## Result

- Aggregate withhold samples: **179**; pressure-error norm p95/max **4.204/28.200**; acceleration-error norm max **1643.7 rad-or-m/s²**.
- A no-contact zero-effort solve is not a calibrated predictor when physical wheel support may still exist but its observation is unavailable. The next online representation must retain a visible interval or hypothesis set over support modes; choosing one fictitious contact mode is not conservative.
- Offline rescoring maximum: **6.562 µs** with zero Rust allocation: **True**.
