# Bonesaw G1 coupled contact-law fresh holdout · r218

> Mechanism **PASS** · frozen profile **REJECTED** · authority **NOT ADMITTED** · controller/policy steps **0**.

## Contract

- R217's coupled eight-point time-step law, 16 sweeps, relaxation mapping, and split root/articulated energy coefficients were frozen before these labels. The new soft/pyramidal/Euler and stiff/elliptic/RK4 laws begin at untouched state offsets 50,000 and 60,000; every sample resets.
- MuJoCo supplies only the completed five-substep scoring label. No controller, policy, successful-rollout selection, or prior state enters either predictor. Rust owns full Delassus coupling, Coulomb projection, split kinetic support, bitwise repeat, and zero-allocation witnesses.
- Strict sample coverage and 2.0/0.5/10.0 p95 root-angular/root-linear/joint widths are conjunctive. A predictor-centered interval cannot be narrower than twice its absolute residual.

## Fresh result

| law | restitution / reg | sample coverage | component coverage | CoP p95 mm | unavoidable joint width p95 | frozen width p95 ω/v/joint | solve p99 µs | zero alloc | profile |
|---|---|---|---|---|---|---|---|---|---|
| soft_pyramidal_euler | 0.000 / 1.500 | 100.000% | 100.0000% | 15.2 | 2.940 | 2.420 / 0.142 / 35.754 | 27.077 | yes | REJECT |
| stiff_elliptic_rk4 | 0.835 / 0.030 | 100.000% | 100.0000% | 16.0 | 17.498 | 13.170 / 0.785 / 175.201 | 21.728 | yes | REJECT |

## Decision

The allocation-free mechanism crosses the untouched laws, but the frozen construction does not satisfy strict coverage and useful width together. No holdout miss is tuned back into the profile. Retain the generic coupled solver and split support; reject this calibration and require typed contact-estimator uncertainty or a higher-order compliant law before authority.
