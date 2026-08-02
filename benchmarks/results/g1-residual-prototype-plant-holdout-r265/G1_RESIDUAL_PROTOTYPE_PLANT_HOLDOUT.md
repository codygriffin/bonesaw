# Bonesaw residual-prototype one-shot plant holdout · r265

> Mechanism **PASS** · frozen profile **REJECTED** · holdout **CONSUMED** · authority **NOT ADMITTED**.

R265 is the single no-refit holdout authorized by R264. The frozen 55-coordinate profile, all 192 prototypes, causal-cell calibration, distance gates, candidates, thresholds, source hashes, and model hash are consumed unchanged. New medium elliptic/Euler and stiff elliptic/RK4 laws use untouched offsets 330,000 and 340,000. Each of 96 states executes exact zero, realized zero-WBC, and realized neutral-recovery branches for five 4 ms MuJoCo steps. No policy or plant command runs.

| fresh law | offset | distance support | selected 0 / 1 / 2 | strict safe+improved / nonzero | component coverage | aggregate coverage |
|---|---|---|---|---|---|---|
| medium_elliptic_euler | 330000 | 43/48 | 47 / 1 / 0 | 0 / 1 | 91.473% | 87.597% |
| stiff_elliptic_rk4 | 340000 | 45/48 | 48 / 0 / 0 | 0 / 0 | 92.593% | 84.444% |

The run executes 1,440 fresh MuJoCo steps. 88/96 rows are inside the frozen distance envelope; unsupported rows fail closed to baseline. Supported component/aggregate coverage is 92.045%/85.985%. The selector emits 1 nonzero actions; 0 are actually strictly nonregressing and improving, with 1/0 selected component/aggregate regression rows. Profile query p99 is 1.00 µs with zero Rust allocations and exact semantic repeat.

The selected failure is medium-Euler row 20, candidate 1, causal group 8, nearest frozen prototype 70 at squared distance 16.471. Actual tilt/angular-rate pressure changes are +0.01186/+0.00349 while the frozen upper bounds predict −0.00642/−0.01144; joint-position and headroom improve −1.485/−0.149 and aggregate improves −0.732, but component non-regression is strict. Across all supported candidate values, joint-position pressure misses 38 values in 25 rows with maximum 27.992, headroom misses 37 values in 24 rows with maximum 2.834, and aggregate misses 37 values in 23 rows with maximum 13.964. Tilt/angular-rate miss maxima are 0.154/0.126; joint-velocity and actuator-effort pressure are fully covered. Candidate 1 was part of the frozen family but never selected in the spent R264 rehearsal, so same-cell state distance is not sufficient action-selection support.

This one-shot result is final evidence. The evaluator refuses overwrite, and the holdout may not be rerun, widened, or reinterpreted as a plant command or authority.
