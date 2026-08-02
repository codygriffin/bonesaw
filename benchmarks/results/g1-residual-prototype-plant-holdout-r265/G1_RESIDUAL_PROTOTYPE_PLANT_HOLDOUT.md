# Bonesaw residual-prototype one-shot plant holdout · r265

> Mechanism **PASS** · frozen profile **REJECTED** · holdout **CONSUMED** · authority **NOT ADMITTED**.

R265 is the single no-refit holdout authorized by R264. The frozen 55-coordinate profile, all 192 prototypes, causal-cell calibration, distance gates, candidates, thresholds, source hashes, and model hash are consumed unchanged. New medium elliptic/Euler and stiff elliptic/RK4 laws use untouched offsets 330,000 and 340,000. Each of 96 states executes exact zero, realized zero-WBC, and realized neutral-recovery branches for five 4 ms MuJoCo steps. No policy or plant command runs.

| fresh law | offset | distance support | selected 0 / 1 / 2 | strict safe+improved / nonzero | component coverage | aggregate coverage |
|---|---|---|---|---|---|---|
| medium_elliptic_euler | 330000 | 43/48 | 47 / 1 / 0 | 0 / 1 | 91.473% | 87.597% |
| stiff_elliptic_rk4 | 340000 | 45/48 | 48 / 0 / 0 | 0 / 0 | 92.593% | 84.444% |

The run executes 1,440 fresh MuJoCo steps. 88/96 rows are inside the frozen distance envelope; unsupported rows fail closed to baseline. Supported component/aggregate coverage is 92.045%/85.985%. The selector emits 1 nonzero actions; 0 are actually strictly nonregressing and improving, with 1/0 selected component/aggregate regression rows. Profile query p99 is 1.00 µs with zero Rust allocations and exact semantic repeat.

This one-shot result is final evidence. The evaluator refuses overwrite, and the holdout may not be rerun, widened, or reinterpreted as a plant command or authority.
