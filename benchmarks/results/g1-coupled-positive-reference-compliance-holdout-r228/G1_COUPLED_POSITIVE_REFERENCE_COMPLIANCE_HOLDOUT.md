# Bonesaw G1 coupled positive-reference fresh holdout · r228

> Mechanism **PASS** · frozen profile **REJECTED** · authority **NOT ADMITTED** · policy/controller/selector/plant steps **0 / 0 / 0 / 0**.

## Contract

- R227's causal convergence rule froze 32 compliant substeps, 32 forward/reverse projection sweeps, label-free total-momentum caps, and the construction residual box before these labels. Soft/pyramidal/Euler and stiff/elliptic/implicit-fast laws begin at untouched offsets 90,000 and 100,000; every transition resets.
- MuJoCo supplies only the completed five-substep label. The Rust query receives causal prestate geometry, velocity, free acceleration, full Delassus response, model mass, and authored law. Each query repeats bitwise, allocates nothing, and must remain below 5 ms.
- Strict sample coverage, frozen 0.449/0.065/9.320 angular/linear/joint width, deadline, repeat, and allocation gates are conjunctive. A pass promotes only this transition profile to terminal selection/non-regression work, never hardware or command authority.
- A complete retained rerun reproduced all 46 non-timing NPZ arrays exactly. Timing is retained as measured evidence but excluded from semantic equality.

## Fresh result

| law | sample coverage | component coverage | residual p95 ω/v/joint | frozen width ω/v/joint | query p99 µs | profile |
|---|---|---|---|---|---|---|
| soft_pyramidal_euler | 100.000% | 100.0000% | 0.016 / 0.004 / 2.000 | 0.449 / 0.065 / 9.320 | 611.774 | PROMOTE |
| stiff_elliptic_implicitfast | 87.500% | 98.9943% | 0.080 / 0.018 / 9.451 | 0.449 / 0.065 / 9.320 | 418.150 | REJECT |

## Decision

At least one untouched law violates a conjunctive gate. Retain the generic coupled mechanism, reject the frozen profile, and do not tune holdout misses back into it.
