# Bonesaw grouped acceleration reachable-set replay · r212

> Rust replay **PASS** · frozen grouped profile strict retained/fresh coverage **PASS** · authority **NOT ADMITTED**.

## Contract

R212 executes **0 physics, 0 policy, and 0 controller steps**. It replays 274 immutable R207 contact transitions and reruns only the allocation-free Rust directional bound. The completed velocity label is scoring-only.

The reserve is a continuous 5 ms generalized-acceleration reachable set around each recorded causal terminal hypothesis. It is not an impact impulse, measured external wrench, learned residual, candidate selector, or completed-contact oracle.

The primary **50/10/50** profile combines round group magnitudes already represented independently in R203. It was selected with knowledge of Upkie calibration results, so this run is construction evidence—not a held-out authority certificate. It is now frozen before the next morphology/contact-law evaluation.

## Replay sweep

| profile | reserve ω/v/q̈ | retained | fresh | max miss /s | root ω p95 | root v p95 | joint p95 | Rust p99 µs |
|---|---|---|---|---|---|---|---|---|
| r204_structured_5_5_50 | 5/5/50 | 98.120% | 100.000% | 0.132260 | 9.687 | 0.991 | 58.020 | 0.591 |
| root_angular_50 | 50/5/50 | 99.624% | 100.000% | 0.004526 | 10.137 | 0.991 | 58.020 | 0.511 |
| frozen_grouped_50_10_50 | 50/10/50 | 100.000% | 100.000% | 0.000000 | 10.137 | 1.041 | 58.020 | 0.491 |
| quarter_hard_limit_50_25_50 | 50/25/50 | 100.000% | 100.000% | 0.000000 | 10.137 | 1.191 | 58.020 | 0.501 |
| hard_limit_sensitivity_200_25_200 | 200/25/200 | 100.000% | 100.000% | 0.000000 | 11.637 | 1.191 | 59.520 | 0.511 |

The original R204/R207 envelope replays **bitwise-identically** (maximum absolute error `0.000e+00`). All timed Rust calls made **0 allocations**.

## Decision

The frozen grouped set closes the current Upkie corpus without the extreme root/joint widths of the momentum residual boxes. It remains diagnostic until the exact same 50/10/50 construction passes a genuinely new morphology and contact law, retains useful normalized width, and demonstrates non-regressing plant consequence under the independent 5 ms deadline gate.
