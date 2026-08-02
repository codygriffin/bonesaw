# Bonesaw discarded-column Jacobi pair A/B · r60

## Decision · REJECT

Skipping coupling dots for cached columns already below the discarded-energy floor is mathematically and bitwise neutral, but it costs 92,433 additional retired instructions per native sentinel tick (+0.420%). Production therefore keeps the original unconditional coupling dot and does not ship the extra pair-level branch.

> Cycle and latency medians are retained but do not overrule retired instructions: the two sequential trial blocks experience different frequency/contention, while instruction counts have sub-part-per-million within-variant spans.

## Native five-run A/B

| variant | instructions/tick M | cycles/tick M | task-clock/tick ms | p50 / p99 / max µs |
|---|---|---|---|---|
| production control | 21.991005 | 5.886 | 1.434 | 1410.215 / 1758.324 / 2070.944 |
| skip experiment | 22.083438 | 5.845 | 1.436 | 1421.878 / 1755.498 / 2247.658 |

## Why the apparently obvious skip loses

The cached energy comparison is executed for every Jacobi column pair. On this admitted G1 sentinel, columns below the discard floor are too sparse to amortize the added branch; most pairs still require the coupling dot. The experiment changes no consumed arithmetic after the rejected check, which is why all semantic outputs remain identical even as instruction work increases.

## Behavioral preservation

- Native control and experiment semantic reports are exactly equal across status, residual, margin, tracking, extrema, bitwise-repeat, and allocation fields.
- The restored production default passes the r54 four-step oracle 43/43 and keeps all 56 non-timing NPZ arrays byte-exact against the admitted baseline.
- The experimental path remains available only behind `bonesaw-core/jacobi-discarded-pair-experiment` for reproduction; it is disabled by default.
- No iteration cap, tolerance change, clock exit, task reprioritization, or feasibility relaxation is accepted.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| benchmarks/results/g1-jacobi-discarded-r60/control/native-wbc-counter-metrics.json | f728bd5b9a9e08fe37e4b252382d3d9c6b8257eaae6fdf972dd2b0bfcea2cea2 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/native-wbc-counter-metrics.json | 9c329439f54423aa8332106e22c27cde225e9caf8255daed57bccb01dac3ccf6 |
| benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz | f1521acd7a0cc777f61407e49865f5953355d4382035421fb2582f2fa0164295 |
| benchmarks/results/g1-jacobi-discarded-r60/r54-default/oracle-wbc-admission-raw.npz | 9af615c529a374c042ecabe6f0f135052c1db60a0380f8f542eee4d85d21149b |
