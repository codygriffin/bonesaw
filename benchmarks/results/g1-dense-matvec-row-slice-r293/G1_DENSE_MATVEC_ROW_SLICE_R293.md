# G1 dense matvec row-slice promotion · R293

> Exact replay PASS · CPU work PASS · production default PROMOTED · 5 ms p99 OPEN.

R293 validates each dense matrix row once before the matrix-vector and residual inner loops. Scalar multiplication, addition, row, column, and store order are unchanged. The former flat-index path remains available through an explicit control feature.

## Policy/physics-free semantic replay

All 6 CPU-4-pinned, 2,317-tick control/production pairs are byte-exact across all 112 non-timing arrays. Every pair has digest `6f087b0ab7fc4741450ae0f8f769e4501bd9de3cd8d9f8a1cd55cebb6233bdcf`. The corpus executes zero policy and zero physics steps.

## Native timing observations

| pair | p50 control / production µs | p95 | p99 | max |
|---|---|---|---|---|
| 1 | 1765.8 / 1742.8 | 4104.1 / 4015.8 | 5195.8 / 5139.6 | 7716.0 / 7736.4 |
| 2 | 1762.9 / 1754.1 | 4037.5 / 4059.5 | 5214.9 / 5192.0 | 7710.9 / 7621.3 |
| 3 | 1759.9 / 1753.1 | 4018.8 / 4011.9 | 5171.8 / 5171.5 | 7691.9 / 7665.8 |
| 4 | 1764.5 / 1751.8 | 4089.7 / 4033.5 | 5215.3 / 5167.7 | 7674.5 / 7624.1 |
| 5 | 1768.4 / 1756.2 | 3998.2 / 3984.7 | 5227.4 / 5186.9 | 7829.1 / 7793.9 |
| 6 | 1777.9 / 1754.8 | 4180.5 / 4082.7 | 5552.0 / 5177.6 | 7800.3 / 7701.3 |

Median p50 moves 1765.2→1753.6 µs and improves in every pair. Median p99 moves 5215.1→5174.6 µs and also improves in every pair, but remains above 5,000 µs. P95 improves in five of six pairs. The maximum RSS is 62872 KiB for both profiles; median user time moves 3.930→3.915 seconds.

Two earlier exploratory windows remain explicit non-decision evidence: one overlapped extension builds and concurrent CLI work; the other ran every control before every candidate and crossed a visible host-load transition. Both were semantically exact, but neither supplies an admissible latency comparison. The six-pair decision series was run AB/BA after builds were idle.

## Complete-process hardware counters

Five alternating pairs cover the same Python admission process and frozen corpus. Every paired retired-instruction delta is negative (-1.969%, -1.967%, -1.967%, -1.969%, -1.964%). Averages are:

| counter | flat control | row-slice production | delta |
|---|---|---|---|
| instructions | 45.025591 B | 44.139847 B | -1.967% |
| cycles | 16.982842 B | 16.945607 B | -0.219% |
| branches | 4.537999 B | 4.165906 B | -8.199% |
| branch-misses | 0.027252 B | 0.026945 B | -1.127% |
| cache-misses | 0.033561 B | 0.033545 B | -0.045% |

The row-slice path is promoted because it preserves every semantic byte while reducing retired instructions and branches in every measured pair. Mean cycles improve slightly; cache misses are effectively flat. This is an addressing/code-generation optimization, not task deletion, changed factorization, reduced solver work, or weakened feasibility.

## Decision

Enable dense matvec row slices by default and retain `dense-matvec-row-slice-control` for reproducible A/B measurement. R280/R290 tracking, contact, timing, plant, actuator, thermal, CUDA, and hardware-authority gates remain unchanged. In particular, R293 does not close the ordinary 5 ms p99 complaint.
