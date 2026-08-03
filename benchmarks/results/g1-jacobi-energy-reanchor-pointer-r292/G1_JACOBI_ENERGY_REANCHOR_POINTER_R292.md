# G1 Jacobi energy re-anchor pointer experiment · R292

> Exact replay PASS · CPU instruction reduction FAIL · candidate REJECTED.

R292 tested whether traversing each contiguous column through a raw pointer could reduce the sweep-boundary energy re-anchor overhead retained by R284. The candidate preserves sweep order, column order, scalar summation order, stores, tolerances, rank selection, clipping, feasibility, and task hierarchy. It is retained only behind an explicit experiment feature; the established R284 kernel remains the default.

## Semantic replay

All 4 CPU-4-pinned, 2,317-tick control/candidate pairs are byte-exact across all 112 non-timing arrays. Every pair has digest `6f087b0ab7fc4741450ae0f8f769e4501bd9de3cd8d9f8a1cd55cebb6233bdcf`. The corpus executes no policy and no physics; timing is the only permitted difference.

## Native timing observations

| pair | p50 control / candidate µs | p95 | p99 | max |
|---|---|---|---|---|
| 1 | 1778.3 / 1780.7 | 4050.6 / 4273.2 | 5222.0 / 5266.4 | 7804.6 / 7757.1 |
| 2 | 1793.4 / 1788.9 | 4331.4 / 4125.8 | 5327.4 / 5321.3 | 9056.9 / 7844.3 |
| 3 | 1788.0 / 1799.3 | 4096.1 / 4229.1 | 5333.3 / 5366.0 | 7847.2 / 7904.9 |
| 4 | 1807.6 / 1794.4 | 4168.2 / 4232.7 | 5286.0 / 5457.6 | 8001.6 / 8554.1 |

The median repeat p99 is 5306.7 µs for control and 5343.7 µs for candidate. Large isolated tails appear in both builds, so wall-clock movement is treated as host scheduling evidence rather than the decision signal. The universal 5 ms p99 gate remains open.

## Pinned hardware counters

Three complete-process pairs use the same corpus and CPU pin. Averages are:

| counter | R284 control | R292 candidate | delta |
|---|---|---|---|
| instructions | 44.496420 B | 44.509928 B | +0.030% |
| cycles | 16.792359 B | 16.712775 B | -0.474% |
| cache-misses | 0.025017 B | 0.025227 B | +0.837% |
| branches | 4.434206 B | 4.444400 B | +0.230% |
| branch-misses | 0.025787 B | 0.025192 B | -2.307% |

The candidate retires +0.030% more instructions (13507650 extra per process), and every paired instruction delta is positive (+0.029%, +0.031%, +0.031%). Its larger dynamic branch count reinforces the rejection.

## Decision

The raw-pointer energy re-anchor is rejected as a production optimization. R284's slice-iterator energy re-anchor and split/slice raw-pointer pair kernel remain the CPU default. The candidate stays opt-in solely so the negative result can be reproduced without reconstructing the patch. No timing, authority, contact-transfer, CUDA, plant, or hardware-realization gate changes.
