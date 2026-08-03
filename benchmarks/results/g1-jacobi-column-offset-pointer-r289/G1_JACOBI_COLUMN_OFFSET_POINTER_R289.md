# G1 direct-offset Jacobi pointer experiment · R289

> Exact replay PASS · CPU instruction reduction FAIL · candidate REJECTED.

R289 tested whether addressing each Jacobi column pair from one base pointer could remove the split/slice wrapper retained by R284. The candidate preserves column traversal, scalar operation order, rotation order, stores, tolerances, rank selection, clipping, feasibility, and task hierarchy. It is retained only behind an explicit experiment feature; the established R284 kernel remains the default.

## Semantic replay

All 4 CPU-4-pinned, 2,317-tick control/candidate pairs are byte-exact across all 112 non-timing arrays. Every pair has digest `6f087b0ab7fc4741450ae0f8f769e4501bd9de3cd8d9f8a1cd55cebb6233bdcf`. The corpus executes no policy and no physics; timing is the only permitted difference.

## Native timing observations

| pair | p50 control / candidate µs | p95 | p99 | max |
|---|---|---|---|---|
| 1 | 1870.9 / 1766.3 | 6635.5 / 4059.0 | 10523.4 / 5263.8 | 12836.6 / 7670.9 |
| 2 | 1774.7 / 1797.9 | 4015.4 / 6456.9 | 5224.8 / 10887.6 | 7761.6 / 16739.9 |
| 3 | 1771.8 / 1771.1 | 4015.7 / 6004.5 | 5215.4 / 7758.3 | 7707.4 / 16771.3 |
| 4 | 1788.9 / 1774.1 | 4032.6 / 4092.5 | 5259.3 / 5310.0 | 7810.0 / 7892.3 |

The median repeat p99 is 5242.1 µs for control and 6534.1 µs for candidate. Large isolated tails appear in both builds, so wall-clock movement is treated as host scheduling evidence rather than the decision signal. The universal 5 ms p99 gate remains open.

## Pinned hardware counters

Three complete-process pairs use the same corpus and CPU pin. Averages are:

| counter | R284 control | R289 candidate | delta |
|---|---|---|---|
| instructions | 44.496389 B | 44.604707 B | +0.243% |
| cycles | 16.988825 B | 16.744910 B | -1.436% |
| cache-misses | 0.025452 B | 0.025286 B | -0.650% |
| branches | 4.434187 B | 4.404005 B | -0.681% |
| branch-misses | 0.024866 B | 0.024585 B | -1.128% |

The candidate retires +0.243% more instructions (108318523 extra per process), and every paired instruction delta is positive (+0.244%, +0.243%, +0.244%). Fewer dynamic branches do not compensate for the larger executed instruction stream.

## Decision

The direct-offset kernel is rejected as a production optimization. R284's split/slice raw-pointer kernel remains the CPU default. The candidate stays opt-in solely so the negative result can be reproduced without reconstructing the patch. No timing, authority, contact-transfer, CUDA, plant, or hardware-realization gate changes.
