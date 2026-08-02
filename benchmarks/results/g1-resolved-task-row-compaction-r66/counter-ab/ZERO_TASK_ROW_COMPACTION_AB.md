# Bonesaw exact-zero task-row compaction A/B · r65

## Decision · REJECT

The candidate removes one identically zero vertical row from the horizontal-only CoM task before each dense Viability pseudoinverse. Across 5 alternating pinned process pairs, all 56 non-timing arrays remain bit-for-bit identical. The retained-instruction median changes by -3.86106%, and every paired change is negative.

> A declared three-row diagnostic task remains a three-row task. Only its mathematically inert solve row is compacted; reported task provenance, residual rows, output arrays, clipping, ranks, feasibility, and allocation behavior remain unchanged.

## Five-run pinned process A/B

| variant | instructions B | cycles B | task-clock s | CPU / wall s | p50 / p99 / max µs | RSS MB |
|---|---|---|---|---|---|---|
| control | 239.795126 | 62.645016 | 15.055975 | 7.327 / 7.330 | 3347.4 / 5626.3 / 7543.1 | 50.28 |
| candidate | 230.536490 | 60.354694 | 14.482483 | 7.062 / 7.065 | 3337.7 / 5632.9 / 7547.2 | 50.31 |

## Relative candidate deltas

| signal | candidate − control |
|---|---|
| retired instructions | -3.8611% |
| cycles | -3.6560% |
| task clock | -3.8091% |
| process CPU | -3.6131% |
| p50 tick | -0.2897% |
| p99 tick | +0.1180% |
| maximum RSS | +0.0570% |

## Preservation and scope

- 56/56 non-timing arrays are exact in every control/candidate pair, including accelerations, efforts, forces, task residuals/clipping, hard residuals/margins, statuses, and solver-work counters.
- The declared task row count and task diagnostics remain unchanged; 8,992 inert row-instances are avoided across the retained corpus's repeated Viability solves.
- Every candidate run passes all 43 admission gates and measures zero Rust hot-loop allocations.
- Hardware counters cover the whole pinned process, not an isolated solve. Process-share values divide by 4,634 WBC calls only to make scale legible and are not per-solve attribution.
- Wall-clock, cycle, and tail changes remain observational host signals. Stable retired instructions plus bit-exact semantics decide promotion.

## Gates

- FAIL `all_56_non_timing_arrays_are_bit_exact_in_every_pair`
- PASS `candidate_reduces_median_retired_instructions`
- PASS `candidate_reduces_retired_instructions_in_every_pair`
- PASS `candidate_keeps_zero_measured_rust_allocations`
