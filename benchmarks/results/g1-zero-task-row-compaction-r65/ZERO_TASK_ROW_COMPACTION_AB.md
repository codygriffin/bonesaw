# Bonesaw exact-zero task-row compaction A/B · r65

## Decision · REJECT

The candidate removes one identically zero vertical row from the horizontal-only CoM task before each dense Viability pseudoinverse. Across 5 alternating pinned process pairs, all 56 non-timing arrays remain bit-for-bit identical. The retained-instruction median changes by -0.00007%; paired changes span -0.00027% to +0.00034%, so the candidate does not reduce stable work in every pair.

> A declared three-row diagnostic task remains a three-row task. Only its mathematically inert solve row is compacted; reported task provenance, residual rows, output arrays, clipping, ranks, feasibility, and allocation behavior remain unchanged.

## Five-run pinned process A/B

| variant | instructions B | cycles B | task-clock s | CPU / wall s | p50 / p99 / max µs | RSS MB |
|---|---|---|---|---|---|---|
| control | 239.795561 | 63.028161 | 15.148908 | 7.420 / 7.431 | 3348.0 / 5674.0 / 7651.4 | 50.36 |
| candidate | 239.795404 | 62.712926 | 15.068312 | 7.309 / 7.312 | 3334.4 / 5629.2 / 7554.5 | 50.43 |

## Relative candidate deltas

| signal | candidate − control |
|---|---|
| retired instructions | -0.0001% |
| cycles | -0.5001% |
| task clock | -0.5320% |
| process CPU | -1.5037% |
| p50 tick | -0.4085% |
| p99 tick | -0.7896% |
| maximum RSS | +0.1383% |

## Preservation and scope

- 56/56 non-timing arrays are exact in every control/candidate pair, including accelerations, efforts, forces, task residuals/clipping, hard residuals/margins, statuses, and solver-work counters.
- The declared task row count and task diagnostics remain unchanged; 8,992 inert row-instances are avoided across the retained corpus's repeated Viability solves.
- Every candidate run passes all 43 admission gates and measures zero Rust hot-loop allocations.
- Hardware counters cover the whole pinned process, not an isolated solve. Process-share values divide by 4,634 WBC calls only to make scale legible and are not per-solve attribution.
- Wall-clock, cycle, and tail changes remain observational host signals. Stable retired instructions plus bit-exact semantics decide promotion.

## Gates

- PASS `all_56_non_timing_arrays_are_bit_exact_in_every_pair`
- PASS `candidate_reduces_median_retired_instructions`
- FAIL `candidate_reduces_retired_instructions_in_every_pair`
- PASS `candidate_keeps_zero_measured_rust_allocations`
