# Bonesaw exact-zero task-row compaction A/B · r65

## Decision · PROMOTE

The candidate removes one identically zero vertical row from the horizontal-only CoM task before each dense Viability pseudoinverse. Across 5 alternating pinned process pairs, all 56 non-timing arrays remain bit-for-bit identical. The retained-instruction median changes by -30.49203%; paired changes span -30.49215% to -30.49166%, so the candidate does not reduce stable work in every pair.

> A declared three-row diagnostic task remains a three-row task. Only its mathematically inert solve row is compacted; reported task provenance, residual rows, output arrays, clipping, ranks, feasibility, and allocation behavior remain unchanged.

## Five-run pinned process A/B

| variant | instructions B | cycles B | task-clock s | CPU / wall s | p50 / p99 / max µs | RSS MB |
|---|---|---|---|---|---|---|
| control | 228.154776 | 59.630714 | 14.330549 | 6.973 / 6.976 | 3301.0 / 5573.7 / 7522.4 | 50.43 |
| candidate | 158.585764 | 56.340682 | 13.497656 | 6.535 / 6.537 | 3093.7 / 5184.6 / 6983.9 | 50.43 |

## Relative candidate deltas

| signal | candidate − control |
|---|---|
| retired instructions | -30.4920% |
| cycles | -5.5173% |
| task clock | -5.8120% |
| process CPU | -6.2808% |
| p50 tick | -6.2817% |
| p99 tick | -6.9819% |
| maximum RSS | +0.0081% |

## Preservation and scope

- 56/56 non-timing arrays are exact in every control/candidate pair, including accelerations, efforts, forces, task residuals/clipping, hard residuals/margins, statuses, and solver-work counters.
- The declared task row count and task diagnostics remain unchanged; 8,992 inert row-instances are avoided across the retained corpus's repeated Viability solves.
- Every candidate run passes all 43 admission gates and measures zero Rust hot-loop allocations.
- Hardware counters cover the whole pinned process, not an isolated solve. Process-share values divide by 4,634 WBC calls only to make scale legible and are not per-solve attribution.
- Wall-clock, cycle, and tail changes remain observational host signals. Stable retired instructions plus bit-exact semantics decide promotion.

## Gates

- PASS `all_56_non_timing_arrays_are_bit_exact_in_every_pair`
- PASS `candidate_reduces_median_retired_instructions`
- PASS `candidate_reduces_retired_instructions_in_every_pair`
- PASS `candidate_keeps_zero_measured_rust_allocations`
