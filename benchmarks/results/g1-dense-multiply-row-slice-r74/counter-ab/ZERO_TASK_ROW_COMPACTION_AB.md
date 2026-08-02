# Bonesaw exact-zero task-row compaction A/B · r65

## Decision · PROMOTE

The candidate removes one identically zero vertical row from the horizontal-only CoM task before each dense Viability pseudoinverse. Across 5 alternating pinned process pairs, all 56 non-timing arrays remain bit-for-bit identical. The retained-instruction median changes by -4.10353%; paired changes span -4.10427% to -4.10275%, so the candidate does not reduce stable work in every pair.

> A declared three-row diagnostic task remains a three-row task. Only its mathematically inert solve row is compacted; reported task provenance, residual rows, output arrays, clipping, ranks, feasibility, and allocation behavior remain unchanged.

## Five-run pinned process A/B

| variant | instructions B | cycles B | task-clock s | CPU / wall s | p50 / p99 / max µs | RSS MB |
|---|---|---|---|---|---|---|
| control | 158.586959 | 56.242305 | 13.483067 | 6.539 / 6.541 | 3095.9 / 5201.5 / 6923.1 | 50.38 |
| candidate | 152.079295 | 55.043310 | 13.199014 | 6.393 / 6.395 | 3028.1 / 5100.7 / 6802.4 | 50.38 |

## Relative candidate deltas

| signal | candidate − control |
|---|---|
| retired instructions | -4.1035% |
| cycles | -2.1318% |
| task clock | -2.1067% |
| process CPU | -2.2332% |
| p50 tick | -2.1919% |
| p99 tick | -1.9377% |
| maximum RSS | +0.0000% |

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
