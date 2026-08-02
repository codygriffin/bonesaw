# Bonesaw exact-zero task-row compaction A/B · r65

## Decision · REJECT

The candidate removes one identically zero vertical row from the horizontal-only CoM task before each dense Viability pseudoinverse. Across 5 alternating pinned process pairs, all 56 non-timing arrays remain bit-for-bit identical. The retained-instruction median changes by +0.00010%; paired changes span -0.00065% to +0.00036%, so the candidate does not reduce stable work in every pair.

> A declared three-row diagnostic task remains a three-row task. Only its mathematically inert solve row is compacted; reported task provenance, residual rows, output arrays, clipping, ranks, feasibility, and allocation behavior remain unchanged.

## Five-run pinned process A/B

| variant | instructions B | cycles B | task-clock s | CPU / wall s | p50 / p99 / max µs | RSS MB |
|---|---|---|---|---|---|---|
| control | 228.153995 | 59.800729 | 14.359395 | 6.996 / 6.999 | 3301.2 / 5606.5 / 7513.6 | 50.32 |
| candidate | 228.154229 | 60.013961 | 14.430985 | 7.026 / 7.029 | 3319.7 / 5623.8 / 7552.5 | 50.43 |

## Relative candidate deltas

| signal | candidate − control |
|---|---|
| retired instructions | +0.0001% |
| cycles | +0.3566% |
| task clock | +0.4986% |
| process CPU | +0.4184% |
| p50 tick | +0.5627% |
| p99 tick | +0.3089% |
| maximum RSS | +0.2035% |

## Preservation and scope

- 56/56 non-timing arrays are exact in every control/candidate pair, including accelerations, efforts, forces, task residuals/clipping, hard residuals/margins, statuses, and solver-work counters.
- The declared task row count and task diagnostics remain unchanged; 8,992 inert row-instances are avoided across the retained corpus's repeated Viability solves.
- Every candidate run passes all 43 admission gates and measures zero Rust hot-loop allocations.
- Hardware counters cover the whole pinned process, not an isolated solve. Process-share values divide by 4,634 WBC calls only to make scale legible and are not per-solve attribution.
- Wall-clock, cycle, and tail changes remain observational host signals. Stable retired instructions plus bit-exact semantics decide promotion.

## Gates

- PASS `all_56_non_timing_arrays_are_bit_exact_in_every_pair`
- FAIL `candidate_reduces_median_retired_instructions`
- FAIL `candidate_reduces_retired_instructions_in_every_pair`
- PASS `candidate_keeps_zero_measured_rust_allocations`
