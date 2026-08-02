# Bonesaw row-sliced dense matrix products · r74

## Decision · PROMOTE

Production now addresses the left, right, and output rows of each dense matrix product through prevalidated contiguous slices. Output initialization, row/shared/column traversal, exact-zero skipping, and every scalar multiply-add remain in the established order. This changes address and bounds work, not solver arithmetic.

Across five alternating complete-process A/B pairs pinned to one logical CPU, all 56 non-timing arrays remain bit-for-bit exact and every candidate retires fewer instructions. Median retired instructions fall 4.104%; process CPU/p50/p99 move -2.23%/-2.19%/-1.94%.

The separate 58-variable native G1 sentinel is semantically exact and allocation-free. Marginal instructions fall from 15.807 M to 15.317 M per tick (-3.098%); native cycles/p50/p99 move -2.17%/-2.28%/-2.17%.

> Hardware counters remain scoped honestly: the primary A/B covers the complete Python admission process; the native measurement subtracts an adjacent one-tick process and remains a marginal upper bound. Stable retired instructions plus exact semantics decide promotion.

## Post-r72 profile attribution

A 5,324-sample, zero-loss cycle profile of 2,000 CPU-pinned native G1 ticks attributed 81.09% self cycles to the pseudoinverse symbol, 16.34% to the Jacobi coupling line, and 8.21% to the general dense multiply. The narrow r73 paired-iterator candidate was exact but changed median instructions by only +0.0000014% with mixed signs, so it is rejected. Row slicing at the multiply boundary produces stable work reduction instead.

## Pinned complete-process A/B

| signal | production − flat-index control |
|---|---|
| retired instructions | -4.104% |
| cycles | -2.13% |
| task clock | -2.11% |
| process CPU | -2.23% |
| p50 tick | -2.19% |
| p99 tick | -1.94% |
| maximum RSS | +0.0000% |

## Native marginal WBC sentinel

| signal | flat-index control | production | delta |
|---|---|---|---|
| instructions / tick | 15.807 M | 15.317 M | -3.098% |
| cycles / tick | 5.499 M | 5.380 M | -2.17% |
| task clock / tick | 1.316 ms | 1.286 ms | -2.30% |
| p50 / p99 | 1311.4 / 1360.6 µs | 1281.5 / 1331.1 µs | -2.28% / -2.17% |

## Preservation and interpretation

- Default production, explicit experiment, and flat-index control all pass 123 core tests; the row-slice product witness covers dense and sparse shapes through 58 × 58.
- Solver work is unchanged: 16,809 task pseudoinverses, 122,031 Jacobi sweeps, and 10,604 clipped steps. This is cheaper execution of identical work.
- Exact r54 outputs inherit the established r63 8/8 fixed-effort consequence and r64 6/6 independent Pinocchio/NumPy oracle evidence without translating a changed trace.
- The `dense-multiply-row-slice-control` feature preserves the pre-r74 flat-index implementation for future differential checks.
- CUDA remains deferred; r74 further tightens and accelerates the CPU semantic reference.

## Gates

- PASS `production_passes_all_43_admission_gates`
- PASS `all_56_non_timing_arrays_are_bit_exact`
- PASS `all_five_pinned_process_pairs_are_bit_exact`
- PASS `retired_instructions_fall_in_every_pinned_process_pair`
- PASS `native_semantic_reports_are_exact`
- PASS `native_marginal_instructions_fall`
- PASS `production_keeps_zero_measured_rust_allocations`
