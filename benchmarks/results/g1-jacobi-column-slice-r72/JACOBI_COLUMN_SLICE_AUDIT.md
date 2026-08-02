# Bonesaw slice-addressed Jacobi columns · r72

## Decision · PROMOTE

Production now addresses each active one-sided-Jacobi column pair through two prevalidated contiguous slices. Column-pair order, scalar coupling accumulation, rotation formulas, right-vector updates, sweep-boundary energy re-anchoring, singular truncation, and every downstream hard-limit comparison remain unchanged. The optimization removes repeated flat-index multiplication and bounds logic; it does not change the factorization or solver semantics.

Across five alternating complete-process A/B pairs pinned to one logical CPU, all 56 non-timing arrays remain bit-for-bit exact and every candidate retires fewer instructions. The median instruction reduction is 30.492%; paired reductions span 30.492% to 30.492%. Process CPU falls 6.28%, p50 falls 6.28%, and p99 falls 6.98%.

A separate native 58-variable G1 WBC sentinel confirms the attribution boundary: marginal retired instructions fall from 22.022 M to 15.807 M per tick (-28.22%), with identical semantic reports, zero infeasible ticks, bitwise repeat, and zero allocations. Native p50 moves -5.39% and median p99 -5.62%.

> Hardware counters remain scoped honestly: the primary A/B covers the complete Python admission process; the native measurement subtracts an adjacent one-tick process and remains a marginal upper bound. Stable retired instructions plus exact semantics decide promotion.

## Pinned complete-process A/B

| signal | production − flat-index control |
|---|---|
| retired instructions | -30.492% |
| cycles | -5.52% |
| task clock | -5.81% |
| process CPU | -6.28% |
| p50 tick | -6.28% |
| p99 tick | -6.98% |
| maximum RSS | +0.0081% |

## Native marginal WBC sentinel

| signal | flat-index control | production | delta |
|---|---|---|---|
| instructions / tick | 22.022 M | 15.807 M | -28.22% |
| cycles / tick | 5.787 M | 5.487 M | -5.18% |
| task clock / tick | 1.387 ms | 1.313 ms | -5.37% |
| p50 / p99 | 1385.4 / 1428.6 µs | 1310.8 / 1348.4 µs | -5.39% / -5.62% |

## Preservation and interpretation

- The promoted default and explicit experiment build are exact across all 56 retained non-timing arrays; the `jacobi-column-slice-control` feature preserves the flat-index A/B control.
- Solver work is unchanged: 16,809 task pseudoinverses, 122,031 Jacobi sweeps, and 10,604 clipped steps. This is cheaper execution of identical work.
- Exact r54 outputs inherit the established r63 8/8 fixed-effort consequence and r64 6/6 independent Pinocchio/NumPy oracle evidence without translating a changed source trace.
- The 30.49% whole-process instruction reduction is larger than the 6.28% process-CPU reduction; instruction count is deterministic work evidence, while cycles and host timing remain observational.
- CUDA remains deferred. The CPU reference becomes both faster and more precisely frozen for a future batch backend.

## Gates

- PASS `production_passes_all_43_admission_gates`
- PASS `all_56_non_timing_arrays_are_bit_exact`
- PASS `all_five_pinned_process_pairs_are_bit_exact`
- PASS `retired_instructions_fall_in_every_pinned_process_pair`
- PASS `median_pinned_process_instructions_fall`
- PASS `native_semantic_reports_are_exact`
- PASS `native_marginal_instructions_fall`
- PASS `production_keeps_zero_measured_rust_allocations`
