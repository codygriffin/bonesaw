# Bonesaw task-nullspace correction repair · r69

## Decision · REJECT

After a hard-limit hit, the candidate analytically redirects the remaining correction through the exact current-task nullspace. It carries that correction only when the new limit can be satisfied without changing the achieved task optimum; otherwise it falls back to the established projected SVD. A redundant two-coordinate witness preserves the exact solution with one pseudoinverse instead of two, and all 117 core tests pass.

The immutable 2,317-state G1 corpus exposes the real boundary: every sequential hit changes the current task optimum. Control and candidate therefore both execute exactly 16,809 task pseudoinverses, 122,031 Jacobi sweeps, and 10,604 clipped steps. All 56 non-timing arrays are bit-exact.

> R68's approximate projected-gradient predecessor was rejected before corpus timing because it selected a different monotonic active set and failed two existing dynamic-WBC tests with 1.24e−7 and 1.69e−7 hard-constraint violations. R69 restores exact semantics but produces no target-work reduction. Neither path is production.

## Exact work comparison

| signal | control | candidate | delta |
|---|---|---|---|
| clipped steps | 10604 | 10604 | 0 |
| clipped steps by priority | 10604 | 10604 | 0 |
| task jacobi sweeps | 122031 | 122031 | 0 |
| task jacobi sweeps by priority | 122031 | 122031 | 0 |
| task pseudoinverse calls | 16809 | 16809 | 0 |
| task pseudoinverse calls by priority | 16809 | 16809 | 0 |

## Interpretation

- Both variants pass 43/43 admission gates, exact replay, and zero-allocation checks.
- No policy, integration, contact simulation, or physics rollout enters the audit.
- Hardware counters are intentionally skipped because the candidate removes zero target operations.
- Bound-local shortcuts are now exhausted on this corpus; the next CPU candidate must accelerate each necessary projected solve or change the exact active-set factorization.

## Gates

- PASS `control_passes_all_43_admission_gates`
- PASS `candidate_passes_all_43_admission_gates`
- PASS `all_56_non_timing_arrays_are_bit_exact`
- PASS `candidate_keeps_zero_measured_rust_allocations`
- FAIL `candidate_reduces_task_pseudoinverse_calls`
