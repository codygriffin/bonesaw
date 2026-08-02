# Bonesaw coincident task-step limit audit · r67

## Decision · REJECT

The candidate freezes every exactly coincident hard limit reached by one semantic-task correction before rebuilding the projected task pseudoinverse. A two-coordinate unit witness proves the mechanism can reduce three inverse evaluations to two. The immutable 2,317-state G1 corpus contains no instance where that mechanism removes actual dense work: control and candidate both execute 16,809 task pseudoinverses and 10,604 clipped steps.

> This is a workload-negative result, not a numerical failure. All physical, decision, residual, authority, clipping, rank, feasibility, and allocation arrays are bit-exact. Two Jacobi-sweep diagnostic samples change by at most two sweeps and the aggregate increases from 122,031 to 122,032. The experiment stays opt-in and production r66 remains unchanged.

## Work comparison

| signal | control | candidate | delta |
|---|---|---|---|
| clipped steps | 10604 | 10604 | 0 |
| clipped steps by priority | 10604 | 10604 | 0 |
| task jacobi sweeps | 122031 | 122032 | 1 |
| task jacobi sweeps by priority | 122031 | 122032 | 1 |
| task pseudoinverse calls | 16809 | 16809 | 0 |
| task pseudoinverse calls by priority | 16809 | 16809 | 0 |

## Boundary and interpretation

- 56 non-timing arrays compared; 50 physical/decision/authority arrays are bit-exact.
- Both variants pass all 43 admission gates and report zero Rust hot-loop allocations.
- The evaluator has no policy, state integration, contact simulation, or physics rollout.
- Hardware counters are intentionally not used: zero target-work reduction already falsifies promotion, and timing the added scan could only encourage a noise-based claim.
- The next CPU experiment must target sequential, non-coincident Viability clipping rather than rare simultaneous boundaries.

## Gates

- PASS `control_passes_all_43_admission_gates`
- PASS `candidate_passes_all_43_admission_gates`
- PASS `all_physical_decision_and_authority_arrays_are_bit_exact`
- PASS `candidate_keeps_zero_measured_rust_allocations`
- FAIL `candidate_reduces_task_pseudoinverse_calls`
- FAIL `candidate_does_not_increase_jacobi_sweeps`
