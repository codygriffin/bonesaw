# G1 Jacobi column-pointer kernel · R284

> Exact replay **PASS** · CPU instruction reduction **PASS** · timing gate **NOT PROMOTED**.

R284 promotes a strictly arithmetic-identical kernel change in the dominant
one-sided Jacobi pseudoinverse. The projected matrix is already stored as
contiguous column slices. R284 uses validated raw pointers for the scalar dot
and two-column rotation loops, preserving the production row order, floating
point association, and store order. It does not change tolerances, rank
selection, task ordering, clipping, hard feasibility, or the universal solver
semantics. `jacobi-column-pointer-control` restores the pre-R284 slice/indexed
kernel for A/B runs.

## Replay contract

Five CPU-4-pinned, 2,317-tick, policy-free/physics-free G1 repeats reproduce
the established 89-array digest:

`3489c58b1259bcb97feb87dde038de8df30da9c2d1de3cb77a4f169b9f228521`

No policy step, physics step, authority admission, allocation, or reference
model change is part of this result. The semantic replay is exact; only native
timing arrays are expected to differ.

## Timing repeats

| build | p50 µs (mean) | p95 µs (mean) | p99 µs (mean) | max µs (mean) |
|---|---:|---:|---:|---:|
| R283 control | 1780.2 | 4151.1 | 5299.6 | 7842.2 |
| R284 pointer | 1776.7 | 4103.2 | 5264.8 | 7883.9 |

The p99 mean moves by −0.66%, but the host-sensitive maximum is higher in one
repeat (8.442 ms). The universal 5 ms p99 gate therefore remains open and is
not silently reclassified as passed.

## Hardware-counter pair

Three process repeats of the same corpus command, pinned to CPU 4, report:

| counter | R283 control | R284 pointer | delta |
|---|---:|---:|---:|
| retired instructions | 45.523 B | 44.494 B | −2.26% |
| cycles | 16.804 B | 16.607 B | −1.17% |
| cache misses | 25.432 M | 24.655 M | −3.06% |
| branches | 4.565 B | 4.434 B | −2.88% |
| branch misses | 25.698 M | 25.528 M | −0.66% |

The counter reduction is a useful CPU result, not a claim of a fixed wall-clock
bound. It is retained behind the explicit A/B control so another host can
retest it without changing the semantic trace.

## Decision

The pointer kernel is promoted as the CPU default because it is exact on all
established non-timing arrays and reduces retired work on the pinned host. The
5 ms p99 timing gate, CUDA device gates, contact-transfer behavior, and physical
authority gates remain unchanged and open.
