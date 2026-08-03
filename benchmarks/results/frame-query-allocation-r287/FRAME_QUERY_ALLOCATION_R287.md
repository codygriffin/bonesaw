# Allocation-stable historical frame queries · R287

> Strict caller-owned path **PASS** · timed-loop allocations **0** · authority **UNCHANGED**.

R287 adds `RobotHistory::reconstruct_into`,
`HistoricalFrameQueryWorkspace`, and strict scalar/batch atlas query entry
points. Robot state, external provenance, atlas input, model cache, snapshot,
and retained outputs are all caller-owned. Undersized output storage is a
typed error instead of an implicit growth operation. The older allocating API
remains available for compatibility and delegates to the same semantics.

## Qualification

| field | value |
|---|---:|
| model | `upkie` |
| independent process repeats | 21 |
| queries per batch | 64 |
| warmup batches per repeat | 8 |
| measured batches per repeat | 32 |
| measured queries per repeat | 2048 |
| latency minimum | 1.327 µs/query |
| latency median | 1.381 µs/query |
| latency mean | 1.974 µs/query |
| latency p95 | 2.756 µs/query |
| latency p99 | 2.761 µs/query |
| latency maximum | 2.762 µs/query |
| peak-to-peak process jitter | 1.435 µs/query |
| coefficient of variation | 33.56% |
| measured allocation calls | 0 |
| measured allocated bytes | 0 |
| measured deallocation calls | 0 |
| output provenance capacity | 2 → 2 |
| workspace provenance capacity | 2 → 2 |
| workspace external-sample capacity | 2 → 2 |
| bitwise repeatability within every process | `True` |

The counting allocator surrounds only the warmed measured batch loop. Every
process repeat independently observed zero allocation, zero allocated bytes,
and zero deallocation. Layout/capacity mistakes are rejected before
reconstruction; the retained output is committed only after all robot and
external reconstruction plus atlas evaluation succeeds.

## R286 context

The prior recorded R286 sample was
2.918 µs/query; R287's multi-process
median is 1.381 µs/query
(-52.67%). R286 retained one recorded timing sample and did not instrument legacy reconstruction allocations; this comparison is contextual, not a paired statistical claim.

No policy step, physics step, actuator command, plant observation, or WBC
authority admission is part of this result.
