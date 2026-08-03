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
| CPU affinity | 4 |
| independent process repeats | 21 |
| queries per batch | 64 |
| warmup batches per repeat | 8 |
| measured batches per repeat | 32 |
| measured queries per repeat | 2048 |
| latency minimum | 1.344 µs/query |
| latency median | 1.380 µs/query |
| latency mean | 1.442 µs/query |
| latency p95 | 1.412 µs/query |
| latency p99 | 2.396 µs/query |
| latency maximum | 2.642 µs/query |
| peak-to-peak process jitter | 1.298 µs/query |
| coefficient of variation | 18.64% |
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
median is 1.380 µs/query
(-52.68%). R286 retained one recorded timing sample and did not instrument legacy reconstruction allocations; this comparison is contextual, not a paired statistical claim.

No policy step, physics step, actuator command, plant observation, or WBC
authority admission is part of this result.
