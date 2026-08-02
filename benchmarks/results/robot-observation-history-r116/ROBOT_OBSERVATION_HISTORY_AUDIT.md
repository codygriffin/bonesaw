# Bonesaw canonical observation history · r116

## Outcome

**PASS.** R116 adds a fixed-capacity Rust authority boundary between asynchronous robot observations and WBC state queries. Equal mapped timestamps resolve to the lowest stable source ID; within that source, the highest sequence wins. The same evidence is produced whether observations arrive as one sorted batch or as separately scheduled chunks: **True**. Unsorted batches reject atomically: **True**. Ring wraparound is bounded and deterministic: **True**.

| query | status | provenance | q[0] | hard eligible | gate |
|---|---|---|---|---|---|
| exact | 0 | exact | 0.020000 | True | True |
| interpolated | 0 | interpolated | 0.010000 | True | True |
| predicted | 0 | predicted | 0.025000 | True | True |
| held | 0 | held | 0.020000 | False | True |

Held state remains queryable but cannot authorize hard constraints. Interpolated and bounded constant-velocity predicted states retain source interval, source identity, sequence, age headroom, and synchronization headroom. Too-old and excessive interpolation-gap queries returned typed status `5` and `3`.

## Ingest fault matrix

| fault | typed disposition | gate |
|---|---|---|
| epoch | rejected_epoch | True |
| future | rejected_future | True |
| stale | rejected_stale | True |
| uncertain | rejected_uncertain | True |
| invalid_state | rejected_invalid_state | True |

## Execution and memory evidence

Across 2000 exact semantic replays, four-record ingest ran in `0.180/0.231/6.192` µs p50/p99/max; reconstruction ran in `0.090/0.140/0.792` µs. Semantic replay was exact: **True**. Timed allocation calls/bytes were `0/0`. Capacity is fixed at construction and this corpus preallocated `4` generalized q/v scalars across its two slots; no online push grows or shifts storage.

## Scope boundary

This audit has no policy, estimator, physics engine, or plant rollout. Python authors observations and statistics; Rust owns validation, canonical ingest, ring retention, manifold interpolation, bounded prediction, provenance, and timing. The next integration step is to make this reconstructed state—not an ad hoc latest sample—the only state accepted by the live WBC transaction.
