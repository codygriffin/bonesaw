# Historical frame-query batch surface · R286

> Batch semantics **PASS** · caller-owned output capacity **PASS** · authority **UNCHANGED**.

R286 introduces `CompiledFrameAtlas::query_history_into` and
`query_history_batch`. They preserve the scalar historical-query semantics,
reuse the external-provenance output storage supplied by the caller, and
return an indexed error when one query in a batch fails. This is a query/data
flow improvement only; it does not change WBC priorities, contact policy,
physics, or command authority.

## Qualification

| field | value |
|---|---:|
| model | `upkie` |
| queries per batch | 64 |
| warmup batches | 8 |
| measured batches | 32 |
| total elapsed | 5.975 ms |
| mean query time | 2.918 µs |
| repeated results bitwise equal | `True` |
| external provenance capacity | 2 → 2 |

The benchmark intentionally does **not** claim that the legacy `RobotHistory`
reconstruction path is allocation-free; that is a separate follow-up slice.
The claim here is limited to the new caller-owned batch output surface and its
semantic repeatability.

No policy step, physics step, actuator command, plant observation, or authority
admission is part of this result.
