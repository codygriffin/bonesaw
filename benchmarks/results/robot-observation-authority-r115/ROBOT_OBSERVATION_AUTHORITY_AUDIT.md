# Bonesaw robot-observation authority · r115

## Outcome

**PASS.** R115 makes robot-state timing an explicit authority boundary. Every transaction retains the producer timestamp, mapped monotonic control timestamp, stable source identity, sequence, and synchronization uncertainty. Rust reads no clock: it derives signed age only from the caller's tick and mapped timestamp. Future, stale, and synchronization-uncertain causes remain independently visible, and any invalid observation withholds both Primary and brake. This is observation admission only—there is no policy, estimator, physics engine, state integration, or plant rollout.

| case | age ns | age headroom ns | sync uncertainty ns | sync headroom ns | causal/age/sync | selection | flags |
|---|---|---|---|---|---|---|---|
| exact | 0 | 10000000 | 0 | 2000000 | True/True/True | primary | 0x0 |
| limits_inclusive | 10000000 | 0 | 2000000 | 0 | True/True/True | primary | 0x0 |
| stale_by_one_ns | 10000001 | -1 | 0 | 2000000 | True/False/True | rejected | 0x200000 |
| future_by_one_ns | -1 | 10000001 | 0 | 2000000 | False/True/True | rejected | 0x100000 |
| sync_uncertain_by_one_ns | 0 | 10000000 | 2000001 | -1 | True/True/False | rejected | 0x400000 |
| negative_sync_uncertainty | 0 | 10000000 | -1 | 2000001 | True/True/False | rejected | 0x400000 |
| future_and_sync_uncertain | -1 | 10000001 | 2000001 | -1 | False/True/False | rejected | 0x500000 |

Limits are inclusive: age `10000000` ns and synchronization uncertainty `2000000` ns remain admitted with exactly zero headroom. A one-nanosecond breach is rejected with `-1` ns headroom. The combined case retains both failure bits rather than collapsing them into one health score.

## Execution evidence

Across 14000 retained transactions, timing was `6.652/10.730/80.883` µs p50/p99/max. Semantic replay was exact: **True**. Timed allocation calls/bytes were `0/0`. Negative configured limits are rejected: **True**.

## Authority boundary

The shell remains responsible for mapping ROS, sensor, and wall clocks into the monotonic control domain and for reconstructing state at the tick. R115 proves that the command transaction no longer silently treats unstamped, future, stale, or insufficiently synchronized state as authoritative. It does not yet merge sorted observation batches into `RobotHistory`, resolve duplicate source/timestamp samples in the full controller transaction, or attach a conservative configuration-error bound to reconstructed state.
