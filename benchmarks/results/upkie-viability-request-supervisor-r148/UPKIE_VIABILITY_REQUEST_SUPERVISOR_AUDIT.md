# Bonesaw viability-request supervisor contract · r148

> Contract **PASS**. This admits the generic request-lifetime mechanism, not the planner or plant consequence.

## State sequence

| event | tick | status | provenance | active | exec | age | request r/l/y |
|---|---|---|---|---|---|---|---|
| activate | 1 | 1 | 1 | True | True | 0 | +40/-40/+20 |
| hold | 2 | 2 | 2 | True | True | 1 | +80/-60/+30 |
| duplicate | 2 | 6 | 0 | True | False | 1 | +0/+0/+0 |
| hold_after_reject | 3 | 2 | 2 | True | True | 2 | +100/-60/+30 |
| evidence_loss | 4 | 5 | 0 | False | False | 0 | +0/+0/+0 |
| reactivate | 5 | 1 | 1 | True | True | 0 | +40/+20/+0 |
| hold_after_reactivate | 6 | 2 | 2 | True | True | 1 | +80/+20/+0 |
| release | 7 | 3 | 3 | False | True | 0 | +40/+0/+0 |
| release_hold | 8 | 0 | 0 | False | False | 0 | +0/+0/+0 |
| inactive | 9 | 0 | 0 | False | False | 0 | +0/+0/+0 |
| bad_candidate | 10 | 7 | 0 | False | False | 0 | +0/+0/+0 |

## Gates

| gate | result |
|---|---|
| fresh_hold_release_sequence | PASS |
| duplicate_is_atomic | PASS |
| evidence_loss_revokes_without_tail | PASS |
| bad_input_fails_closed | PASS |
| exact_replay | PASS |
| zero_rust_hot_path_allocation | PASS |
| zero_python_gc | PASS |

## Timing and ownership

The caller-owned NumPy boundary ran **20,000** transitions at p50/p95/p99 **0.391 / 0.441 / 0.470 µs**. Python GC collections were **0** and RSS delta was **2.293 MiB**. The PyO3 method snapshots the Rust allocator counters around the state transition and raises if either changes; every retained call passed.

Rust owns tick ordering, activation/release hysteresis, candidate age, per-axis bounds and slew, target/request state, typed provenance, transition count, and exact-evidence revocation. Python owns only event construction and reporting. A nonzero output is still a request: downstream WBC admission is required every tick.
