# Bonesaw bounded viability coordinate planner · r147

> Evaluation **PASS**. This is a frozen-state action selector with no external plant step or contact integration. It is not live or physically promoted.

## Outcome

A 0.10 pressure gate keeps near-equilibrium rows at zero request. Active rows run two deterministic yaw→roll→lateral coordinate passes over the r145 lattice and one exact verification solve: 40 fixed Rust WBC queries per planner event. Every active row strictly improves the 250 ms local forecast, every inactive row stays zero, every verified solve is hard-feasible, exact replay passes, and the Rust hot path allocates nothing.

The compact search finishes inside the unit boundary in 8/16 rows, exactly matches the 245-query oracle in 7/16, and has active-row median candidate/oracle pressure 1.011. It still selects a lattice edge in 11/16 rows. This admits the bounded search mechanism, not its command smoothness or plant consequence.

| case | before fall s | L/R | active | queries | zero | candidate | oracle | cand/oracle | request r/l/y | Rust µs |
|---|---|---|---|---|---|---|---|---|---|---|
| left_1n | 0.50 | 10 | YES | 40 | 3.200 | 0.129 | 0.129 | 1.000 | +0/+250/+80 | 3796.9 |
| left_1n | 0.25 | 00 | YES | 40 | 5.761 | 2.663 | 0.635 | 4.196 | +0/+0/-80 | 2569.1 |
| left_1n | 0.10 | 00 | YES | 40 | 2.902 | 0.624 | 0.624 | 1.000 | +0/+0/+80 | 2480.1 |
| left_1n | 0.05 | 00 | YES | 40 | 2.830 | 0.954 | 0.523 | 1.822 | -100/+0/+0 | 2745.5 |
| left_2n | 0.50 | 11 | NO | 1 | 0.006 | 0.006 | 0.003 | 2.249 | +0/+0/+0 | 119.5 |
| left_2n | 0.25 | 10 | YES | 40 | 11.308 | 6.718 | 6.718 | 1.000 | +0/+250/-80 | 3723.3 |
| left_2n | 0.10 | 00 | YES | 40 | 1.505 | 1.098 | 1.098 | 1.000 | +0/+0/-80 | 2596.6 |
| left_2n | 0.05 | 00 | YES | 40 | 29.369 | 21.213 | 20.331 | 1.043 | +250/+100/-20 | 2837.5 |
| right_2n | 0.50 | 11 | NO | 1 | 0.001 | 0.001 | 0.001 | 1.199 | +0/+0/+0 | 110.5 |
| right_2n | 0.25 | 11 | NO | 1 | 0.002 | 0.002 | 0.002 | 1.000 | +0/+0/+0 | 125.5 |
| right_2n | 0.10 | 00 | YES | 40 | 2.607 | 1.609 | 1.609 | 1.000 | -250/+100/+80 | 2563.8 |
| right_2n | 0.05 | 00 | YES | 40 | 2.677 | 0.117 | 0.105 | 1.105 | -250/+250/+0 | 2898.8 |
| forward_6n_overload | 0.50 | 11 | NO | 1 | 0.016 | 0.016 | 0.001 | 26.120 | +0/+0/+0 | 109.6 |
| forward_6n_overload | 0.25 | 10 | YES | 40 | 4.236 | 2.228 | 2.183 | 1.021 | -40/+100/-80 | 3809.7 |
| forward_6n_overload | 0.10 | 00 | YES | 40 | 3.601 | 2.211 | 1.331 | 1.661 | +0/+0/+80 | 2801.4 |
| forward_6n_overload | 0.05 | 00 | YES | 40 | 13.085 | 7.436 | 7.436 | 1.000 | -250/+0/-80 | 2831.9 |

## Integrity gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| inactive_rows_emit_zero_request | PASS |
| every_active_row_has_strict_descent | PASS |
| bounded_query_count | PASS |
| verified_candidates_hard_feasible | PASS |
| exact_replay | PASS |
| zero_rust_allocation | PASS |
| finite | PASS |

## Timing

| scope | p50 µs | p95 µs | p99 µs | max µs |
|---|---|---|---|---|
| summed Rust queries per frozen state | 2671.0 | 3800.1 | 3807.8 | 3809.7 |

## Boundary

- The discrete request can jump between lattice edges. A live candidate needs Rust-owned activation hysteresis, request slew, a fixed update cadence, and a fresh exact verification solve each control tick.
- Timing sums the retained per-query Rust clocks on this host; Python batching and a target-hardware tail certificate remain separate.
- The next gate must execute a smoothed version through the external plant matrix. Failure there leaves r137 live behavior unchanged.
