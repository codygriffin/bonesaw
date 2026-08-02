# Bonesaw anytime hard-feasibility timing A/B · r166

> Authoritative WBC profile **PASS** · synchronous supervisor profile **FAIL** · controller promotion **NO**.

## Result

The candidate compiles exact nonzero coordinate/value lists for each sparse linear row and caps only speculative planner queries at **512 sweeps**. The executable WBC keeps the established uncapped fallback. Coefficient order, cached norms, multipliers, stopping tests, and active-set behavior are unchanged within each solve.
Across **20** frozen plant cases, synchronous-loop deadline overruns change **58 → 5**, while physical execution traces are **exact in every row**. Every authoritative WBC call stays below 5 ms; the worst candidate call is **2.946 ms**.
Logical half-space projections remain **11,904,894 → 11,904,894**; the optimization skips arithmetic at every compiled zero coordinate rather than deleting solver work.

| case | full row | sparse nonzeros | overrun full | overrun sparse | max sweep full | max sweep sparse | loop p99 full µs | loop p99 sparse µs | final WBC max µs | execution exact | replay exact |
|---|---|---|---|---|---|---|---|---|---|---|---|
| nominal | FALL 3.405s | FALL 3.405s | 4 | 1 | 2496 | 2496 | 1155 | 1139 | 2247 | YES | YES |
| forward_2n | FALL 3.680s | FALL 3.680s | 2 | 1 | 2688 | 2688 | 1118 | 1524 | 2917 | YES | YES |
| forward_4n_reference | FALL 2.865s | FALL 2.865s | 6 | 2 | 2688 | 2688 | 4872 | 2203 | 2792 | YES | YES |
| forward_6n_overload | FALL 1.230s | FALL 1.230s | 0 | 0 | 1 | 1 | 1148 | 1148 | 159 | YES | YES |
| backward_2n | FALL 3.420s | FALL 3.420s | 3 | 0 | 2688 | 2688 | 1170 | 1171 | 2946 | YES | YES |
| backward_4n | FALL 2.730s | FALL 2.730s | 2 | 0 | 2688 | 2688 | 1414 | 1299 | 2902 | YES | YES |
| left_1n | FALL 2.295s | FALL 2.295s | 8 | 0 | 2688 | 2688 | 12510 | 3081 | 2764 | YES | YES |
| left_2n | FALL 2.630s | FALL 2.630s | 4 | 0 | 2496 | 2496 | 1228 | 1379 | 2343 | YES | YES |
| left_4n | FALL 1.850s | FALL 1.850s | 1 | 0 | 2496 | 2496 | 1165 | 1127 | 2176 | YES | YES |
| right_2n | FALL 2.030s | FALL 2.030s | 4 | 0 | 2496 | 2496 | 1116 | 1163 | 2209 | YES | YES |
| right_4n | FALL 2.195s | FALL 2.195s | 4 | 0 | 2056 | 2056 | 1176 | 1196 | 2032 | YES | YES |
| diagonal_4n | FALL 2.605s | FALL 2.605s | 2 | 0 | 2688 | 2688 | 1184 | 1171 | 2830 | YES | YES |
| up_4n | FALL 4.215s | FALL 4.215s | 1 | 0 | 2496 | 2496 | 1188 | 1183 | 2112 | YES | YES |
| down_4n | FALL 3.450s | FALL 3.450s | 7 | 0 | 2688 | 2688 | 2401 | 1362 | 2762 | YES | YES |
| handle_forward_4n | FALL 2.045s | FALL 2.045s | 1 | 0 | 2380 | 2380 | 1172 | 1189 | 2492 | YES | YES |
| short_8n_50ms | FALL 1.785s | FALL 1.785s | 1 | 0 | 2393 | 2393 | 1149 | 1151 | 2495 | YES | YES |
| long_2n_200ms | FALL 3.155s | FALL 3.155s | 4 | 0 | 2496 | 2496 | 1180 | 1190 | 2151 | YES | YES |
| forward_2n_three_pulses | FALL 3.625s | FALL 3.625s | 4 | 1 | 2496 | 2496 | 1201 | 1202 | 2383 | YES | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | 0 | 0 | 1 | 1 | 1122 | 1128 | 195 | YES | YES |
| forward_4n_friction_0p03 | FALL 0.715s | FALL 0.715s | 0 | 0 | 1 | 1 | 1112 | 1112 | 175 | YES | YES |

## Admission gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| established_fallback_exercised | PASS |
| projection_work_is_exact | PASS |
| execution_trace_is_exact | PASS |
| candidate_replay_is_exact | PASS |
| zero_authoritative_wbc_deadline_overruns | PASS |
| zero_synchronous_loop_deadline_overruns | FAIL |
| zero_rust_allocation | PASS |
| zero_python_gc | PASS |
| finite | PASS |
| no_new_numeric_fault | PASS |
| no_new_fall | PASS |

## Architecture contract

- Rust owns sparse-nonzero metadata, Dykstra projection ordering, work counters, typed solver status, allocation witness, hybrid guard, confirmation, and request freshness.
- Python owns the immutable MuJoCo matrix, exact A/B/replay sequencing, timing statistics, and artifact generation.
- Admission requires exact executable commands, planner requests/statuses, final-solver diagnostics, and plant traces. A timing win with changed execution is rejected.
- Planner budget exhaustion is typed `MaxIterations` and carries no torque authority; multi-update confirmation remains the temporal admission boundary.
- The five remaining deadline misses are synchronous planner-continuation scheduling failures. They do not invalidate the authoritative WBC kernel gate, but they do reject the combined supervisor profile.
- This can admit a CPU kernel profile, not the learned realization certificate rejected by r159 and not a hardware controller. The retained r137 worker remains live.

## Scope

Host timing is descriptive for this Ryzen 7 3700X. No real-time kernel, priority elevation, core isolation, hardware plant, estimator delay/noise, or authenticated command transport is claimed.
