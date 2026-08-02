# Bonesaw bit-identical hard-feasibility reuse timing A/B · r169

> Authoritative WBC profile **PASS** · synchronous supervisor profile **FAIL** · controller promotion **NO**.

## Result

Both sides use the admitted r167 sparse/anytime profile. The candidate additionally caches a terminal hard-feasibility result inside one planner session only when the complete ordered hard problem, bounds, and feasibility configuration are bit-identical. Exact seeds still run every soft hierarchy; cached exhausted results return the same typed `MaxIterations` and never gain authority.
Across **20** frozen plant cases, synchronous-loop deadline overruns change **5 → 2**, with **9,669** witnessed reuse hits. The complete non-timing authority trace is **exact**, physical execution is **exact in every row**, and replay is exact. Every authoritative WBC call stays below 5 ms; the worst candidate call is **2.910 ms**.
Logical half-space projections remain **18,332,262 → 18,332,262** across planner and final solves. Reuse removes physical recomputation but preserves the cold logical counters for auditability.

| case | cold | reuse | overrun cold | overrun reuse | cache hits | max sweep cold | max sweep reuse | loop p99 cold µs | loop p99 reuse µs | final WBC max µs | authority exact | execution exact | replay exact |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nominal | FALL 3.405s | FALL 3.405s | 1 | 0 | 114 | 2496 | 2496 | 1085 | 1094 | 2230 | YES | YES | YES |
| forward_2n | FALL 3.680s | FALL 3.680s | 1 | 1 | 639 | 2688 | 2688 | 1089 | 1062 | 2910 | YES | YES | YES |
| forward_4n_reference | FALL 2.865s | FALL 2.865s | 2 | 1 | 810 | 2688 | 2688 | 2046 | 1929 | 2691 | YES | YES | YES |
| forward_6n_overload | FALL 1.230s | FALL 1.230s | 0 | 0 | 99 | 1 | 1 | 1083 | 1098 | 134 | YES | YES | YES |
| backward_2n | FALL 3.420s | FALL 3.420s | 0 | 0 | 624 | 2688 | 2688 | 1401 | 1081 | 2808 | YES | YES | YES |
| backward_4n | FALL 2.730s | FALL 2.730s | 0 | 0 | 849 | 2688 | 2688 | 1184 | 1169 | 2825 | YES | YES | YES |
| left_1n | FALL 2.295s | FALL 2.295s | 0 | 0 | 282 | 2688 | 2688 | 3109 | 3065 | 2770 | YES | YES | YES |
| left_2n | FALL 2.630s | FALL 2.630s | 0 | 0 | 222 | 2496 | 2496 | 1200 | 1225 | 2341 | YES | YES | YES |
| left_4n | FALL 1.850s | FALL 1.850s | 0 | 0 | 288 | 2496 | 2496 | 1125 | 1124 | 2231 | YES | YES | YES |
| right_2n | FALL 2.030s | FALL 2.030s | 0 | 0 | 114 | 2496 | 2496 | 1242 | 1084 | 2216 | YES | YES | YES |
| right_4n | FALL 2.195s | FALL 2.195s | 0 | 0 | 426 | 2056 | 2056 | 1151 | 1155 | 2049 | YES | YES | YES |
| diagonal_4n | FALL 2.605s | FALL 2.605s | 0 | 0 | 714 | 2688 | 2688 | 1124 | 1127 | 2802 | YES | YES | YES |
| up_4n | FALL 4.215s | FALL 4.215s | 0 | 0 | 540 | 2496 | 2496 | 1166 | 1181 | 2229 | YES | YES | YES |
| down_4n | FALL 3.450s | FALL 3.450s | 0 | 0 | 252 | 2688 | 2688 | 1393 | 1369 | 2791 | YES | YES | YES |
| handle_forward_4n | FALL 2.045s | FALL 2.045s | 0 | 0 | 501 | 2380 | 2380 | 1169 | 1153 | 2536 | YES | YES | YES |
| short_8n_50ms | FALL 1.785s | FALL 1.785s | 0 | 0 | 360 | 2393 | 2393 | 1130 | 1137 | 2568 | YES | YES | YES |
| long_2n_200ms | FALL 3.155s | FALL 3.155s | 0 | 0 | 957 | 2496 | 2496 | 1206 | 1144 | 2140 | YES | YES | YES |
| forward_2n_three_pulses | FALL 3.625s | FALL 3.625s | 1 | 0 | 1173 | 2496 | 2496 | 1713 | 1211 | 2404 | YES | YES | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | 0 | 0 | 636 | 1 | 1 | 1083 | 1081 | 149 | YES | YES | YES |
| forward_4n_friction_0p03 | FALL 0.715s | FALL 0.715s | 0 | 0 | 69 | 1 | 1 | 1081 | 1078 | 208 | YES | YES | YES |

## Admission gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| established_fallback_exercised | PASS |
| projection_work_is_exact | PASS |
| candidate_cache_exercised | PASS |
| control_cache_disabled | PASS |
| execution_trace_is_exact | PASS |
| authority_trace_is_exact | PASS |
| candidate_replay_is_exact | PASS |
| cache_witness_replay_is_exact | PASS |
| zero_authoritative_wbc_deadline_overruns | PASS |
| zero_synchronous_loop_deadline_overruns | FAIL |
| zero_rust_allocation | PASS |
| zero_python_gc | PASS |
| finite | PASS |
| no_new_numeric_fault | PASS |
| no_new_fall | PASS |

## Architecture contract

- Rust owns the session-local cache witness, sparse-nonzero metadata, Dykstra projection ordering, logical work counters, typed solver status, allocation witness, hybrid guard, confirmation, and request freshness.
- Python owns the immutable MuJoCo matrix, exact A/B/replay sequencing, timing statistics, and artifact generation.
- Admission requires exact executable commands, planner requests/statuses, final-solver diagnostics, and plant traces. A timing win with changed execution is rejected.
- The cache key contains every ordered hard coefficient and bound as exact IEEE-754 bits plus feasibility-profile parameters. It is never shared between planner and final-authority sessions.
- Planner budget exhaustion remains typed `MaxIterations` and carries no torque authority even when its deterministic terminal result is reused; multi-update confirmation remains the temporal admission boundary.
- The combined synchronous supervisor profile has 2 remaining deadline misses; zero is required for admission.
- This can admit a CPU kernel profile, not the learned realization certificate rejected by r159 and not a hardware controller. The retained r137 worker remains live.

## Scope

Host timing is descriptive for this Ryzen 7 3700X and the explicit `-C target-cpu=native` build profile. The portable generic build is not admitted. No real-time kernel, priority elevation, core isolation, hardware plant, estimator delay/noise, or authenticated command transport is claimed.
