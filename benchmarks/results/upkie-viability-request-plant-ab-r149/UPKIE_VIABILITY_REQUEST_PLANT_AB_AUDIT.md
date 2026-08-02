# Bonesaw supervised viability-request plant A/B · r149

> Mechanism **PASS** · physical consequence **FAIL** · live promotion **NO**.

## Causal comparison

Each row has four deterministic runs: the retained r137 assumed-contact sentinel, a measured-contact control with no planner, the supervised planner candidate, and an exact candidate replay. The physical verdict compares candidate against the measured-contact control so contact-observer consequences are not credited to or blamed on the planner. The r137 arm remains visible as the current-behavior reference.

| case | r137 | measured control | candidate | searches | active ticks | max |r/l/y| | loop p99 µs | >5 ms | replay |
|---|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | FALL 3.395s | FALL 3.175s | 18 | 71 | 250/250/80 | 5205 | 14 | YES |
| forward_2n | RECOVERED | FALL 3.680s | FALL 3.915s | 22 | 87 | 250/250/80 | 5361 | 18 | YES |
| forward_4n_reference | RECOVERED | FALL 2.865s | FALL 2.120s | 18 | 72 | 250/250/80 | 5678 | 17 | YES |
| forward_6n_overload | FALL 4.820s | FALL 1.230s | FALL 1.220s | 7 | 28 | 250/250/80 | 5603 | 6 | YES |
| backward_2n | RECOVERED | FALL 3.420s | FALL 3.970s | 41 | 162 | 250/250/80 | 6236 | 36 | YES |
| backward_4n | RECOVERED | FALL 2.730s | FALL 2.835s | 24 | 95 | 250/250/80 | 5742 | 18 | YES |
| left_1n | FALL 2.470s | FALL 2.295s | FALL 1.965s | 24 | 93 | 250/250/80 | 5462 | 21 | YES |
| left_2n | FALL 2.350s | FALL 2.535s | FALL 1.460s | 16 | 68 | 250/250/80 | 8551 | 22 | YES |
| left_4n | FALL 2.610s | FALL 1.850s | FALL 1.630s | 19 | 78 | 160/250/80 | 6282 | 21 | YES |
| right_2n | FALL 2.015s | FALL 2.030s | FALL 2.430s | 38 | 158 | 250/250/80 | 7094 | 34 | YES |
| right_4n | FALL 2.415s | FALL 2.195s | FALL 2.295s | 26 | 107 | 250/250/80 | 6014 | 27 | YES |
| diagonal_4n | FALL 2.125s | FALL 2.605s | FALL 2.710s | 34 | 134 | 250/250/80 | 5768 | 29 | YES |
| up_4n | RECOVERED | FALL 4.215s | FALL 3.675s | 16 | 67 | 250/250/80 | 5313 | 15 | YES |
| down_4n | RECOVERED | FALL 3.450s | FALL 3.400s | 16 | 64 | 250/250/80 | 5311 | 15 | YES |
| handle_forward_4n | FALL 4.630s | FALL 2.045s | FALL 1.360s | 6 | 24 | 160/250/80 | 5223 | 5 | YES |
| short_8n_50ms | RECOVERED | FALL 1.785s | FALL 2.760s | 59 | 240 | 250/250/80 | 6406 | 56 | YES |
| long_2n_200ms | RECOVERED | FALL 3.155s | FALL 1.665s | 17 | 65 | 250/250/80 | 5819 | 16 | YES |
| forward_2n_three_pulses | RECOVERED | FALL 3.625s | FALL 3.720s | 17 | 68 | 250/250/80 | 5085 | 10 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | RECOVERED | 0 | 0 | 0/0/0 | 632 | 0 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.715s | FALL 0.490s | 18 | 70 | 200/250/80 | 5910 | 15 | YES |

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| candidate_replay_exact | PASS |
| planner_search_exercised | PASS |
| inactive_planner_is_execution_neutral | PASS |
| query_cardinality_is_bounded | PASS |
| every_admitted_search_strictly_descends | PASS |
| request_bounds_hold | PASS |
| normal_slew_bounds_hold | PASS |
| candidate_age_is_bounded | PASS |
| zero_rust_allocation | PASS |
| finite | PASS |

## Physical gates

| gate | result |
|---|---|
| no_new_numeric_fault | PASS |
| no_new_fall | PASS |
| no_earlier_fall_boundary | FAIL |
| green_control_qualification_preserved | PASS |
| no_loop_deadline_overrun | FAIL |

## Authority and timing contract

- Python owns the bounded coordinate-search schedule and retained MuJoCo experiment only. Rust owns each WBC query, the final command solve, request activation hysteresis, maximum age, per-tick slew, exact-evidence revocation, and allocation witness.
- A planner event costs one baseline query when inactive or 40 exact WBC queries when searching. Between events, no planner query runs; the Rust supervisor either holds a fresh candidate, slews release, or revokes it.
- A nonzero request is not a command. The downstream WBC re-solves every control tick against the current measured hard-contact mask; rejected results cannot become fresh torque.
- Candidate Python GC collections: **0**; summed within-run RSS delta: **1.38 MiB**; loop overruns: **395**.

## Scope

This plant A/B is downstream of the policy- and physics-free r146/r147 frozen-state evaluations. A green mechanism gate proves bounded execution semantics, not universal recovery. Live promotion additionally requires every physical regression gate; otherwise the planner remains evaluation-only and the retained controller is unchanged.
