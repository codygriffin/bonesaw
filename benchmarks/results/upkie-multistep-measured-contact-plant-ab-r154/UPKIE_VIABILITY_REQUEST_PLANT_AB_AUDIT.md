# Bonesaw budgeted multi-step measured-contact plant A/B · r154

> Mechanism **PASS** · physical consequence **FAIL** · live promotion **NO**.

## Causal comparison

Each row has four deterministic runs: the retained r137 assumed-contact sentinel, a measured-contact control with no planner, the supervised planner candidate, and an exact candidate replay. The physical verdict compares candidate against the measured-contact control so contact-observer consequences are not credited to or blamed on the planner. The r137 arm remains visible as the current-behavior reference.

| case | r137 | measured control | candidate | searches | active ticks | max r/l/y | loop p99 µs | >5 ms | replay |
|---|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | FALL 3.395s | FALL 3.395s | 41 | 75 | 40/0/20 | 1078 | 4 | YES |
| forward_2n | RECOVERED | FALL 3.680s | FALL 3.680s | 215 | 237 | 40/0/20 | 1112 | 2 | YES |
| forward_4n_reference | RECOVERED | FALL 2.865s | FALL 2.615s | 264 | 301 | 40/0/0 | 1232 | 5 | YES |
| forward_6n_overload | FALL 4.820s | FALL 1.230s | FALL 1.260s | 37 | 50 | 0/0/20 | 1131 | 0 | YES |
| backward_2n | RECOVERED | FALL 3.420s | FALL 3.420s | 209 | 255 | 0/0/0 | 1134 | 3 | YES |
| backward_4n | RECOVERED | FALL 2.730s | FALL 2.730s | 285 | 304 | 40/0/0 | 1174 | 2 | YES |
| left_1n | FALL 2.470s | FALL 2.295s | FALL 2.295s | 94 | 150 | 0/0/0 | 12720 | 8 | YES |
| left_2n | FALL 2.350s | FALL 2.535s | FALL 2.530s | 45 | 107 | 40/0/20 | 1102 | 4 | YES |
| left_4n | FALL 2.610s | FALL 1.850s | FALL 1.850s | 96 | 137 | 0/0/0 | 1136 | 1 | YES |
| right_2n | FALL 2.015s | FALL 2.030s | FALL 2.030s | 38 | 86 | 0/0/0 | 1195 | 4 | YES |
| right_4n | FALL 2.415s | FALL 2.195s | FALL 2.365s | 150 | 211 | 40/0/20 | 13996 | 7 | YES |
| diagonal_4n | FALL 2.125s | FALL 2.605s | FALL 2.475s | 237 | 268 | 40/0/0 | 1162 | 2 | YES |
| up_4n | RECOVERED | FALL 4.215s | FALL 4.065s | 127 | 191 | 40/0/20 | 1388 | 7 | YES |
| down_4n | RECOVERED | FALL 3.450s | FALL 3.470s | 91 | 130 | 40/0/0 | 1132 | 4 | YES |
| handle_forward_4n | FALL 4.630s | FALL 2.045s | FALL 2.060s | 170 | 210 | 0/0/20 | 1148 | 1 | YES |
| short_8n_50ms | RECOVERED | FALL 1.785s | FALL 2.345s | 202 | 267 | 0/0/20 | 1135 | 1 | YES |
| long_2n_200ms | RECOVERED | FALL 3.155s | FALL 3.155s | 319 | 381 | 0/0/0 | 1155 | 4 | YES |
| forward_2n_three_pulses | RECOVERED | FALL 3.625s | FALL 3.625s | 392 | 450 | 0/0/0 | 1196 | 4 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | RECOVERED | 212 | 216 | 0/0/0 | 1113 | 0 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.715s | FALL 0.715s | 23 | 65 | 0/0/0 | 1093 | 0 | YES |

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

## Consequence deltas

- Earlier falling boundaries: **4/19**; later: **5/19**; neutral: **10/19**.
- Worst boundary delta: **-0.250 s** (forward_4n_reference); best: **+0.560 s** (short_8n_50ms); median: **+0.000 s**.
- Candidate-minus-control mean / worst increases: maximum_translation_m +0.001255 / +0.04628; maximum_tilt_deg +0.1731 / +36.64; maximum_torque_utilization +0.07602 / +0.357; final_station_error_m -0.01653 / +0.01718.
- Against retained r149: earlier boundaries fall from **11/19** to **4/19**; worst regression contracts from **-1.490 s** to **-0.250 s**; >5 ms loop events fall from **395** to **63**. Total exact queries are **20,957** versus **19,649**, so the win is burst size and jitter, not less aggregate work.

## Authority and timing contract

- Python owns the bounded coordinate-search schedule and retained MuJoCo experiment only. Rust owns each WBC query, the final command solve, request activation hysteresis, maximum age, per-tick slew, exact-evidence revocation, and allocation witness.
- The r154 planner performs one baseline query when inactive or four exact WBC queries when searching: baseline plus one three-point local coordinate question. Rust scores each candidate at eight fixed future knots, and work advances every control tick rather than arriving as a 40-query burst.
- The r154 score retains roll/lateral capture, pitch-path non-regression, yaw, single-support closing direction, actuator utilization, joint headroom, action magnitude, and request delta as separate evidence. A 40/40/20 one-tick trust region prevents the lattice-edge/execution mismatch seen in r149.
- A nonzero request is not a command. The downstream WBC re-solves every control tick against the current measured hard-contact mask; rejected results cannot become fresh torque.
- Candidate Python GC collections: **0**; summed within-run RSS delta: **2.00 MiB**; loop overruns: **63**.

## Scope

This plant A/B is downstream of the policy- and physics-free r146/r147 frozen-state evaluations. A green mechanism gate proves bounded execution semantics, not universal recovery. Live promotion additionally requires every physical regression gate; otherwise the planner remains evaluation-only and the retained controller is unchanged.
