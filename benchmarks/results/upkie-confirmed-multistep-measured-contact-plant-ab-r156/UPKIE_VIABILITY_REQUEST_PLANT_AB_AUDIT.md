# Bonesaw confirmed multi-step measured-contact plant A/B · r156

> Mechanism **PASS** · physical consequence **FAIL** · live promotion **NO**.

## Causal comparison

Each row has four deterministic runs: the retained r137 assumed-contact sentinel, a measured-contact control with no planner, the supervised planner candidate, and an exact candidate replay. The physical verdict compares candidate against the measured-contact control so contact-observer consequences are not credited to or blamed on the planner. The r137 arm remains visible as the current-behavior reference.

| case | r137 | measured control | candidate | searches | shadow ticks | confirmed ticks | active ticks | max r/l/y | loop p99 µs | planner p99 µs | final WBC p99 µs | >5 ms | replay |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | FALL 3.395s | FALL 3.430s | 40 | 2 | 1 | 80 | 40/0/0 | 2367 | 1576 | 390 | 6 | YES |
| forward_2n | RECOVERED | FALL 3.680s | FALL 3.680s | 213 | 2 | 4 | 237 | 40/0/20 | 1104 | 562 | 157 | 2 | YES |
| forward_4n_reference | RECOVERED | FALL 2.865s | FALL 2.865s | 271 | 1 | 0 | 350 | 0/0/0 | 4782 | 2474 | 1225 | 6 | YES |
| forward_6n_overload | FALL 4.820s | FALL 1.230s | FALL 1.230s | 33 | 1 | 1 | 44 | 0/0/20 | 1092 | 559 | 156 | 0 | YES |
| backward_2n | RECOVERED | FALL 3.420s | FALL 3.420s | 209 | 0 | 0 | 255 | 0/0/0 | 1140 | 575 | 161 | 3 | YES |
| backward_4n | RECOVERED | FALL 2.730s | FALL 2.730s | 284 | 1 | 0 | 304 | 0/0/0 | 1173 | 594 | 153 | 2 | YES |
| left_1n | FALL 2.470s | FALL 2.295s | FALL 2.295s | 94 | 0 | 0 | 150 | 0/0/0 | 12382 | 6025 | 6044 | 8 | YES |
| left_2n | FALL 2.350s | FALL 2.535s | FALL 2.630s | 75 | 4 | 18 | 126 | 40/0/20 | 1350 | 576 | 168 | 4 | YES |
| left_4n | FALL 2.610s | FALL 1.850s | FALL 1.850s | 96 | 0 | 0 | 137 | 0/0/0 | 1094 | 562 | 149 | 1 | YES |
| right_2n | FALL 2.015s | FALL 2.030s | FALL 2.030s | 38 | 0 | 0 | 86 | 0/0/0 | 1041 | 524 | 174 | 4 | YES |
| right_4n | FALL 2.415s | FALL 2.195s | FALL 2.195s | 146 | 2 | 0 | 183 | 0/0/0 | 1164 | 596 | 164 | 4 | YES |
| diagonal_4n | FALL 2.125s | FALL 2.605s | FALL 2.485s | 234 | 1 | 1 | 270 | 40/0/0 | 1141 | 568 | 155 | 2 | YES |
| up_4n | RECOVERED | FALL 4.215s | FALL 3.765s | 94 | 1 | 1 | 133 | 40/0/0 | 1124 | 552 | 148 | 1 | YES |
| down_4n | RECOVERED | FALL 3.450s | FALL 3.450s | 85 | 1 | 0 | 123 | 0/0/0 | 2594 | 1261 | 728 | 7 | YES |
| handle_forward_4n | FALL 4.630s | FALL 2.045s | FALL 2.045s | 168 | 1 | 0 | 207 | 0/0/0 | 1162 | 570 | 156 | 1 | YES |
| short_8n_50ms | RECOVERED | FALL 1.785s | FALL 1.785s | 121 | 2 | 0 | 155 | 0/0/0 | 1082 | 559 | 177 | 1 | YES |
| long_2n_200ms | RECOVERED | FALL 3.155s | FALL 3.155s | 319 | 0 | 0 | 381 | 0/0/0 | 1150 | 597 | 152 | 4 | YES |
| forward_2n_three_pulses | RECOVERED | FALL 3.625s | FALL 3.625s | 392 | 0 | 0 | 450 | 0/0/0 | 1157 | 597 | 156 | 4 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | RECOVERED | 212 | 0 | 0 | 216 | 0/0/0 | 1068 | 524 | 138 | 0 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.715s | FALL 0.715s | 23 | 0 | 0 | 65 | 0/0/0 | 1068 | 544 | 154 | 0 | YES |

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
| confirmation_shadow_exercised | PASS |
| confirmation_execution_exercised | PASS |
| request_only_after_confirmation | PASS |
| support_change_revocation_exercised | PASS |

## Physical gates

| gate | result |
|---|---|
| no_new_numeric_fault | PASS |
| no_new_fall | PASS |
| no_earlier_fall_boundary | FAIL |
| green_control_qualification_preserved | PASS |
| no_loop_deadline_overrun | FAIL |

## Consequence deltas

- Earlier falling boundaries: **2/19**; later: **2/19**; neutral: **15/19**.
- Worst boundary delta: **-0.450 s** (up_4n); best: **+0.095 s** (left_2n); median: **+0.000 s**.
- Candidate-minus-control mean / worst increases: maximum_translation_m -0.01907 / +0.04119; maximum_tilt_deg +0.6358 / +8.104; maximum_torque_utilization +0.009497 / +0.1792; final_station_error_m -0.02129 / +0.01344.
- Against retained r149: earlier boundaries fall from **11/19** to **2/19**; worst regression contracts from **-1.490 s** to **-0.450 s**; >5 ms loop events fall from **395** to **60**. Total exact queries are **20,517** versus **19,649**, so the win is burst size and jitter, not less aggregate work.
- Against its direct r154 predecessor, confirmation changes earlier boundaries **4 → 2**, later boundaries **5 → 2**, neutral boundaries **10 → 15**, exact planner queries **20,957 → 20,517**, and >5 ms loops **63 → 60**. The worst regression worsens **-0.250 → -0.450 s**, so r156 is not promoted despite rejecting more transient actions.

## Authority and timing contract

- Python owns retained MuJoCo experiment sequencing. Rust owns each WBC query, fixed-knot scoring, the r155 signed-proposal phase, the r156 confirmation transition, the final command solve, request activation hysteresis, maximum age, per-tick slew, exact-evidence revocation, and allocation witnesses.
- The r156 Rust confirmation layer requires **2** consecutive, directionally aligned improving proposals under the same exact raw support mask. Shadow proposals cannot reach request supervision; evidence loss, support change, failed update, expiry, invalid input, or reordered sequence revokes execution immediately.
- The r154 planner performs one baseline query when inactive or four exact WBC queries when searching: baseline plus one three-point local coordinate question. Rust scores each candidate at eight fixed future knots, and work advances every control tick rather than arriving as a 40-query burst.
- The r154 score retains roll/lateral capture, pitch-path non-regression, yaw, single-support closing direction, actuator utilization, joint headroom, action magnitude, and request delta as separate evidence. A 40/40/20 one-tick trust region prevents the lattice-edge/execution mismatch seen in r149.
- A nonzero request is not a command. The downstream WBC re-solves every control tick against the current measured hard-contact mask; rejected results cannot become fresh torque.
- Candidate Python GC collections: **0**; summed within-run RSS delta: **1.60 MiB**; loop overruns: **60**.
- Worst per-case p99 split: planner queries **6025.2 µs**; final current-support WBC **6044.2 µs**. These clocks are measured separately inside the Rust query boundary; their sum is the controller solve work, not Python loop overhead.

## Scope

This plant A/B is downstream of the policy- and physics-free r146/r147 frozen-state evaluations. A green mechanism gate proves bounded execution semantics, not universal recovery. Live promotion additionally requires every physical regression gate; otherwise the planner remains evaluation-only and the retained controller is unchanged.
