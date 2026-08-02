# Bonesaw hybrid-support guarded measured-contact plant A/B · r158

> Mechanism **PASS** · physical consequence **FAIL** · live promotion **NO**.

## Causal comparison

Each row has four deterministic runs: the retained r137 assumed-contact sentinel, a measured-contact control with no planner, the supervised planner candidate, and an exact candidate replay. The physical verdict compares candidate against the measured-contact control so contact-observer consequences are not credited to or blamed on the planner. The r137 arm remains visible as the current-behavior reference.

| case | r137 | measured control | candidate | searches | guard admit | guard reject | shadow ticks | confirmed ticks | active ticks | max r/l/y | loop p99 µs | planner p99 µs | final WBC p99 µs | >5 ms | replay |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | FALL 3.395s | FALL 3.405s | 38 | 5 | 1 | 2 | 3 | 77 | 40/0/20 | 1141 | 558 | 154 | 4 | YES |
| forward_2n | RECOVERED | FALL 3.680s | FALL 3.680s | 213 | 6 | 0 | 2 | 4 | 237 | 40/0/20 | 1110 | 565 | 157 | 2 | YES |
| forward_4n_reference | RECOVERED | FALL 2.865s | FALL 2.865s | 271 | 1 | 0 | 1 | 0 | 350 | 0/0/0 | 4914 | 2589 | 1301 | 6 | YES |
| forward_6n_overload | FALL 4.820s | FALL 1.230s | FALL 1.230s | 33 | 2 | 0 | 1 | 1 | 44 | 0/0/20 | 1118 | 555 | 162 | 0 | YES |
| backward_2n | RECOVERED | FALL 3.420s | FALL 3.420s | 209 | 0 | 0 | 0 | 0 | 255 | 0/0/0 | 1161 | 558 | 169 | 3 | YES |
| backward_4n | RECOVERED | FALL 2.730s | FALL 2.730s | 284 | 1 | 0 | 1 | 0 | 304 | 0/0/0 | 1299 | 612 | 162 | 2 | YES |
| left_1n | FALL 2.470s | FALL 2.295s | FALL 2.295s | 94 | 0 | 0 | 0 | 0 | 150 | 0/0/0 | 12787 | 6139 | 6149 | 8 | YES |
| left_2n | FALL 2.350s | FALL 2.535s | FALL 2.630s | 75 | 22 | 0 | 4 | 18 | 126 | 40/0/20 | 1235 | 567 | 175 | 4 | YES |
| left_4n | FALL 2.610s | FALL 1.850s | FALL 1.850s | 96 | 0 | 0 | 0 | 0 | 137 | 0/0/0 | 1139 | 568 | 153 | 1 | YES |
| right_2n | FALL 2.015s | FALL 2.030s | FALL 2.030s | 38 | 0 | 0 | 0 | 0 | 86 | 0/0/0 | 1061 | 504 | 169 | 4 | YES |
| right_4n | FALL 2.415s | FALL 2.195s | FALL 2.195s | 146 | 0 | 2 | 0 | 0 | 183 | 0/0/0 | 1635 | 792 | 208 | 4 | YES |
| diagonal_4n | FALL 2.125s | FALL 2.605s | FALL 2.605s | 238 | 1 | 2 | 1 | 0 | 294 | 0/0/0 | 1217 | 588 | 159 | 2 | YES |
| up_4n | RECOVERED | FALL 4.215s | FALL 4.215s | 180 | 1 | 11 | 3 | 6 | 223 | 0/0/0 | 1157 | 573 | 151 | 1 | YES |
| down_4n | RECOVERED | FALL 3.450s | FALL 3.450s | 85 | 0 | 1 | 0 | 0 | 123 | 0/0/0 | 2370 | 1172 | 736 | 7 | YES |
| handle_forward_4n | FALL 4.630s | FALL 2.045s | FALL 2.045s | 168 | 1 | 0 | 1 | 0 | 207 | 0/0/0 | 1149 | 574 | 156 | 1 | YES |
| short_8n_50ms | RECOVERED | FALL 1.785s | FALL 1.785s | 121 | 2 | 0 | 2 | 0 | 155 | 0/0/0 | 1195 | 577 | 169 | 1 | YES |
| long_2n_200ms | RECOVERED | FALL 3.155s | FALL 3.155s | 319 | 0 | 0 | 0 | 0 | 381 | 0/0/0 | 1222 | 610 | 164 | 4 | YES |
| forward_2n_three_pulses | RECOVERED | FALL 3.625s | FALL 3.625s | 392 | 0 | 0 | 0 | 0 | 450 | 0/0/0 | 1183 | 613 | 156 | 4 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | RECOVERED | 212 | 0 | 0 | 0 | 0 | 216 | 0/0/0 | 1113 | 541 | 148 | 0 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.715s | FALL 0.715s | 23 | 0 | 0 | 0 | 0 | 65 | 0/0/0 | 1092 | 559 | 176 | 0 | YES |

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
| hybrid_guard_admission_exercised | PASS |
| hybrid_guard_rejection_exercised | PASS |
| support_discontinuity_prevention_exercised | PASS |
| hybrid_guard_shadow_only_exercised | PASS |
| request_only_after_hybrid_guard | PASS |

## Physical gates

| gate | result |
|---|---|
| no_new_numeric_fault | PASS |
| no_new_fall | PASS |
| no_earlier_fall_boundary | PASS |
| green_control_qualification_preserved | PASS |
| no_loop_deadline_overrun | FAIL |

## Consequence deltas

- Earlier falling boundaries: **0/19**; later: **2/19**; neutral: **17/19**.
- Worst boundary delta: **+0.000 s** (forward_2n); best: **+0.095 s** (left_2n); median: **+0.000 s**.
- Candidate-minus-control mean / worst increases: maximum_translation_m +0.002036 / +0.04119; maximum_tilt_deg +0.1749 / +4.711; maximum_torque_utilization +0.005278 / +0.1056; final_station_error_m -0.002416 / +0.001551.
- Against retained r149: earlier boundaries fall from **11/19** to **0/19**; worst regression contracts from **-1.490 s** to **+0.000 s**; >5 ms loop events fall from **395** to **58**. Total exact queries are **20,890** versus **19,649**, so the win is burst size and jitter, not less aggregate work.
- Against direct predecessor upkie-confirmed-multistep-measured-contact-plant-ab-r156, earlier boundaries change **2 → 0**, later boundaries **2 → 2**, neutral boundaries **15 → 17**, exact planner queries **20,517 → 20,890**, and >5 ms loops **60 → 58**. Worst boundary delta changes **-0.450 → +0.000 s**.

## Authority and timing contract

- Python owns retained MuJoCo experiment sequencing. Rust owns each WBC query, fixed-knot scoring, the r155 signed-proposal phase, the r156 confirmation transition, the r158 hybrid-support guard, the final command solve, request activation hysteresis, maximum age, per-tick slew, exact-evidence revocation, and allocation witnesses.
- The r158 guard requires the established three exact raw-support samples used by hard contact eligibility and at least 5% candidate-WBC load on each wheel before double-support execution. A single-support roll request that opens the missing wheel may advance the bounded confirmation shadow but never receives physical execution authority; age, load, evidence, input, and sequence rejections cannot seed or refresh that shadow.
- The r156 Rust confirmation layer requires **2** consecutive, directionally aligned improving proposals under the same exact raw support mask. Shadow proposals cannot reach request supervision; evidence loss, support change, failed update, expiry, invalid input, or reordered sequence revokes execution immediately.
- The r154 planner performs one baseline query when inactive or four exact WBC queries when searching: baseline plus one three-point local coordinate question. Rust scores each candidate at eight fixed future knots, and work advances every control tick rather than arriving as a 40-query burst.
- The r154 score retains roll/lateral capture, pitch-path non-regression, yaw, single-support closing direction, actuator utilization, joint headroom, action magnitude, and request delta as separate evidence. A 40/40/20 one-tick trust region prevents the lattice-edge/execution mismatch seen in r149.
- A nonzero request is not a command. The downstream WBC re-solves every control tick against the current measured hard-contact mask; rejected results cannot become fresh torque.
- Candidate Python GC collections: **0**; summed within-run RSS delta: **4.64 MiB**; loop overruns: **58**.
- Worst per-case p99 split: planner queries **6138.6 µs**; final current-support WBC **6149.0 µs**. These clocks are measured separately inside the Rust query boundary; their sum is the controller solve work, not Python loop overhead.

## Scope

This plant A/B is downstream of the policy- and physics-free r146/r147 frozen-state evaluations. A green mechanism gate proves bounded execution semantics, not universal recovery. Live promotion additionally requires every physical regression gate; otherwise the planner remains evaluation-only and the retained controller is unchanged.
