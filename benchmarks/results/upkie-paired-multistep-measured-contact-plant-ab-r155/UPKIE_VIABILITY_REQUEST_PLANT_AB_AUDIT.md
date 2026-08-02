# Bonesaw paired-query multi-step measured-contact plant A/B · r155

> Mechanism **PASS** · physical consequence **FAIL** · live promotion **NO**.

## Causal comparison

Each row has four deterministic runs: the retained r137 assumed-contact sentinel, a measured-contact control with no planner, the supervised planner candidate, and an exact candidate replay. The physical verdict compares candidate against the measured-contact control so contact-observer consequences are not credited to or blamed on the planner. The r137 arm remains visible as the current-behavior reference.

| case | r137 | measured control | candidate | searches | active ticks | max r/l/y | loop p99 µs | planner p99 µs | final WBC p99 µs | >5 ms | replay |
|---|---|---|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | FALL 3.395s | FALL 3.985s | 110 | 171 | 40/40/20 | 851 | 293 | 160 | 6 | YES |
| forward_2n | RECOVERED | FALL 3.680s | FALL 3.675s | 25 | 46 | 40/0/0 | 749 | 250 | 147 | 2 | YES |
| forward_4n_reference | RECOVERED | FALL 2.865s | FALL 2.920s | 73 | 140 | 40/40/20 | 12820 | 6320 | 5221 | 8 | YES |
| forward_6n_overload | FALL 4.820s | FALL 1.230s | FALL 1.230s | 4 | 11 | 0/0/0 | 748 | 235 | 147 | 0 | YES |
| backward_2n | RECOVERED | FALL 3.420s | FALL 3.475s | 46 | 81 | 40/40/20 | 841 | 281 | 148 | 3 | YES |
| backward_4n | RECOVERED | FALL 2.730s | FALL 2.690s | 20 | 38 | 40/0/0 | 761 | 262 | 157 | 2 | YES |
| left_1n | FALL 2.470s | FALL 2.295s | FALL 2.300s | 75 | 129 | 40/0/20 | 12251 | 5950 | 5970 | 6 | YES |
| left_2n | FALL 2.350s | FALL 2.535s | FALL 2.695s | 75 | 123 | 40/40/20 | 12424 | 6100 | 5984 | 9 | YES |
| left_4n | FALL 2.610s | FALL 1.850s | FALL 1.850s | 89 | 123 | 0/0/0 | 808 | 293 | 151 | 1 | YES |
| right_2n | FALL 2.015s | FALL 2.030s | FALL 2.030s | 28 | 76 | 0/0/0 | 819 | 259 | 169 | 4 | YES |
| right_4n | FALL 2.415s | FALL 2.195s | FALL 2.015s | 113 | 138 | 40/40/0 | 1046 | 401 | 232 | 4 | YES |
| diagonal_4n | FALL 2.125s | FALL 2.605s | FALL 2.605s | 41 | 90 | 0/0/0 | 821 | 299 | 148 | 2 | YES |
| up_4n | RECOVERED | FALL 4.215s | FALL 3.900s | 60 | 102 | 40/0/0 | 1057 | 348 | 175 | 0 | YES |
| down_4n | RECOVERED | FALL 3.450s | FALL 3.495s | 24 | 67 | 40/40/20 | 827 | 282 | 154 | 4 | YES |
| handle_forward_4n | FALL 4.630s | FALL 2.045s | FALL 2.045s | 42 | 68 | 0/0/0 | 821 | 276 | 152 | 1 | YES |
| short_8n_50ms | RECOVERED | FALL 1.785s | FALL 1.785s | 51 | 72 | 0/0/0 | 827 | 295 | 158 | 1 | YES |
| long_2n_200ms | RECOVERED | FALL 3.155s | FALL 3.155s | 13 | 67 | 0/0/0 | 785 | 275 | 149 | 4 | YES |
| forward_2n_three_pulses | RECOVERED | FALL 3.625s | FALL 3.625s | 7 | 63 | 0/0/0 | 717 | 228 | 153 | 4 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | RECOVERED | 0 | 0 | 0/0/0 | 671 | 150 | 143 | 0 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.715s | FALL 0.710s | 7 | 44 | 40/0/0 | 771 | 257 | 151 | 0 | YES |

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

- Earlier falling boundaries: **5/19**; later: **6/19**; neutral: **8/19**.
- Worst boundary delta: **-0.315 s** (up_4n); best: **+0.590 s** (nominal); median: **+0.000 s**.
- Candidate-minus-control mean / worst increases: maximum_translation_m +0.005295 / +0.4001; maximum_tilt_deg +0.4204 / +36.21; maximum_torque_utilization +0.02149 / +0.2334; final_station_error_m +0.0009231 / +0.4846.
- Against retained r149: earlier boundaries fall from **11/19** to **5/19**; worst regression contracts from **-1.490 s** to **-0.315 s**; >5 ms loop events fall from **395** to **61**. Total exact queries are **12,140** versus **19,649**, so the win is burst size and jitter, not less aggregate work.

## Authority and timing contract

- Python owns the bounded coordinate-search schedule and retained MuJoCo experiment only. Rust owns each WBC query, the final command solve, request activation hysteresis, maximum age, per-tick slew, exact-evidence revocation, and allocation witness.
- The r155 planner performs one baseline query when inactive or two exact WBC queries when searching: the same-state zero baseline plus one Rust-scheduled signed proposal. The final current-support command WBC remains a separate full-budget solve.
- The r155 proposal phase and clipping witness are fixed-size Rust state. Only current roll/lateral capture wakes it; pitch-path non-regression and all other forecast pressures remain independent proposal vetoes.
- A nonzero request is not a command. The downstream WBC re-solves every control tick against the current measured hard-contact mask; rejected results cannot become fresh torque.
- Candidate Python GC collections: **0**; summed within-run RSS delta: **2.23 MiB**; loop overruns: **61**.
- Worst per-case p99 split: planner queries **6319.8 µs**; final current-support WBC **5983.6 µs**. These clocks are measured separately inside the Rust query boundary; their sum is the controller solve work, not Python loop overhead.

## Scope

This plant A/B is downstream of the policy- and physics-free r146/r147 frozen-state evaluations. A green mechanism gate proves bounded execution semantics, not universal recovery. Live promotion additionally requires every physical regression gate; otherwise the planner remains evaluation-only and the retained controller is unchanged.
