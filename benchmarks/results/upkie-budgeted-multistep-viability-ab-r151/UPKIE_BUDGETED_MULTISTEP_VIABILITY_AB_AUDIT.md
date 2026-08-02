# Bonesaw fixed-budget multistep viability plant A/B · r151

> Mechanism evaluation **PASS**. Live deployment **REJECTED**. The candidate is never promoted when any physical gate fails.

## Outcome

The r146 one-shot maximum-pressure score is replaced by an eight-knot Rust forecast. Acceleration is held for 60 ms, then coasted through a 240 ms horizon. Capture, terminal rate, yaw, support, torque/joint resource, action magnitude, and action change remain separate witnesses. Python schedules one three-point coordinate poll per tick; including the baseline, no tick performs more than four planner WBC queries, and the final current-support WBC solve remains mandatory.

Green regressions: **['backward_2n', 'backward_4n', 'forward_4n_reference', 'long_2n_200ms', 'short_8n_50ms']**. New falls: **['forward_4n_reference', 'backward_2n', 'backward_4n', 'short_8n_50ms', 'long_2n_200ms', 'forward_4n_friction_0p1']**. Earlier adverse boundaries: **['forward_6n_overload', 'left_1n', 'left_4n', 'handle_forward_4n', 'forward_4n_friction_0p1']**. Newly recovered adverse rows: **none**.

| case | r137 | candidate | Δ s | queries | max/tick | dominant | p99 µs | replay |
|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 304.1 | YES |
| forward_2n | RECOVERED | RECOVERED | +0.000 | 1740 | 4 | peak_sagittal | 663.8 | YES |
| forward_4n_reference | RECOVERED | FALL 2.940s | -3.060 | 1404 | 4 | peak_sagittal | 18808.0 | YES |
| forward_6n_overload | FALL 4.820s | FALL 3.940s | -0.880 | 2225 | 4 | peak_sagittal | 11921.3 | YES |
| backward_2n | RECOVERED | FALL 2.860s | -3.140 | 1052 | 4 | peak_sagittal | 14120.3 | YES |
| backward_4n | RECOVERED | FALL 1.835s | -4.165 | 724 | 4 | peak_sagittal | 16281.5 | YES |
| left_1n | FALL 2.470s | FALL 2.385s | -0.085 | 690 | 4 | peak_sagittal | 9627.6 | YES |
| left_2n | FALL 2.350s | FALL 2.520s | +0.170 | 867 | 4 | peak_sagittal | 19107.9 | YES |
| left_4n | FALL 2.610s | FALL 2.055s | -0.555 | 507 | 4 | peak_sagittal | 16513.9 | YES |
| right_2n | FALL 2.015s | FALL 2.410s | +0.395 | 593 | 4 | peak_sagittal | 10149.6 | YES |
| right_4n | FALL 2.415s | FALL 3.050s | +0.635 | 1237 | 4 | peak_capture | 9698.5 | YES |
| diagonal_4n | FALL 2.125s | FALL 2.610s | +0.485 | 882 | 4 | peak_sagittal | 16448.4 | YES |
| up_4n | RECOVERED | RECOVERED | +0.000 | 1329 | 4 | peak_sagittal | 672.6 | YES |
| down_4n | RECOVERED | RECOVERED | +0.000 | 1350 | 4 | peak_sagittal | 699.0 | YES |
| handle_forward_4n | FALL 4.630s | FALL 3.190s | -1.440 | 1424 | 4 | peak_sagittal | 16525.6 | YES |
| short_8n_50ms | RECOVERED | FALL 2.830s | -3.170 | 1286 | 4 | peak_sagittal | 16809.9 | YES |
| long_2n_200ms | RECOVERED | FALL 2.780s | -3.220 | 1306 | 4 | peak_sagittal | 9442.5 | YES |
| forward_2n_three_pulses | RECOVERED | RECOVERED | +0.000 | 2733 | 4 | peak_sagittal | 700.3 | YES |
| forward_4n_friction_0p1 | RECOVERED | FALL 1.640s | -4.360 | 583 | 4 | peak_sagittal | 9462.3 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 1.610s | +0.000 | 478 | 4 | peak_sagittal | 9592.5 | YES |

## Mechanism gates

| gate | result |
|---|---|
| complete_frozen_matrix | PASS |
| current_live_reproduced | PASS |
| fixed_query_budget_at_most_four | PASS |
| multistep_pressure_stack_finite | PASS |
| candidate_exact_replay | PASS |
| zero_timed_rust_allocation | PASS |
| zero_python_gc | PASS |
| finite_and_numeric_clean | PASS |

## Physical gates

| gate | result |
|---|---|
| every_green_qualification_preserved | FAIL |
| no_new_fall | FAIL |
| no_earlier_adverse_boundary | FAIL |
| at_least_one_new_adverse_recovery | FAIL |
| candidate_p99_inside_5ms | FAIL |

## Interpretation

- This is a reduced-order forecast scorer, not a learned policy, physics engine, or proof that future contact follows the forecast. MuJoCo is used only as the external consequence gate.
- Request lifetime, exact evidence, slew, reduced-support admission, and final WBC verification remain the independent r148/r150 authority boundaries.
- Each pressure component is retained independently so a capture, support, joint/torque resource, yaw, rate, or action limit can be displayed without hiding it in the total score.
- Total exact planner queries: **23610**; >5 ms controller ticks: **1044**; request-edge ticks: **0**.
