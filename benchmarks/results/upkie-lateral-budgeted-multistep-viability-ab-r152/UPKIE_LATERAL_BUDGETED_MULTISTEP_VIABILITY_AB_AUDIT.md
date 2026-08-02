# Bonesaw lateral-authority budgeted multistep viability plant A/B · r152

> Mechanism evaluation **PASS**. Live deployment **REJECTED**. The candidate is never promoted when any physical gate fails.

## Outcome

The r146 one-shot maximum-pressure score is replaced by an eight-knot Rust forecast. Acceleration is held for 60 ms, then coasted through a 240 ms horizon. Capture, terminal rate, yaw, support, torque/joint resource, action magnitude, and action change remain separate witnesses. Python schedules one three-point coordinate poll per tick; including the baseline, no tick performs more than four planner WBC queries, and the final current-support WBC solve remains mandatory.

Only current roll/lateral capture pressure may wake the planner. Sagittal, support-mask, rate, yaw, resource, action, and action-change pressures remain separate visible proposal vetoes; they are not wake-up authority.

Green regressions: **none**. New falls: **none**. Earlier adverse boundaries: **['diagonal_4n']**. Newly recovered adverse rows: **none**.

| case | r137 | candidate | Δ s | queries | max/tick | dominant | p99 µs | replay |
|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 319.3 | YES |
| forward_2n | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 365.8 | YES |
| forward_4n_reference | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 293.5 | YES |
| forward_6n_overload | FALL 4.820s | FALL 4.860s | +0.040 | 1107 | 4 | peak_sagittal | 20158.1 | YES |
| backward_2n | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 292.1 | YES |
| backward_4n | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 296.9 | YES |
| left_1n | FALL 2.470s | FALL 2.490s | +0.020 | 657 | 4 | peak_sagittal | 10128.7 | YES |
| left_2n | FALL 2.350s | FALL 2.350s | +0.000 | 551 | 4 | peak_sagittal | 14700.6 | YES |
| left_4n | FALL 2.610s | FALL 2.610s | +0.000 | 747 | 4 | peak_sagittal | 16622.6 | YES |
| right_2n | FALL 2.015s | FALL 2.015s | +0.000 | 427 | 4 | peak_sagittal | 8768.5 | YES |
| right_4n | FALL 2.415s | FALL 2.415s | +0.000 | 744 | 4 | peak_capture | 9851.0 | YES |
| diagonal_4n | FALL 2.125s | FALL 2.040s | -0.085 | 723 | 4 | peak_sagittal | 18514.3 | YES |
| up_4n | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 361.2 | YES |
| down_4n | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 304.5 | YES |
| handle_forward_4n | FALL 4.630s | FALL 4.715s | +0.085 | 1162 | 4 | peak_sagittal | 9866.4 | YES |
| short_8n_50ms | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 293.7 | YES |
| long_2n_200ms | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 366.8 | YES |
| forward_2n_three_pulses | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 304.6 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | +0.000 | 1200 | 1 | peak_sagittal | 309.4 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 1.610s | +0.000 | 322 | 1 | peak_sagittal | 10172.9 | YES |

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
| every_green_qualification_preserved | PASS |
| no_new_fall | PASS |
| no_earlier_adverse_boundary | FAIL |
| at_least_one_new_adverse_recovery | FAIL |
| candidate_p99_inside_5ms | FAIL |

## Interpretation

- This is a reduced-order forecast scorer, not a learned policy, physics engine, or proof that future contact follows the forecast. MuJoCo is used only as the external consequence gate.
- Request lifetime, exact evidence, slew, reduced-support admission, and final WBC verification remain the independent r148/r150 authority boundaries.
- Each pressure component is retained independently so a capture, support, joint/torque resource, yaw, rate, or action limit can be displayed without hiding it in the total score.
- Total exact planner queries: **19640**; >5 ms controller ticks: **660**; global-request-edge ticks: **0**; one-tick trust-region-edge ticks: **19**.
