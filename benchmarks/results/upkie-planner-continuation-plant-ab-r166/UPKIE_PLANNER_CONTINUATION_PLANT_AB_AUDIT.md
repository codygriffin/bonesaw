# Bonesaw bounded planner continuation plant A/B · r166

> Profile **FAIL** · controller promotion **NO**.

## Result

The candidate caps each speculative planner solve at **512 Dykstra sweeps**. If the exact prefix ends within **0.1** normalized violation, Rust resumes the same point and multipliers to the established bound; otherwise the proposal remains typed `MaxIterations` and has no authority. The executable WBC remains uncapped and dense/full-row on both arms.
Across **20** frozen plant cases, >5 ms loops change **58 → 58**. Plant execution is **exact**; the complete non-timing authority trace is **not exact**.

## Retained causal matrix

| case | control | candidate | overrun C | overrun B | planner p99 C µs | planner p99 B µs | authority exact | plant exact | replay exact |
|---|---|---|---|---|---|---|---|---|---|
| nominal | FALL 3.405s | FALL 3.405s | 4 | 4 | 556 | 554 | YES | YES | YES |
| forward_2n | FALL 3.680s | FALL 3.680s | 2 | 2 | 561 | 551 | YES | YES | YES |
| forward_4n_reference | FALL 2.865s | FALL 2.865s | 6 | 6 | 2489 | 1166 | NO | YES | YES |
| forward_6n_overload | FALL 1.230s | FALL 1.230s | 0 | 0 | 550 | 553 | YES | YES | YES |
| backward_2n | FALL 3.420s | FALL 3.420s | 3 | 3 | 551 | 555 | NO | YES | YES |
| backward_4n | FALL 2.730s | FALL 2.730s | 2 | 2 | 588 | 592 | NO | YES | YES |
| left_1n | FALL 2.295s | FALL 2.295s | 8 | 8 | 6008 | 1290 | YES | YES | YES |
| left_2n | FALL 2.630s | FALL 2.630s | 4 | 4 | 549 | 553 | NO | YES | YES |
| left_4n | FALL 1.850s | FALL 1.850s | 1 | 1 | 565 | 563 | YES | YES | YES |
| right_2n | FALL 2.030s | FALL 2.030s | 4 | 4 | 505 | 496 | YES | YES | YES |
| right_4n | FALL 2.195s | FALL 2.195s | 4 | 4 | 596 | 601 | NO | YES | YES |
| diagonal_4n | FALL 2.605s | FALL 2.605s | 2 | 2 | 602 | 595 | NO | YES | YES |
| up_4n | FALL 4.215s | FALL 4.215s | 1 | 1 | 586 | 594 | YES | YES | YES |
| down_4n | FALL 3.450s | FALL 3.450s | 7 | 7 | 1176 | 665 | NO | YES | YES |
| handle_forward_4n | FALL 2.045s | FALL 2.045s | 1 | 1 | 568 | 568 | NO | YES | YES |
| short_8n_50ms | FALL 1.785s | FALL 1.785s | 1 | 1 | 575 | 573 | NO | YES | YES |
| long_2n_200ms | FALL 3.155s | FALL 3.155s | 4 | 4 | 600 | 609 | YES | YES | YES |
| forward_2n_three_pulses | FALL 3.625s | FALL 3.625s | 4 | 4 | 764 | 605 | NO | YES | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | 0 | 0 | 538 | 591 | YES | YES | YES |
| forward_4n_friction_0p03 | FALL 0.715s | FALL 0.715s | 0 | 0 | 545 | 552 | YES | YES | YES |

## Gates

| gate | result |
|---|---|
| retained_matrix_complete | PASS |
| authority_trace_exact | FAIL |
| plant_execution_trace_exact | PASS |
| candidate_replay_exact | PASS |
| deadline_overruns_reduced | FAIL |
| zero_deadline_overruns | FAIL |
| zero_rust_allocation | PASS |
| zero_python_gc | PASS |
| finite | PASS |

## Authority boundary

Only speculative planner work is bounded. Budget exhaustion cannot create or refresh a request, and no candidate torque crosses this boundary. Promotion remains false until the timing profile passes the frozen matrix and actual-browser/hardware scheduling is separately established.
