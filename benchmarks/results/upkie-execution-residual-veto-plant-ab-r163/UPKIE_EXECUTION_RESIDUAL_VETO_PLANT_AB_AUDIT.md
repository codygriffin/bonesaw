# Bonesaw execution-residual veto plant A/B · r163

> Causal physical gate **FAIL** · live authority promotion **NO**.

## Result

The control and candidate run the same r158 hybrid-confirmed planner. The candidate adds only r162's confidence-lowering rule: an execution residual that exceeds the pre-update rolling envelope makes the current proposal unavailable before confirmation/request supervision. It cannot create a request, reuse rejected torque, or alter the retained r137 fallback.

The candidate reports **70** exceedances / **70** veto ticks and changes later trajectory-dependent executable-request exposure **7 → 26 ticks**, while executable-request overlap on the vetoed tick itself remains **0**. Green rows lost: **0**. Earlier/later/neutral first-boundary rows: **0/2/18**. Worst/best boundary delta: **+0.000/+0.785 s**. Timed Rust monitor allocation: **0 calls / 0 bytes**; 5 ms loop overruns: **692**.

## Retained causal matrix

| case | control | veto | boundary Δs | control exec | veto exec | veto ticks | replay |
|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 0 | YES |
| forward_2n | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 1 | YES |
| forward_4n_reference | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 2 | YES |
| forward_6n_overload | FALL 2.880s | FALL 3.665s | +0.785 | 2 | 20 | 6 | YES |
| backward_2n | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 1 | YES |
| backward_4n | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 2 | YES |
| left_1n | FALL 2.465s | FALL 2.465s | +0.000 | 3 | 3 | 13 | YES |
| left_2n | FALL 2.350s | FALL 2.350s | +0.000 | 0 | 0 | 9 | YES |
| left_4n | FALL 2.610s | FALL 2.610s | +0.000 | 0 | 0 | 4 | YES |
| right_2n | FALL 2.015s | FALL 2.015s | +0.000 | 0 | 0 | 7 | YES |
| right_4n | FALL 2.415s | FALL 2.415s | +0.000 | 0 | 0 | 6 | YES |
| diagonal_4n | FALL 2.125s | FALL 2.125s | +0.000 | 0 | 0 | 3 | YES |
| up_4n | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 1 | YES |
| down_4n | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 1 | YES |
| handle_forward_4n | FALL 2.525s | FALL 2.920s | +0.395 | 2 | 3 | 7 | YES |
| short_8n_50ms | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 1 | YES |
| long_2n_200ms | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 1 | YES |
| forward_2n_three_pulses | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 3 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 1 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 1.610s | +0.000 | 0 | 0 | 1 | YES |

## Gates

| gate | result |
|---|---|
| retained_matrix_complete | PASS |
| exact_candidate_replay | PASS |
| residual_exceedance_exercised | PASS |
| veto_path_exercised | PASS |
| vetoed_tick_never_executes_request | PASS |
| rust_monitor_zero_allocation | PASS |
| all_control_green_rows_preserved | PASS |
| no_earlier_fall_boundary | PASS |
| five_ms_control_deadline | FAIL |

## Authority boundary

Even a physical pass would admit only a fail-lowering veto mechanism. It would not prove the planner action reachable or promote r158. Live use remains withheld until deterministic deadline and independently solved support/contingency action gates pass.
