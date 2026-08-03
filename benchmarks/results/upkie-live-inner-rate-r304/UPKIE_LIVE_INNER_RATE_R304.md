# Upkie live inner-rate evaluation — R304

Status: **FAST EVALUATION PASS; public 250/50 cadence unchanged; lateral recovery remains unpromoted.**

## Outcome

The browser still receives 50 Hz state frames, while this evaluation profile
contains five
250 Hz Rust WBC ticks and each WBC tick contains four 1 kHz MuJoCo substeps. With
the Rust-balanced initialization also used as the nominal joint target, the live
plant completes 20.000 s in
bilateral support. The frozen 50 Hz WBC / 250 Hz plant baseline reaches its fall
boundary after 1.860 s.

This records a bounded execution experiment, not a public cadence promotion or
recovery policy. The same 8 N
lateral pull still loses bilateral support at stream tick
34 and reaches a
fall boundary at tick 56.

## Rate contract

| path | browser stream | Rust WBC | MuJoCo | substeps / WBC |
|---|---:|---:|---:|---:|
| frozen baseline | 50 Hz | 50 Hz | 250 Hz | 5 |
| fast evaluation R304 | 50 Hz | 250 Hz | 1000 Hz | 4 |

## Nominal 20-second hold

| measure | result |
|---|---:|
| completed ticks | 1000 |
| measured contact patterns | 11 |
| minimum root height | 0.539310 m |
| maximum root tilt | 0.005983 rad |
| maximum hard residual | 2.197e-11 |
| WBC status counts | `{"Solved": 1000}` |
| Rust allocations / bytes | 0 / 0 |
| controller p50 / p95 / p99 | 135.291 / 145.367 / 162.000 us |
| controller adjacent jitter p99 | 33.351 us |
| worker p50 / p95 / p99 | 7045.952 / 7294.837 / 7592.668 us |
| worker adjacent jitter p99 | 611.794 us |

## Frozen baseline consequence

The baseline completes 93 stream ticks, loses bilateral wheel
support at tick 65, and reaches
`fall` at tick 92. Its peak tilt is
0.806599 rad and minimum root height is
0.295726 m. This comparison isolates execution rate;
the model, WBC formulation, and 50 Hz observation/visualization cadence are unchanged.

## Disturbance holdout

The 8 N lateral wrench runs for 0.200 s. It first loses
bilateral support at tick 34, reaches flight at
tick 34, and terminates with
`fall` at tick 56.
MaxIterations occurs at stream ticks
`[25, 33]`; those outputs are non-admitted.
The hot Rust solve still reports 0 allocations.

## Fast-profile evaluation gates

- PASS `fast_eval_rate_contract_is_50_250_1000`
- PASS `fast_eval_uses_explicit_balanced_nominal_joint_target`
- PASS `legacy_50hz_control_reaches_fall_boundary`
- PASS `fast_nominal_completes_horizon`
- PASS `fast_nominal_stays_bilateral`
- PASS `fast_nominal_stays_upright`
- PASS `fast_nominal_all_solved_and_admitted`
- PASS `fast_nominal_hard_residuals_under_1e8`
- PASS `fast_nominal_rust_hot_loop_zero_allocations`
- PASS `fast_nominal_controller_p99_under_1ms`
- PASS `fast_nominal_worker_p99_under_20ms_stream_budget`
- PASS `disturbance_trace_is_finite_and_fail_closed`

## Deliberately open recovery gates

- OPEN `eight_newton_lateral_pull_recovers`
- OPEN `eight_newton_lateral_pull_has_no_max_iterations`
- OPEN `eight_newton_lateral_pull_stays_within_20ms_stream_budget`

## Authority boundary

R304 is evaluation-only: it exercises a faster inner cadence and an explicit balanced nominal target without changing the public 250/50 worker or promoting disturbance-recovery authority. Python declares the immutable cases, aggregates
metrics, and renders this report. The WBC and actuator solve remain Rust-owned.
Full per-tick state summaries are retained in `traces.json` beside `metrics.json`.
