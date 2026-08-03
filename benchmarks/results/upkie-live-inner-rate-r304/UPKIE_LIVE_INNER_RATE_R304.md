# Upkie live inner-rate qualification — R304

Status: **QUALIFIED for nominal live execution; lateral recovery remains unpromoted.**

## Outcome

The browser still receives 50 Hz state frames, while each frame now contains five
250 Hz Rust WBC ticks and each WBC tick contains four 1 kHz MuJoCo substeps. With
the Rust-balanced initialization also used as the nominal joint target, the live
plant completes 20.000 s in
bilateral support. The frozen 50 Hz WBC / 250 Hz plant baseline reaches its fall
boundary after 1.860 s.

This promotes an execution architecture, not a recovery policy. The same 8 N
lateral pull still loses bilateral support at stream tick
34 and reaches a
fall boundary at tick 56.

## Rate contract

| path | browser stream | Rust WBC | MuJoCo | substeps / WBC |
|---|---:|---:|---:|---:|
| frozen baseline | 50 Hz | 50 Hz | 250 Hz | 5 |
| public R304 | 50 Hz | 250 Hz | 1000 Hz | 4 |

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
| controller p50 / p95 / p99 | 133.437 / 141.308 / 147.415 us |
| controller adjacent jitter p99 | 20.816 us |
| worker p50 / p95 / p99 | 6993.335 / 7134.167 / 7267.763 us |
| worker adjacent jitter p99 | 346.201 us |

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

## Qualification gates

- PASS `public_rate_contract_is_50_250_1000`
- PASS `public_uses_rust_balanced_nominal_joint_target`
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

R304 changes the public execution cadence and nominal joint target only. It does not add a Python policy or promote disturbance-recovery authority. Python declares the immutable cases, aggregates
metrics, and renders this report. The WBC and actuator solve remain Rust-owned.
Full per-tick state summaries are retained in `traces.json` beside `metrics.json`.
