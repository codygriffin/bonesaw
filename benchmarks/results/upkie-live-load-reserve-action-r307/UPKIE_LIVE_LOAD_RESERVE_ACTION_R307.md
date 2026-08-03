# Upkie measured load-reserve action — R307

Status: **RETAINED AS DEFAULT-OFF NEGATIVE EVIDENCE**

The Rust action is causal, bounded, allocation-free, and nominally dormant. It
activates at stream tick 25. The
baseline first loses bilateral support at tick 34
and reaches its fall boundary at tick 56; the candidate
first loses bilateral support at tick 53 and
reaches its fall boundary at tick 73.

That delay is not recovery. Promotion requires a 5.0 s
bilateral upright tail, no body-ground stall, no non-admitted solve, and no late
fall. The candidate fails those consequence gates and remains default-off.

## Measured action

| measure | result |
|---|---:|
| maximum authority | 1.000000 |
| minimum filtered weaker-wheel fraction | 0.260602 |
| maximum absolute lateral DCM | 0.043555 m |
| maximum commanded lateral acceleration | 3.000299 m/s² |
| Rust action p99 | 3.069 µs |
| Rust allocations / bytes | 0 / 0 |

## Bounded-experiment qualification

- PASS `nominal_completes_requested_horizon`
- PASS `nominal_stays_bilateral_upright`
- PASS `nominal_action_remains_dormant`
- PASS `rust_action_hot_path_zero_allocations`
- PASS `rust_action_p99_under_100us`
- PASS `disturbance_activates_before_first_support_loss`

## Recovery promotion

- OPEN `disturbed_completes_requested_horizon`
- OPEN `disturbed_has_five_second_bilateral_upright_tail`
- OPEN `disturbed_never_enters_body_ground_stall`
- OPEN `disturbed_has_no_nonadmitted_or_max_iterations_ticks`
- OPEN `disturbed_hard_residuals_under_1e8`

## Architectural conclusion

Measured load reserve is a useful causal trigger, but this root-task-only action does not achieve sustained wheel-supported recovery. The next controller chunk is contact-mode-aware reacquisition, not more timeout.
