# Upkie live dynamic measured-contact transition · R300

> Fixture PASS · controller behavior REJECTED · actual 250 Hz MuJoCo integration / 50 Hz Rust WBC · authority closed.

The control and candidate use the same live worker, Upkie reference capture composition, persistent Rust WBC, model, and initial state. Each 50 Hz solve consumes the final completed sample of the prior five-frame 250 Hz contact window; the fixture checks the public frame indices and loss/gain edges. The candidate alone receives an 8 N lateral world-frame wrench at the measured base COM for 200 ms. There is no learned policy. The run stops on the first fall boundary before the worker can consume its pending automatic reset.

| metric | zero-wrench control | 8 N lateral candidate |
|---|---:|---:|
| duration s | 1.060 | 1.060 |
| contact patterns | 11 | 00, 01, 10, 11 |
| first non-double / flight tick | — / — | 32 / 36 |
| terminal boundary | — | fall |
| supported-upright recovery tick (10-tick dwell) | 35 | — |
| longest low-body / no-wheel-contact stall ticks | 0 | 2 |
| root position RMS m | 0.00168 | 0.04297 |
| CoM position RMS m | 0.00044 | 0.02558 |
| max lateral displacement m | 0.00034 | 0.06432 |
| paired root / CoM trace delta RMS m | — | 0.04286 / 0.02552 |
| paired actuator-effort delta RMS N·m | — | 0.75425 |
| max tilt rad | 0.03291 | 0.62450 |
| min height m | 0.53905 | 0.29304 |
| controller p50 / p99 / max µs | 132.2 / 147.9 / 149.1 | 132.4 / 5352.8 / 5408.4 |
| controller adjacent jitter p50 / p99 / max µs | 3.0 / 21.1 / 22.3 | 6.1 / 5288.3 / 5290.3 |
| controller calls >5 ms | 0 | 2 |
| worker p50 / p99 / max µs | 1410.0 / 1616.7 / 1687.0 | 1418.0 / 6615.9 / 6706.9 |
| worker adjacent jitter p50 / p99 / max µs | 12.6 / 179.0 / 228.9 | 25.9 / 5252.8 / 5316.0 |
| worker calls >20 ms | 0 | 0 |
| max constraint / dynamics / contact residual | 1.788e-11 / 1.788e-11 / 1.526e-12 | 2.911e-02 / 9.139e+01 / 1.290e+01 |
| MaxIterations ticks | — | [48, 50] |
| max controller / constraint tick | 50 / 8 | 48 / 50 |
| sampled ground impulse N·s | 55.930 | 82.369 |
| Rust allocation calls / bytes | 0 / 0 | 0 / 0 |

## Measured support transitions

| tick | observed | hard |
|---:|:---:|:---:|
| 0 | `11` | `00` |
| 2 | `11` | `11` |
| 32 | `10` | `10` |
| 33 | `01` | `01` |
| 36 | `00` | `00` |
| 38 | `01` | `00` |
| 40 | `01` | `01` |
| 41 | `00` | `00` |
| 46 | `01` | `00` |
| 48 | `01` | `01` |
| 49 | `00` | `00` |
| 50 | `01` | `01` |
| 51 | `00` | `00` |

## Fixture gates

| gate | result |
|---|:---:|
| live_rate_split_is_250_50 | PASS |
| control_stays_double_support | PASS |
| control_has_no_boundary | PASS |
| candidate_measures_all_contact_modes | PASS |
| candidate_reaches_single_then_flight | PASS |
| candidate_stops_before_automatic_reset | PASS |
| no_numeric_reset_or_warning | PASS |
| hard_rows_always_subset_measured_contact | PASS |
| wbc_boundary_consumes_prior_window | PASS |
| exact_contact_loss_removes_hard_row | PASS |
| external_wrench_is_bounded_and_ten_ticks | PASS |
| all_wbc_outputs_finite | PASS |
| rust_hot_loop_zero_allocations | PASS |
| max_iterations_are_nonadmitted | PASS |
| executable_hard_rows_require_admission | PASS |
| worker_p99_under_20ms | PASS |
| semantic_replay_exact | PASS |
| disturbance_has_visible_consequence | PASS |

## Controller behavior gates

| gate | result |
|---|:---:|
| controller_p99_under_5ms | FAIL |
| hard_residuals_under_1e8 | FAIL |
| candidate_recovers_supported_upright | FAIL |
| candidate_recovers_without_boundary | FAIL |

This is a discriminating consequence fixture, not a recovery pass. It proves that actual integrated contact loss crosses an explicit prior-window causality boundary and reaches the streamed authority layers without stale hard rows or reset masking. The controller still falls; the next behavior candidate must improve this same frozen trace without weakening any transport, residual, timing, or allocation gate. Rust allocation counters are in-process and exact; process RSS, Python GC, and hardware thermal/power telemetry are deliberately not claimed by this fixture and remain separate benchmark work.
