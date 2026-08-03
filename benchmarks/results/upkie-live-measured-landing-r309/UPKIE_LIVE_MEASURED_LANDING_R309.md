# Upkie measured landing phase boundary — R309

Status: **QUALIFIED DEFAULT-OFF EXPERIMENT**; recovery **RETAINED AS NEGATIVE EVIDENCE**.

R309 runs the Rust-owned phase-aware measured-landing candidate at the public
250 Hz MuJoCo / 50 Hz WBC rate.  The receiver is primed from four measured
prestart samples only to avoid cold-start rejection; no contact authority is
authored by that option.  The candidate uses the frozen configuration
`[0.05, 90.0, 14.0, 20.0, 0.0001, 20.0, 30.0, 120.0, 0.2]` and remains default-off.

## Candidate outcome

| measure | nominal | disturbed |
|---|---:|---:|
| ticks / terminal boundary | 1000 / — | 54 / fall |
| observed contact patterns | 11 | 00, 01, 11 |
| first precontact / request tick | — | 33 / 33 |
| first force-backed qualification tick | — | None |
| contact-mode patterns | 33 | 22, 23, 33 |
| maximum request authority | 0.000000 | 1.000000 |
| max precontact acceleration norm | 0.000000 | 4.969295 m/s² |
| measured-landing p99 / max µs | 3.648 / 9.588 | 4.323 / 4.769 |
| measured-landing allocations / bytes | 0 / 0 | 0 / 0 |
| min root height / max tilt | 0.53932 m / 0.00002 rad | 0.29209 m / 0.79167 rad |

## Disturbed baseline comparison

| measure | candidate off | R309 candidate |
|---|---:|---:|
| terminal tick / boundary | 50 / fall | 53 / fall |
| MaxIterations ticks | [50] | — |
| max dynamics / contact residual | 1.308e+02 / 2.566e+01 | 4.642e-09 / 1.859e-09 |
| controller p99 µs | 2740.690 | 155.711 |

## Bounded-experiment qualification

- PASS `live_rate_split_is_250_50`
- PASS `nominal_completes_requested_horizon`
- PASS `nominal_stays_bilateral_upright`
- PASS `nominal_measured_landing_remains_dormant`
- PASS `candidate_default_is_off`
- PASS `measured_phase_activates_after_physics_loss`
- PASS `mode_firewall_never_promotes_unobserved_leg`
- PASS `causal_physics_window_and_exact_snapshot`
- PASS `finite_measured_landing_outputs`
- PASS `rust_measured_landing_hot_path_zero_allocations`
- PASS `measured_landing_p99_under_100us`
- PASS `wbc_hard_rows_subset_measured_contact`
- PASS `wbc_max_iterations_are_nonadmitted`
- PASS `all_wbc_outputs_finite`
- PASS `semantic_replay_exact`

## Recovery promotion

- OPEN `disturbed_completes_requested_horizon`
- OPEN `disturbed_has_ten_tick_bilateral_upright_tail`
- OPEN `disturbed_never_enters_body_ground_stall`
- PASS `disturbed_has_no_nonadmitted_or_max_iterations_ticks`
- PASS `disturbed_hard_residuals_under_1e8`

## Explicit negative evidence

- PASS `candidate_disturbed_negative_recovery_is_explicit`
- PASS `baseline_and_candidate_both_expose_same_boundary_class`

## Architectural conclusion

R309 is a causal, phase-aware, force-backed landing boundary. The nominal candidate remains dormant and allocation-free; the current 8 N disturbed trace is retained as negative recovery evidence when it falls, rather than allowing automatic reset to turn an incomplete recovery into a pass.
