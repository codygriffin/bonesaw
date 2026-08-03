# Upkie measured landing phase boundary — R312

Status: **QUALIFIED DEFAULT-OFF EXPERIMENT**; recovery **RETAINED AS NEGATIVE EVIDENCE**.

R312 runs the Rust-owned phase-aware measured-landing candidate at the public
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
| measured-landing p99 / max µs | 3.672 / 10.840 | 3.768 / 3.847 |
| measured-landing allocations / bytes | 0 / 0 | 0 / 0 |
| min root height / max tilt | 0.53932 m / 0.00002 rad | 0.29209 m / 0.79167 rad |

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
- PASS `controller_and_worker_deadlines_hold`
- PASS `mechanism_replay_exact`
- PASS `r311_public_composition_harness_valid`

## Recovery promotion

- OPEN `disturbed_completes_requested_horizon`
- OPEN `disturbed_has_ten_tick_bilateral_upright_tail`
- OPEN `disturbed_never_enters_body_ground_stall`
- PASS `disturbed_has_no_nonadmitted_or_max_iterations_ticks`
- PASS `disturbed_hard_residuals_under_1e8`

## Explicit negative evidence

- PASS `candidate_disturbed_negative_recovery_is_explicit`
- PASS `baseline_and_candidate_both_expose_same_boundary_class`

## R311 public-profile composition

The retained R310 controller profile is evaluated on repeated upper-base pulls
at a 250 mm lever. This is the physical promotion holdout; the earlier table is
only the frozen mechanism-isolation trace.

| force Y N | baseline ticks / terminal | R312 ticks / terminal | terminal Δ ticks | active ticks | qualified tick |
|---:|---:|---:|---:|---:|---:|
| -8 | 75 / fall | 80 / fall | 5 | 22 | — |
| -6 | 165 / fall | 85 / fall | -80 | 25 | — |
| -4 | 450 / — | 450 / — | — | 0 | — |
| -2 | 450 / — | 450 / — | — | 0 | — |
| +2 | 450 / — | 450 / — | — | 0 | — |
| +4 | 450 / — | 450 / — | — | 0 | — |
| +6 | 88 / fall | 84 / fall | -4 | 21 | — |
| +8 | 70 / fall | 70 / fall | 0 | 13 | — |

Baseline/candidate falls: **4 / 4**.
Activated/force-qualified cases: **4 / 0**.

### Composition harness

- PASS `r311_upper_repeated_holdout_has_both_signs`
- PASS `baseline_reproduces_terminal_boundary`
- PASS `landing_activates_only_after_measured_loss`
- PASS `mode_firewall_holds_on_public_composition`
- PASS `finite_zero_allocation_landing_boundary`
- PASS `controller_and_worker_deadlines_hold`
- PASS `representative_replay_exact`

### Recovery promotion

- OPEN `candidate_eliminates_every_terminal_fall`
- OPEN `candidate_reduces_terminal_fall_count`
- OPEN `candidate_never_moves_terminal_boundary_earlier`
- OPEN `every_activated_case_requalifies_bilateral_contact`
- OPEN `every_case_finishes_requested_horizon`

## Architectural conclusion

R312 is a causal, phase-aware, force-backed landing boundary, but not a recovery controller. The isolated mechanism is default-off, causal, deterministic, and allocation-free. On the R311 upper-body moment holdout it does not reduce fall count, never reaches force-backed bilateral requalification, and moves at least one terminal boundary earlier. This negative physical composition is retained without reset masking or promotion.
