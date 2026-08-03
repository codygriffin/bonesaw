# Upkie single-support touchdown request — R308

Status: **RETAINED AS DEFAULT-OFF NEGATIVE EVIDENCE**

R308 adds a Rust-owned free-leg request after an exact measured single-support
observation. It computes a wheel-height/velocity error and a damped Jacobian-
transpose joint acceleration, with fixed capacities and explicit authority
slew. The ordinary floating WBC still owns contact rows, effort limits, hard
residuals, and executable command authority.

## Measured request

| measure | result |
|---|---:|
| first measured single-support tick | 40 |
| first physics-window non-double/flight tick | 34 |
| first request-active tick | 40 |
| active ticks | 12 |
| maximum authority | 1.000000 |
| maximum vertical request | 2.774038 m/s² |
| maximum commanded vertical request | 2.774038 m/s² |
| Rust request p99 | 2.202 µs |
| Rust allocations / bytes | 0 / 0 |

## Bounded-experiment qualification

- PASS `nominal_completes_requested_horizon`
- PASS `nominal_stays_bilateral_upright`
- PASS `nominal_request_remains_dormant`
- PASS `rust_request_hot_path_zero_allocations`
- PASS `rust_request_p99_under_100us`
- PASS `request_activates_only_after_measured_single_support`

## Physical recovery promotion

- OPEN `disturbed_completes_requested_horizon`
- OPEN `disturbed_has_ten_tick_bilateral_upright_tail`
- PASS `disturbed_never_enters_body_ground_stall`
- OPEN `disturbed_has_no_nonadmitted_or_max_iterations_ticks`
- OPEN `disturbed_hard_residuals_under_1e8`

## Architectural conclusion

The request is causal, bounded, allocation-free, and passes the ordinary WBC boundary. The plant first leaves the bilateral physics window at tick `34`; the 50 Hz WBC observes/debounces single support and can issue the request at tick `40`. This is intentionally not a precontact controller. The frozen wrench trace still does not produce a ten-tick bilateral/upright tail; it remains default-off pending a contact-mode phase policy and a physically sustained touchdown.
