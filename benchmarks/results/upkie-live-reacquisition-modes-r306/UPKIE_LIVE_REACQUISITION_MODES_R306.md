# Upkie live contact-reacquisition mode comparison · R306

> Negative evidence **PASS** when transport/causality/finite/allocation gates pass and every existing mode is rejected by the strict 10-tick wheel-supported recovery predicate.

R306 replays the frozen R300 8 N lateral wrench against the existing capture, planar-capture, viability, and R302 support-contingency profiles. Every candidate is an explicit evaluation-only worker override; the public 250 Hz MuJoCo / 50 Hz WBC `production_default` path is not changed. The Rust request remains downstream of measured contact and is admitted only by the ordinary WBC boundary.

Recovery requires ten consecutive 50 Hz samples after the disturbance with both measured wheel contacts, root height at least 0.48 m, root tilt at most 0.20 rad, and no pending reset. Delayed fall, body-ground support, and a geometric touch do not count.

| mode | terminal tick | first flight | recovery tick | max body/no-wheel stall | min height m | max tilt rad | controller p99 µs | max residual |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| production_capture | 52 | 36 | — | 2 | 0.29304 | 0.62450 | 5293.7 | 9.139e+01 |
| planar_capture | 48 | 35 | — | 1 | 0.29820 | 0.90303 | 4511.0 | 5.324e+01 |
| viability_capture | 47 | 33 | — | 4 | 0.28479 | 1.44011 | 6932.2 | 5.324e+01 |
| viability_support_capture | 47 | 33 | — | 4 | 0.28479 | 1.44011 | 6882.0 | 5.324e+01 |
| viability_coordinate | 48 | 36 | — | 7 | 0.29953 | 1.03334 | 6988.3 | 7.151e+01 |
| support_contingency_r302 | — | 35 | — | 51 | 0.30301 | 0.62216 | 275.2 | 1.788e-11 |

## Gates

| gate | result |
|---|:---:|
| all_modes_use_250hz_physics_50hz_wbc | PASS |
| all_modes_preserve_causal_prior_window | PASS |
| all_outputs_finite | PASS |
| hard_rows_subset_measured_contact | PASS |
| max_iterations_never_admitted | PASS |
| zero_timed_rust_allocations | PASS |
| worker_p99_under_20ms | PASS |
| public_capture_profile_is_unchanged | PASS |
| no_mode_reacquires_wheels_for_ten_ticks | PASS |

## Interpretation

The existing profiles provide useful fall-delay and residual experiments, but none produces a measured, force-backed wheel reacquisition on this frozen physical trace. R302's support-contingency request can keep the plant alive longer than the baseline, yet it does not create a new contact target when both wheels are absent. The next behavior slice must add a separately reviewed wheel-gap/landing request and then pass it through the ordinary WBC admission path; increasing QP iterations or relaxing timeout handling is not a contact-reacquisition mechanism.

No policy, hidden estimator, automatic reset, public cadence change, or promoted actuator authority is used by this artifact.
