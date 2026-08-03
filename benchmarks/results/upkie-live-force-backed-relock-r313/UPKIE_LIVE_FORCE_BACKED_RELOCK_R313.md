# Upkie force-backed relock persistence — R313

Status: **BOUNDED RELOCK QUALIFIED; FULL RECOVERY REJECTED; DEFAULT-OFF**.

R313 keeps a bounded normal request active through measured-load debounce. A model-radius-minus-0.9-mm target uniquely passes the frozen screen, force-qualifies at least three mirrored ±6 N relocks, and moves their terminal boundaries to tick 236 without moving the ±8 N guardrails earlier. All four terminal cases still fall, so the mechanism remains default-off.

## Selected public-profile consequence

| force Y N | baseline terminal | R312 terminal | R313 terminal | force-qualified ticks |
|---:|---:|---:|---:|---:|
| -8 | 74 | 79 | 79 | — |
| -6 | 164 | 84 | 236 | 57, 137, 154 |
| -4 | — | — | — | — |
| -2 | — | — | — | — |
| +2 | — | — | — | — |
| +4 | — | — | — | — |
| +6 | 87 | 83 | 236 | 57, 137, 154 |
| +8 | 69 | 69 | 69 | — |

## Mechanism gates

- PASS `candidate_is_default_off`
- PASS `model_radius_minus_declared_preload_is_target`
- PASS `screen_selects_exactly_declared_profile`
- PASS `nominal_is_bilateral_dormant`
- PASS `request_starts_only_after_measured_loss`
- PASS `request_persists_during_force_debounce`
- PASS `mirrored_force_backed_relock_occurs`
- PASS `measured_mask_and_mode_firewall_hold`
- PASS `finite_zero_allocation`
- PASS `deadlines_hold`
- PASS `exact_replay`
- PASS `no_new_nonadmission_and_active_rows_clean`
- PASS `never_earlier_than_r312_terminal_boundary`

## Full recovery promotion

- OPEN `terminal_fall_count_reduced`
- OPEN `every_case_finishes_horizon`
- OPEN `every_activated_case_force_relocks`
- OPEN `relocked_cases_retain_final_upright_tail`

## Architectural conclusion

The request mask used to retain the relock command is private to the Rust request author. Measured raw/stable/hard masks still own WBC contact rows and RollingWheel promotion. R313 proves repeated force-backed requalification in the ±6 N class, but not survival of the later repeated pull. The next action must reject accumulated body moment before the third loss rather than deepen preload or extend a timeout.
