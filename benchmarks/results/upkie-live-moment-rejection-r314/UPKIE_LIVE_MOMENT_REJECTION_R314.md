# Upkie causal moment rejection — R314

Status: **MECHANISM QUALIFIED; RECOVERY REJECTED; DEFAULT-OFF**.

The delayed external-moment target is causal, finite, allocation-free, and never earlier than the frozen R313 boundary for this profile. It still reaches terminal falls in the repeated ±6/±8 N holdout, so the mechanism remains default-off and does not promote recovery.

Frozen simulator: **MuJoCo 3.3.7**.

| force Y N | baseline terminal | R313 terminal | R314 terminal | max moment N·m |
|---:|---:|---:|---:|---:|
| -8 | 74 | 79 | 80 | 2.000 |
| -6 | 164 | 236 | 331 | 1.500 |
| -4 | — | — | — | 1.000 |
| -2 | — | — | — | 0.500 |
| +2 | — | — | — | 0.500 |
| +4 | — | — | — | 1.000 |
| +6 | 87 | 236 | 322 | 1.500 |
| +8 | 69 | 69 | 70 | 2.000 |

## Mechanism gates

- PASS `frozen_mujoco_version_matches`
- PASS `candidate_is_default_off`
- PASS `external_moments_finite`
- PASS `zero_allocations_and_no_nonadmission`
- PASS `deadlines_hold`
- PASS `mode_firewall_holds`
- PASS `exact_replay`
- PASS `never_earlier_than_r313`

## Promotion gates

- OPEN `terminal_fall_count_reduced`
- OPEN `every_case_finishes_horizon`
- OPEN `relock_cases_retain_upright_tail`

## Architectural conclusion

The plant applies the current declared wrench only after the WBC solve, then stores the completed MuJoCo moment for the next 50 Hz call. Rust owns the centroidal contact-moment target; Python only transports the fixed-size observation. The remaining failure is physical support/moment capacity, not a missing timeout or hidden reset.
