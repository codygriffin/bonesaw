# Upkie causal moment rejection — R314

Status: **RECOVERY QUALIFIED FOR EVALUATION; PUBLIC PROFILE UNCHANGED; DEFAULT-OFF**.

The delayed root-origin wrench observation is causal and finite; the dynamics feed-forward path is explicit, Rust-owned, and confidence scaled to 0.7 for the one-tick delayed/model-mismatch boundary. Every bounded mechanism gate passes. Every holdout finishes, so the frozen recovery profile qualifies.

Frozen simulator: **MuJoCo 3.3.7**.

| force Y N | baseline terminal | R313 terminal | R314 terminal | max moment N·m |
|---:|---:|---:|---:|---:|
| -8 | 74 | 79 | — | 2.000 |
| -6 | 164 | 236 | — | 1.500 |
| -4 | — | — | — | 1.000 |
| -2 | — | — | — | 0.500 |
| +2 | — | — | — | 0.500 |
| +4 | — | — | — | 1.000 |
| +6 | 87 | 236 | — | 1.500 |
| +8 | 69 | 69 | — | 2.000 |

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

- PASS `terminal_fall_count_reduced`
- PASS `every_case_finishes_horizon`
- PASS `relock_cases_retain_upright_tail`

## Architectural conclusion

The plant applies the current declared wrench only after the WBC solve, then retains its application point and force and re-expresses the root-origin moment at the next 50 Hz boundary. Rust owns the bounded 0.7 confidence-scaled floating dynamics/feed-forward rows; Python only transports the fixed-size observation. The optional centroidal moment remains a separate frame/objective input and is zero in this candidate. All eight rows finish this frozen holdout, but public promotion remains unchanged pending independent delay/model, reference-controller, and hardware/thermal evidence.
