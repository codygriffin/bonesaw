# Upkie composed moment rejection — R315

Status: **REJECTED; DEFAULT-OFF NEGATIVE EVIDENCE**.

Correct root-origin wrench feed-forward at the R314 0.7 confidence scale is causal and deterministic and its root-only profile clears the frozen holdout. Composing it with the state-local body-moment law is still rejected: the lower-damping composition delays both terminal boundaries to ticks 84/85 while introducing four nonadmitted solves and a >7 ms controller p99; the higher-damping composition also regresses the -8 N fall to tick 63. The root-only recovery result remains evaluation-only; public authority and the state-local R314 qualification are unchanged.

| profile | falls | nonadmitted ticks | max torque | controller p99 µs | worker p99 µs |
|---|---:|---:|---:|---:|---:|
| body_only | 2 | 0 | 0.256 | 207.7 | 3248.5 |
| root_wrench_only | 0 | 0 | 0.327 | 167.5 | 2598.5 |
| body_plus_root_wrench | 2 | 4 | 0.219 | 7322.1 | 9148.3 |
| body_plus_root_wrench_d12 | 2 | 3 | 0.998 | 7287.0 | 9107.9 |

## Composition gates

- PASS `frozen_mujoco_version_matches`
- PASS `all_profiles_default_off`
- PASS `semantic_replay_exact`
- OPEN `composition_finishes_every_case`
- OPEN `composition_has_zero_nonadmission`
- OPEN `composition_meets_controller_deadline`
- PASS `composition_meets_worker_deadline`
- PASS `composition_does_not_raise_fall_count`

The screen changes no public authority, timeout, iteration budget, contact mode, or reset behavior. The rejected profiles remain executable research options only.
