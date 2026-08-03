# Upkie root-wrench robustness — R316

Status: **FROZEN MATRIX RECOVERY; ROBUSTNESS REJECTED; DEFAULT-OFF**.

Scale 0.70 clears the selected frozen terminal matrix, but the scale response is discontinuous and the independent 40-case R311 holdout retains one failure: repeated centered +8 N reaches a nonadmitted solve at tick 338 and falls at tick 351. The mechanism remains default-off pending a robust interval and independent holdout pass.

Passing sampled scales: **[0.62, 0.7, 0.72]**.

| scale | falls | nonadmitted ticks | all upright tails | max torque |
|---:|---:|---:|:---:|---:|
| 0.00 | 4 | 0 | no | 0.432 |
| 0.40 | 2 | 2 | no | 0.242 |
| 0.60 | 1 | 1 | no | 0.413 |
| 0.62 | 0 | 0 | yes | 0.330 |
| 0.64 | 2 | 0 | no | 0.556 |
| 0.68 | 1 | 1 | no | 0.374 |
| 0.70 | 0 | 0 | yes | 0.327 |
| 0.72 | 0 | 0 | yes | 0.231 |
| 0.74 | 2 | 0 | no | 0.771 |
| 0.80 | 1 | 0 | no | 0.457 |
| 1.00 | 2 | 4 | no | 1.000 |

## Independent schedule/lever holdout

- 40 rows exclude the eight upper/repeated selection rows.
- Falls: **1**; nonadmitted ticks: **1**.
- Max torque utilization: **0.334**.
- Max controller p99: **187.5 µs**.

## Gates

- PASS `frozen_mujoco_version_matches`
- PASS `candidate_is_default_off`
- PASS `selected_scale_clears_terminal_rows`
- OPEN `adjacent_scale_interval_qualifies`
- OPEN `independent_40_case_holdout_has_zero_falls`
- OPEN `independent_40_case_holdout_has_zero_nonadmission`
- PASS `independent_completed_rows_finish_upright`
- PASS `independent_holdout_meets_deadlines`
- PASS `failed_holdout_replays_exactly`

R310 remains the public profile. R314's exact 0.70 result is retained as behavior-positive selection evidence, not generalized into authority.
