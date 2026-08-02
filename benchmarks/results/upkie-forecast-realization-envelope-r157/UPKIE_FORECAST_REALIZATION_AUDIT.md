# Bonesaw forecast-versus-realized envelope · r157

> Measurement gate **PASS**. This is calibration evidence, not an executable safety certificate.

## Result

The retained r156 two-update confirmation produced **26** executable forecast origins and **173** complete knot comparisons. **148** comparisons cross at least one raw support change. Replay is **exact** and timed Rust allocation is **zero**.

Same-support normalized max error reaches p50/p95/p99/max **0.121/0.277/0.313/0.324** of the forecast limits. Support-changing comparisons reach **0.377/1.181/1.222/1.235**. These empirical errors are descriptive and must not be consumed online until coverage, uncertainty, and an upper-confidence rule are admitted.

## Error by knot

| knot | t s | samples | support changed | p50 | p95 | p99 | max |
|---|---|---|---|---|---|---|---|
| 1 | 0.03 | 26 | 13 | 0.121 | 0.284 | 0.620 | 0.732 |
| 2 | 0.06 | 22 | 15 | 0.234 | 0.348 | 0.866 | 1.002 |
| 3 | 0.09 | 22 | 18 | 0.273 | 0.323 | 0.887 | 1.037 |
| 4 | 0.12 | 22 | 21 | 0.318 | 0.386 | 0.835 | 0.954 |
| 5 | 0.15 | 21 | 21 | 0.377 | 0.585 | 1.092 | 1.219 |
| 6 | 0.18 | 20 | 20 | 0.484 | 1.007 | 1.039 | 1.047 |
| 7 | 0.21 | 20 | 20 | 0.845 | 1.165 | 1.184 | 1.189 |
| 8 | 0.24 | 20 | 20 | 1.140 | 1.225 | 1.233 | 1.235 |

## Absolute state error

| state | p50 | p95 | p99 | max |
|---|---|---|---|---|
| roll_rad | 0.026717 | 0.13565 | 0.20162 | 0.2803 |
| roll_rate_rad_s | 0.49941 | 1.1514 | 2.294 | 2.3611 |
| pitch_rad | 0.045311 | 0.21707 | 0.31104 | 0.36281 |
| pitch_rate_rad_s | 0.8799 | 4.6403 | 4.872 | 4.9405 |
| lateral_position_m | 0.019978 | 0.082053 | 0.088141 | 0.12188 |
| lateral_velocity_m_s | 0.30961 | 0.81847 | 0.96737 | 1.0366 |
| yaw_rad | 0.075616 | 0.33112 | 0.40924 | 0.42873 |
| yaw_rate_rad_s | 0.96879 | 3.127 | 3.2076 | 3.2104 |

## Measurement gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| forecast_paths_exercised | PASS |
| support_transition_paths_exercised | PASS |
| path_replay_exact | PASS |
| finite | PASS |
| zero_timed_rust_allocation | PASS |
| fixed_knot_times_exact | PASS |

## Contract

- Rust emits the exact eight fixed reduced-order knots used by scoring into caller-owned `[8, 9]` storage with validation-before-mutation and zero hot-path allocation.
- Python aligns each confirmed origin with later 200 Hz plant observations at 30 ms intervals and keeps same-support and support-changing evidence separate.
- This experiment does not train a policy, alter the live r137 worker, infer a worst-case bound from a percentile, or claim that MuJoCo is hardware.
