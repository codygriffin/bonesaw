# Bonesaw rolling-balance corpus

Overall: **PASS**  
Ticks/scenario: 500 at 5 ms  
Isolated repeats/scenario: 2  
Initial velocity cases set root and both wheel rates to satisfy the no-slip rows exactly.

| split | scenario | v₀ (m/s) | pass | infeasible | height RMS (m) | horizontal RMS (m) | CoM max (m) | drift (m) | rotation (rad) | p99 (µs) | alloc/step |
|---|---|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|
| train | nominal | 0.000 | PASS | 0 | 0.017168 | 0.010316 | 0.017133 | 0.000006 | 0.000000 | 347.9 | 0.0000 |
| train | slow_forward | 0.010 | PASS | 0 | 0.017184 | 0.017150 | 0.025023 | 0.000006 | 0.000000 | 338.0 | 0.0000 |
| train | slow_backward | -0.010 | PASS | 0 | 0.017151 | 0.003553 | 0.013259 | 0.000006 | 0.000000 | 506.9 | 0.0000 |
| train | medium_forward | 0.030 | PASS | 0 | 0.017213 | 0.030860 | 0.044181 | 0.000006 | 0.000000 | 504.5 | 0.0000 |
| train | medium_backward | -0.030 | PASS | 0 | 0.017113 | 0.010282 | 0.025583 | 0.000006 | 0.000000 | 360.3 | 0.0000 |
| heldout | heldout_forward | 0.020 | PASS | 0 | 0.017199 | 0.024000 | 0.034323 | 0.000006 | 0.000000 | 353.9 | 0.0000 |
| heldout | heldout_backward | -0.020 | PASS | 0 | 0.017133 | 0.003533 | 0.017691 | 0.000006 | 0.000000 | 280.1 | 0.0000 |
| heldout | heldout_fast_forward | 0.050 | PASS | 0 | 0.017236 | 0.044601 | 0.064579 | 0.000006 | 0.000000 | 477.7 | 0.0000 |
| heldout | heldout_fast_backward | -0.050 | PASS | 0 | 0.017071 | 0.023899 | 0.044595 | 0.000006 | 0.000000 | 269.2 | 0.0000 |

## Nominal endurance

This single long run uses the same raw integrated WBC path after all train and held-out perturbation cases.

| ticks | achieved lowering cm | root z RMS cm | root xy RMS cm | CoM max cm | constrained drift mm | wheel travel cm | dynamics max | contact max | infeasible | p50 µs | p99 µs | max µs | alloc/step |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5,000 | 12.000 | 0.543 | 0.521 | 1.713 | 0.0059 | 1.732 | 2.19e-11 | 4.99e-12 | 0 | 262.2 | 277.8 | 585.6 | 0.0000 |

## Acceptance thresholds

```json
{
  "infeasible_ticks": 0,
  "final_root_height_error_abs_m": 0.015,
  "root_height_tracking_rms_m": 0.02,
  "root_horizontal_tracking_rms_m": 0.045,
  "final_root_horizontal_error_m": 0.015,
  "maximum_center_of_mass_tracking_error_m": 0.065,
  "maximum_constrained_contact_drift_m": 0.002,
  "maximum_root_rotation_rad": 0.15,
  "minimum_friction_margin": -1e-07,
  "minimum_torque_margin": -1e-07,
  "allocations_per_step": 0.01
}
```

A scenario passes only when every named check passes. Latency is retained for diagnosis, but does not become an acceptance metric until the controller completes every scenario feasibly.
