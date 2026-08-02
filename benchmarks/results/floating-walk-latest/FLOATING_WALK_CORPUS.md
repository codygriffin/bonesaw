# Bonesaw floating CMU walking corpus

This is the first moving-root, contact-aware retarget through the Rust floating inverse-dynamics WBC. Python owns source reconstruction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target stride: `1.217 m` forward per `1.308 s` source cycle.
- Mean source-scaled forward speed: `0.930 m/s`.
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Stance feet begin as hard three-axis locks at Rust-latched measured touchdown anchors. A typed normal-only and then contact-release contingency keeps a rejected lock observable instead of freezing the trace.

## Acceptance

Overall: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | FAIL |
| `no_contact_contingency_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `hand_tracking_rms_le_5cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `hand_tracking_rms_le_5cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 119 | 0.595 s | 0.430 cm | 0.878 cm | 8.532 cm | 14.021 cm | 1.780° | 6.982 rad/s | 1421.1 µs |

Nominal hard residual maxima: dynamics `1.409e-09`, contact acceleration `5.454e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 115.074 cm |
| stance foot RMS | 122.043 cm |
| swing foot RMS | 163.037 cm |
| hand RMS | 123.392 cm |
| maximum root rotation | 171.070° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.409e-09 |
| contact acceleration residual | 5.454e-11 |
| raw max dynamics residual, including rejected ticks | 1.442e+03 |
| raw max contact residual, including rejected ticks | 5.454e-11 |
| active normal force range | 0.000–821.481 N |
| point-task acceleration RMS max | 196.581 m/s² |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 70883.4 µs | 72526.9 µs | 80186.7 µs | 119477.1 µs | 110 | 9 | 137 | 8 | 336 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 115.074 | 140.376 | 123.392 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
