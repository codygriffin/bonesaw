# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `intent` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.100` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; contact forces remain off until the authored contact edge.
- Reference touchdown blend: `0` ticks (`0.000 s`) with a quintic zero-velocity endpoint.
- Touchdown stabilization point: `ankle origin`.

## Acceptance

Functional: **FAIL**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | FAIL |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 428 | 2.140 s | 8.643 cm | 0.468 cm | 29.226 cm | 20.368 cm | 9.201° | 8.000 rad/s | 4305.0 µs |

Nominal hard residual maxima: dynamics `1.610e-09`, contact acceleration `6.551e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 36.436 cm |
| stance foot RMS | 33.435 cm |
| swing foot RMS | 69.949 cm |
| hand RMS | 42.888 cm |
| maximum root rotation | 13.826° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.610e-09 |
| contact acceleration residual | 6.551e-11 |
| raw max dynamics residual, including rejected ticks | 1.610e-09 |
| raw max contact residual, including rejected ticks | 6.551e-11 |
| active normal force range | 0.000–581.324 N |
| centroidal momentum-rate residual RMS / max | 31.378 / 125.054 N·m |
| point-task acceleration RMS max | 111.093 m/s² |
| frame-angular acceleration RMS max | 124.795 rad/s² |
| longest pre-contact / touchdown transition | 200 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2536.0 µs | 60074.6 µs | 101296.2 µs | 270928.7 µs | 148 | 80 | 200 | 0 | 166 | 6 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10362.5 | 23637.8 | 589.4 | 36591.8 | 186218.4 | 262457.7 | 82996.9 | 600 | 108 | 74 | 96.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 148 | 2479.9 | 2575.2 | 2711.5 | 2829.6 |
| solved_with_slack | 80 | 2817.0 | 3807.7 | 4446.4 | 4457.7 |
| normal_contact_contingency | 166 | 7531.4 | 88696.4 | 97997.3 | 121594.9 |
| contact_release_contingency | 6 | 115764.8 | 235573.8 | 263857.8 | 270928.7 |
| precontact_transition | 200 | 2087.2 | 3940.3 | 4340.4 | 4401.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.55 | 10.0 | 12.0 | 16 | 4.43 | 11.0 | 14 | 0.1120 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 416.62/3365.1/6142.1/6544 | 90069.60/726861.6/1326704.4/1452768 | 0.92/9.0/10 | 0.68/8.0/9 | 5.60/67.0/76 | 0.7763 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 148 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 80 | 7.75/16.0/16 | 5.31/14.0/14 |
| normal_contact_contingency | 166 | 7.25/11.0/12 | 6.00/10.0/11 |
| contact_release_contingency | 6 | 6.17/8.0/8 | 5.00/7.0/7 |
| precontact_transition | 200 | 7.38/11.0/14 | 6.04/10.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.63/5.0/7 | 6.27/20.0/28 | 0.93/5.0/7 | 276 |
| viability | 2.28/7.0/13 | 13.78/49.0/64 | 2.02/7.0/13 | 451 |
| intent | 1.60/6.0/8 | 13.21/47.0/73 | 1.19/5.0/8 | 392 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.04/2.0/4 | 8.95/22.0/29 | 0.29/2.0/4 | 162 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 0 | 199 | 172 |
| right_ankle_roll_link | 168 | 0 | 0 | 428 | 4 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `0` ticks, normal fallback `175` ticks.
Precontact sole-center tangential speed: p50 `1.9662 m/s`, p95 `2.8500 m/s`, max `3.6579 m/s` over 200 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.218 | 6.217 | 6.217 | 1.000 | 1.000 | 47.621 | 47.848 | 0.227 | 47.848 | 0.001 | 0 | 40 | 0 | 0 | 46 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2519.7 | 3851.0 | 5.25 | 31.30 | 1.52 | 1.00 | 234.00 | 0.810 | 0.000 | 1.61e-09 | 5.50e-11 | 0 |
| 60–119 | 2473.6 | 2576.9 | 4.00 | 25.28 | 0.00 | 1.00 | 234.00 | 1.327 | 0.000 | 1.25e-09 | 4.51e-11 | 0 |
| 120–179 | 2490.0 | 3615.7 | 5.38 | 32.93 | 1.83 | 1.00 | 234.00 | 1.376 | 0.000 | 1.58e-09 | 6.55e-11 | 0 |
| 180–239 | 2729.9 | 4282.9 | 7.12 | 45.77 | 4.95 | 3.45 | 769.70 | 5.672 | 5.114 | 1.47e-09 | 5.75e-11 | 12 |
| 240–299 | 1873.3 | 3612.4 | 6.92 | 43.88 | 5.02 | 2.28 | 506.90 | 7.754 | 21.675 | 9.57e-10 | 1.85e-11 | 60 |
| 300–359 | 2049.2 | 3891.0 | 7.37 | 47.52 | 6.30 | 2.17 | 481.00 | 9.151 | 17.370 | 1.19e-09 | 1.28e-11 | 60 |
| 360–419 | 2357.5 | 4365.1 | 7.82 | 49.13 | 6.70 | 4.03 | 895.40 | 17.410 | 24.347 | 1.34e-09 | 1.53e-11 | 60 |
| 420–479 | 7099.3 | 187491.2 | 7.23 | 49.25 | 5.93 | 1196.90 | 259190.70 | 24.213 | 37.439 | 1.28e-10 | 6.12e-12 | 60 |
| 480–539 | 19416.6 | 94556.1 | 7.33 | 50.88 | 6.05 | 1903.23 | 411098.40 | 59.683 | 80.548 | 4.72e-10 | 5.89e-12 | 60 |
| 540–599 | 4957.7 | 97979.1 | 7.07 | 46.05 | 6.02 | 1051.17 | 227051.90 | 92.973 | 119.950 | 5.54e-10 | 6.46e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 36.436 | 48.650 | 42.888 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
