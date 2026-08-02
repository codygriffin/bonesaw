# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.206 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.157 m/s` (`0.20×` forward, `0.10×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
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
| 428 | 2.140 s | 8.914 cm | 0.278 cm | 34.782 cm | 29.730 cm | 12.601° | 8.000 rad/s | 5524.5 µs |

Nominal hard residual maxima: dynamics `2.032e-09`, contact acceleration `6.821e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 31.831 cm |
| stance foot RMS | 34.646 cm |
| swing foot RMS | 49.833 cm |
| hand RMS | 41.599 cm |
| maximum root rotation | 12.741° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.032e-09 |
| contact acceleration residual | 6.821e-11 |
| raw max dynamics residual, including rejected ticks | 2.032e-09 |
| raw max contact residual, including rejected ticks | 6.821e-11 |
| active normal force range | 0.000–515.110 N |
| centroidal momentum-rate residual RMS / max | 23.553 / 93.559 N·m |
| point-task acceleration RMS max | 113.789 m/s² |
| frame-angular acceleration RMS max | 119.167 rad/s² |
| longest pre-contact / touchdown transition | 200 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2744.2 µs | 27602.9 µs | 74859.5 µs | 267890.7 µs | 141 | 87 | 200 | 0 | 169 | 3 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5707.9 | 15538.1 | 347.8 | 5734.4 | 170583.3 | 258160.0 | 69522.3 | 600 | 65 | 33 | 175.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2465.1 | 2745.0 | 2958.7 | 2980.0 |
| solved_with_slack | 87 | 2772.7 | 5501.8 | 6480.5 | 7173.0 |
| normal_contact_contingency | 169 | 2910.5 | 41266.0 | 79075.0 | 105441.0 |
| contact_release_contingency | 3 | 102399.8 | 251341.6 | 264580.9 | 267890.7 |
| precontact_transition | 200 | 2878.4 | 3781.8 | 4332.1 | 4884.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.59 | 11.0 | 12.0 | 25 | 5.23 | 11.0 | 22 | 0.0707 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 144.84/1115.0/3182.7/5679 | 31359.02/240850.8/687474.0/1260738 | 1.13/7.0/9 | 0.71/6.0/8 | 5.72/49.0/65 | 0.6138 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 87 | 8.83/24.1/25 | 6.32/21.1/22 |
| normal_contact_contingency | 169 | 8.31/12.0/12 | 6.85/11.0/11 |
| contact_release_contingency | 3 | 8.33/10.0/10 | 7.00/9.0/9 |
| precontact_transition | 200 | 8.26/12.0/14 | 7.04/11.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.76/4.0/7 | 6.76/16.0/28 | 1.14/4.0/7 | 356 |
| viability | 2.17/7.0/12 | 12.73/37.0/49 | 1.92/7.0/12 | 451 |
| intent | 1.40/4.0/6 | 7.52/24.0/32 | 1.12/4.0/6 | 434 |
| preference | 1.18/4.0/17 | 11.79/44.0/221 | 0.70/4.0/16 | 326 |
| style | 1.08/3.0/3 | 9.83/27.0/43 | 0.34/3.0/3 | 173 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 0 | 199 | 172 |
| right_ankle_roll_link | 168 | 0 | 0 | 428 | 4 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `0` ticks, normal fallback `175` ticks.
Precontact sole-center tangential speed: p50 `1.6844 m/s`, p95 `3.1294 m/s`, max `3.2332 m/s` over 200 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.425 | 3.424 | 3.424 | 1.000 | 1.000 | 47.438 | 47.680 | 0.242 | 47.863 | 0.001 | 0 | 43 | 0 | 0 | 40 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2492.1 | 3040.5 | 6.07 | 36.82 | 1.67 | 1.00 | 234.00 | 0.172 | 0.000 | 1.62e-09 | 5.63e-11 | 0 |
| 60–119 | 2442.6 | 3522.3 | 5.10 | 34.98 | 0.17 | 1.00 | 234.00 | 0.032 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–179 | 2521.8 | 4372.2 | 6.03 | 38.58 | 1.55 | 1.12 | 261.30 | 0.002 | 0.000 | 1.46e-09 | 6.31e-11 | 0 |
| 180–239 | 2913.6 | 6697.9 | 9.05 | 60.82 | 7.22 | 4.15 | 934.90 | 3.276 | 3.388 | 1.47e-09 | 4.99e-11 | 12 |
| 240–299 | 2945.8 | 4216.3 | 8.22 | 51.03 | 6.78 | 4.97 | 1102.60 | 9.405 | 19.525 | 1.08e-09 | 7.74e-12 | 60 |
| 300–359 | 2865.3 | 3556.7 | 8.17 | 50.68 | 6.90 | 4.50 | 999.00 | 13.340 | 32.002 | 2.03e-09 | 6.82e-11 | 60 |
| 360–419 | 2215.5 | 4745.7 | 8.30 | 53.45 | 7.27 | 3.62 | 802.90 | 16.050 | 25.380 | 8.53e-10 | 1.02e-11 | 60 |
| 420–479 | 4113.7 | 172045.4 | 8.57 | 55.12 | 7.32 | 426.22 | 92636.30 | 15.408 | 41.928 | 9.13e-11 | 4.16e-12 | 60 |
| 480–539 | 2798.8 | 49873.3 | 8.00 | 50.98 | 6.57 | 288.35 | 62283.60 | 40.191 | 38.964 | 1.05e-10 | 2.28e-12 | 60 |
| 540–599 | 3442.8 | 40167.5 | 8.40 | 53.87 | 6.87 | 713.43 | 154101.60 | 88.003 | 104.456 | 2.08e-10 | 2.00e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 31.831 | 40.309 | 41.599 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
