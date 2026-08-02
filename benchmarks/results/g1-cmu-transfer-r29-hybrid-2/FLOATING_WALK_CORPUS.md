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
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 264 | 1.320 s | 3.622 cm | 0.009 cm | 12.419 cm | 27.458 cm | 6.048° | 8.000 rad/s | 5169.5 µs |

Nominal hard residual maxima: dynamics `1.281e-09`, contact acceleration `7.305e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 145.413 cm |
| stance foot RMS | 137.202 cm |
| swing foot RMS | 191.255 cm |
| hand RMS | 155.993 cm |
| maximum root rotation | 8.817° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.281e-09 |
| contact acceleration residual | 7.305e-11 |
| raw max dynamics residual, including rejected ticks | 1.281e-09 |
| raw max contact residual, including rejected ticks | 7.305e-11 |
| active normal force range | 0.000–702.120 N |
| centroidal momentum-rate residual RMS / max | 28.381 / 114.760 N·m |
| point-task acceleration RMS max | 113.240 m/s² |
| frame-angular acceleration RMS max | 161.154 rad/s² |
| longest pre-contact / touchdown transition | 36 / 168 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2679.5 µs | 4203.2 µs | 5186.2 µs | 6218.6 µs | 138 | 90 | 36 | 168 | 147 | 21 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2759.2 | 745.3 | 451.9 | 3647.4 | 6091.2 | 6205.9 | 2080.8 | 600 | 8 | 0 | 362.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 138 | 2636.7 | 2719.1 | 2784.2 | 2829.1 |
| solved_with_slack | 90 | 2955.7 | 4892.4 | 5852.2 | 6218.6 |
| normal_contact_contingency | 147 | 3003.4 | 4206.0 | 4636.9 | 6005.8 |
| contact_release_contingency | 21 | 1872.2 | 3166.4 | 3175.2 | 3177.5 |
| touchdown_transition | 168 | 2088.6 | 3487.8 | 5210.1 | 5434.6 |
| precontact_transition | 36 | 3198.8 | 3896.9 | 3935.8 | 3946.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.44 | 11.0 | 13.0 | 24 | 4.85 | 13.0 | 21 | 0.2163 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.38/2.0/2.0/2 | 303.87/444.0/468.0/468 | 0.90/4.0/4 | 0.53/3.0/3 | 4.24/24.0/27 | 0.6498 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 138 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 90 | 8.77/24.0/24 | 6.41/21.0/21 |
| normal_contact_contingency | 147 | 8.20/11.5/15 | 6.69/10.5/13 |
| contact_release_contingency | 21 | 7.86/11.6/12 | 6.52/10.6/11 |
| touchdown_transition | 168 | 7.92/11.3/14 | 5.80/11.0/13 |
| precontact_transition | 36 | 7.94/11.9/13 | 6.64/11.6/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.67/4.0/5 | 6.49/17.0/24 | 0.95/4.0/5 | 280 |
| viability | 2.18/7.0/13 | 13.13/42.0/49 | 1.93/7.0/13 | 452 |
| intent | 1.41/4.0/6 | 7.36/24.0/35 | 1.15/4.0/6 | 441 |
| preference | 1.15/3.0/16 | 11.15/35.0/192 | 0.58/3.0/15 | 270 |
| style | 1.03/2.0/4 | 9.23/27.0/40 | 0.24/2.0/3 | 130 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 172 | 199 | 0 |
| right_ankle_roll_link | 168 | 0 | 0 | 264 | 168 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `172` ticks, normal fallback `171` ticks.
Precontact sole-center tangential speed: p50 `1.8405 m/s`, p95 `7.0498 m/s`, max `7.4766 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `4.8698 m/s`, p95 `6.0394 m/s`, max `6.1952 m/s` over 171 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.656 | 1.654 | 1.654 | 0.999 | 0.999 | 47.297 | 47.539 | 0.242 | 47.539 | 0.001 | 0 | 40 | 0 | 0 | 108 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2658.0 | 4334.7 | 6.43 | 39.43 | 2.07 | 1.00 | 234.00 | 0.156 | 0.000 | 1.25e-09 | 4.91e-11 | 0 |
| 60–119 | 2636.9 | 4048.7 | 5.17 | 36.52 | 0.28 | 1.00 | 234.00 | 0.025 | 0.000 | 1.28e-09 | 7.30e-11 | 0 |
| 120–179 | 2666.9 | 5399.8 | 6.43 | 42.97 | 2.17 | 1.00 | 234.00 | 0.033 | 0.000 | 1.00e-09 | 5.04e-11 | 0 |
| 180–239 | 3071.2 | 5684.6 | 8.15 | 51.68 | 6.40 | 1.47 | 330.80 | 4.510 | 3.091 | 1.21e-09 | 4.17e-11 | 12 |
| 240–299 | 3151.6 | 4458.4 | 8.30 | 51.50 | 6.83 | 1.60 | 349.10 | 9.699 | 15.749 | 7.91e-10 | 7.16e-12 | 60 |
| 300–359 | 3074.3 | 5067.3 | 8.35 | 50.98 | 7.00 | 1.70 | 367.20 | 13.164 | 33.389 | 3.69e-10 | 1.21e-11 | 60 |
| 360–419 | 2883.5 | 3963.1 | 7.80 | 47.92 | 6.15 | 1.67 | 359.30 | 68.375 | 50.278 | 1.03e-09 | 7.56e-12 | 60 |
| 420–479 | 2989.9 | 5331.7 | 7.77 | 47.60 | 6.03 | 1.75 | 375.90 | 171.316 | 167.170 | 3.68e-10 | 2.92e-12 | 12 |
| 480–539 | 2076.8 | 3487.2 | 7.95 | 53.78 | 5.85 | 1.30 | 280.80 | 265.887 | 289.528 | 5.06e-10 | 5.56e-12 | 0 |
| 540–599 | 1933.4 | 3102.1 | 8.08 | 51.13 | 5.72 | 1.27 | 273.60 | 326.252 | 362.364 | 5.06e-10 | 3.08e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 145.413 | 157.156 | 155.993 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
