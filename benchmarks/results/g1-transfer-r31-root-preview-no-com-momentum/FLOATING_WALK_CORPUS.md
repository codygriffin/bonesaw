# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `rooted` reference at `viability` priority with weight `0.000` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `support-preview`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
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
| 428 | 2.140 s | 9.809 cm | 0.835 cm | 35.970 cm | 28.063 cm | 18.156° | 8.000 rad/s | 3394.9 µs |

Nominal hard residual maxima: dynamics `1.871e-09`, contact acceleration `5.328e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 27.022 cm |
| stance foot RMS | 31.996 cm |
| swing foot RMS | 49.495 cm |
| hand RMS | 54.697 cm |
| maximum root rotation | 24.124° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.494e-09 |
| contact acceleration residual | 1.460e-10 |
| raw max dynamics residual, including rejected ticks | 4.494e-09 |
| raw max contact residual, including rejected ticks | 1.460e-10 |
| active normal force range | 0.000–556.566 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 176.798 m/s² |
| frame-angular acceleration RMS max | 139.559 rad/s² |
| longest pre-contact / touchdown transition | 200 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2316.3 µs | 105602.9 µs | 199674.1 µs | 269910.5 µs | 152 | 76 | 200 | 0 | 122 | 50 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20770.4 | 43562.9 | 578.8 | 92281.6 | 236451.5 | 266564.6 | 98537.6 | 600 | 134 | 107 | 48.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 152 | 2311.1 | 2377.6 | 2403.6 | 2423.3 |
| solved_with_slack | 76 | 2386.2 | 3487.7 | 4000.0 | 4323.0 |
| normal_contact_contingency | 122 | 10535.0 | 97346.4 | 101443.3 | 105998.4 |
| contact_release_contingency | 50 | 107099.9 | 209227.0 | 242540.1 | 269910.5 |
| precontact_transition | 200 | 1756.8 | 3056.9 | 3271.2 | 3395.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.21 | 11.0 | 14.0 | 22 | 4.97 | 12.0 | 21 | 0.1603 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 461.31/4900.0/6457.0/6904 | 99651.01/1058400.0/1394720.6/1491264 | 0.82/10.0/15 | 0.64/9.0/14 | 5.30/76.1/116 | 0.4224 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 152 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 76 | 8.59/16.8/22 | 6.21/15.0/21 |
| normal_contact_contingency | 122 | 8.62/13.6/15 | 7.27/11.8/13 |
| contact_release_contingency | 50 | 7.82/10.5/11 | 6.54/9.5/10 |
| precontact_transition | 200 | 8.12/13.0/14 | 6.49/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.68/5.0/9 | 6.64/21.0/41 | 1.02/5.0/8 | 241 |
| viability | 1.26/6.0/7 | 6.90/30.0/42 | 1.12/6.0/7 | 319 |
| intent | 1.62/7.0/11 | 3.39/14.0/22 | 1.24/6.0/11 | 383 |
| preference | 1.59/7.0/18 | 11.17/50.0/109 | 1.31/7.0/18 | 434 |
| style | 1.06/3.0/5 | 8.76/22.0/40 | 0.29/2.0/4 | 152 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 0 | 199 | 172 |
| right_ankle_roll_link | 168 | 0 | 0 | 428 | 4 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `0` ticks, normal fallback `175` ticks.
Precontact sole-center tangential speed: p50 `1.4125 m/s`, p95 `6.3089 m/s`, max `7.4580 m/s` over 200 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12.462 | 12.461 | 12.461 | 1.000 | 1.000 | 47.445 | 47.707 | 0.262 | 47.707 | 0.001 | 0 | 45 | 0 | 0 | 73 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2323.0 | 3324.6 | 4.53 | 26.43 | 0.65 | 1.00 | 234.00 | 0.632 | 0.000 | 1.25e-09 | 5.33e-11 | 0 |
| 60–119 | 2292.7 | 2370.6 | 4.00 | 22.82 | 0.00 | 1.00 | 234.00 | 0.048 | 0.000 | 7.07e-10 | 3.80e-11 | 0 |
| 120–179 | 2349.5 | 4068.9 | 6.08 | 30.95 | 2.93 | 1.00 | 234.00 | 1.244 | 0.001 | 1.87e-09 | 3.95e-11 | 0 |
| 180–239 | 2132.9 | 3781.6 | 8.05 | 40.57 | 5.23 | 1.00 | 225.80 | 4.087 | 0.159 | 1.07e-09 | 3.91e-11 | 12 |
| 240–299 | 2041.9 | 3351.4 | 7.85 | 39.22 | 5.97 | 4.03 | 895.40 | 3.032 | 14.381 | 5.15e-10 | 9.09e-12 | 60 |
| 300–359 | 1638.4 | 2331.7 | 7.82 | 37.10 | 5.90 | 1.00 | 222.00 | 8.243 | 32.206 | 1.08e-09 | 2.48e-11 | 60 |
| 360–419 | 1733.5 | 2978.7 | 8.50 | 42.80 | 7.63 | 1.23 | 273.80 | 21.077 | 27.948 | 1.17e-09 | 1.96e-11 | 60 |
| 420–479 | 7077.2 | 236954.3 | 8.37 | 42.92 | 7.43 | 812.63 | 175526.80 | 44.799 | 57.476 | 4.49e-09 | 1.10e-10 | 60 |
| 480–539 | 73382.1 | 102960.6 | 8.55 | 44.98 | 6.72 | 3435.88 | 742150.80 | 51.620 | 78.080 | 4.33e-10 | 1.70e-11 | 60 |
| 540–599 | 104145.4 | 208544.5 | 8.37 | 40.87 | 7.27 | 354.33 | 76513.50 | 45.722 | 59.414 | 4.13e-09 | 1.46e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 27.022 | 38.672 | 54.697 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
