# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response. Foot tracking: `invariant` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Touchdown admission limits: `0.025 m` position, `0.200 m/s` tangential, and `0.200 m/s` normal; the outgoing support is retained until its replacement locks.
- Reference touchdown blend: `0` ticks (`0.000 s`) with a quintic zero-velocity endpoint.
- Touchdown stabilization point: `ankle origin`.

## Acceptance

Functional: **FAIL**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.000 s | 25.688 cm | 33.418 cm | 34.234 cm | 49.185 cm | 40.164° | 8.000 rad/s | 77048.6 µs |

Nominal hard residual maxima: dynamics `1.622e-09`, contact acceleration `6.301e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 25.688 cm |
| stance foot RMS | 33.418 cm |
| swing foot RMS | 34.234 cm |
| hand RMS | 49.185 cm |
| maximum root rotation | 40.164° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.622e-09 |
| contact acceleration residual | 6.301e-11 |
| raw max dynamics residual, including rejected ticks | 1.622e-09 |
| raw max contact residual, including rejected ticks | 6.301e-11 |
| active normal force range | 0.000–353.727 N |
| centroidal momentum-rate residual RMS / max | 31.037 / 120.551 N·m |
| point-task acceleration RMS max | 54.858 m/s² |
| frame-angular acceleration RMS max | 168.076 rad/s² |
| longest pre-contact / touchdown transition | 372 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2516.1 µs | 4941.0 µs | 77048.6 µs | 85159.9 µs | 141 | 87 | 372 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4851.8 | 11765.0 | 574.0 | 3690.3 | 84914.1 | 85135.3 | 17711.2 | 600 | 30 | 22 | 206.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2440.7 | 2693.4 | 2813.6 | 2903.1 |
| solved_with_slack | 87 | 2721.3 | 4860.3 | 5373.2 | 6199.7 |
| precontact_transition | 372 | 3024.1 | 30595.6 | 82842.8 | 85159.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.67 | 11.0 | 13.0 | 16 | 5.47 | 12.0 | 14 | 0.2069 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 146.94/8.0/4874.7/5316 | 32624.67/1872.0/1082179.0/1180152 | 0.74/3.0/3 | 0.41/2.0/2 | 3.34/16.0/17 | 0.9984 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 87 | 8.31/12.6/16 | 5.83/11.3/13 |
| precontact_transition | 372 | 8.54/13.0/14 | 7.45/12.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.79/5.0/6 | 7.27/20.0/25 | 1.23/5.0/6 | 329 |
| viability | 2.23/7.0/12 | 13.10/42.0/63 | 1.98/7.0/12 | 451 |
| intent | 1.39/4.0/6 | 7.45/24.0/36 | 1.11/4.0/6 | 434 |
| preference | 1.22/5.0/7 | 12.01/48.0/77 | 0.84/5.0/6 | 382 |
| style | 1.05/2.0/4 | 9.91/26.0/41 | 0.31/2.0/4 | 172 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 600 | 0 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `2.5133 m/s`, p95 `3.5200 m/s`, max `5.3138 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.911 | 2.911 | 2.911 | 1.000 | 1.000 | 47.684 | 47.930 | 0.246 | 48.105 | 0.001 | 0 | 41 | 0 | 0 | 21 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2472.3 | 3004.1 | 6.07 | 36.82 | 1.67 | 1.00 | 234.00 | 0.172 | 0.000 | 1.62e-09 | 5.63e-11 | 0 |
| 60–119 | 2429.8 | 3606.2 | 5.10 | 34.98 | 0.17 | 1.00 | 234.00 | 0.032 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–179 | 2502.9 | 5189.4 | 6.12 | 39.43 | 1.62 | 1.12 | 261.30 | 0.002 | 0.000 | 1.46e-09 | 6.30e-11 | 0 |
| 180–239 | 2829.2 | 5532.1 | 8.25 | 50.77 | 6.32 | 4.03 | 909.00 | 3.471 | 0.538 | 1.47e-09 | 5.34e-11 | 12 |
| 240–299 | 1947.3 | 3986.3 | 8.43 | 55.18 | 6.95 | 2.28 | 506.90 | 7.497 | 21.157 | 1.19e-09 | 1.44e-11 | 60 |
| 300–359 | 3185.8 | 4162.0 | 8.93 | 59.78 | 7.82 | 6.25 | 1387.50 | 10.882 | 31.857 | 9.81e-10 | 2.86e-11 | 60 |
| 360–419 | 3060.2 | 3724.5 | 8.50 | 52.75 | 7.45 | 5.35 | 1187.70 | 15.594 | 37.916 | 6.18e-10 | 1.75e-11 | 60 |
| 420–479 | 3087.8 | 4006.2 | 8.13 | 54.40 | 6.97 | 6.13 | 1361.60 | 24.339 | 31.141 | 1.04e-09 | 2.07e-11 | 60 |
| 480–539 | 2934.8 | 3869.9 | 8.22 | 51.48 | 7.42 | 4.73 | 1050.80 | 40.968 | 49.873 | 9.43e-10 | 1.24e-11 | 60 |
| 540–599 | 2666.4 | 84917.8 | 9.00 | 61.72 | 8.30 | 1437.45 | 319113.90 | 62.434 | 70.660 | 1.21e-09 | 2.31e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 25.688 | 33.690 | 49.185 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
