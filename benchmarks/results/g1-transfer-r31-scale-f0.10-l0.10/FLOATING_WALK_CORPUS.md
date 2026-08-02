# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.103 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.079 m/s` (`0.10×` forward, `0.10×` lateral retarget scale).
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
| 582 | 2.910 s | 53.999 cm | 32.158 cm | 48.890 cm | 60.327 cm | 18.271° | 8.000 rad/s | 32166.8 µs |

Nominal hard residual maxima: dynamics `1.622e-09`, contact acceleration `5.634e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 65.936 cm |
| stance foot RMS | 40.977 cm |
| swing foot RMS | 60.498 cm |
| hand RMS | 72.092 cm |
| maximum root rotation | 18.271° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.340e-09 |
| contact acceleration residual | 2.347e-10 |
| raw max dynamics residual, including rejected ticks | 4.340e-09 |
| raw max contact residual, including rejected ticks | 2.347e-10 |
| active normal force range | 0.000–467.655 N |
| centroidal momentum-rate residual RMS / max | 25.649 / 247.068 N·m |
| point-task acceleration RMS max | 109.760 m/s² |
| frame-angular acceleration RMS max | 100.592 rad/s² |
| longest pre-contact / touchdown transition | 200 / 154 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2593.8 µs | 4924.8 µs | 107750.0 µs | 217472.6 µs | 141 | 87 | 200 | 154 | 4 | 14 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6516.4 | 22748.7 | 527.6 | 3865.8 | 215197.8 | 217245.1 | 30429.3 | 600 | 30 | 21 | 153.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2484.5 | 2601.4 | 2674.4 | 2693.7 |
| solved_with_slack | 87 | 2789.1 | 4826.2 | 5917.7 | 6199.0 |
| normal_contact_contingency | 4 | 2727.8 | 7386.3 | 8043.3 | 8207.5 |
| contact_release_contingency | 14 | 106914.4 | 215004.1 | 216978.9 | 217472.6 |
| touchdown_transition | 154 | 2816.6 | 4570.5 | 39045.3 | 76307.9 |
| precontact_transition | 200 | 3002.3 | 4490.6 | 44356.8 | 76799.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.43 | 11.0 | 12.0 | 16 | 5.03 | 11.0 | 13 | 0.0523 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 40.39/8.0/1953.7/4903 | 8862.57/1776.0/422128.7/1059048 | 0.95/4.0/10 | 0.55/3.0/9 | 4.42/24.0/73 | 0.2296 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 87 | 8.21/12.6/16 | 5.76/11.3/13 |
| normal_contact_contingency | 4 | 8.00/9.9/10 | 7.00/8.9/9 |
| contact_release_contingency | 14 | 7.79/10.7/11 | 6.71/9.7/10 |
| touchdown_transition | 154 | 8.19/12.5/13 | 6.67/10.9/13 |
| precontact_transition | 200 | 8.18/12.0/13 | 6.85/11.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.90/5.0/7 | 7.51/25.0/35 | 1.34/5.0/7 | 337 |
| viability | 2.06/7.0/12 | 12.23/40.0/50 | 1.82/7.0/12 | 452 |
| intent | 1.33/4.0/6 | 6.99/24.0/34 | 1.05/4.0/6 | 435 |
| preference | 1.10/3.0/7 | 10.86/33.0/70 | 0.58/3.0/7 | 296 |
| style | 1.03/2.0/4 | 9.31/26.0/33 | 0.23/2.0/4 | 129 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 154 | 199 | 18 |
| right_ankle_roll_link | 168 | 0 | 0 | 432 | 0 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `154` ticks, normal fallback `21` ticks.
Precontact sole-center tangential speed: p50 `1.8610 m/s`, p95 `2.6705 m/s`, max `2.9689 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `2.8534 m/s`, p95 `4.0729 m/s`, max `4.5639 m/s` over 153 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.910 | 3.909 | 3.909 | 1.000 | 1.000 | 47.539 | 47.770 | 0.230 | 47.816 | 0.001 | 0 | 43 | 0 | 0 | 60 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2512.5 | 2959.3 | 6.07 | 36.82 | 1.67 | 1.00 | 234.00 | 0.172 | 0.000 | 1.62e-09 | 5.63e-11 | 0 |
| 60–119 | 2479.7 | 3554.0 | 5.10 | 34.98 | 0.17 | 1.00 | 234.00 | 0.032 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–179 | 2522.1 | 5242.9 | 6.17 | 39.07 | 1.70 | 1.23 | 288.60 | 0.003 | 0.000 | 1.46e-09 | 4.52e-11 | 0 |
| 180–239 | 2835.8 | 6006.0 | 7.78 | 50.33 | 5.90 | 4.27 | 959.40 | 3.607 | 3.613 | 9.12e-10 | 5.41e-11 | 12 |
| 240–299 | 3013.9 | 3658.4 | 8.10 | 50.00 | 6.77 | 5.35 | 1187.70 | 10.838 | 23.647 | 4.26e-10 | 1.22e-11 | 60 |
| 300–359 | 3645.0 | 4872.6 | 8.37 | 51.15 | 7.27 | 6.37 | 1413.40 | 12.142 | 31.692 | 4.25e-10 | 7.31e-12 | 60 |
| 360–419 | 1994.3 | 59158.6 | 8.20 | 52.43 | 6.75 | 202.82 | 45025.30 | 18.809 | 14.850 | 1.15e-09 | 2.48e-11 | 60 |
| 420–479 | 3355.8 | 5467.0 | 8.45 | 55.47 | 7.12 | 6.48 | 1411.80 | 29.349 | 30.928 | 1.31e-09 | 1.23e-11 | 8 |
| 480–539 | 2849.8 | 3621.6 | 7.73 | 50.08 | 6.25 | 7.07 | 1526.40 | 86.036 | 51.150 | 2.86e-10 | 2.19e-12 | 0 |
| 540–599 | 1800.2 | 215232.0 | 8.32 | 48.70 | 6.75 | 168.30 | 36345.10 | 185.956 | 134.070 | 4.34e-09 | 2.35e-10 | 18 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 65.936 | 48.317 | 72.092 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
