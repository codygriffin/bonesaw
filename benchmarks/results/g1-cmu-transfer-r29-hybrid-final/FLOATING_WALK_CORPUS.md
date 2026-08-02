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
| 386 | 1.930 s | 11.086 cm | 9.010 cm | 43.951 cm | 34.026 cm | 50.795° | 8.000 rad/s | 5303.4 µs |

Nominal hard residual maxima: dynamics `1.281e-09`, contact acceleration `7.305e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 126.196 cm |
| stance foot RMS | 95.801 cm |
| swing foot RMS | 137.885 cm |
| hand RMS | 142.778 cm |
| maximum root rotation | 179.407° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.145e-09 |
| contact acceleration residual | 1.758e-10 |
| raw max dynamics residual, including rejected ticks | 8.145e-09 |
| raw max contact residual, including rejected ticks | 1.758e-10 |
| active normal force range | 0.000–785.072 N |
| centroidal momentum-rate residual RMS / max | 83.535 / 499.512 N·m |
| point-task acceleration RMS max | 203.703 m/s² |
| frame-angular acceleration RMS max | 122.559 rad/s² |
| longest pre-contact / touchdown transition | 158 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3080.0 µs | 8921.9 µs | 13547.5 µs | 116484.3 µs | 138 | 90 | 158 | 0 | 160 | 54 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4074.9 | 5137.0 | 489.1 | 7226.9 | 55484.9 | 110384.4 | 4264.8 | 600 | 109 | 1 | 245.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 138 | 2639.3 | 2719.2 | 2770.7 | 2834.4 |
| solved_with_slack | 90 | 2959.0 | 5091.4 | 6959.4 | 8105.3 |
| normal_contact_contingency | 160 | 6136.6 | 12897.0 | 14024.1 | 14649.0 |
| contact_release_contingency | 54 | 2732.4 | 6477.8 | 62333.9 | 116484.3 |
| precontact_transition | 158 | 3273.6 | 3968.4 | 4191.0 | 4408.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.87 | 12.0 | 14.0 | 25 | 5.77 | 13.0 | 22 | 0.1852 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 4.76/8.0/8.0/8 | 1043.46/1776.0/1872.0/1872 | 2.35/14.0/14 | 1.81/13.0/13 | 14.63/111.0/113 | 0.2603 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 138 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 90 | 9.11/24.1/25 | 6.76/21.1/22 |
| normal_contact_contingency | 160 | 9.15/13.0/14 | 8.22/13.0/13 |
| contact_release_contingency | 54 | 7.70/10.5/11 | 6.43/9.5/10 |
| precontact_transition | 158 | 8.42/13.4/14 | 7.53/13.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.19/6.0/8 | 8.96/25.0/35 | 1.78/6.0/8 | 382 |
| viability | 1.86/7.0/13 | 10.73/40.0/56 | 1.61/7.0/13 | 452 |
| intent | 1.39/4.0/6 | 6.93/24.0/36 | 1.12/4.0/6 | 439 |
| preference | 1.27/6.0/17 | 12.82/54.0/192 | 0.82/6.0/16 | 350 |
| style | 1.15/4.0/5 | 9.57/29.0/40 | 0.43/3.0/5 | 202 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 4 | 199 | 168 |
| right_ankle_roll_link | 168 | 0 | 0 | 386 | 46 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `4` ticks, normal fallback `171` ticks.
Precontact sole-center tangential speed: p50 `1.4524 m/s`, p95 `3.5818 m/s`, max `5.6429 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `5.9062 m/s`, p95 `5.9520 m/s`, max `5.9570 m/s` over 3 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.445 | 2.444 | 2.444 | 1.000 | 1.000 | 47.434 | 47.590 | 0.156 | 47.965 | 0.001 | 0 | 39 | 0 | 0 | 32 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2659.1 | 4311.7 | 6.43 | 39.43 | 2.07 | 1.00 | 234.00 | 0.156 | 0.000 | 1.25e-09 | 4.91e-11 | 0 |
| 60–119 | 2638.4 | 3983.3 | 5.17 | 36.52 | 0.28 | 1.00 | 234.00 | 0.025 | 0.000 | 1.28e-09 | 7.30e-11 | 0 |
| 120–179 | 2668.5 | 5353.0 | 6.43 | 42.97 | 2.17 | 1.00 | 234.00 | 0.033 | 0.000 | 1.00e-09 | 5.04e-11 | 0 |
| 180–239 | 3182.2 | 7345.7 | 8.77 | 57.28 | 7.02 | 4.62 | 1038.50 | 4.085 | 2.967 | 1.21e-09 | 5.28e-11 | 12 |
| 240–299 | 3257.2 | 4015.5 | 8.45 | 50.98 | 7.32 | 6.37 | 1413.40 | 11.477 | 20.588 | 4.30e-10 | 1.86e-11 | 60 |
| 300–359 | 3145.9 | 4100.3 | 8.40 | 51.08 | 7.78 | 4.97 | 1102.60 | 16.108 | 39.818 | 1.01e-09 | 1.36e-11 | 60 |
| 360–419 | 3395.1 | 14374.9 | 8.50 | 47.87 | 7.47 | 6.83 | 1475.00 | 45.144 | 79.613 | 4.34e-10 | 1.72e-11 | 60 |
| 420–479 | 3958.1 | 53837.6 | 8.97 | 54.75 | 7.82 | 6.40 | 1376.00 | 137.067 | 144.662 | 3.18e-09 | 1.37e-10 | 60 |
| 480–539 | 6186.5 | 13804.3 | 8.83 | 55.52 | 8.17 | 7.65 | 1649.70 | 243.162 | 188.699 | 8.15e-09 | 1.76e-10 | 60 |
| 540–599 | 5772.2 | 11374.1 | 8.72 | 53.67 | 7.58 | 7.77 | 1677.40 | 280.880 | 243.770 | 2.35e-09 | 8.28e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 126.196 | 111.496 | 142.778 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
