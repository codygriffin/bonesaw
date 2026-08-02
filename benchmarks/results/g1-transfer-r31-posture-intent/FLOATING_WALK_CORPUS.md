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
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.000 s | 31.268 cm | 27.282 cm | 47.646 cm | 32.305 cm | 5.668° | 8.000 rad/s | 40198.9 µs |

Nominal hard residual maxima: dynamics `1.729e-09`, contact acceleration `6.161e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 31.268 cm |
| stance foot RMS | 27.282 cm |
| swing foot RMS | 47.646 cm |
| hand RMS | 32.305 cm |
| maximum root rotation | 5.668° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.729e-09 |
| contact acceleration residual | 6.161e-11 |
| raw max dynamics residual, including rejected ticks | 1.729e-09 |
| raw max contact residual, including rejected ticks | 6.161e-11 |
| active normal force range | 0.000–466.451 N |
| centroidal momentum-rate residual RMS / max | 25.660 / 82.038 N·m |
| point-task acceleration RMS max | 105.879 m/s² |
| frame-angular acceleration RMS max | 106.718 rad/s² |
| longest pre-contact / touchdown transition | 200 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2490.3 µs | 4283.5 µs | 40198.9 µs | 89070.6 µs | 148 | 80 | 200 | 172 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3756.5 | 7760.1 | 424.3 | 3528.0 | 87172.2 | 88880.8 | 17403.1 | 600 | 21 | 16 | 266.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 148 | 2484.0 | 2634.0 | 2857.3 | 2867.4 |
| solved_with_slack | 80 | 2654.2 | 3864.5 | 4528.1 | 4844.3 |
| touchdown_transition | 172 | 2902.8 | 36110.9 | 80416.6 | 89070.6 |
| precontact_transition | 200 | 2102.0 | 3515.5 | 4095.1 | 4281.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.58 | 10.0 | 12.0 | 16 | 4.29 | 10.0 | 13 | 0.1408 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 72.15/8.0/2491.8/4371 | 15910.37/1776.0/538222.3/996588 | 0.47/4.0/5 | 0.27/3.0/4 | 2.17/24.0/32 | 0.9917 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 148 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 80 | 7.59/12.8/16 | 5.14/11.4/13 |
| touchdown_transition | 172 | 7.30/12.3/14 | 5.66/10.3/12 |
| precontact_transition | 200 | 7.46/11.0/12 | 5.94/10.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.45/4.0/5 | 5.51/16.0/20 | 0.67/4.0/5 | 207 |
| viability | 2.50/7.0/13 | 14.87/47.0/56 | 2.25/7.0/13 | 452 |
| intent | 1.62/6.0/7 | 14.72/54.0/68 | 1.19/6.0/6 | 382 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.01/2.0/2 | 8.68/16.0/24 | 0.19/1.0/2 | 110 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 172 | 199 | 0 |
| right_ankle_roll_link | 168 | 0 | 0 | 432 | 0 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `172` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `1.4033 m/s`, p95 `3.1587 m/s`, max `3.2920 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `2.6992 m/s`, p95 `3.3525 m/s`, max `3.5014 m/s` over 171 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.254 | 2.254 | 2.253 | 1.000 | 1.000 | 46.688 | 46.914 | 0.227 | 47.840 | 0.001 | 0 | 40 | 0 | 0 | 29 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2516.8 | 3224.4 | 5.03 | 29.83 | 1.30 | 1.00 | 234.00 | 0.803 | 0.000 | 1.61e-09 | 5.50e-11 | 0 |
| 60–119 | 2478.9 | 2860.1 | 4.00 | 26.17 | 0.00 | 1.00 | 234.00 | 1.323 | 0.000 | 1.37e-09 | 4.71e-11 | 0 |
| 120–179 | 2509.5 | 3685.9 | 5.30 | 32.13 | 1.75 | 1.00 | 234.00 | 1.376 | 0.000 | 1.50e-09 | 6.16e-11 | 0 |
| 180–239 | 3072.2 | 4608.2 | 7.28 | 48.00 | 5.10 | 4.15 | 925.10 | 5.630 | 5.291 | 1.50e-09 | 4.23e-11 | 12 |
| 240–299 | 1930.1 | 4171.4 | 7.33 | 49.22 | 5.77 | 1.93 | 429.20 | 8.790 | 24.937 | 9.35e-10 | 1.07e-11 | 60 |
| 300–359 | 2261.1 | 3584.4 | 7.67 | 51.45 | 6.30 | 2.52 | 558.70 | 8.092 | 20.372 | 4.93e-10 | 1.73e-11 | 60 |
| 360–419 | 2042.1 | 3318.3 | 7.23 | 47.32 | 5.70 | 1.35 | 299.70 | 17.368 | 27.564 | 1.73e-09 | 1.68e-11 | 60 |
| 420–479 | 2302.1 | 87200.7 | 7.25 | 50.70 | 5.25 | 313.73 | 70908.60 | 19.765 | 47.624 | 1.18e-09 | 7.81e-12 | 8 |
| 480–539 | 3589.6 | 40433.9 | 7.33 | 52.15 | 6.02 | 390.43 | 84333.60 | 32.821 | 42.040 | 2.58e-10 | 3.57e-12 | 0 |
| 540–599 | 2801.6 | 4627.9 | 7.33 | 50.95 | 5.70 | 4.38 | 946.80 | 88.478 | 81.431 | 3.12e-10 | 4.74e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 31.268 | 35.343 | 32.305 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
