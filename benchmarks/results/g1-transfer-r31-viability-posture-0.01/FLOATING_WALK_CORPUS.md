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
- Whole-body posture: `viability` priority with weight `0.010`.
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
| 427 | 2.135 s | 14.676 cm | 0.740 cm | 21.982 cm | 22.685 cm | 53.248° | 8.000 rad/s | 4662.7 µs |

Nominal hard residual maxima: dynamics `1.939e-09`, contact acceleration `6.139e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 73.320 cm |
| stance foot RMS | 33.676 cm |
| swing foot RMS | 67.003 cm |
| hand RMS | 87.143 cm |
| maximum root rotation | 179.212° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.978e-09 |
| contact acceleration residual | 2.494e-10 |
| raw max dynamics residual, including rejected ticks | 7.978e-09 |
| raw max contact residual, including rejected ticks | 2.494e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 56.570 / 279.100 N·m |
| point-task acceleration RMS max | 172.798 m/s² |
| frame-angular acceleration RMS max | 108.748 rad/s² |
| longest pre-contact / touchdown transition | 199 / 6 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2500.8 µs | 70038.8 µs | 145407.2 µs | 343264.2 µs | 171 | 57 | 199 | 6 | 148 | 19 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9950.2 | 29797.6 | 508.3 | 6777.0 | 253721.2 | 334309.9 | 98485.3 | 600 | 72 | 40 | 100.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 171 | 2339.1 | 2425.6 | 2462.1 | 2501.0 |
| solved_with_slack | 57 | 2852.8 | 3769.3 | 4245.2 | 4673.5 |
| normal_contact_contingency | 148 | 3494.0 | 75911.7 | 139331.9 | 182445.0 |
| contact_release_contingency | 19 | 107469.1 | 208725.4 | 316356.5 | 343264.2 |
| touchdown_transition | 6 | 34563.7 | 76230.8 | 77974.4 | 78410.3 |
| precontact_transition | 199 | 2770.6 | 4232.3 | 4958.5 | 6774.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.01 | 9.0 | 11.0 | 12 | 3.48 | 10.0 | 11 | 0.1422 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 151.47/8.0/5024.4/6885 | 32886.08/1776.0/1093825.2/1487160 | 0.67/9.0/21 | 0.50/8.0/20 | 4.03/69.1/177 | 0.4802 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 171 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 57 | 6.88/11.4/12 | 4.04/8.9/10 |
| normal_contact_contingency | 148 | 7.13/12.0/12 | 5.47/10.5/11 |
| contact_release_contingency | 19 | 6.11/7.8/8 | 4.00/5.8/6 |
| touchdown_transition | 6 | 8.00/10.9/11 | 6.67/9.0/9 |
| precontact_transition | 199 | 6.59/10.0/12 | 4.69/9.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.85/6.0/7 | 7.70/25.0/35 | 1.31/6.0/7 | 280 |
| viability | 2.07/7.0/9 | 17.19/57.0/74 | 1.75/7.0/9 | 419 |
| intent | 1.00/1.0/1 | 4.33/5.0/6 | 0.00/0.0/0 | 0 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.09/3.0/7 | 8.55/29.1/60 | 0.41/3.0/6 | 211 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 10 | 199 | 162 |
| right_ankle_roll_link | 168 | 0 | 0 | 427 | 5 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `10` ticks, normal fallback `165` ticks.
Precontact sole-center tangential speed: p50 `1.9321 m/s`, p95 `3.8679 m/s`, max `4.1148 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `4.6313 m/s`, p95 `5.4899 m/s`, max `5.6025 m/s` over 9 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.970 | 5.969 | 5.969 | 1.000 | 1.000 | 47.613 | 47.836 | 0.223 | 47.906 | 0.001 | 0 | 39 | 0 | 0 | 39 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2335.1 | 2441.1 | 4.00 | 23.67 | 0.00 | 1.00 | 234.00 | 0.459 | 0.000 | 1.36e-09 | 6.14e-11 | 0 |
| 60–119 | 2368.4 | 2467.8 | 4.00 | 23.80 | 0.00 | 1.00 | 234.00 | 0.905 | 0.000 | 1.53e-09 | 5.15e-11 | 0 |
| 120–179 | 2381.5 | 4083.4 | 5.12 | 30.67 | 1.50 | 1.00 | 234.00 | 1.010 | 0.000 | 1.14e-09 | 4.70e-11 | 0 |
| 180–239 | 2663.1 | 3855.3 | 6.00 | 38.60 | 2.88 | 1.00 | 225.80 | 4.959 | 0.283 | 1.48e-09 | 3.06e-11 | 12 |
| 240–299 | 2799.5 | 4686.3 | 6.48 | 42.17 | 4.08 | 1.00 | 222.00 | 6.310 | 7.098 | 1.17e-09 | 1.71e-11 | 60 |
| 300–359 | 2584.5 | 4384.2 | 6.53 | 41.82 | 4.65 | 1.00 | 222.00 | 13.505 | 18.975 | 1.94e-09 | 2.00e-11 | 60 |
| 360–419 | 3075.0 | 6149.0 | 6.77 | 40.35 | 5.52 | 3.68 | 817.70 | 32.563 | 21.963 | 1.25e-09 | 1.70e-11 | 60 |
| 420–479 | 3149.8 | 248380.9 | 7.32 | 49.37 | 5.92 | 712.18 | 155441.00 | 59.134 | 44.714 | 4.11e-09 | 8.98e-11 | 54 |
| 480–539 | 6302.4 | 138883.9 | 6.98 | 43.32 | 5.13 | 735.05 | 158766.30 | 99.253 | 51.853 | 1.28e-09 | 2.31e-11 | 60 |
| 540–599 | 3888.1 | 192574.1 | 6.90 | 43.93 | 5.13 | 57.73 | 12464.00 | 197.738 | 129.847 | 7.98e-09 | 2.49e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 73.320 | 47.372 | 87.143 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
