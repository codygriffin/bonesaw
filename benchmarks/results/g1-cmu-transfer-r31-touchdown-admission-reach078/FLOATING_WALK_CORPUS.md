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
| `no_infeasible_or_failed_ticks` | FAIL |
| `no_contact_contingency_ticks` | FAIL |
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

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 455 | 2.275 s | 19.554 cm | 20.541 cm | 39.768 cm | 42.624 cm | 73.967° | 8.000 rad/s | 42237.9 µs |

Nominal hard residual maxima: dynamics `3.450e-09`, contact acceleration `1.220e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 79.106 cm |
| stance foot RMS | 58.838 cm |
| swing foot RMS | 70.536 cm |
| hand RMS | 88.689 cm |
| maximum root rotation | 179.537° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.574e-09 |
| contact acceleration residual | 1.761e-10 |
| raw max dynamics residual, including rejected ticks | 2.234e+02 |
| raw max contact residual, including rejected ticks | 1.761e-10 |
| active normal force range | 0.000–684.264 N |
| centroidal momentum-rate residual RMS / max | 55.811 / 246.808 N·m |
| point-task acceleration RMS max | 139.268 m/s² |
| frame-angular acceleration RMS max | 332.892 rad/s² |
| longest pre-contact / touchdown transition | 227 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3104.7 µs | 229655.1 µs | 232603.9 µs | 314954.5 µs | 141 | 87 | 227 | 0 | 99 | 11 | 35 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 21631.1 | 58817.2 | 667.1 | 25538.6 | 278829.0 | 311341.9 | 80193.1 | 600 | 125 | 67 | 46.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2479.9 | 2584.8 | 2617.8 | 2629.8 |
| solved_with_slack | 87 | 2785.0 | 4913.0 | 5462.7 | 6143.3 |
| primal_infeasible | 35 | 230433.1 | 241198.6 | 294449.2 | 314954.5 |
| normal_contact_contingency | 99 | 8588.8 | 26387.0 | 30929.1 | 116121.8 |
| contact_release_contingency | 11 | 192406.3 | 212718.3 | 215597.4 | 216317.1 |
| precontact_transition | 227 | 3257.0 | 4923.1 | 44701.0 | 49650.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.25 | 11.0 | 13.0 | 16 | 5.14 | 12.0 | 14 | -0.5781 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 467.39/6720.0/6720.0/6720 | 98852.02/1411200.0/1411200.0/1411200 | 4.56/46.0/46 | 4.06/45.0/45 | 33.19/370.0/370 | 0.8930 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 87 | 8.29/12.6/16 | 5.83/11.3/13 |
| primal_infeasible | 35 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 99 | 9.40/13.0/14 | 8.64/13.0/14 |
| contact_release_contingency | 11 | 7.36/8.0/8 | 6.18/7.0/7 |
| precontact_transition | 227 | 8.42/13.0/14 | 7.30/11.7/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.83/5.0/6 | 7.62/25.0/30 | 1.37/5.0/6 | 320 |
| viability | 1.90/7.0/12 | 11.32/42.0/56 | 1.65/7.0/12 | 417 |
| intent | 1.31/4.0/6 | 6.87/23.0/34 | 1.04/4.0/6 | 399 |
| preference | 1.14/4.0/7 | 11.14/40.0/77 | 0.70/4.0/6 | 320 |
| style | 1.07/4.0/6 | 9.56/31.0/41 | 0.39/3.0/6 | 177 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 455 | 145 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `148` ticks.
Precontact sole-center tangential speed: p50 `2.3321 m/s`, p95 `6.3796 m/s`, max `9.3412 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12.979 | 12.976 | 12.976 | 1.000 | 1.000 | 47.648 | 47.805 | 0.156 | 47.859 | 0.001 | 0 | 39 | 0 | 0 | 260 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2505.9 | 2948.2 | 6.07 | 36.82 | 1.67 | 1.00 | 234.00 | 0.172 | 0.000 | 1.62e-09 | 5.63e-11 | 0 |
| 60–119 | 2470.3 | 3561.9 | 5.10 | 34.98 | 0.17 | 1.00 | 234.00 | 0.032 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–179 | 2527.6 | 5293.3 | 6.12 | 39.43 | 1.62 | 1.12 | 261.30 | 0.002 | 0.000 | 1.46e-09 | 6.30e-11 | 0 |
| 180–239 | 2834.0 | 5552.5 | 8.28 | 53.07 | 6.50 | 3.92 | 883.10 | 3.367 | 3.391 | 1.47e-09 | 5.34e-11 | 12 |
| 240–299 | 3180.3 | 4146.6 | 8.50 | 56.97 | 7.17 | 6.13 | 1361.60 | 9.191 | 21.270 | 6.67e-10 | 1.61e-11 | 60 |
| 300–359 | 3309.2 | 4233.6 | 8.25 | 51.35 | 7.25 | 7.30 | 1620.60 | 16.264 | 28.529 | 9.25e-10 | 1.00e-11 | 60 |
| 360–419 | 3271.6 | 13355.3 | 8.40 | 53.88 | 7.25 | 28.43 | 6312.20 | 30.200 | 39.227 | 3.45e-09 | 1.22e-10 | 60 |
| 420–479 | 4031.3 | 76903.7 | 8.73 | 56.77 | 7.82 | 695.17 | 152301.80 | 64.810 | 73.904 | 8.72e-10 | 2.43e-11 | 60 |
| 480–539 | 6968.1 | 212070.6 | 8.85 | 54.70 | 8.00 | 6.48 | 1391.60 | 137.181 | 113.920 | 9.57e-09 | 1.76e-10 | 60 |
| 540–599 | 229662.0 | 279371.7 | 4.18 | 27.17 | 4.02 | 3923.33 | 823920.00 | 195.669 | 135.535 | 2.23e+02 | 1.25e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 79.106 | 62.949 | 88.689 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
