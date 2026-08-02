# Bonesaw floating G1 synthetic-step acceptance

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: deterministic, predeclared G1-sized left step; no mocap or learned policy contributes contact labels or target motion.
- Contact schedule: liftoff tick `53`, touchdown tick `93`, step `0.010 m`, clearance `0.010 m`.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.010 m` forward per `0.800 s` source cycle.
- Applied mean forward speed: `0.012 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `constant synthetic phase rate` in `0.8 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-centroid-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `40` ticks (`0.200 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
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
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.800 s | 8.532 cm | 3.604 cm | 0.949 cm | 33.499 cm | 4.739° | 8.000 rad/s | 19005.4 µs |

Nominal hard residual maxima: dynamics `1.361e-09`, contact acceleration `5.033e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 8.532 cm |
| CoM RMS / p95 | 8.841 / 15.924 cm |
| stance foot RMS | 3.604 cm |
| swing foot RMS | 0.949 cm |
| hand RMS | 33.499 cm |
| maximum root rotation | 4.739° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.361e-09 |
| contact acceleration residual | 5.033e-11 |
| raw max dynamics residual, including rejected ticks | 1.361e-09 |
| raw max contact residual, including rejected ticks | 5.033e-11 |
| active normal force range | 0.000–289.596 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 42.987 m/s² |
| frame-angular acceleration RMS max | 46.713 rad/s² |
| longest pre-contact / touchdown transition | 42 / 65 ticks |
| delayed touchdown admission ticks / longest delay | 2 / 2 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `13.845` / `26.276 cm`.
- Virtual ZMP clipped on `74.38%` of ticks; clip-distance RMS / max `19.757` / `46.415 cm`.
- Measured-height natural frequency min / p50 / max: `3.710` / `3.766` / `3.803 rad/s`.
- CoM command acceleration p95 / max: `6.470` / `7.215 m/s²`; support hull `4–6` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 2870.3 µs | 4609.0 µs | 19005.4 µs | 95280.0 µs | 12 | 41 | 42 | 65 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3800.2 | 7834.0 | 492.9 | 4259.5 | 86308.4 | 94382.9 | 43911.4 | 160 | 4 | 2 | 263.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 12 | 2413.3 | 3390.6 | 3454.0 | 3469.8 |
| solved_with_slack | 41 | 2487.2 | 3309.8 | 3634.4 | 3718.2 |
| touchdown_transition | 65 | 4074.7 | 5030.0 | 59167.9 | 95280.0 |
| precontact_transition | 42 | 2135.0 | 3246.1 | 3395.7 | 3476.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 10.58 | 17.0 | 23.0 | 23 | 8.23 | 20.0 | 21 | 0.1002 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 40.28/8.0/701.3/4238 | 9180.34/1824.0/159898.7/966264 | 0.64/2.0/2 | 0.33/1.0/1 | 2.76/9.0/9 | 0.9945 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 12 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 41 | 13.41/23.0/23 | 10.83/20.6/21 |
| touchdown_transition | 65 | 11.43/17.4/18 | 9.49/15.4/16 |
| precontact_transition | 42 | 8.10/12.0/12 | 6.10/11.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.77/7.0/8 | 6.98/28.0/32 | 1.08/6.4/8 | 77 |
| viability | 4.03/13.4/14 | 15.53/52.0/60 | 3.67/13.0/13 | 135 |
| intent | 1.90/8.4/10 | 3.84/16.8/20 | 1.82/8.4/10 | 148 |
| preference | 1.71/6.4/7 | 15.56/58.2/64 | 1.23/6.0/6 | 110 |
| style | 1.16/3.0/6 | 11.36/33.0/68 | 0.43/2.0/5 | 60 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 42 | 65 | 53 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 160 | 0 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `42` ticks, planned normal touchdown `65` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.1439 m/s`, p95 `0.1962 m/s`, max `0.2184 m/s` over 42 samples.
Touchdown Normal sole-center tangential speed: p50 `0.4443 m/s`, p95 `1.3462 m/s`, max `1.7246 m/s` over 65 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.608 | 0.608 | 0.608 | 1.000 | 1.000 | 42.402 | 42.605 | 0.203 | 42.605 | 0.001 | 0 | 51 | 0 | 0 | 6 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2413.3 | 3448.2 | 7.62 | 32.75 | 3.12 | 1.00 | 234.00 | 0.008 | 0.000 | 1.36e-09 | 3.85e-11 | 0 |
| 16–31 | 2435.0 | 3183.0 | 14.88 | 51.56 | 12.56 | 1.00 | 234.00 | 0.352 | 0.000 | 5.82e-10 | 5.03e-11 | 0 |
| 32–47 | 2547.7 | 3478.8 | 11.75 | 47.38 | 9.06 | 1.00 | 234.00 | 1.585 | 0.000 | 9.48e-10 | 3.94e-11 | 0 |
| 48–63 | 2330.8 | 3611.1 | 8.88 | 42.69 | 6.31 | 1.00 | 225.75 | 3.115 | 0.222 | 9.63e-10 | 2.55e-11 | 11 |
| 64–79 | 1760.9 | 2950.9 | 7.69 | 41.44 | 4.94 | 1.44 | 319.12 | 3.971 | 0.934 | 1.21e-09 | 1.84e-11 | 16 |
| 80–95 | 3164.4 | 81509.6 | 9.38 | 55.94 | 8.50 | 272.38 | 62056.50 | 5.892 | 0.458 | 3.07e-12 | 1.54e-13 | 15 |
| 96–111 | 4265.5 | 33808.4 | 11.19 | 63.69 | 8.94 | 113.69 | 25920.75 | 8.615 | 1.540 | 2.61e-12 | 1.10e-13 | 0 |
| 112–127 | 4180.2 | 5044.1 | 10.38 | 56.81 | 8.38 | 7.12 | 1624.50 | 11.501 | 2.741 | 5.25e-10 | 3.87e-12 | 0 |
| 128–143 | 3123.7 | 4654.7 | 12.62 | 76.06 | 10.25 | 1.00 | 228.00 | 13.896 | 4.551 | 9.00e-10 | 1.36e-11 | 0 |
| 144–159 | 2903.0 | 4702.6 | 11.44 | 64.44 | 10.25 | 3.19 | 726.75 | 16.297 | 9.114 | 6.63e-10 | 1.31e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 8.532 | 3.388 | 33.499 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
