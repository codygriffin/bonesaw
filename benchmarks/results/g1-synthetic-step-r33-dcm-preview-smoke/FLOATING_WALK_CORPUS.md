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
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.800 s | 11.872 cm | 16.351 cm | 3.310 cm | 34.324 cm | 10.588° | 8.000 rad/s | 32009.1 µs |

Nominal hard residual maxima: dynamics `1.397e-09`, contact acceleration `6.035e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 11.872 cm |
| CoM RMS / p95 | 11.121 / 20.976 cm |
| stance foot RMS | 16.351 cm |
| swing foot RMS | 3.310 cm |
| hand RMS | 34.324 cm |
| maximum root rotation | 10.588° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.397e-09 |
| contact acceleration residual | 6.035e-11 |
| raw max dynamics residual, including rejected ticks | 1.397e-09 |
| raw max contact residual, including rejected ticks | 6.035e-11 |
| active normal force range | 0.000–375.799 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 49.627 m/s² |
| frame-angular acceleration RMS max | 69.515 rad/s² |
| longest pre-contact / touchdown transition | 107 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 67 / 67 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `22.355` / `33.241 cm`.
- Virtual ZMP clipped on `93.12%` of ticks; clip-distance RMS / max `93.628` / `623.208 cm`.
- Measured-height natural frequency min / p50 / max: `3.769` / `3.786` / `3.862 rad/s`.
- CoM command acceleration p95 / max: `8.774` / `11.209 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 2510.5 µs | 4236.3 µs | 32009.1 µs | 35993.0 µs | 11 | 42 | 107 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3494.7 | 4797.0 | 620.3 | 3765.1 | 35710.6 | 35964.8 | 27462.8 | 160 | 6 | 4 | 286.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 11 | 2475.1 | 2653.7 | 2782.1 | 2814.2 |
| solved_with_slack | 42 | 2500.2 | 3534.7 | 3761.3 | 3896.9 |
| precontact_transition | 107 | 2916.3 | 6634.9 | 33992.2 | 35993.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 10.02 | 17.0 | 22.4 | 23 | 8.19 | 19.4 | 20 | -0.0273 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 54.80/8.0/1861.1/2108 | 12169.58/1776.0/413168.6/467976 | 0.79/3.0/7 | 0.47/2.0/6 | 3.83/16.8/48 | 0.9872 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 11 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 42 | 14.55/23.0/23 | 12.33/20.0/20 |
| precontact_transition | 107 | 8.76/13.9/14 | 7.41/13.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.70/4.4/5 | 6.67/17.6/20 | 1.09/4.4/5 | 85 |
| viability | 3.94/14.0/14 | 18.99/53.0/54 | 3.79/13.0/13 | 149 |
| intent | 1.82/7.4/8 | 3.83/15.4/16 | 1.72/7.0/7 | 146 |
| preference | 1.51/5.0/7 | 13.96/46.6/60 | 1.16/4.4/6 | 125 |
| style | 1.04/2.0/2 | 8.63/20.2/23 | 0.42/2.0/2 | 65 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 107 | 0 | 53 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 160 | 0 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `107` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `1.0929 m/s`, p95 `3.3540 m/s`, max `3.4488 m/s` over 107 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.559 | 0.559 | 0.559 | 1.000 | 0.999 | 42.383 | 42.695 | 0.312 | 42.695 | 0.001 | 0 | 62 | 0 | 0 | 8 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2440.7 | 3072.2 | 8.00 | 40.75 | 3.81 | 1.00 | 234.00 | 0.014 | 0.000 | 1.40e-09 | 3.77e-11 | 0 |
| 16–31 | 2547.2 | 3847.3 | 15.44 | 64.81 | 13.25 | 1.00 | 234.00 | 0.333 | 0.000 | 1.25e-09 | 3.27e-11 | 0 |
| 32–47 | 2527.2 | 3533.1 | 14.31 | 69.19 | 12.06 | 1.00 | 234.00 | 1.558 | 0.000 | 1.18e-09 | 6.03e-11 | 0 |
| 48–63 | 2164.4 | 3707.5 | 9.56 | 52.00 | 7.12 | 1.44 | 322.88 | 3.687 | 0.178 | 5.00e-10 | 3.12e-11 | 11 |
| 64–79 | 3690.3 | 4222.1 | 9.56 | 51.25 | 8.19 | 8.00 | 1776.00 | 5.541 | 1.508 | 8.70e-12 | 3.82e-13 | 16 |
| 80–95 | 3110.5 | 4587.5 | 8.44 | 47.44 | 7.31 | 8.00 | 1776.00 | 8.985 | 4.110 | 7.96e-13 | 3.91e-14 | 16 |
| 96–111 | 3050.9 | 3425.3 | 8.31 | 45.75 | 6.19 | 6.25 | 1387.50 | 13.060 | 8.482 | 2.08e-10 | 2.86e-12 | 16 |
| 112–127 | 1980.1 | 13840.8 | 8.75 | 46.88 | 7.81 | 52.25 | 11599.50 | 16.889 | 14.585 | 3.12e-10 | 9.35e-12 | 16 |
| 128–143 | 1806.0 | 2914.5 | 9.19 | 53.00 | 8.31 | 1.00 | 222.00 | 19.555 | 24.750 | 7.56e-10 | 1.95e-11 | 16 |
| 144–159 | 1850.3 | 35726.6 | 8.62 | 49.69 | 7.88 | 468.06 | 103909.88 | 21.061 | 37.904 | 8.04e-10 | 7.60e-12 | 16 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 11.872 | 15.340 | 34.324 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
