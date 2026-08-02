# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `dcm-backward-preview` `DCM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `intent` priority with weight `0.250`.
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
| 366 | 1.830 s | 6.683 cm | 0.377 cm | 26.394 cm | 18.163 cm | 9.664° | 8.000 rad/s | 4482.5 µs |

Nominal hard residual maxima: dynamics `1.998e-09`, contact acceleration `5.283e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 39.615 cm |
| authored reference vs measured CoM RMS / p95 | 40.115 / 93.457 cm |
| stance foot RMS | 40.009 cm |
| swing foot RMS | 40.250 cm |
| hand RMS | 54.988 cm |
| maximum root rotation | 106.474° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.139e-09 |
| contact acceleration residual | 1.693e-10 |
| raw max dynamics residual, including rejected ticks | 1.014e-08 |
| raw max contact residual, including rejected ticks | 1.693e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 169.833 / 489.155 N·m |
| point-task acceleration RMS max | 220.610 m/s² |
| frame-angular acceleration RMS max | 264.663 rad/s² |
| longest pre-contact / touchdown transition | 138 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `19.469` / `38.140 cm`.
- Virtual ZMP clipped on `60.17%` of ticks; clip-distance RMS / max `23.289` / `75.050 cm`.
- Measured-height natural frequency min / p50 / max: `3.698` / `3.769` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.188 m`; height-floor ticks: `95`.
- CoM command acceleration p95 / max: `9.116` / `9.116 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2496.3 µs | 17826.6 µs | 186796.3 µs | 210326.8 µs | 224 | 4 | 138 | 0 | 157 | 9 | 0 | 68 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7633.6 | 23864.3 | 570.4 | 17383.0 | 209360.3 | 210230.2 | 177717.1 | 600 | 92 | 11 | 131.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 224 | 2429.0 | 2536.5 | 2583.1 | 2940.9 |
| solved_with_slack | 4 | 2315.5 | 2543.3 | 2562.3 | 2567.1 |
| failed | 68 | 17462.7 | 18228.7 | 18414.3 | 18455.7 |
| normal_contact_contingency | 157 | 3427.7 | 6111.9 | 44267.7 | 155637.1 |
| contact_release_contingency | 9 | 189938.7 | 209681.4 | 210197.8 | 210326.8 |
| precontact_transition | 138 | 2344.3 | 4364.8 | 5627.1 | 6640.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.09 | 10.0 | 11.0 | 15 | 3.95 | 10.0 | 14 | 0.1239 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 13.65/8.0/8.0/5904 | 2959.44/1776.0/1776.0/1275264 | 3.14/19.0/19 | 2.74/18.0/18 | 22.28/148.0/148 | 0.1370 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 224 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 4 | 7.50/9.0/9 | 5.25/7.9/8 |
| failed | 68 | 8.85/10.0/10 | 8.85/10.0/10 |
| normal_contact_contingency | 157 | 6.83/11.4/15 | 5.66/10.9/14 |
| contact_release_contingency | 9 | 6.22/7.0/7 | 5.00/6.0/6 |
| precontact_transition | 138 | 7.23/11.6/13 | 5.89/11.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.06/6.0/6 | 8.85/30.0/30 | 1.50/6.0/6 | 286 |
| viability | 1.59/5.0/9 | 9.00/35.0/63 | 1.20/5.0/9 | 371 |
| intent | 1.40/5.0/8 | 12.05/45.0/83 | 0.97/5.0/8 | 361 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.04/2.0/3 | 9.05/19.0/27 | 0.27/2.0/2 | 144 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 366 | 234 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `237` ticks.
Precontact sole-center tangential speed: p50 `1.4025 m/s`, p95 `3.7192 m/s`, max `4.0326 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.580 | 4.579 | 4.579 | 1.000 | 1.000 | 48.324 | 48.555 | 0.230 | 48.602 | 0.001 | 0 | 58 | 0 | 0 | 101 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2462.8 | 2739.5 | 4.00 | 25.02 | 0.00 | 1.00 | 234.00 | 0.269 | 0.000 | 1.29e-09 | 4.72e-11 | 0 |
| 60–119 | 2430.4 | 2573.0 | 4.00 | 25.50 | 0.00 | 1.00 | 234.00 | 1.114 | 0.000 | 1.43e-09 | 4.89e-11 | 0 |
| 120–179 | 2427.9 | 2510.1 | 4.00 | 25.02 | 0.00 | 1.00 | 234.00 | 3.171 | 0.000 | 2.00e-09 | 5.23e-11 | 0 |
| 180–239 | 1976.3 | 3164.9 | 5.08 | 32.17 | 1.63 | 1.12 | 251.70 | 7.819 | 0.519 | 1.45e-09 | 3.19e-11 | 12 |
| 240–299 | 2103.4 | 6236.7 | 7.07 | 45.87 | 5.60 | 2.22 | 492.10 | 7.273 | 15.157 | 1.48e-09 | 4.36e-11 | 60 |
| 300–359 | 2946.5 | 4765.4 | 7.27 | 45.28 | 6.10 | 4.38 | 973.10 | 11.731 | 26.096 | 7.58e-10 | 4.38e-11 | 60 |
| 360–419 | 2475.3 | 67906.4 | 6.97 | 47.68 | 5.53 | 3.68 | 798.30 | 8.657 | 29.245 | 1.22e-09 | 5.28e-11 | 60 |
| 420–479 | 3506.1 | 4705.7 | 6.48 | 41.12 | 5.42 | 7.88 | 1702.80 | 20.600 | 56.912 | 3.87e-09 | 1.69e-10 | 60 |
| 480–539 | 4678.7 | 209374.8 | 7.23 | 45.92 | 6.40 | 106.27 | 22946.40 | 72.372 | 74.964 | 1.01e-08 | 1.62e-10 | 60 |
| 540–599 | 17462.7 | 18419.3 | 8.80 | 55.95 | 8.80 | 8.00 | 1728.00 | 98.457 | 73.798 | 1.01e-08 | 1.62e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 39.615 | 40.089 | 54.988 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
