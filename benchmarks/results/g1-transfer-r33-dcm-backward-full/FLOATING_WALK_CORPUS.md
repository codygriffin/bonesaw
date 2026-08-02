# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `dcm-backward-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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

Failed checks: `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 427 | 2.135 s | 16.355 cm | 12.026 cm | 46.080 cm | 34.887 cm | 62.586° | 8.000 rad/s | 56502.1 µs |

Nominal hard residual maxima: dynamics `3.196e-09`, contact acceleration `1.125e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 30.346 cm |
| CoM RMS / p95 | 40.138 / 86.419 cm |
| stance foot RMS | 51.837 cm |
| swing foot RMS | 47.778 cm |
| hand RMS | 44.187 cm |
| maximum root rotation | 64.219° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.196e-09 |
| contact acceleration residual | 1.125e-10 |
| raw max dynamics residual, including rejected ticks | 3.196e-09 |
| raw max contact residual, including rejected ticks | 1.125e-10 |
| active normal force range | 0.000–491.299 N |
| centroidal momentum-rate residual RMS / max | 34.731 / 188.731 N·m |
| point-task acceleration RMS max | 96.381 m/s² |
| frame-angular acceleration RMS max | 178.136 rad/s² |
| longest pre-contact / touchdown transition | 199 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `43.355` / `87.305 cm`.
- Virtual ZMP clipped on `61.50%` of ticks; clip-distance RMS / max `67.502` / `180.286 cm`.
- Measured-height natural frequency min / p50 / max: `3.573` / `3.769` / `5.134 rad/s`.
- CoM command acceleration p95 / max: `39.147` / `56.202 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2879.4 µs | 10315.4 µs | 48085.6 µs | 156089.4 µs | 97 | 131 | 199 | 0 | 173 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4533.5 | 10217.0 | 441.5 | 4278.3 | 126055.7 | 153086.1 | 27671.2 | 600 | 42 | 18 | 220.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 97 | 2426.8 | 2493.2 | 2538.2 | 2540.9 |
| solved_with_slack | 131 | 2554.2 | 4640.9 | 5322.4 | 5591.6 |
| normal_contact_contingency | 173 | 2827.0 | 21798.2 | 31250.7 | 156089.4 |
| precontact_transition | 199 | 3144.3 | 10462.6 | 77383.5 | 105949.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.47 | 11.0 | 12.0 | 16 | 5.31 | 12.0 | 16 | 0.0730 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 99.93/229.0/2921.6/6638 | 21913.28/49474.8/648401.8/1473636 | 1.33/6.0/11 | 0.81/5.0/10 | 6.63/41.3/85 | 0.9034 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 97 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 131 | 6.85/10.0/11 | 4.37/8.7/9 |
| normal_contact_contingency | 173 | 8.43/12.0/13 | 6.75/11.0/12 |
| precontact_transition | 199 | 8.24/13.0/16 | 7.26/12.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.89/5.0/6 | 7.60/20.0/26 | 1.30/5.0/6 | 359 |
| viability | 1.81/6.0/7 | 9.98/42.0/47 | 1.47/6.0/7 | 419 |
| intent | 1.65/5.0/7 | 8.86/25.0/42 | 1.49/5.0/7 | 503 |
| preference | 1.10/3.0/6 | 10.59/31.0/63 | 0.69/3.0/6 | 363 |
| style | 1.02/2.0/5 | 9.34/17.0/24 | 0.36/1.0/5 | 207 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 427 | 173 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `176` ticks.
Precontact sole-center tangential speed: p50 `1.8694 m/s`, p95 `6.6466 m/s`, max `7.0562 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.720 | 2.720 | 2.720 | 1.000 | 1.000 | 47.141 | 47.434 | 0.293 | 48.344 | 0.001 | 0 | 59 | 0 | 0 | 45 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2420.3 | 2539.3 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2450.8 | 3498.3 | 5.73 | 38.37 | 1.63 | 1.00 | 234.00 | 0.087 | 0.000 | 1.36e-09 | 5.46e-11 | 0 |
| 120–179 | 2588.7 | 5438.4 | 6.95 | 42.42 | 4.78 | 3.10 | 725.40 | 1.481 | 0.000 | 1.36e-09 | 5.99e-11 | 0 |
| 180–239 | 2950.9 | 4353.3 | 6.85 | 40.98 | 4.25 | 5.20 | 1163.80 | 4.562 | 0.481 | 1.16e-09 | 5.83e-11 | 12 |
| 240–299 | 3164.9 | 4118.3 | 8.28 | 49.63 | 7.07 | 8.00 | 1776.00 | 10.051 | 12.823 | 2.86e-10 | 9.39e-12 | 60 |
| 300–359 | 3080.2 | 6589.8 | 8.05 | 48.97 | 7.22 | 13.88 | 3082.10 | 14.543 | 33.411 | 3.69e-10 | 1.27e-11 | 60 |
| 360–419 | 3135.7 | 80321.1 | 8.32 | 49.07 | 7.53 | 391.35 | 86879.70 | 35.048 | 50.705 | 5.59e-10 | 2.01e-11 | 60 |
| 420–479 | 2907.6 | 126507.0 | 8.70 | 53.30 | 7.65 | 317.05 | 69151.40 | 62.014 | 97.204 | 3.20e-09 | 1.12e-10 | 60 |
| 480–539 | 2128.6 | 24695.6 | 8.50 | 54.75 | 6.12 | 253.00 | 54648.00 | 45.242 | 68.009 | 8.47e-10 | 1.49e-11 | 60 |
| 540–599 | 2802.7 | 3959.5 | 8.30 | 52.62 | 6.83 | 5.73 | 1238.40 | 41.863 | 87.221 | 9.62e-10 | 9.96e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 30.346 | 50.530 | 44.187 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
