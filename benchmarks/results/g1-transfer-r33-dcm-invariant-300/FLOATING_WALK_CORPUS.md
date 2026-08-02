# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-centroid-preview` reference at `invariant` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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

Functional: **PASS**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.500 s | 3.262 cm | 0.758 cm | 5.299 cm | 23.439 cm | 3.138° | 8.000 rad/s | 37168.4 µs |

Nominal hard residual maxima: dynamics `1.878e-09`, contact acceleration `5.855e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 3.262 cm |
| CoM RMS / p95 | 3.536 / 7.761 cm |
| stance foot RMS | 0.758 cm |
| swing foot RMS | 5.299 cm |
| hand RMS | 23.439 cm |
| maximum root rotation | 3.138° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.878e-09 |
| contact acceleration residual | 5.855e-11 |
| raw max dynamics residual, including rejected ticks | 1.878e-09 |
| raw max contact residual, including rejected ticks | 5.855e-11 |
| active normal force range | 0.000–230.893 N |
| centroidal momentum-rate residual RMS / max | 7.542 / 30.643 N·m |
| point-task acceleration RMS max | 69.449 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `1.927` / `4.975 cm`.
- Virtual ZMP clipped on `22.67%` of ticks; clip-distance RMS / max `2.730` / `11.504 cm`.
- Measured-height natural frequency min / p50 / max: `3.707` / `3.777` / `3.787 rad/s`.
- CoM command acceleration p95 / max: `5.618` / `6.215 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.5 s | 2412.8 µs | 3121.7 µs | 37168.4 µs | 53070.9 µs | 158 | 142 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2977.2 | 4915.4 | 90.8 | 3006.6 | 51841.7 | 52948.0 | 35395.3 | 300 | 5 | 4 | 335.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 158 | 2398.4 | 2532.3 | 2786.3 | 3201.4 |
| solved_with_slack | 142 | 2480.8 | 3615.0 | 44776.0 | 53070.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.98 | 11.0 | 13.0 | 22 | 2.96 | 13.0 | 20 | 0.1956 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 37.64/8.0/2221.2/3175 | 8363.30/1776.0/493099.7/704850 | 0.33/3.0/9 | 0.19/2.0/8 | 1.54/15.0/65 | 0.9946 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 158 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 142 | 8.18/14.6/22 | 6.25/14.6/20 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.88/9.0/19 | 7.53/36.1/76 | 1.09/9.0/18 | 78 |
| viability | 0.66/4.0/6 | 2.91/20.0/30 | 0.52/4.0/6 | 69 |
| intent | 1.41/5.0/5 | 7.16/25.0/30 | 0.89/5.0/5 | 142 |
| preference | 1.01/2.0/2 | 10.86/20.0/24 | 0.30/1.0/2 | 88 |
| style | 1.01/1.0/3 | 9.92/13.0/17 | 0.16/1.0/3 | 46 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 101 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 300 | 0 |
| left_wrist_roll_rubber_hand | 300 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 300 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.893 | 0.892 | 0.892 | 0.999 | 0.999 | 47.195 | 47.195 | 0.000 | 47.859 | 0.001 | 0 | 0 | 0 | 0 | 73 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–29 | 2400.3 | 3013.8 | 4.00 | 31.30 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.88e-09 | 4.95e-11 | 0 |
| 30–59 | 2391.5 | 2496.7 | 4.00 | 31.50 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 9.00e-10 | 5.86e-11 | 0 |
| 60–89 | 2434.1 | 2591.6 | 4.00 | 30.80 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 9.81e-10 | 5.70e-11 | 0 |
| 90–119 | 2412.8 | 2505.7 | 4.00 | 30.83 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.43e-09 | 3.94e-11 | 0 |
| 120–149 | 2372.3 | 2871.0 | 4.00 | 29.97 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.25e-09 | 4.22e-11 | 0 |
| 150–179 | 2485.0 | 2801.8 | 8.90 | 49.13 | 6.90 | 1.00 | 234.00 | 0.426 | 0.000 | 1.23e-09 | 3.32e-11 | 0 |
| 180–209 | 2433.3 | 2856.5 | 7.70 | 44.23 | 5.50 | 1.23 | 281.40 | 2.977 | 0.001 | 1.21e-09 | 5.20e-11 | 0 |
| 210–239 | 1700.7 | 3168.9 | 7.20 | 43.30 | 4.63 | 3.80 | 843.60 | 4.430 | 0.145 | 7.85e-10 | 8.91e-12 | 0 |
| 240–269 | 3008.4 | 3667.4 | 7.80 | 45.47 | 5.97 | 7.30 | 1620.60 | 5.627 | 1.325 | 2.10e-10 | 2.10e-12 | 0 |
| 270–299 | 1807.1 | 51878.7 | 8.20 | 47.30 | 6.57 | 358.03 | 79483.40 | 6.790 | 7.091 | 9.62e-10 | 2.61e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 300 | 3.262 | 2.282 | 23.439 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
