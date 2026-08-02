# Bonesaw floating G1 synthetic-step acceptance

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: deterministic, predeclared G1-sized left step; no mocap or learned policy contributes contact labels or target motion.
- Contact schedule: liftoff tick `53`, touchdown tick `93`, step `0.010 m`, clearance `0.010 m`.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.010 m` forward per `0.800 s` source cycle.
- Applied mean forward speed: `0.012 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `constant synthetic phase rate` in `0.8 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `rooted` reference at `viability` priority with weight `0.000` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `40` ticks (`0.200 s`) with a receding cubic landing law capped at `25.000 m/s²`; contact forces remain off until the authored contact edge.
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
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `root_tracking_rms_le_5cm`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.800 s | 5.204 cm | 0.592 cm | 0.639 cm | 26.870 cm | 2.383° | 8.000 rad/s | 6151.7 µs |

Nominal hard residual maxima: dynamics `2.124e-09`, contact acceleration `8.149e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 5.204 cm |
| stance foot RMS | 0.592 cm |
| swing foot RMS | 0.639 cm |
| hand RMS | 26.870 cm |
| maximum root rotation | 2.383° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.124e-09 |
| contact acceleration residual | 8.149e-11 |
| raw max dynamics residual, including rejected ticks | 2.124e-09 |
| raw max contact residual, including rejected ticks | 8.149e-11 |
| active normal force range | 0.000–241.268 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 0.002 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 40 / 3 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 2490.3 µs | 5096.7 µs | 6151.7 µs | 6270.0 µs | 53 | 64 | 40 | 3 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3096.2 | 1143.8 | 483.9 | 4747.4 | 6264.8 | 6269.4 | 1776.3 | 160 | 9 | 0 | 323.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 53 | 2472.2 | 2980.0 | 3669.9 | 3771.7 |
| solved_with_slack | 64 | 4357.8 | 6022.5 | 6249.6 | 6270.0 |
| touchdown_transition | 3 | 4272.0 | 4368.1 | 4376.7 | 4378.8 |
| precontact_transition | 40 | 2001.2 | 2580.0 | 3091.0 | 3410.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.88 | 12.0 | 14.4 | 15 | 4.03 | 13.0 | 14 | 0.4587 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.00/0.0/0.0/0 | 0.00/0.0/0.0/0 | 1.26/2.0/2 | 0.26/1.0/1 | 2.27/9.4/10 | 0.0000 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 53 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 64 | 8.61/15.0/15 | 7.30/13.4/14 |
| touchdown_transition | 3 | 9.33/11.0/11 | 6.67/9.0/9 |
| precontact_transition | 40 | 7.75/11.0/11 | 3.95/7.6/8 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.61/6.4/8 | 6.12/25.6/32 | 0.82/6.4/8 | 60 |
| viability | 0.27/1.0/1 | 1.31/5.0/5 | 0.00/0.0/0 | 0 |
| intent | 1.69/6.4/7 | 3.41/12.8/14 | 1.16/6.4/7 | 76 |
| preference | 2.22/7.0/7 | 14.96/47.4/51 | 1.85/7.0/7 | 106 |
| style | 1.09/2.4/3 | 11.24/26.0/32 | 0.20/2.0/2 | 28 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 40 | 3 | 117 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 160 | 0 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `40` ticks, planned normal touchdown `3` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.0563 m/s`, p95 `0.0751 m/s`, max `0.0754 m/s` over 40 samples.
Touchdown Normal sole-center tangential speed: p50 `0.0001 m/s`, p95 `0.0001 m/s`, max `0.0001 m/s` over 2 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.496 | 0.496 | 0.495 | 1.000 | 1.000 | 41.520 | 41.750 | 0.230 | 41.750 | 0.001 | 0 | 58 | 0 | 0 | 1 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2458.7 | 3742.4 | 4.00 | 23.75 | 0.00 | 0.00 | 0.00 | 0.000 | 0.000 | 2.12e-09 | 7.30e-11 | 0 |
| 16–31 | 2479.1 | 2520.2 | 4.00 | 23.75 | 0.00 | 0.00 | 0.00 | 0.000 | 0.000 | 2.12e-09 | 7.30e-11 | 0 |
| 32–47 | 2471.2 | 2522.4 | 4.00 | 23.75 | 0.00 | 0.00 | 0.00 | 0.000 | 0.000 | 2.12e-09 | 7.30e-11 | 0 |
| 48–63 | 2337.6 | 3285.4 | 6.50 | 37.06 | 2.50 | 0.00 | 0.00 | 0.000 | 0.176 | 2.12e-09 | 7.30e-11 | 11 |
| 64–79 | 2020.8 | 2551.1 | 7.31 | 41.69 | 3.31 | 0.00 | 0.00 | 0.000 | 0.635 | 1.79e-09 | 7.39e-11 | 16 |
| 80–95 | 1896.2 | 4362.8 | 8.56 | 37.31 | 5.31 | 0.00 | 0.00 | 0.188 | 0.277 | 7.17e-10 | 3.53e-11 | 13 |
| 96–111 | 4506.0 | 6083.0 | 8.06 | 42.56 | 7.00 | 0.00 | 0.00 | 2.264 | 0.001 | 6.03e-11 | 2.13e-12 | 0 |
| 112–127 | 4542.6 | 6197.0 | 7.75 | 42.56 | 6.25 | 0.00 | 0.00 | 5.790 | 0.001 | 5.05e-11 | 1.97e-12 | 0 |
| 128–143 | 4123.3 | 6218.4 | 10.00 | 51.44 | 8.31 | 0.00 | 0.00 | 9.117 | 0.005 | 2.08e-09 | 7.93e-11 | 0 |
| 144–159 | 2753.6 | 4652.8 | 8.62 | 46.62 | 7.62 | 0.00 | 0.00 | 12.208 | 1.751 | 2.07e-09 | 8.15e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 5.204 | 0.598 | 26.870 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
