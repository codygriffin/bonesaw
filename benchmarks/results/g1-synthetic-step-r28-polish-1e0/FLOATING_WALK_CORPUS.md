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

Functional: **PASS**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
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
| 160 | 0.800 s | 4.163 cm | 0.083 cm | 0.639 cm | 25.756 cm | 1.764° | 8.000 rad/s | 7118.9 µs |

Nominal hard residual maxima: dynamics `1.212e-09`, contact acceleration `4.625e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 4.163 cm |
| stance foot RMS | 0.083 cm |
| swing foot RMS | 0.639 cm |
| hand RMS | 25.756 cm |
| maximum root rotation | 1.764° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.212e-09 |
| contact acceleration residual | 4.625e-11 |
| raw max dynamics residual, including rejected ticks | 1.212e-09 |
| raw max contact residual, including rejected ticks | 4.625e-11 |
| active normal force range | 0.000–250.557 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 0.044 m/s² |
| frame-angular acceleration RMS max | 0.016 rad/s² |
| longest pre-contact / touchdown transition | 40 / 3 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 3153.6 µs | 5597.5 µs | 7118.9 µs | 7434.9 µs | 53 | 64 | 40 | 3 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3343.4 | 1211.8 | 673.1 | 5069.0 | 7386.3 | 7430.0 | 2749.9 | 160 | 17 | 0 | 299.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 53 | 3336.2 | 4708.5 | 5676.4 | 6072.7 |
| solved_with_slack | 64 | 3663.2 | 6914.0 | 7242.4 | 7434.9 |
| touchdown_transition | 3 | 2654.3 | 2679.1 | 2681.3 | 2681.9 |
| precontact_transition | 40 | 2132.7 | 2540.8 | 2609.8 | 2652.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.20 | 13.0 | 14.0 | 14 | 4.29 | 13.0 | 13 | 0.3169 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.00/1.0/1.0/1 | 230.89/234.0/234.0/234 | 0.14/1.0/1 | 0.14/1.0/1 | 1.19/9.0/10 | 0.5693 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 53 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 64 | 9.41/14.0/14 | 7.86/13.0/13 |
| touchdown_transition | 3 | 9.33/12.9/13 | 6.67/10.9/11 |
| precontact_transition | 40 | 7.75/10.6/11 | 4.10/7.0/7 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.34/5.8/11 | 5.02/23.3/44 | 0.48/5.8/11 | 32 |
| viability | 0.27/1.0/1 | 1.31/5.0/5 | 0.00/0.0/0 | 0 |
| intent | 1.95/7.4/8 | 3.91/14.8/16 | 1.42/7.4/8 | 76 |
| preference | 2.48/8.8/10 | 16.54/55.1/70 | 2.09/8.8/10 | 107 |
| style | 1.17/3.0/3 | 11.96/33.0/34 | 0.30/2.0/3 | 35 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 40 | 3 | 117 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 160 | 0 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `40` ticks, planned normal touchdown `3` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.0563 m/s`, p95 `0.0752 m/s`, max `0.0755 m/s` over 40 samples.
Touchdown Normal sole-center tangential speed: p50 `0.0000 m/s`, p95 `0.0000 m/s`, max `0.0000 m/s` over 2 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.535 | 0.526 | 0.526 | 0.982 | 0.982 | 41.547 | 41.777 | 0.230 | 41.777 | 0.001 | 0 | 58 | 0 | 0 | 102 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 3327.2 | 5133.5 | 4.00 | 23.62 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 16–31 | 3417.3 | 5823.7 | 4.00 | 23.81 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 32–47 | 2975.0 | 4978.4 | 4.00 | 23.75 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 48–63 | 2467.6 | 2670.8 | 6.81 | 39.00 | 2.94 | 1.00 | 225.75 | 0.000 | 0.176 | 1.21e-09 | 3.03e-11 | 11 |
| 64–79 | 2178.1 | 2625.5 | 7.62 | 43.88 | 3.62 | 1.00 | 222.00 | 0.001 | 0.635 | 1.01e-09 | 9.22e-12 | 16 |
| 80–95 | 1998.1 | 2677.7 | 7.94 | 36.62 | 4.94 | 1.00 | 223.12 | 0.223 | 0.277 | 6.70e-10 | 1.91e-11 | 13 |
| 96–111 | 3050.7 | 4541.9 | 8.75 | 43.31 | 7.00 | 1.00 | 234.00 | 2.035 | 0.001 | 8.94e-10 | 4.62e-11 | 0 |
| 112–127 | 4974.3 | 7389.0 | 10.12 | 57.62 | 8.56 | 1.00 | 234.00 | 4.823 | 0.001 | 4.57e-10 | 8.99e-12 | 0 |
| 128–143 | 4670.4 | 6784.4 | 9.56 | 49.06 | 8.00 | 1.00 | 234.00 | 7.287 | 0.000 | 5.30e-10 | 1.67e-11 | 0 |
| 144–159 | 3103.1 | 4737.8 | 9.19 | 46.81 | 7.88 | 1.00 | 234.00 | 9.631 | 0.245 | 4.97e-10 | 3.49e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 4.163 | 0.239 | 25.756 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
