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
| `no_contact_contingency_ticks` | FAIL |
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 112 | 0.560 s | 0.800 cm | 1.373 cm | 0.639 cm | 16.207 cm | 4.244° | 8.000 rad/s | 96247.2 µs |

Nominal hard residual maxima: dynamics `1.212e-09`, contact acceleration `3.026e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 7.062 cm |
| stance foot RMS | 5.605 cm |
| swing foot RMS | 0.639 cm |
| hand RMS | 28.150 cm |
| maximum root rotation | 8.689° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.212e-09 |
| contact acceleration residual | 3.026e-11 |
| raw max dynamics residual, including rejected ticks | 1.212e-09 |
| raw max contact residual, including rejected ticks | 3.026e-11 |
| active normal force range | 0.000–371.831 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 61.172 m/s² |
| frame-angular acceleration RMS max | 25.901 rad/s² |
| longest pre-contact / touchdown transition | 40 / 19 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 3386.8 µs | 92084.7 µs | 178782.5 µs | 327156.1 µs | 53 | 0 | 40 | 19 | 46 | 2 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 27743.5 | 41527.0 | 1373.2 | 69524.6 | 303857.3 | 324826.3 | 150558.9 | 160 | 72 | 65 | 36.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 53 | 2441.0 | 2889.8 | 3606.2 | 3770.6 |
| normal_contact_contingency | 46 | 39428.2 | 68780.4 | 249689.3 | 327156.1 |
| contact_release_contingency | 2 | 179063.2 | 180466.7 | 180591.5 | 180622.7 |
| touchdown_transition | 19 | 78656.8 | 96512.9 | 97733.0 | 98038.1 |
| precontact_transition | 40 | 2826.1 | 63124.4 | 65872.6 | 66400.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.54 | 12.0 | 14.4 | 18 | 5.36 | 14.4 | 17 | 0.5044 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 819.50/2969.4/4222.4/6110 | 183796.05/677034.6/949676.9/1356420 | 0.44/1.0/1 | 0.44/1.0/1 | 4.28/13.4/14 | 0.8107 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 53 | 4.00/4.0/4 | 0.00/0.0/0 |
| normal_contact_contingency | 46 | 9.48/15.7/18 | 9.00/15.2/17 |
| contact_release_contingency | 2 | 7.00/7.0/7 | 5.50/6.0/6 |
| touchdown_transition | 19 | 9.42/14.8/15 | 9.42/14.8/15 |
| precontact_transition | 40 | 9.12/13.0/13 | 6.33/12.6/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.97/7.0/7 | 7.54/28.0/28 | 1.41/7.0/7 | 73 |
| viability | 1.14/5.4/11 | 5.19/27.0/55 | 0.94/5.4/11 | 79 |
| intent | 1.54/5.4/7 | 3.08/10.8/14 | 1.01/5.0/7 | 99 |
| preference | 1.48/5.0/5 | 10.56/33.2/40 | 1.14/5.0/5 | 106 |
| style | 1.41/4.0/4 | 13.72/34.4/46 | 0.86/4.0/4 | 87 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 40 | 19 | 53 | 48 |
| right_ankle_roll_link | 0 | 0 | 0 | 112 | 48 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `40` ticks, planned normal touchdown `19` ticks, normal fallback `51` ticks.
Precontact sole-center tangential speed: p50 `0.0581 m/s`, p95 `0.0751 m/s`, max `0.0754 m/s` over 40 samples.
Touchdown Normal sole-center tangential speed: p50 `0.9803 m/s`, p95 `2.0046 m/s`, max `2.0946 m/s` over 18 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.439 | 4.438 | 4.438 | 1.000 | 1.000 | 41.648 | 42.016 | 0.367 | 42.016 | 0.001 | 0 | 72 | 0 | 0 | 57 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2440.3 | 2566.7 | 4.00 | 23.88 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 16–31 | 2440.8 | 2460.0 | 4.00 | 23.94 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.14e-13 | 4.73e-16 | 0 |
| 32–47 | 2442.2 | 3723.2 | 4.00 | 23.88 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 5.68e-14 | 4.53e-16 | 0 |
| 48–63 | 2445.6 | 3589.8 | 7.38 | 40.94 | 3.81 | 1.00 | 225.75 | 0.000 | 0.176 | 6.97e-11 | 1.14e-12 | 11 |
| 64–79 | 2500.2 | 3476.9 | 8.62 | 45.31 | 5.00 | 1.00 | 222.00 | 0.000 | 0.635 | 7.28e-12 | 6.60e-14 | 16 |
| 80–95 | 45273.9 | 80049.9 | 9.81 | 43.75 | 8.75 | 1494.00 | 333989.62 | 0.198 | 0.279 | 6.86e-12 | 7.36e-14 | 13 |
| 96–111 | 79994.9 | 97783.9 | 9.44 | 43.06 | 9.44 | 2726.94 | 621741.75 | 2.106 | 3.293 | 1.42e-13 | 8.55e-15 | 0 |
| 112–127 | 30555.9 | 305176.1 | 9.00 | 48.44 | 8.12 | 1171.25 | 259965.00 | 6.248 | 9.301 | 5.07e-11 | 7.11e-15 | 16 |
| 128–143 | 29548.3 | 139005.2 | 9.88 | 55.88 | 9.31 | 1156.00 | 256632.00 | 12.662 | 6.626 | 1.03e-10 | 1.60e-12 | 16 |
| 144–159 | 47884.5 | 70880.2 | 9.25 | 51.75 | 9.12 | 1641.81 | 364482.38 | 17.174 | 11.558 | 1.14e-13 | 5.33e-15 | 16 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 7.062 | 5.247 | 28.150 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
