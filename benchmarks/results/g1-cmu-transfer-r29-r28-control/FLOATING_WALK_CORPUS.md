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
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; contact forces remain off until the authored contact edge.
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
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 390 | 1.950 s | 14.761 cm | 14.561 cm | 44.428 cm | 33.651 cm | 73.875° | 8.000 rad/s | 70899.6 µs |

Nominal hard residual maxima: dynamics `1.281e-09`, contact acceleration `7.305e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 100.205 cm |
| stance foot RMS | 97.187 cm |
| swing foot RMS | 87.108 cm |
| hand RMS | 131.406 cm |
| maximum root rotation | 179.360° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.281e-09 |
| contact acceleration residual | 7.305e-11 |
| raw max dynamics residual, including rejected ticks | 3.889e+02 |
| raw max contact residual, including rejected ticks | 7.305e-11 |
| active normal force range | 0.000–357.944 N |
| centroidal momentum-rate residual RMS / max | 50.562 / 133.451 N·m |
| point-task acceleration RMS max | 103.337 m/s² |
| frame-angular acceleration RMS max | 216.929 rad/s² |
| longest pre-contact / touchdown transition | 162 / 13 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 10657.1 µs | 173330.4 µs | 177348.0 µs | 201687.9 µs | 138 | 90 | 162 | 13 | 22 | 49 | 126 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 54087.1 | 69283.9 | 8594.5 | 172449.2 | 191154.4 | 200634.6 | 103274.6 | 600 | 340 | 248 | 18.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 138 | 2618.7 | 2694.6 | 2756.0 | 2804.6 |
| solved_with_slack | 90 | 2944.8 | 28246.3 | 30080.8 | 31978.3 |
| primal_infeasible | 126 | 172410.7 | 175066.1 | 182318.7 | 184102.8 |
| normal_contact_contingency | 22 | 54066.1 | 83827.0 | 154326.0 | 173024.3 |
| contact_release_contingency | 49 | 106974.2 | 142971.7 | 193142.7 | 201687.9 |
| touchdown_transition | 13 | 56032.8 | 77236.2 | 77786.1 | 77923.5 |
| precontact_transition | 162 | 11059.6 | 59409.9 | 84514.3 | 103297.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.01 | 11.0 | 14.0 | 25 | 4.05 | 13.0 | 22 | -0.6526 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1806.37/6720.0/6720.0/6720 | 383297.42/1411200.0/1411200.0/1411200 | 0.31/1.0/1 | 0.31/1.0/1 | 2.54/9.0/11 | 0.8830 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 138 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 90 | 9.13/24.1/25 | 6.81/21.1/22 |
| primal_infeasible | 126 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 22 | 9.73/14.6/15 | 8.64/13.8/14 |
| contact_release_contingency | 49 | 7.63/10.0/10 | 6.33/9.0/9 |
| touchdown_transition | 13 | 10.69/13.9/14 | 8.92/12.0/12 |
| precontact_transition | 162 | 8.46/13.0/14 | 7.41/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.51/5.0/6 | 6.08/24.0/30 | 1.10/5.0/6 | 250 |
| viability | 1.50/6.0/13 | 8.30/35.0/42 | 1.25/6.0/13 | 323 |
| intent | 1.09/4.0/6 | 5.29/24.0/30 | 0.83/4.0/6 | 315 |
| preference | 1.07/6.0/17 | 10.68/65.0/192 | 0.68/6.0/16 | 248 |
| style | 0.83/2.0/3 | 7.25/27.0/40 | 0.19/2.0/2 | 101 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 17 | 199 | 155 |
| right_ankle_roll_link | 168 | 0 | 0 | 390 | 42 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `17` ticks, normal fallback `158` ticks.
Precontact sole-center tangential speed: p50 `1.5671 m/s`, p95 `4.9763 m/s`, max `5.2981 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `3.5640 m/s`, p95 `4.4163 m/s`, max `4.4868 m/s` over 16 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 32.452 | 32.438 | 32.438 | 1.000 | 1.000 | 47.383 | 47.625 | 0.242 | 47.695 | 0.001 | 0 | 40 | 0 | 0 | 195 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2635.1 | 4256.1 | 6.43 | 39.43 | 2.07 | 1.00 | 234.00 | 0.156 | 0.000 | 1.25e-09 | 4.91e-11 | 0 |
| 60–119 | 2618.7 | 3937.4 | 5.17 | 36.52 | 0.28 | 1.00 | 234.00 | 0.025 | 0.000 | 1.28e-09 | 7.30e-11 | 0 |
| 120–179 | 2646.8 | 5326.9 | 6.43 | 42.97 | 2.17 | 1.00 | 234.00 | 0.033 | 0.000 | 1.00e-09 | 5.04e-11 | 0 |
| 180–239 | 5356.5 | 30720.4 | 8.73 | 56.30 | 6.97 | 549.97 | 123352.60 | 4.436 | 3.115 | 1.21e-09 | 4.78e-11 | 12 |
| 240–299 | 11990.9 | 25878.1 | 8.43 | 52.77 | 7.18 | 471.40 | 104650.80 | 10.422 | 24.184 | 7.94e-10 | 1.01e-11 | 60 |
| 300–359 | 10029.2 | 22662.8 | 8.30 | 50.43 | 7.52 | 452.93 | 100551.20 | 21.386 | 38.105 | 6.68e-10 | 8.27e-12 | 60 |
| 360–419 | 65741.5 | 134940.5 | 8.65 | 50.80 | 7.57 | 1406.75 | 309628.80 | 57.086 | 94.291 | 6.38e-10 | 1.31e-11 | 60 |
| 420–479 | 102241.4 | 191184.5 | 8.00 | 46.82 | 6.75 | 1739.68 | 371688.80 | 144.907 | 159.776 | 3.89e+02 | 1.50e-12 | 47 |
| 480–539 | 172414.2 | 183206.6 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 194.482 | 161.645 | 3.89e+02 | 0.00e+00 | 60 |
| 540–599 | 172373.4 | 176018.3 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 194.282 | 160.352 | 3.89e+02 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 100.205 | 93.972 | 131.406 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
