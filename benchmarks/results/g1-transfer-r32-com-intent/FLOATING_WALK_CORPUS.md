# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `intent` priority with weight `0.100` and `2.000 Hz` response. Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 386 | 1.930 s | 14.826 cm | 1.098 cm | 11.866 cm | 32.318 cm | 56.766° | 8.000 rad/s | 11361.8 µs |

Nominal hard residual maxima: dynamics `5.003e-09`, contact acceleration `1.556e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 52.232 cm |
| CoM RMS / p95 | 45.797 / 75.259 cm |
| stance foot RMS | 26.153 cm |
| swing foot RMS | 45.065 cm |
| hand RMS | 81.734 cm |
| maximum root rotation | 148.717° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.418e-09 |
| contact acceleration residual | 3.396e-10 |
| raw max dynamics residual, including rejected ticks | 5.914e+02 |
| raw max contact residual, including rejected ticks | 3.396e-10 |
| active normal force range | 0.000–773.824 N |
| centroidal momentum-rate residual RMS / max | 72.421 / 296.199 N·m |
| point-task acceleration RMS max | 104.529 m/s² |
| frame-angular acceleration RMS max | 253.270 rad/s² |
| longest pre-contact / touchdown transition | 158 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3778.8 µs | 249757.0 µs | 252581.9 µs | 316377.9 µs | 41 | 187 | 158 | 0 | 24 | 22 | 168 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 79674.3 | 111398.3 | 1939.2 | 248842.0 | 280967.8 | 312836.9 | 81686.0 | 600 | 261 | 196 | 12.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 41 | 2426.5 | 2483.5 | 2501.7 | 2506.0 |
| solved_with_slack | 187 | 3020.0 | 5634.0 | 6756.7 | 7445.3 |
| primal_infeasible | 168 | 248381.1 | 251968.6 | 255433.7 | 316377.9 |
| normal_contact_contingency | 24 | 14521.8 | 73728.3 | 172702.7 | 199567.7 |
| contact_release_contingency | 22 | 189810.0 | 209949.5 | 211659.7 | 212102.9 |
| precontact_transition | 158 | 2219.4 | 9368.3 | 18462.0 | 49213.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.29 | 11.0 | 13.0 | 18 | 3.72 | 13.0 | 18 | -0.8049 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1899.03/6720.0/6720.0/6720 | 398957.46/1411200.0/1411200.0/1411200 | 11.35/36.0/36 | 10.79/35.0/35 | 96.64/315.0/315 | 0.9468 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 41 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 187 | 6.73/12.1/13 | 4.60/10.1/11 |
| primal_infeasible | 168 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 24 | 11.50/15.0/15 | 10.75/14.0/14 |
| contact_release_contingency | 22 | 7.55/9.8/10 | 6.00/8.8/9 |
| precontact_transition | 158 | 8.27/13.0/18 | 6.22/13.0/18 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.33/5.0/8 | 5.44/23.0/40 | 0.91/5.0/8 | 212 |
| viability | 0.53/4.0/6 | 3.04/24.0/36 | 0.33/4.0/6 | 123 |
| intent | 1.52/7.0/9 | 8.71/42.0/50 | 1.36/7.0/9 | 343 |
| preference | 1.10/7.0/9 | 10.84/57.0/99 | 0.83/6.0/9 | 274 |
| style | 0.80/3.0/5 | 7.55/28.0/40 | 0.29/2.0/5 | 139 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 386 | 214 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `217` ticks.
Precontact sole-center tangential speed: p50 `4.9559 m/s`, p95 `5.0207 m/s`, max `6.9066 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 47.805 | 47.789 | 47.789 | 1.000 | 1.000 | 47.660 | 47.832 | 0.172 | 48.012 | 0.001 | 0 | 43 | 0 | 0 | 817 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2591.7 | 5466.2 | 5.92 | 40.20 | 3.73 | 2.52 | 588.90 | 0.385 | 0.000 | 1.25e-09 | 5.19e-11 | 0 |
| 60–119 | 2660.7 | 5412.4 | 5.57 | 42.73 | 3.10 | 2.52 | 588.90 | 0.177 | 0.000 | 1.32e-09 | 7.09e-11 | 0 |
| 120–179 | 3297.8 | 7107.7 | 6.48 | 52.52 | 4.33 | 3.57 | 834.60 | 0.200 | 0.000 | 1.36e-09 | 4.82e-11 | 0 |
| 180–239 | 1848.9 | 5727.0 | 7.23 | 46.42 | 3.88 | 2.55 | 588.50 | 1.498 | 0.078 | 1.13e-09 | 2.54e-11 | 12 |
| 240–299 | 1862.2 | 3537.1 | 7.60 | 49.80 | 4.63 | 3.45 | 765.90 | 6.095 | 2.591 | 8.37e-10 | 1.87e-11 | 60 |
| 300–359 | 3180.0 | 4221.0 | 8.40 | 52.33 | 7.18 | 4.40 | 976.80 | 21.874 | 13.184 | 7.64e-10 | 1.96e-11 | 60 |
| 360–419 | 13263.3 | 210857.8 | 9.85 | 61.77 | 8.72 | 153.65 | 33531.00 | 59.816 | 23.564 | 5.00e-09 | 3.40e-10 | 60 |
| 420–479 | 248115.4 | 254343.8 | 1.80 | 10.02 | 1.63 | 5377.60 | 1129300.00 | 88.426 | 60.510 | 5.91e+02 | 2.22e-10 | 60 |
| 480–539 | 248429.3 | 279653.5 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 87.195 | 58.365 | 5.91e+02 | 0.00e+00 | 60 |
| 540–599 | 248612.6 | 254125.2 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 88.112 | 59.091 | 5.91e+02 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 52.232 | 33.609 | 81.734 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
