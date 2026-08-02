# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `0.300 Hz` response (`position-only`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 335 | 1.675 s | 12.050 cm | 1.180 cm | 10.926 cm | 24.944 cm | 14.759° | 8.000 rad/s | 22056.8 µs |

Nominal hard residual maxima: dynamics `2.136e-09`, contact acceleration `1.103e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 145.463 cm |
| CoM RMS / p95 | 137.336 / 289.834 cm |
| stance foot RMS | 102.962 cm |
| swing foot RMS | 155.448 cm |
| hand RMS | 169.877 cm |
| maximum root rotation | 142.811° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.438e-09 |
| contact acceleration residual | 1.758e-10 |
| raw max dynamics residual, including rejected ticks | 7.438e-09 |
| raw max contact residual, including rejected ticks | 1.758e-10 |
| active normal force range | 0.000–900.214 N |
| centroidal momentum-rate residual RMS / max | 63.013 / 466.202 N·m |
| point-task acceleration RMS max | 128.802 m/s² |
| frame-angular acceleration RMS max | 251.973 rad/s² |
| longest pre-contact / touchdown transition | 107 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2703.3 µs | 147734.3 µs | 197402.0 µs | 219446.8 µs | 175 | 53 | 107 | 0 | 230 | 35 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 15493.1 | 41652.1 | 566.4 | 28992.8 | 216780.6 | 219180.2 | 166566.5 | 600 | 115 | 69 | 64.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 175 | 2458.7 | 2561.0 | 2799.6 | 3927.8 |
| solved_with_slack | 53 | 2969.2 | 4294.2 | 4452.3 | 4529.8 |
| normal_contact_contingency | 230 | 2810.0 | 43090.2 | 75934.0 | 197379.8 |
| contact_release_contingency | 35 | 179653.2 | 214627.5 | 217933.4 | 219446.8 |
| precontact_transition | 107 | 3065.5 | 19500.4 | 61634.5 | 70602.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.36 | 11.0 | 13.0 | 16 | 4.91 | 12.0 | 15 | 0.0596 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 142.59/1257.4/2859.0/5815 | 30912.96/271609.2/617776.4/1256040 | 1.63/16.0/20 | 1.25/15.0/19 | 10.36/138.0/170 | 0.1758 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 175 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 53 | 7.58/12.0/13 | 4.51/9.5/10 |
| normal_contact_contingency | 230 | 8.59/13.7/16 | 7.49/13.0/15 |
| contact_release_contingency | 35 | 7.29/9.0/9 | 5.83/8.0/8 |
| precontact_transition | 107 | 8.50/13.0/14 | 7.32/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.13/6.0/9 | 9.12/29.0/37 | 1.71/6.0/9 | 376 |
| viability | 1.59/6.0/7 | 10.51/40.0/56 | 1.25/6.0/7 | 401 |
| intent | 1.35/5.0/7 | 6.85/25.0/42 | 1.02/5.0/7 | 398 |
| preference | 1.22/6.0/8 | 12.07/55.1/87 | 0.64/5.0/8 | 264 |
| style | 1.07/3.0/6 | 9.01/26.0/49 | 0.30/2.0/6 | 150 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 335 | 265 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `268` ticks.
Precontact sole-center tangential speed: p50 `1.3870 m/s`, p95 `6.8936 m/s`, max `7.7838 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9.296 | 9.294 | 9.294 | 1.000 | 1.000 | 47.711 | 47.945 | 0.234 | 47.945 | 0.001 | 0 | 44 | 0 | 0 | 115 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2510.2 | 4441.8 | 5.68 | 41.38 | 1.08 | 1.00 | 234.00 | 0.004 | 0.000 | 1.33e-09 | 5.83e-11 | 0 |
| 60–119 | 2469.1 | 2589.6 | 5.00 | 34.42 | 0.00 | 1.00 | 234.00 | 0.001 | 0.000 | 1.04e-09 | 5.09e-11 | 0 |
| 120–179 | 2436.0 | 3734.8 | 5.05 | 34.48 | 0.08 | 1.00 | 234.00 | 0.001 | 0.000 | 1.39e-09 | 4.40e-11 | 0 |
| 180–239 | 2883.7 | 3980.2 | 7.13 | 45.85 | 4.10 | 4.97 | 1106.40 | 1.964 | 2.187 | 7.78e-10 | 4.50e-11 | 12 |
| 240–299 | 3035.2 | 4291.9 | 8.20 | 50.60 | 6.95 | 6.72 | 1491.10 | 12.680 | 6.366 | 1.05e-09 | 1.01e-11 | 60 |
| 300–359 | 4088.8 | 122581.3 | 8.78 | 57.92 | 7.90 | 445.48 | 97260.70 | 44.620 | 18.052 | 2.29e-09 | 1.10e-10 | 60 |
| 360–419 | 1730.1 | 15334.3 | 8.35 | 50.78 | 6.92 | 31.93 | 6897.60 | 121.222 | 54.825 | 7.83e-10 | 3.97e-12 | 60 |
| 420–479 | 6255.9 | 203464.7 | 8.23 | 48.87 | 7.17 | 314.43 | 67910.20 | 208.622 | 159.505 | 7.44e-09 | 1.76e-10 | 60 |
| 480–539 | 10509.1 | 183730.3 | 9.10 | 60.35 | 8.13 | 616.45 | 133147.60 | 233.382 | 218.741 | 2.95e-09 | 1.02e-10 | 60 |
| 540–599 | 1835.0 | 216820.6 | 8.10 | 50.93 | 6.80 | 2.87 | 614.00 | 311.052 | 272.426 | 2.61e-09 | 5.91e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 145.463 | 122.834 | 169.877 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
