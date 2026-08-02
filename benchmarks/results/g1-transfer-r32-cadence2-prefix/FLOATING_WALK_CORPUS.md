# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `2.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `120` ticks (`0.600 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
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
| 366 | 1.830 s | 12.075 cm | 26.333 cm | 35.008 cm | 28.455 cm | 49.403° | 8.000 rad/s | 99792.5 µs |

Nominal hard residual maxima: dynamics `1.822e-09`, contact acceleration `5.634e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 18.809 cm |
| CoM RMS / p95 | 22.520 / 43.666 cm |
| stance foot RMS | 29.262 cm |
| swing foot RMS | 48.575 cm |
| hand RMS | 31.949 cm |
| maximum root rotation | 90.580° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.822e-09 |
| contact acceleration residual | 5.634e-11 |
| raw max dynamics residual, including rejected ticks | 1.822e-09 |
| raw max contact residual, including rejected ticks | 5.634e-11 |
| active normal force range | 0.000–378.326 N |
| centroidal momentum-rate residual RMS / max | 28.347 / 94.008 N·m |
| point-task acceleration RMS max | 98.088 m/s² |
| frame-angular acceleration RMS max | 120.501 rad/s² |
| longest pre-contact / touchdown transition | 185 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 91 / 91 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 400 | 2.0 s | 2649.4 µs | 9577.8 µs | 100488.0 µs | 117915.3 µs | 113 | 68 | 185 | 0 | 34 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5996.5 | 15079.0 | 529.5 | 6931.4 | 115471.2 | 117670.9 | 23833.0 | 400 | 55 | 15 | 166.8 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 113 | 2444.7 | 2573.1 | 2657.5 | 3009.5 |
| solved_with_slack | 68 | 2821.7 | 100094.0 | 107978.2 | 111789.7 |
| normal_contact_contingency | 34 | 8379.8 | 26809.3 | 88565.3 | 117915.3 |
| precontact_transition | 185 | 3149.7 | 7202.9 | 73197.8 | 85830.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.68 | 12.0 | 13.0 | 16 | 5.24 | 13.0 | 13 | 0.2230 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 165.97/17.8/6187.2/6971 | 36730.92/3981.6/1373567.3/1547562 | 1.36/10.0/10 | 1.00/9.0/9 | 8.32/76.0/77 | 0.9198 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 113 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 68 | 9.04/14.7/16 | 6.66/13.0/13 |
| normal_contact_contingency | 34 | 9.44/12.0/12 | 8.12/11.0/11 |
| precontact_transition | 185 | 8.49/13.0/14 | 7.39/13.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.88/5.0/6 | 7.72/24.0/29 | 1.32/5.0/6 | 204 |
| viability | 2.01/8.0/12 | 11.00/37.0/49 | 1.71/8.0/12 | 280 |
| intent | 1.36/4.0/5 | 7.16/24.0/30 | 1.01/4.0/5 | 262 |
| preference | 1.27/5.0/7 | 12.15/48.0/84 | 0.83/5.0/7 | 230 |
| style | 1.16/4.0/5 | 11.20/29.1/41 | 0.37/3.0/5 | 114 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 44 | 185 | 0 | 171 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 366 | 34 |
| left_wrist_roll_rubber_hand | 400 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 400 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `185` ticks, planned normal touchdown `0` ticks, normal fallback `37` ticks.
Precontact sole-center tangential speed: p50 `1.7844 m/s`, p95 `3.8887 m/s`, max `4.2661 m/s` over 185 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.399 | 2.398 | 2.398 | 1.000 | 1.000 | 47.637 | 47.859 | 0.223 | 47.859 | 0.001 | 0 | 42 | 0 | 0 | 34 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–39 | 2476.4 | 2979.6 | 6.60 | 39.05 | 2.50 | 1.00 | 234.00 | 0.194 | 0.000 | 1.62e-09 | 5.31e-11 | 0 |
| 40–79 | 2473.4 | 3863.4 | 5.15 | 34.88 | 0.25 | 1.00 | 234.00 | 0.091 | 0.000 | 9.73e-10 | 5.63e-11 | 0 |
| 80–119 | 2429.3 | 2562.8 | 5.00 | 33.77 | 0.00 | 1.00 | 234.00 | 0.016 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–159 | 2608.0 | 4866.0 | 7.47 | 46.58 | 4.00 | 2.40 | 561.60 | 0.056 | 0.000 | 7.96e-10 | 5.38e-11 | 0 |
| 160–199 | 2816.5 | 4784.0 | 8.47 | 56.65 | 6.80 | 2.92 | 652.65 | 2.125 | 2.596 | 1.33e-09 | 3.72e-11 | 29 |
| 200–239 | 2635.7 | 16519.9 | 7.83 | 50.52 | 6.65 | 38.48 | 8541.45 | 9.319 | 25.667 | 1.38e-09 | 4.22e-11 | 40 |
| 240–279 | 3079.5 | 3845.8 | 8.32 | 51.83 | 7.30 | 4.67 | 1037.85 | 14.580 | 42.132 | 1.82e-09 | 1.48e-11 | 40 |
| 280–319 | 3745.1 | 5309.8 | 8.97 | 55.33 | 7.80 | 6.42 | 1426.35 | 20.218 | 44.410 | 5.91e-10 | 8.28e-12 | 40 |
| 320–359 | 5845.5 | 81724.2 | 9.43 | 58.73 | 8.85 | 582.27 | 129265.05 | 21.935 | 51.314 | 4.82e-10 | 1.30e-11 | 36 |
| 360–399 | 8673.3 | 115526.3 | 9.53 | 65.03 | 8.25 | 1019.50 | 225122.25 | 48.415 | 75.669 | 3.63e-10 | 4.44e-12 | 34 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.50x | 400 | 18.809 | 35.738 | 31.949 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
