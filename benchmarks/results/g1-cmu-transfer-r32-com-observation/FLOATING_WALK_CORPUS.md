# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
5 ms p99 deadline: **PASS**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | PASS |

Failed checks: `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.000 s | 20.026 cm | 28.940 cm | 27.249 cm | 43.656 cm | 13.921° | 8.000 rad/s | 4794.7 µs |

Nominal hard residual maxima: dynamics `2.027e-09`, contact acceleration `7.067e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 20.026 cm |
| CoM RMS / p95 | 26.808 / 50.361 cm |
| stance foot RMS | 28.940 cm |
| swing foot RMS | 27.249 cm |
| hand RMS | 43.656 cm |
| maximum root rotation | 13.921° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.027e-09 |
| contact acceleration residual | 7.067e-11 |
| raw max dynamics residual, including rejected ticks | 2.027e-09 |
| raw max contact residual, including rejected ticks | 7.067e-11 |
| active normal force range | 0.000–357.782 N |
| centroidal momentum-rate residual RMS / max | 25.319 / 109.138 N·m |
| point-task acceleration RMS max | 95.319 m/s² |
| frame-angular acceleration RMS max | 128.887 rad/s² |
| longest pre-contact / touchdown transition | 372 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2450.9 µs | 3709.3 µs | 4794.7 µs | 6112.9 µs | 141 | 87 | 372 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2488.6 | 680.2 | 554.9 | 3274.8 | 5667.7 | 6068.4 | 2029.5 | 600 | 4 | 0 | 401.8 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2479.1 | 2753.2 | 2975.1 | 3031.0 |
| solved_with_slack | 87 | 2766.0 | 4976.3 | 5473.7 | 6112.9 |
| precontact_transition | 372 | 1956.4 | 3490.9 | 3938.8 | 4501.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.64 | 11.0 | 13.0 | 16 | 5.26 | 12.0 | 13 | 0.1024 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 2.61/8.0/8.0/8 | 585.26/1776.0/1872.0/1872 | 0.48/3.0/3 | 0.25/2.0/2 | 2.04/16.0/18 | 0.7281 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 87 | 8.18/12.6/16 | 5.79/11.3/13 |
| precontact_transition | 372 | 8.51/13.0/13 | 7.13/12.3/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.67/5.0/7 | 6.49/20.0/28 | 1.02/5.0/7 | 260 |
| viability | 2.36/7.0/12 | 14.39/42.0/63 | 2.12/7.0/12 | 452 |
| intent | 1.37/4.0/5 | 7.40/24.0/29 | 1.10/4.0/5 | 434 |
| preference | 1.19/5.0/7 | 11.64/45.1/75 | 0.75/4.0/7 | 346 |
| style | 1.04/2.0/3 | 9.57/22.0/41 | 0.28/2.0/3 | 161 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 600 | 0 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `1.6789 m/s`, p95 `2.9692 m/s`, max `3.2863 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.493 | 1.493 | 1.493 | 1.000 | 1.000 | 46.730 | 46.965 | 0.234 | 47.898 | 0.001 | 0 | 44 | 0 | 0 | 20 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2509.1 | 2990.4 | 6.07 | 36.82 | 1.67 | 1.00 | 234.00 | 0.172 | 0.000 | 1.62e-09 | 5.63e-11 | 0 |
| 60–119 | 2473.2 | 3845.1 | 5.10 | 34.98 | 0.17 | 1.00 | 234.00 | 0.032 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–179 | 2544.5 | 5309.8 | 6.12 | 39.43 | 1.62 | 1.12 | 261.30 | 0.002 | 0.000 | 1.46e-09 | 6.30e-11 | 0 |
| 180–239 | 2814.2 | 5628.2 | 7.90 | 50.87 | 6.07 | 3.92 | 883.10 | 3.281 | 3.526 | 1.47e-09 | 5.34e-11 | 12 |
| 240–299 | 3086.6 | 3743.3 | 8.45 | 55.45 | 7.15 | 5.43 | 1206.20 | 7.626 | 23.100 | 1.61e-09 | 7.07e-11 | 60 |
| 300–359 | 3077.0 | 4193.8 | 8.68 | 54.75 | 7.52 | 5.43 | 1206.20 | 11.865 | 27.214 | 7.87e-10 | 1.32e-11 | 60 |
| 360–419 | 1933.7 | 4149.5 | 8.68 | 57.62 | 7.25 | 2.63 | 584.60 | 15.120 | 21.371 | 1.28e-09 | 1.62e-11 | 60 |
| 420–479 | 1995.0 | 3876.8 | 8.43 | 54.97 | 7.32 | 3.60 | 799.20 | 24.191 | 33.994 | 1.22e-09 | 1.14e-11 | 60 |
| 480–539 | 1830.5 | 2170.1 | 8.43 | 54.72 | 6.43 | 1.00 | 222.00 | 31.410 | 54.393 | 2.03e-09 | 1.75e-11 | 60 |
| 540–599 | 1887.1 | 2887.7 | 8.50 | 55.30 | 7.45 | 1.00 | 222.00 | 44.723 | 46.940 | 7.09e-10 | 2.06e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 20.026 | 28.392 | 43.656 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
