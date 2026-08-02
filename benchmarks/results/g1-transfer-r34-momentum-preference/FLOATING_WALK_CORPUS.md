# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `dcm-backward-preview` `DCM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `preference` priority with weight `1.000` and `1.000 Hz` response.
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
| 391 | 1.955 s | 22.893 cm | 22.983 cm | 62.867 cm | 35.313 cm | 101.912° | 8.000 rad/s | 57940.9 µs |

Nominal hard residual maxima: dynamics `1.530e-09`, contact acceleration `6.074e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 131.910 cm |
| authored reference vs measured CoM RMS / p95 | 130.227 / 251.002 cm |
| stance foot RMS | 114.538 cm |
| swing foot RMS | 128.903 cm |
| hand RMS | 141.851 cm |
| maximum root rotation | 179.540° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.320e-09 |
| contact acceleration residual | 4.079e-10 |
| raw max dynamics residual, including rejected ticks | 9.320e-09 |
| raw max contact residual, including rejected ticks | 4.079e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 75.650 / 616.756 N·m |
| point-task acceleration RMS max | 187.053 m/s² |
| frame-angular acceleration RMS max | 346.444 rad/s² |
| longest pre-contact / touchdown transition | 163 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `117.427` / `242.407 cm`.
- Virtual ZMP clipped on `65.83%` of ticks; clip-distance RMS / max `183.435` / `488.883 cm`.
- Measured-height natural frequency min / p50 / max: `3.693` / `3.834` / `7.004 rad/s`.
- Minimum measured CoM height: `-1.261 m`; height-floor ticks: `212`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3285.3 µs | 101312.5 µs | 187048.9 µs | 209553.8 µs | 154 | 74 | 163 | 0 | 164 | 45 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 15576.1 | 36777.4 | 899.3 | 31109.4 | 208554.2 | 209453.8 | 152022.8 | 600 | 192 | 64 | 64.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 154 | 2443.4 | 2520.2 | 2543.4 | 2579.9 |
| solved_with_slack | 74 | 4263.3 | 5923.1 | 6884.3 | 7026.2 |
| normal_contact_contingency | 164 | 5124.3 | 19758.0 | 78579.6 | 121176.4 |
| contact_release_contingency | 45 | 109267.9 | 202186.6 | 208819.5 | 209553.8 |
| precontact_transition | 163 | 3198.0 | 34334.5 | 76999.7 | 108471.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.04 | 12.0 | 13.0 | 15 | 5.89 | 13.0 | 14 | 0.0404 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 122.78/266.4/3467.5/6926 | 26917.18/57553.2/769789.4/1537572 | 2.37/14.0/16 | 1.82/13.0/15 | 15.20/116.0/138 | 0.2189 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 154 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 74 | 8.92/13.3/14 | 6.82/13.0/13 |
| normal_contact_contingency | 164 | 9.23/13.0/15 | 8.35/13.0/13 |
| contact_release_contingency | 45 | 7.96/11.6/12 | 6.62/10.6/11 |
| precontact_transition | 163 | 9.34/14.0/14 | 8.37/13.4/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.24/7.0/9 | 9.90/30.0/45 | 1.85/7.0/9 | 416 |
| viability | 1.62/6.0/9 | 8.87/36.0/54 | 1.30/6.0/8 | 418 |
| intent | 1.50/5.0/6 | 3.06/12.0/17 | 1.18/5.0/6 | 414 |
| preference | 1.59/6.0/7 | 14.85/57.0/70 | 1.23/6.0/7 | 409 |
| style | 1.09/3.0/5 | 9.00/24.0/35 | 0.34/3.0/4 | 171 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 391 | 209 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `212` ticks.
Precontact sole-center tangential speed: p50 `3.5463 m/s`, p95 `8.9669 m/s`, max `13.0352 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9.346 | 9.343 | 9.343 | 1.000 | 1.000 | 47.180 | 47.477 | 0.297 | 48.523 | 0.001 | 0 | 60 | 0 | 0 | 166 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2427.4 | 2536.6 | 5.00 | 28.55 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.44e-09 | 5.15e-11 | 0 |
| 60–119 | 2456.4 | 2557.2 | 5.00 | 28.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.46e-09 | 6.07e-11 | 0 |
| 120–179 | 2480.2 | 6911.5 | 6.82 | 38.45 | 3.03 | 3.22 | 752.70 | 0.554 | 0.000 | 1.05e-09 | 5.42e-11 | 0 |
| 180–239 | 3868.8 | 5502.1 | 8.90 | 49.30 | 7.00 | 8.00 | 1806.40 | 6.058 | 0.854 | 6.70e-11 | 3.05e-12 | 12 |
| 240–299 | 2338.8 | 61577.9 | 9.12 | 51.12 | 7.77 | 367.82 | 81655.30 | 12.987 | 21.146 | 1.53e-09 | 2.13e-11 | 60 |
| 300–359 | 3044.0 | 18427.4 | 9.13 | 51.73 | 8.52 | 41.88 | 9298.10 | 30.726 | 51.493 | 1.18e-09 | 1.72e-11 | 60 |
| 360–419 | 9190.8 | 192131.1 | 10.22 | 61.13 | 9.42 | 318.15 | 70075.90 | 92.574 | 132.517 | 8.64e-10 | 4.25e-11 | 60 |
| 420–479 | 3917.8 | 170746.5 | 8.65 | 48.48 | 7.85 | 7.88 | 1697.20 | 215.619 | 168.178 | 8.76e-09 | 2.82e-10 | 60 |
| 480–539 | 4794.7 | 208569.2 | 8.63 | 48.88 | 7.53 | 7.65 | 1641.10 | 246.635 | 202.295 | 9.32e-09 | 4.08e-10 | 60 |
| 540–599 | 9014.9 | 174137.4 | 8.93 | 50.57 | 7.83 | 471.25 | 101777.10 | 238.666 | 229.981 | 5.70e-09 | 1.73e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 131.910 | 119.482 | 141.851 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
