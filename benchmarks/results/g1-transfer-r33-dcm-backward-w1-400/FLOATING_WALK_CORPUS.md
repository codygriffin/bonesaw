# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `dcm-backward-preview` reference at `viability` priority with weight `1.000` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 384 | 1.920 s | 14.504 cm | 12.994 cm | 40.461 cm | 32.721 cm | 91.723° | 8.000 rad/s | 38357.9 µs |

Nominal hard residual maxima: dynamics `1.212e-09`, contact acceleration `5.554e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 18.440 cm |
| CoM RMS / p95 | 17.029 / 46.902 cm |
| stance foot RMS | 19.870 cm |
| swing foot RMS | 50.142 cm |
| hand RMS | 35.786 cm |
| maximum root rotation | 124.125° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.829e-09 |
| contact acceleration residual | 3.386e-10 |
| raw max dynamics residual, including rejected ticks | 3.829e-09 |
| raw max contact residual, including rejected ticks | 3.386e-10 |
| active normal force range | 0.000–345.610 N |
| centroidal momentum-rate residual RMS / max | 29.637 / 301.969 N·m |
| point-task acceleration RMS max | 149.688 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `21.700` / `59.392 cm`.
- Virtual ZMP clipped on `45.25%` of ticks; clip-distance RMS / max `34.969` / `96.718 cm`.
- Measured-height natural frequency min / p50 / max: `3.703` / `3.769` / `7.004 rad/s`.
- CoM command acceleration p95 / max: `74.033` / `83.491 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 400 | 2.0 s | 2872.6 µs | 34154.6 µs | 146714.6 µs | 211090.0 µs | 94 | 290 | 0 | 0 | 1 | 15 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8477.6 | 25122.4 | 470.7 | 5311.0 | 204515.9 | 210432.6 | 63993.9 | 400 | 44 | 22 | 118.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 94 | 2411.2 | 2501.6 | 2535.1 | 2542.1 |
| solved_with_slack | 290 | 3023.4 | 6619.2 | 58691.8 | 64671.3 |
| normal_contact_contingency | 1 | 16199.8 | 16199.8 | 16199.8 | 16199.8 |
| contact_release_contingency | 15 | 102539.3 | 199556.5 | 208783.3 | 211090.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.34 | 11.0 | 13.0 | 14 | 4.90 | 12.0 | 14 | 0.1136 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 57.20/8.0/2056.2/3908 | 12710.94/1872.0/456487.5/867576 | 1.18/7.0/17 | 0.71/6.0/16 | 5.92/51.0/138 | 0.2281 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 94 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 290 | 8.04/13.0/13 | 6.37/12.0/13 |
| normal_contact_contingency | 1 | 14.00/14.0/14 | 14.00/14.0/14 |
| contact_release_contingency | 15 | 7.93/10.0/10 | 6.53/8.9/9 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.89/5.0/6 | 8.02/24.0/30 | 1.34/5.0/6 | 223 |
| viability | 1.61/6.0/9 | 6.66/36.0/49 | 1.16/6.0/8 | 238 |
| intent | 1.62/5.0/7 | 8.60/26.0/35 | 1.38/5.0/7 | 303 |
| preference | 1.18/4.0/6 | 12.35/47.0/60 | 0.69/4.0/6 | 215 |
| style | 1.03/2.0/2 | 9.44/19.0/27 | 0.32/2.0/2 | 124 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 201 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 384 | 16 |
| left_wrist_roll_rubber_hand | 400 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 400 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `19` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.391 | 3.391 | 3.391 | 1.000 | 1.000 | 47.031 | 47.293 | 0.262 | 47.934 | 0.001 | 0 | 46 | 0 | 0 | 38 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–39 | 2394.0 | 2498.6 | 5.00 | 33.20 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| 40–79 | 2421.4 | 2534.0 | 5.00 | 34.25 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 8.38e-10 | 4.71e-11 | 0 |
| 80–119 | 2491.9 | 4467.1 | 6.33 | 43.05 | 2.60 | 1.00 | 234.00 | 0.114 | 0.000 | 9.17e-10 | 5.50e-11 | 0 |
| 120–159 | 2468.5 | 4424.5 | 6.75 | 41.52 | 4.05 | 3.27 | 766.35 | 0.520 | 0.000 | 9.84e-10 | 5.46e-11 | 0 |
| 160–199 | 4385.3 | 5636.4 | 8.10 | 46.85 | 6.42 | 6.42 | 1501.05 | 3.184 | 0.001 | 5.60e-10 | 2.22e-11 | 0 |
| 200–239 | 3029.0 | 3786.8 | 8.75 | 49.12 | 6.65 | 7.83 | 1737.15 | 8.593 | 1.057 | 1.71e-10 | 2.53e-12 | 0 |
| 240–279 | 1778.8 | 5672.5 | 7.88 | 46.15 | 6.75 | 3.10 | 688.20 | 14.621 | 15.441 | 7.59e-10 | 9.33e-12 | 0 |
| 280–319 | 2946.8 | 32614.9 | 7.97 | 48.75 | 6.92 | 111.20 | 24686.40 | 14.549 | 21.100 | 9.22e-10 | 6.41e-12 | 0 |
| 320–359 | 3123.5 | 4659.2 | 8.32 | 51.35 | 7.35 | 8.00 | 1776.00 | 23.096 | 33.309 | 1.46e-10 | 5.26e-12 | 0 |
| 360–399 | 25175.5 | 204664.2 | 9.30 | 56.33 | 8.22 | 429.18 | 95252.25 | 48.551 | 86.480 | 3.83e-09 | 3.39e-10 | 16 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 400 | 18.440 | 30.452 | 35.786 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
