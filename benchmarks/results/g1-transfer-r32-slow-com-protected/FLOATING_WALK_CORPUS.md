# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `0.300 Hz` response (`position-only`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `viability` priority with weight `0.010` over 13 waist/arm coordinates.
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
| 346 | 1.730 s | 13.382 cm | 1.432 cm | 29.222 cm | 24.068 cm | 37.511° | 8.000 rad/s | 17600.6 µs |

Nominal hard residual maxima: dynamics `4.293e-09`, contact acceleration `1.010e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 89.462 cm |
| CoM RMS / p95 | 88.487 / 180.051 cm |
| stance foot RMS | 68.564 cm |
| swing foot RMS | 53.904 cm |
| hand RMS | 112.016 cm |
| maximum root rotation | 179.834° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.569e-09 |
| contact acceleration residual | 2.182e-10 |
| raw max dynamics residual, including rejected ticks | 9.569e-09 |
| raw max contact residual, including rejected ticks | 2.182e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 90.499 / 588.382 N·m |
| point-task acceleration RMS max | 202.160 m/s² |
| frame-angular acceleration RMS max | 246.047 rad/s² |
| longest pre-contact / touchdown transition | 118 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2551.7 µs | 174129.0 µs | 207899.7 µs | 213866.5 µs | 199 | 29 | 118 | 0 | 196 | 58 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20877.8 | 49698.9 | 493.3 | 72016.5 | 213501.2 | 213830.0 | 152448.4 | 600 | 149 | 79 | 47.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 2473.6 | 2562.5 | 2588.7 | 2605.9 |
| solved_with_slack | 29 | 2342.9 | 2923.2 | 2989.4 | 3002.2 |
| normal_contact_contingency | 196 | 4550.3 | 34911.6 | 66809.9 | 195965.6 |
| contact_release_contingency | 58 | 174272.5 | 209842.5 | 213518.9 | 213866.5 |
| precontact_transition | 118 | 2461.5 | 11601.7 | 63180.9 | 70209.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.95 | 10.0 | 12.0 | 14 | 3.42 | 9.0 | 11 | 0.1407 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 103.73/8.0/3706.0/5781 | 22531.80/1776.0/800673.1/1248696 | 1.84/15.0/17 | 1.51/14.0/16 | 12.32/121.0/139 | 0.1164 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 29 | 7.31/10.0/10 | 3.66/6.7/7 |
| normal_contact_contingency | 196 | 8.13/13.0/14 | 5.56/10.1/11 |
| contact_release_contingency | 58 | 7.45/10.9/12 | 4.66/8.4/9 |
| precontact_transition | 118 | 7.92/11.8/13 | 4.98/9.0/10 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.14/6.0/8 | 9.20/30.0/36 | 1.68/6.0/8 | 329 |
| viability | 1.70/6.0/8 | 12.86/43.0/69 | 1.37/6.0/8 | 401 |
| intent | 1.00/1.0/1 | 3.66/5.0/6 | 0.00/0.0/0 | 0 |
| preference | 1.00/1.0/1 | 3.87/6.0/7 | 0.02/1.0/1 | 12 |
| style | 1.10/4.0/7 | 9.14/30.0/42 | 0.35/4.0/6 | 154 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 346 | 254 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `257` ticks.
Precontact sole-center tangential speed: p50 `3.1491 m/s`, p95 `7.7940 m/s`, max `9.9793 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12.527 | 12.524 | 12.524 | 1.000 | 1.000 | 46.738 | 46.973 | 0.234 | 48.020 | 0.001 | 0 | 44 | 0 | 0 | 193 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2478.6 | 2580.0 | 5.00 | 29.40 | 0.00 | 1.00 | 234.00 | 0.079 | 0.000 | 1.36e-09 | 4.14e-11 | 0 |
| 60–119 | 2479.2 | 2596.8 | 5.00 | 29.03 | 0.00 | 1.00 | 234.00 | 0.322 | 0.000 | 1.19e-09 | 4.06e-11 | 0 |
| 120–179 | 2463.3 | 2543.9 | 5.00 | 29.30 | 0.00 | 1.00 | 234.00 | 0.610 | 0.000 | 1.17e-09 | 5.12e-11 | 0 |
| 180–239 | 2452.2 | 3474.8 | 6.77 | 39.33 | 2.70 | 1.00 | 225.80 | 1.445 | 2.963 | 1.66e-09 | 3.26e-11 | 12 |
| 240–299 | 2390.5 | 65633.5 | 7.60 | 41.83 | 4.38 | 140.28 | 31142.90 | 12.511 | 11.352 | 1.01e-09 | 1.93e-11 | 60 |
| 300–359 | 2485.8 | 117710.5 | 8.27 | 45.25 | 5.73 | 67.72 | 15031.00 | 39.299 | 36.686 | 4.29e-09 | 1.01e-10 | 60 |
| 360–419 | 13619.0 | 209779.2 | 8.37 | 46.17 | 5.87 | 103.48 | 22340.80 | 100.208 | 56.275 | 3.31e-09 | 9.18e-11 | 60 |
| 420–479 | 12946.7 | 204622.8 | 8.00 | 40.78 | 5.38 | 373.18 | 80589.90 | 120.508 | 64.660 | 5.95e-09 | 8.47e-11 | 60 |
| 480–539 | 4550.1 | 166722.7 | 7.83 | 44.90 | 5.00 | 6.83 | 1468.80 | 149.473 | 108.290 | 9.38e-09 | 1.99e-10 | 60 |
| 540–599 | 5284.6 | 213506.7 | 7.63 | 41.32 | 5.15 | 341.77 | 73816.80 | 177.270 | 143.222 | 9.57e-09 | 2.18e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 89.462 | 64.086 | 112.016 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
