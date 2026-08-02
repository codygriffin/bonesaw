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
- Whole-body posture: `viability` priority with weight `0.001`.
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
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | FAIL |
| `touchdown_transition_completes_within_8_ticks` | PASS |
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
| 528 | 2.640 s | 16.161 cm | 12.680 cm | 31.660 cm | 32.547 cm | 78.184° | 8.000 rad/s | 99419.7 µs |

Nominal hard residual maxima: dynamics `1.758e-09`, contact acceleration `5.441e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 41.889 cm |
| stance foot RMS | 27.129 cm |
| swing foot RMS | 33.022 cm |
| hand RMS | 62.803 cm |
| maximum root rotation | 179.783° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.188e-09 |
| contact acceleration residual | 1.284e-10 |
| raw max dynamics residual, including rejected ticks | 6.188e-09 |
| raw max contact residual, including rejected ticks | 1.284e-10 |
| active normal force range | 0.000–935.957 N |
| centroidal momentum-rate residual RMS / max | 65.889 / 379.601 N·m |
| point-task acceleration RMS max | 170.633 m/s² |
| frame-angular acceleration RMS max | 241.976 rad/s² |
| longest pre-contact / touchdown transition | 300 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2794.9 µs | 12187.5 µs | 101258.6 µs | 196665.3 µs | 150 | 78 | 300 | 0 | 69 | 3 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7709.5 | 20577.9 | 455.0 | 6951.4 | 176116.9 | 194610.4 | 87451.5 | 600 | 89 | 28 | 129.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 150 | 2389.3 | 2471.2 | 2524.4 | 2628.0 |
| solved_with_slack | 78 | 2891.4 | 4416.5 | 5075.1 | 5106.5 |
| normal_contact_contingency | 69 | 7640.4 | 92442.5 | 105139.6 | 117055.2 |
| contact_release_contingency | 3 | 162360.8 | 193234.8 | 195979.2 | 196665.3 |
| precontact_transition | 300 | 2966.5 | 39671.6 | 101258.6 | 105352.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.32 | 10.0 | 11.0 | 13 | 3.81 | 9.0 | 12 | 0.2347 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 220.90/8.0/6157.3/6445 | 48566.77/1776.0/1358282.6/1430790 | 0.96/12.0/13 | 0.79/11.0/12 | 6.61/92.0/108 | 0.8124 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 150 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 78 | 7.31/12.2/13 | 4.41/9.2/10 |
| normal_contact_contingency | 69 | 7.75/10.6/12 | 6.29/9.3/10 |
| contact_release_contingency | 3 | 6.33/7.0/7 | 4.33/5.0/5 |
| precontact_transition | 300 | 6.90/11.0/12 | 4.99/10.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.76/5.0/7 | 7.05/24.0/34 | 1.14/5.0/7 | 230 |
| viability | 2.38/8.0/10 | 19.14/63.0/70 | 2.11/8.0/10 | 447 |
| intent | 1.00/1.0/1 | 4.05/5.0/6 | 0.00/0.0/1 | 1 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.18/5.0/6 | 9.67/35.0/42 | 0.56/4.0/6 | 253 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 528 | 72 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `75` ticks.
Precontact sole-center tangential speed: p50 `2.2370 m/s`, p95 `10.2446 m/s`, max `10.5813 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.626 | 4.625 | 4.625 | 1.000 | 1.000 | 47.570 | 47.789 | 0.219 | 47.980 | 0.001 | 0 | 40 | 0 | 0 | 29 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2420.3 | 3884.5 | 4.77 | 28.87 | 0.88 | 1.00 | 234.00 | 0.764 | 0.000 | 1.47e-09 | 4.71e-11 | 0 |
| 60–119 | 2356.9 | 2539.1 | 4.00 | 24.05 | 0.00 | 1.00 | 234.00 | 1.313 | 0.000 | 1.76e-09 | 5.44e-11 | 0 |
| 120–179 | 2424.5 | 5082.4 | 5.20 | 32.17 | 1.65 | 1.00 | 234.00 | 1.387 | 0.000 | 1.36e-09 | 4.60e-11 | 0 |
| 180–239 | 2891.4 | 4165.5 | 6.87 | 45.43 | 4.03 | 1.00 | 225.80 | 5.032 | 3.276 | 1.19e-09 | 4.58e-11 | 12 |
| 240–299 | 2753.1 | 4830.5 | 6.50 | 41.12 | 4.27 | 1.00 | 222.00 | 7.466 | 21.387 | 1.50e-09 | 2.46e-11 | 60 |
| 300–359 | 3136.2 | 5096.7 | 7.22 | 48.95 | 5.32 | 1.00 | 222.00 | 10.346 | 31.685 | 1.26e-09 | 1.66e-11 | 60 |
| 360–419 | 2900.5 | 4492.7 | 6.72 | 43.87 | 4.45 | 1.00 | 222.00 | 18.544 | 33.151 | 1.52e-09 | 1.63e-11 | 60 |
| 420–479 | 3159.1 | 5433.1 | 6.77 | 39.88 | 5.18 | 4.15 | 921.30 | 25.296 | 25.595 | 1.27e-09 | 1.01e-11 | 60 |
| 480–539 | 6998.7 | 110150.3 | 7.73 | 47.80 | 6.43 | 2086.88 | 459189.80 | 42.943 | 30.672 | 1.40e-09 | 4.70e-11 | 60 |
| 540–599 | 6980.1 | 176425.6 | 7.45 | 46.98 | 5.92 | 110.95 | 23962.80 | 120.527 | 66.072 | 6.19e-09 | 1.28e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 41.889 | 29.210 | 62.803 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
