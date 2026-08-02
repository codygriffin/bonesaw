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
- Whole-body posture: `intent` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `1.000` over 11 waist/arm coordinates.
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
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.000 s | 30.471 cm | 25.435 cm | 58.406 cm | 35.975 cm | 5.338° | 8.000 rad/s | 41938.6 µs |

Nominal hard residual maxima: dynamics `1.610e-09`, contact acceleration `5.497e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 30.471 cm |
| stance foot RMS | 25.435 cm |
| swing foot RMS | 58.406 cm |
| hand RMS | 35.975 cm |
| maximum root rotation | 5.338° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.610e-09 |
| contact acceleration residual | 5.497e-11 |
| raw max dynamics residual, including rejected ticks | 1.610e-09 |
| raw max contact residual, including rejected ticks | 5.497e-11 |
| active normal force range | 0.000–503.302 N |
| centroidal momentum-rate residual RMS / max | 21.132 / 67.555 N·m |
| point-task acceleration RMS max | 107.488 m/s² |
| frame-angular acceleration RMS max | 85.012 rad/s² |
| longest pre-contact / touchdown transition | 200 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2972.4 µs | 5666.2 µs | 41938.6 µs | 89283.6 µs | 147 | 81 | 200 | 172 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4154.2 | 7168.7 | 524.9 | 4815.5 | 81886.9 | 88543.9 | 11871.7 | 600 | 50 | 9 | 240.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 147 | 2881.1 | 3007.2 | 3085.6 | 3126.2 |
| solved_with_slack | 81 | 3165.1 | 4832.7 | 8032.1 | 8042.7 |
| touchdown_transition | 172 | 3538.1 | 22010.1 | 69898.3 | 89283.6 |
| precontact_transition | 200 | 2507.9 | 5388.3 | 6570.6 | 7193.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.70 | 11.0 | 14.0 | 16 | 4.44 | 12.0 | 14 | 0.2591 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 49.96/8.0/2653.9/3977 | 11069.95/1776.0/573567.0/906756 | 0.48/3.0/4 | 0.27/2.0/3 | 2.13/16.0/24 | 0.9817 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 147 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 81 | 7.36/16.0/16 | 4.88/14.0/14 |
| touchdown_transition | 172 | 7.81/14.3/15 | 6.12/13.3/14 |
| precontact_transition | 200 | 7.46/12.0/14 | 6.08/11.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.48/4.0/8 | 5.66/16.0/32 | 0.72/4.0/8 | 217 |
| viability | 2.56/7.0/12 | 15.23/49.0/57 | 2.31/7.0/12 | 453 |
| intent | 1.62/6.0/8 | 12.31/51.0/65 | 1.20/6.0/8 | 390 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.04/2.0/9 | 8.85/21.0/100 | 0.22/2.0/8 | 116 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 172 | 199 | 0 |
| right_ankle_roll_link | 168 | 0 | 0 | 432 | 0 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `172` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `1.4832 m/s`, p95 `2.7187 m/s`, max `2.8617 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `2.0846 m/s`, p95 `3.0010 m/s`, max `3.3968 m/s` over 171 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.493 | 2.492 | 2.492 | 1.000 | 1.000 | 47.621 | 47.855 | 0.234 | 48.027 | 0.001 | 0 | 41 | 0 | 0 | 40 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2942.6 | 5820.3 | 5.13 | 28.97 | 1.43 | 1.00 | 234.00 | 0.788 | 0.000 | 1.61e-09 | 5.50e-11 | 0 |
| 60–119 | 2892.0 | 3015.5 | 4.00 | 23.72 | 0.00 | 1.00 | 234.00 | 1.324 | 0.000 | 1.19e-09 | 5.27e-11 | 0 |
| 120–179 | 2825.8 | 4418.4 | 5.10 | 28.25 | 1.53 | 1.00 | 234.00 | 1.379 | 0.000 | 1.13e-09 | 4.38e-11 | 0 |
| 180–239 | 3519.8 | 6579.1 | 6.90 | 42.60 | 4.63 | 3.68 | 821.50 | 5.507 | 5.095 | 1.18e-09 | 5.12e-11 | 12 |
| 240–299 | 2517.0 | 5730.0 | 7.57 | 48.75 | 6.22 | 2.05 | 455.10 | 9.499 | 26.987 | 8.93e-10 | 9.87e-12 | 60 |
| 300–359 | 2397.7 | 5684.4 | 7.08 | 45.50 | 5.67 | 1.00 | 222.00 | 8.620 | 22.975 | 1.20e-09 | 1.85e-11 | 60 |
| 360–419 | 2915.3 | 6549.3 | 7.62 | 47.28 | 6.30 | 2.40 | 532.80 | 16.224 | 25.543 | 7.77e-10 | 2.27e-11 | 60 |
| 420–479 | 3406.7 | 81998.1 | 7.62 | 51.58 | 5.95 | 477.07 | 105716.10 | 17.720 | 42.677 | 4.67e-10 | 5.28e-12 | 8 |
| 480–539 | 3798.4 | 7546.6 | 7.70 | 49.20 | 6.08 | 6.72 | 1450.80 | 39.133 | 39.667 | 3.82e-10 | 2.82e-12 | 0 |
| 540–599 | 3741.1 | 6785.6 | 8.28 | 54.60 | 6.58 | 3.70 | 799.20 | 83.529 | 101.416 | 4.63e-10 | 4.73e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 30.471 | 39.515 | 35.975 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
