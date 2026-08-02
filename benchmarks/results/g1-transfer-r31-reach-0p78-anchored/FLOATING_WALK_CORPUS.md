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
| `no_infeasible_or_failed_ticks` | PASS |
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

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 541 | 2.705 s | 45.704 cm | 30.654 cm | 43.434 cm | 71.518 cm | 138.491° | 8.000 rad/s | 76537.5 µs |

Nominal hard residual maxima: dynamics `3.664e-09`, contact acceleration `1.220e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 66.016 cm |
| stance foot RMS | 46.787 cm |
| swing foot RMS | 51.366 cm |
| hand RMS | 96.809 cm |
| maximum root rotation | 167.857° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.581e-09 |
| contact acceleration residual | 1.932e-10 |
| raw max dynamics residual, including rejected ticks | 6.581e-09 |
| raw max contact residual, including rejected ticks | 1.932e-10 |
| active normal force range | 0.000–939.514 N |
| centroidal momentum-rate residual RMS / max | 52.966 / 329.651 N·m |
| point-task acceleration RMS max | 149.326 m/s² |
| frame-angular acceleration RMS max | 134.653 rad/s² |
| longest pre-contact / touchdown transition | 200 / 113 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2893.8 µs | 12625.9 µs | 80092.5 µs | 340366.6 µs | 141 | 87 | 200 | 113 | 56 | 3 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6550.6 | 19766.4 | 478.9 | 8932.0 | 237951.8 | 330125.1 | 53360.1 | 600 | 93 | 20 | 152.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2475.8 | 2586.4 | 2608.5 | 2613.9 |
| solved_with_slack | 87 | 2776.0 | 4934.6 | 5997.3 | 6092.5 |
| normal_contact_contingency | 56 | 2931.0 | 8136.4 | 8839.9 | 8916.6 |
| contact_release_contingency | 3 | 169390.4 | 323269.0 | 336947.0 | 340366.6 |
| touchdown_transition | 113 | 8823.9 | 76386.1 | 87434.2 | 90040.1 |
| precontact_transition | 200 | 3243.8 | 4058.3 | 4826.3 | 24881.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.83 | 12.0 | 14.0 | 19 | 5.62 | 13.0 | 18 | 0.1919 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 121.75/8.0/4788.9/5756 | 26333.89/1824.0/1034408.9/1243296 | 2.16/14.0/16 | 1.67/13.0/15 | 13.73/108.0/133 | 0.5452 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 87 | 8.29/12.6/16 | 5.83/11.3/13 |
| normal_contact_contingency | 56 | 7.96/10.0/10 | 7.12/10.0/10 |
| contact_release_contingency | 3 | 8.33/10.0/10 | 7.00/8.9/9 |
| touchdown_transition | 113 | 9.93/15.0/19 | 8.82/14.0/18 |
| precontact_transition | 200 | 8.40/13.0/14 | 7.25/11.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.01/5.0/6 | 8.59/25.0/30 | 1.54/5.0/6 | 355 |
| viability | 1.98/7.0/12 | 11.69/42.0/56 | 1.74/7.0/12 | 452 |
| intent | 1.41/5.0/11 | 7.56/24.0/52 | 1.14/5.0/11 | 434 |
| preference | 1.26/5.0/10 | 12.64/55.1/102 | 0.79/5.0/10 | 331 |
| style | 1.17/4.0/5 | 10.44/35.0/69 | 0.42/4.0/5 | 182 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 113 | 199 | 59 |
| right_ankle_roll_link | 168 | 0 | 0 | 432 | 0 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `113` ticks, normal fallback `62` ticks.
Precontact sole-center tangential speed: p50 `1.9381 m/s`, p95 `4.1791 m/s`, max `5.2342 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `1.7329 m/s`, p95 `5.9558 m/s`, max `7.0576 m/s` over 112 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.931 | 3.930 | 3.930 | 1.000 | 1.000 | 47.629 | 47.852 | 0.223 | 47.871 | 0.001 | 0 | 41 | 0 | 0 | 38 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2504.5 | 2941.7 | 6.07 | 36.82 | 1.67 | 1.00 | 234.00 | 0.172 | 0.000 | 1.62e-09 | 5.63e-11 | 0 |
| 60–119 | 2470.7 | 3550.7 | 5.10 | 34.98 | 0.17 | 1.00 | 234.00 | 0.032 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–179 | 2531.6 | 5539.0 | 6.12 | 39.43 | 1.62 | 1.12 | 261.30 | 0.002 | 0.000 | 1.46e-09 | 6.30e-11 | 0 |
| 180–239 | 2818.7 | 5474.6 | 8.28 | 53.07 | 6.50 | 3.92 | 883.10 | 3.367 | 3.391 | 1.47e-09 | 5.34e-11 | 12 |
| 240–299 | 3188.5 | 4130.2 | 8.50 | 56.97 | 7.17 | 6.13 | 1361.60 | 9.191 | 21.270 | 6.67e-10 | 1.61e-11 | 60 |
| 300–359 | 3270.7 | 4089.0 | 8.25 | 51.35 | 7.25 | 7.30 | 1620.60 | 16.264 | 28.529 | 9.25e-10 | 1.00e-11 | 60 |
| 360–419 | 3290.1 | 13115.0 | 8.40 | 53.88 | 7.25 | 28.43 | 6312.20 | 30.200 | 39.227 | 3.45e-09 | 1.22e-10 | 60 |
| 420–479 | 10842.9 | 88903.3 | 10.15 | 69.55 | 9.30 | 1122.35 | 242440.40 | 60.491 | 68.158 | 1.04e-09 | 3.46e-11 | 8 |
| 480–539 | 2938.6 | 16191.5 | 9.47 | 60.85 | 8.17 | 38.35 | 8283.60 | 116.137 | 59.966 | 1.65e-09 | 1.76e-11 | 0 |
| 540–599 | 3098.4 | 239490.6 | 8.00 | 52.25 | 7.13 | 7.93 | 1708.10 | 158.624 | 110.991 | 6.58e-09 | 1.93e-10 | 59 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 66.016 | 48.350 | 96.809 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
