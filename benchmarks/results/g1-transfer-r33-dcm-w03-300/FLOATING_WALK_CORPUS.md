# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-centroid-preview` reference at `viability` priority with weight `0.300` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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

Functional: **PASS**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.500 s | 3.544 cm | 1.079 cm | 7.644 cm | 25.978 cm | 2.865° | 8.000 rad/s | 54455.3 µs |

Nominal hard residual maxima: dynamics `1.538e-09`, contact acceleration `5.926e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 3.544 cm |
| CoM RMS / p95 | 3.643 / 7.794 cm |
| stance foot RMS | 1.079 cm |
| swing foot RMS | 7.644 cm |
| hand RMS | 25.978 cm |
| maximum root rotation | 2.865° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.538e-09 |
| contact acceleration residual | 5.926e-11 |
| raw max dynamics residual, including rejected ticks | 1.538e-09 |
| raw max contact residual, including rejected ticks | 5.926e-11 |
| active normal force range | 0.000–355.362 N |
| centroidal momentum-rate residual RMS / max | 11.581 / 79.632 N·m |
| point-task acceleration RMS max | 73.460 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `3.173` / `7.637 cm`.
- Virtual ZMP clipped on `32.33%` of ticks; clip-distance RMS / max `4.669` / `22.516 cm`.
- Measured-height natural frequency min / p50 / max: `3.674` / `3.774` / `3.787 rad/s`.
- CoM command acceleration p95 / max: `6.322` / `7.962 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.5 s | 2440.0 µs | 3689.4 µs | 54455.3 µs | 67087.9 µs | 158 | 142 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3464.8 | 7484.2 | 70.3 | 3113.2 | 66991.0 | 67078.2 | 10190.6 | 300 | 6 | 5 | 288.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 158 | 2434.6 | 2527.0 | 2567.6 | 2578.2 |
| solved_with_slack | 142 | 2490.7 | 3953.9 | 65801.4 | 67087.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.67 | 12.0 | 16.0 | 17 | 3.12 | 15.0 | 16 | 0.2757 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 65.22/8.0/3312.1/4091 | 14486.80/1776.0/735292.9/908202 | 0.33/3.0/3 | 0.19/2.0/2 | 1.61/17.0/18 | 0.9981 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 158 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 142 | 8.54/17.0/17 | 6.59/15.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.18/3.0/4 | 4.19/12.0/16 | 0.28/3.0/4 | 45 |
| viability | 2.07/11.0/12 | 6.57/30.0/35 | 1.43/11.0/12 | 126 |
| intent | 1.34/4.0/6 | 6.72/24.0/40 | 0.81/4.0/6 | 141 |
| preference | 1.05/2.0/6 | 11.24/23.0/61 | 0.39/2.0/6 | 103 |
| style | 1.03/2.0/3 | 10.21/23.0/32 | 0.21/2.0/3 | 57 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 101 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 300 | 0 |
| left_wrist_roll_rubber_hand | 300 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 300 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.040 | 1.039 | 1.039 | 1.000 | 1.000 | 48.375 | 48.375 | 0.000 | 48.375 | 0.001 | 0 | 0 | 0 | 0 | 10 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–29 | 2443.0 | 2561.8 | 5.00 | 32.30 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.23e-09 | 3.48e-11 | 0 |
| 30–59 | 2441.4 | 2576.4 | 5.00 | 32.63 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 7.59e-10 | 5.86e-11 | 0 |
| 60–89 | 2425.3 | 2515.9 | 5.00 | 31.70 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 9.55e-10 | 5.70e-11 | 0 |
| 90–119 | 2440.1 | 2525.8 | 5.00 | 31.90 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.43e-09 | 3.84e-11 | 0 |
| 120–149 | 2422.9 | 2529.1 | 5.00 | 31.17 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.25e-09 | 4.58e-11 | 0 |
| 150–179 | 2477.7 | 4072.6 | 10.70 | 45.00 | 7.83 | 1.00 | 234.00 | 0.355 | 0.000 | 1.54e-09 | 4.12e-11 | 0 |
| 180–209 | 2466.0 | 2877.5 | 7.17 | 42.40 | 4.50 | 1.23 | 281.40 | 1.793 | 0.006 | 8.19e-10 | 5.93e-11 | 0 |
| 210–239 | 3166.1 | 3939.9 | 7.83 | 45.83 | 5.87 | 8.00 | 1776.00 | 4.031 | 0.405 | 1.12e-11 | 5.62e-13 | 0 |
| 240–269 | 1743.7 | 3059.0 | 7.23 | 43.30 | 5.30 | 2.17 | 481.00 | 6.102 | 5.049 | 7.34e-10 | 1.27e-11 | 0 |
| 270–299 | 1888.0 | 66993.9 | 8.80 | 53.10 | 7.70 | 634.80 | 140925.60 | 8.293 | 9.077 | 1.11e-09 | 2.07e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 300 | 3.544 | 3.287 | 25.978 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
