# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-centroid-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.500 s | 4.301 cm | 0.607 cm | 3.153 cm | 25.230 cm | 7.394° | 8.000 rad/s | 34704.9 µs |

Nominal hard residual maxima: dynamics `1.878e-09`, contact acceleration `7.798e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 4.301 cm |
| CoM RMS / p95 | 3.451 / 7.732 cm |
| stance foot RMS | 0.607 cm |
| swing foot RMS | 3.153 cm |
| hand RMS | 25.230 cm |
| maximum root rotation | 7.394° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.878e-09 |
| contact acceleration residual | 7.798e-11 |
| raw max dynamics residual, including rejected ticks | 1.878e-09 |
| raw max contact residual, including rejected ticks | 7.798e-11 |
| active normal force range | 0.000–413.112 N |
| centroidal momentum-rate residual RMS / max | 9.258 / 43.334 N·m |
| point-task acceleration RMS max | 41.053 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `3.816` / `10.875 cm`.
- Virtual ZMP clipped on `27.33%` of ticks; clip-distance RMS / max `6.514` / `25.400 cm`.
- Measured-height natural frequency min / p50 / max: `3.718` / `3.776` / `3.787 rad/s`.
- CoM command acceleration p95 / max: `5.800` / `9.151 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.5 s | 2474.2 µs | 3761.1 µs | 34704.9 µs | 49507.9 µs | 158 | 142 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3136.2 | 4842.4 | 80.4 | 3581.2 | 48860.0 | 49443.1 | 6808.8 | 300 | 7 | 4 | 318.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 158 | 2451.6 | 2558.5 | 2584.1 | 2654.1 |
| solved_with_slack | 142 | 2618.0 | 3848.4 | 46147.6 | 49507.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.73 | 12.0 | 15.0 | 17 | 3.11 | 14.0 | 15 | 0.1425 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 37.97/8.0/2074.3/3048 | 8438.04/1776.0/460490.2/676656 | 0.53/3.0/8 | 0.34/2.0/7 | 2.70/17.2/56 | 0.9905 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 158 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 142 | 8.65/17.0/17 | 6.56/14.6/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.28/4.0/5 | 4.57/16.0/21 | 0.41/4.0/5 | 65 |
| viability | 2.01/10.0/13 | 6.25/26.0/50 | 1.31/10.0/13 | 98 |
| intent | 1.39/4.0/5 | 6.97/24.0/30 | 0.87/4.0/5 | 142 |
| preference | 1.03/2.0/3 | 11.02/22.0/30 | 0.32/2.0/3 | 90 |
| style | 1.02/2.0/3 | 9.97/16.0/26 | 0.20/1.0/3 | 55 |

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
| 0.941 | 0.941 | 0.941 | 1.000 | 1.000 | 47.262 | 47.262 | 0.000 | 47.855 | 0.001 | 0 | 0 | 0 | 0 | 11 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–29 | 2456.7 | 2637.8 | 5.00 | 32.40 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.88e-09 | 4.95e-11 | 0 |
| 30–59 | 2465.7 | 2571.6 | 5.00 | 32.60 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 8.98e-10 | 5.86e-11 | 0 |
| 60–89 | 2466.5 | 2559.2 | 5.00 | 31.83 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 9.81e-10 | 5.70e-11 | 0 |
| 90–119 | 2471.7 | 2563.1 | 5.00 | 31.83 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.43e-09 | 3.94e-11 | 0 |
| 120–149 | 2420.8 | 2533.1 | 5.00 | 30.97 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.25e-09 | 4.22e-11 | 0 |
| 150–179 | 2468.7 | 3069.0 | 10.57 | 42.80 | 7.77 | 1.00 | 234.00 | 0.396 | 0.000 | 1.29e-09 | 7.80e-11 | 0 |
| 180–209 | 2463.5 | 3065.3 | 7.57 | 43.63 | 4.73 | 1.00 | 229.60 | 2.029 | 0.003 | 1.17e-09 | 4.77e-11 | 0 |
| 210–239 | 3375.3 | 3813.4 | 8.50 | 49.70 | 6.20 | 7.07 | 1568.80 | 4.049 | 0.403 | 3.71e-10 | 4.73e-12 | 0 |
| 240–269 | 3121.0 | 3807.7 | 7.70 | 44.43 | 6.10 | 6.83 | 1517.00 | 7.383 | 2.490 | 2.39e-10 | 3.00e-12 | 0 |
| 270–299 | 1740.9 | 48879.5 | 7.93 | 47.67 | 6.27 | 358.83 | 79661.00 | 10.478 | 3.665 | 7.36e-10 | 1.28e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 300 | 4.301 | 1.407 | 25.230 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
