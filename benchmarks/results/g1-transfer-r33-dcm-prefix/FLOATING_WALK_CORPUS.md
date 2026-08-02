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

Functional: **PASS**  
5 ms p99 deadline: **PASS**  
Combined: **PASS**

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
| `p99_tick_le_5ms` | PASS |

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.250 s | 2.043 cm | 0.000 cm | 0.075 cm | 18.154 cm | 0.000° | 8.000 rad/s | 3191.9 µs |

Nominal hard residual maxima: dynamics `1.878e-09`, contact acceleration `5.855e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2.043 cm |
| CoM RMS / p95 | 3.745 / 7.948 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 0.075 cm |
| hand RMS | 18.154 cm |
| maximum root rotation | 0.000° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.878e-09 |
| contact acceleration residual | 5.855e-11 |
| raw max dynamics residual, including rejected ticks | 1.878e-09 |
| raw max contact residual, including rejected ticks | 5.855e-11 |
| active normal force range | 0.000–162.822 N |
| centroidal momentum-rate residual RMS / max | 2.672 / 10.960 N·m |
| point-task acceleration RMS max | 8.733 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `0.716` / `1.450 cm`.
- Virtual ZMP clipped on `10.00%` of ticks; clip-distance RMS / max `0.590` / `8.159 cm`.
- Measured-height natural frequency min / p50 / max: `3.705` / `3.781` / `3.787 rad/s`.
- CoM command acceleration p95 / max: `6.291` / `7.071 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.2 s | 2414.7 µs | 2914.3 µs | 3191.9 µs | 4062.1 µs | 158 | 92 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2385.6 | 326.0 | 51.9 | 2593.5 | 3864.7 | 4042.3 | 1220.7 | 250 | 0 | 0 | 419.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 158 | 2411.1 | 2501.7 | 2556.9 | 2710.3 |
| solved_with_slack | 92 | 2425.0 | 3106.8 | 3340.7 | 4062.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.29 | 12.0 | 15.5 | 17 | 2.08 | 12.5 | 15 | 0.0734 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.48/8.0/8.0/8 | 337.22/1776.0/1776.0/1776 | 0.14/2.0/2 | 0.07/1.0/1 | 0.56/9.0/9 | 0.5369 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 158 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 92 | 8.50/17.0/17 | 5.65/15.0/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.00/1.0/1 | 3.35/4.0/4 | 0.00/0.0/0 | 0 |
| viability | 1.87/11.0/13 | 4.48/22.0/26 | 1.00/10.5/12 | 46 |
| intent | 1.38/5.0/6 | 6.76/24.5/36 | 0.75/5.0/6 | 92 |
| preference | 1.02/1.5/4 | 11.04/16.6/41 | 0.24/1.5/4 | 53 |
| style | 1.01/1.5/2 | 10.50/15.5/21 | 0.09/1.0/1 | 23 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 51 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 250 | 0 |
| left_wrist_roll_rubber_hand | 250 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 250 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.597 | 0.596 | 0.596 | 0.999 | 0.999 | 47.230 | 47.230 | 0.000 | 47.855 | 0.001 | 0 | 0 | 0 | 0 | 9 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–24 | 2419.7 | 2516.2 | 5.00 | 32.44 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.23e-09 | 3.48e-11 | 0 |
| 25–49 | 2441.1 | 2566.3 | 5.00 | 32.68 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.88e-09 | 5.86e-11 | 0 |
| 50–74 | 2414.6 | 2508.4 | 5.00 | 32.24 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 9.55e-10 | 3.23e-11 | 0 |
| 75–99 | 2437.5 | 2483.2 | 5.00 | 31.48 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.43e-09 | 5.70e-11 | 0 |
| 100–124 | 2400.4 | 2489.5 | 5.00 | 31.84 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 6.03e-10 | 3.94e-11 | 0 |
| 125–149 | 2378.6 | 2644.2 | 5.00 | 30.88 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.25e-09 | 4.22e-11 | 0 |
| 150–174 | 2447.5 | 3202.3 | 9.64 | 42.04 | 6.36 | 1.00 | 234.00 | 0.230 | 0.000 | 1.18e-09 | 4.19e-11 | 0 |
| 175–199 | 2444.0 | 2733.7 | 9.40 | 42.04 | 6.76 | 1.00 | 233.52 | 2.138 | 0.000 | 1.19e-09 | 4.35e-11 | 0 |
| 200–224 | 1709.3 | 3747.5 | 7.00 | 42.44 | 3.88 | 1.84 | 408.48 | 3.893 | 0.010 | 1.15e-09 | 2.47e-11 | 0 |
| 225–249 | 2858.3 | 3231.1 | 6.84 | 43.28 | 3.80 | 4.92 | 1092.24 | 4.688 | 0.075 | 3.21e-10 | 2.78e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 250 | 2.043 | 0.024 | 18.154 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
