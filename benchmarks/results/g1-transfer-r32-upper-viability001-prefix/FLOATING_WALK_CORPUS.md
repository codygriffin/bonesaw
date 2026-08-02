# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response. Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `viability` priority with weight `0.010` over 11 waist/arm coordinates.
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
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | PASS |

Failed checks: `swing_foot_tracking_rms_le_8cm`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.250 s | 3.053 cm | 0.000 cm | 10.568 cm | 5.437 cm | 0.000° | 8.000 rad/s | 4814.0 µs |

Nominal hard residual maxima: dynamics `1.637e-09`, contact acceleration `5.941e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 3.053 cm |
| CoM RMS / p95 | 3.610 / 8.141 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 10.568 cm |
| hand RMS | 5.437 cm |
| maximum root rotation | 0.000° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.637e-09 |
| contact acceleration residual | 5.941e-11 |
| raw max dynamics residual, including rejected ticks | 1.637e-09 |
| raw max contact residual, including rejected ticks | 5.941e-11 |
| active normal force range | 0.000–187.038 N |
| centroidal momentum-rate residual RMS / max | 4.133 / 12.457 N·m |
| point-task acceleration RMS max | 88.955 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.2 s | 2451.7 µs | 4031.1 µs | 4814.0 µs | 5355.9 µs | 148 | 102 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2723.6 | 614.9 | 62.1 | 3769.7 | 5226.5 | 5342.9 | 2185.8 | 250 | 1 | 0 | 367.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 148 | 2430.6 | 2504.1 | 2626.5 | 3767.2 |
| solved_with_slack | 102 | 2845.0 | 4756.9 | 4836.0 | 5355.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.55 | 11.0 | 13.0 | 15 | 2.27 | 10.0 | 11 | 0.7008 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.56/8.0/8.0/8 | 355.87/1776.0/1776.0/1776 | 0.16/2.0/2 | 0.08/1.0/1 | 0.66/9.0/9 | 0.4488 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 148 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 102 | 8.79/14.0/15 | 5.57/11.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.00/1.0/1 | 3.34/4.0/4 | 0.00/0.0/0 | 0 |
| viability | 2.30/9.0/11 | 13.83/54.0/66 | 1.70/9.0/11 | 102 |
| intent | 1.16/3.5/4 | 4.66/15.0/20 | 0.30/3.0/4 | 48 |
| preference | 1.00/1.0/2 | 4.92/6.0/12 | 0.08/1.0/2 | 20 |
| style | 1.09/3.0/3 | 11.10/33.0/34 | 0.18/3.0/3 | 33 |

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
| 0.681 | 0.681 | 0.681 | 1.000 | 1.000 | 47.984 | 47.984 | 0.000 | 47.984 | 0.001 | 0 | 0 | 0 | 0 | 17 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–24 | 2496.3 | 3892.6 | 7.08 | 41.56 | 2.48 | 1.00 | 234.00 | 0.318 | 0.000 | 1.40e-09 | 3.38e-11 | 0 |
| 25–49 | 2453.3 | 3499.2 | 5.00 | 29.24 | 0.00 | 1.00 | 234.00 | 0.940 | 0.000 | 1.48e-09 | 4.83e-11 | 0 |
| 50–74 | 2412.9 | 2498.3 | 5.00 | 28.92 | 0.00 | 1.00 | 234.00 | 1.270 | 0.000 | 1.61e-09 | 4.29e-11 | 0 |
| 75–99 | 2423.2 | 2477.8 | 5.00 | 28.68 | 0.00 | 1.00 | 234.00 | 1.387 | 0.000 | 1.39e-09 | 5.94e-11 | 0 |
| 100–124 | 2414.5 | 2466.8 | 5.00 | 29.28 | 0.00 | 1.00 | 234.00 | 1.425 | 0.000 | 1.64e-09 | 3.28e-11 | 0 |
| 125–149 | 2439.9 | 2512.9 | 5.00 | 29.64 | 0.00 | 1.00 | 234.00 | 1.437 | 0.000 | 6.86e-10 | 5.27e-11 | 0 |
| 150–174 | 2823.0 | 5231.2 | 7.64 | 47.60 | 3.44 | 1.00 | 234.00 | 1.445 | 0.000 | 1.12e-09 | 3.11e-11 | 0 |
| 175–199 | 2830.1 | 4817.6 | 8.04 | 50.28 | 4.36 | 1.00 | 233.52 | 2.450 | 0.002 | 1.17e-09 | 2.82e-11 | 0 |
| 200–224 | 2228.9 | 3271.4 | 8.88 | 47.08 | 6.52 | 1.00 | 222.00 | 5.239 | 2.540 | 1.34e-09 | 1.56e-11 | 0 |
| 225–249 | 3553.5 | 4204.3 | 8.84 | 46.28 | 5.92 | 6.60 | 1465.20 | 7.004 | 10.367 | 1.44e-09 | 1.39e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 250 | 3.053 | 3.375 | 5.437 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
