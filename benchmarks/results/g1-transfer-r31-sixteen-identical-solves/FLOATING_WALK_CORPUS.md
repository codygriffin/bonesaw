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
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 434 | 2.170 s | 9.593 cm | 3.304 cm | 37.173 cm | 20.519 cm | 2.726° | 8.000 rad/s | 5418.8 µs |

Nominal hard residual maxima: dynamics `1.758e-09`, contact acceleration `5.441e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 10.501 cm |
| stance foot RMS | 6.765 cm |
| swing foot RMS | 35.964 cm |
| hand RMS | 20.662 cm |
| maximum root rotation | 2.726° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.758e-09 |
| contact acceleration residual | 5.441e-11 |
| raw max dynamics residual, including rejected ticks | 1.758e-09 |
| raw max contact residual, including rejected ticks | 5.441e-11 |
| active normal force range | 0.000–407.113 N |
| centroidal momentum-rate residual RMS / max | 23.223 / 111.273 N·m |
| point-task acceleration RMS max | 99.188 m/s² |
| frame-angular acceleration RMS max | 141.153 rad/s² |
| longest pre-contact / touchdown transition | 200 / 6 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 450 | 2.2 s | 2598.0 µs | 4781.7 µs | 889037.0 µs | 1760726.7 µs | 150 | 78 | 200 | 6 | 11 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 22643.5 | 174319.4 | 270.5 | 4124.4 | 1709093.1 | 1755563.4 | 58782.1 | 450 | 19 | 11 | 44.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 150 | 2400.8 | 2570.0 | 2672.0 | 2715.4 |
| solved_with_slack | 78 | 3068.2 | 4554.4 | 5394.5 | 5494.2 |
| normal_contact_contingency | 11 | 4117.4 | 112463.0 | 117174.7 | 118352.6 |
| contact_release_contingency | 5 | 1642386.7 | 1737727.3 | 1756126.8 | 1760726.7 |
| touchdown_transition | 6 | 5558.7 | 106409.4 | 108034.2 | 108440.4 |
| precontact_transition | 200 | 2925.9 | 4591.6 | 5078.1 | 5356.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.95 | 10.0 | 11.0 | 13 | 3.05 | 9.5 | 12 | 0.0079 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 78.16/1.0/4425.3/6864 | 16893.61/234.0/955873.4/1482624 | 0.01/1.0/1 | 0.01/1.0/1 | 0.12/8.0/10 | 0.0534 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 150 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 78 | 7.31/12.2/13 | 4.41/9.2/10 |
| normal_contact_contingency | 11 | 6.09/8.0/8 | 4.00/5.9/6 |
| contact_release_contingency | 5 | 6.00/7.0/7 | 3.40/5.0/5 |
| touchdown_transition | 6 | 9.00/12.9/13 | 6.83/9.9/10 |
| precontact_transition | 200 | 6.78/11.0/12 | 4.64/10.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.30/4.5/6 | 4.84/18.0/24 | 0.46/4.5/6 | 75 |
| viability | 2.60/8.0/10 | 20.16/65.0/70 | 2.24/8.0/10 | 294 |
| intent | 1.00/1.0/1 | 3.96/5.0/5 | 0.00/0.0/1 | 1 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.04/2.0/3 | 9.05/23.0/35 | 0.35/1.5/3 | 150 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 6 | 199 | 16 |
| right_ankle_roll_link | 18 | 0 | 0 | 432 | 0 |
| left_wrist_roll_rubber_hand | 450 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 450 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `6` ticks, normal fallback `19` ticks.
Precontact sole-center tangential speed: p50 `0.8785 m/s`, p95 `2.4373 m/s`, max `3.6125 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `4.1025 m/s`, p95 `4.6282 m/s`, max `4.6963 m/s` over 5 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10.190 | 10.188 | 10.188 | 1.000 | 1.000 | 46.594 | 46.797 | 0.203 | 47.652 | 0.001 | 0 | 51 | 0 | 0 | 73 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–44 | 2428.6 | 4016.6 | 5.02 | 30.33 | 1.18 | 1.00 | 234.00 | 0.602 | 0.000 | 1.47e-09 | 4.71e-11 | 0 |
| 45–89 | 2385.5 | 2652.4 | 4.00 | 24.11 | 0.00 | 1.00 | 234.00 | 1.225 | 0.000 | 1.76e-09 | 4.45e-11 | 0 |
| 90–134 | 2400.4 | 2698.5 | 4.00 | 24.00 | 0.00 | 1.00 | 234.00 | 1.356 | 0.000 | 1.36e-09 | 5.44e-11 | 0 |
| 135–179 | 2603.0 | 5437.2 | 5.60 | 35.00 | 2.20 | 1.00 | 234.00 | 1.394 | 0.000 | 9.62e-10 | 4.60e-11 | 0 |
| 180–224 | 2967.6 | 4232.7 | 6.87 | 45.62 | 3.96 | 1.00 | 227.07 | 4.238 | 1.573 | 9.33e-10 | 4.58e-11 | 0 |
| 225–269 | 2873.2 | 5002.1 | 6.73 | 43.36 | 4.27 | 1.00 | 222.00 | 7.503 | 10.950 | 1.19e-09 | 1.36e-11 | 42 |
| 270–314 | 2713.2 | 4821.1 | 6.51 | 40.58 | 4.56 | 1.00 | 222.00 | 7.429 | 28.626 | 1.50e-09 | 2.46e-11 | 45 |
| 315–359 | 3390.3 | 5202.6 | 7.33 | 51.11 | 5.38 | 1.00 | 222.00 | 11.000 | 31.952 | 1.16e-09 | 1.66e-11 | 45 |
| 360–404 | 2924.9 | 4253.6 | 6.76 | 43.38 | 4.62 | 1.00 | 222.00 | 17.423 | 34.167 | 1.52e-09 | 1.63e-11 | 45 |
| 405–449 | 3455.0 | 1710128.1 | 6.64 | 42.73 | 4.38 | 772.60 | 166885.07 | 23.304 | 27.416 | 8.24e-10 | 1.36e-11 | 39 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 450 | 10.501 | 19.702 | 20.662 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
