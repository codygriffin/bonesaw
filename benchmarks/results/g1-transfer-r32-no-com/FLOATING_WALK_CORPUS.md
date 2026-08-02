# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `rooted` reference at `viability` priority with weight `0.000` and `2.000 Hz` response. Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 280 | 1.400 s | 3.042 cm | 0.011 cm | 5.861 cm | 13.446 cm | 3.574° | 8.000 rad/s | 9773.6 µs |

Nominal hard residual maxima: dynamics `1.388e-09`, contact acceleration `5.449e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 197.450 cm |
| CoM RMS / p95 | 195.942 / 468.618 cm |
| stance foot RMS | 169.572 cm |
| swing foot RMS | 231.869 cm |
| hand RMS | 204.152 cm |
| maximum root rotation | 8.513° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.823e-09 |
| contact acceleration residual | 7.519e-11 |
| raw max dynamics residual, including rejected ticks | 2.823e-09 |
| raw max contact residual, including rejected ticks | 7.519e-11 |
| active normal force range | 0.000–350.662 N |
| centroidal momentum-rate residual RMS / max | 29.294 / 115.375 N·m |
| point-task acceleration RMS max | 113.464 m/s² |
| frame-angular acceleration RMS max | 234.224 rad/s² |
| longest pre-contact / touchdown transition | 52 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2289.3 µs | 29268.2 µs | 102288.4 µs | 209421.7 µs | 199 | 29 | 52 | 0 | 294 | 26 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8793.0 | 25129.7 | 461.5 | 9439.3 | 202029.3 | 208682.5 | 131842.7 | 600 | 101 | 44 | 113.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 2274.2 | 2405.5 | 2418.6 | 2472.4 |
| solved_with_slack | 29 | 2220.4 | 2761.4 | 2949.4 | 2988.4 |
| normal_contact_contingency | 294 | 2751.7 | 23314.9 | 29305.0 | 172129.7 |
| contact_release_contingency | 26 | 100828.0 | 194599.6 | 206336.4 | 209421.7 |
| precontact_transition | 52 | 1835.6 | 10179.4 | 24928.6 | 31949.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.20 | 12.0 | 13.0 | 19 | 4.89 | 12.0 | 18 | 0.1104 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 62.18/8.0/1688.1/4058 | 13466.04/1776.0/364638.2/876528 | 1.25/12.0/28 | 0.97/11.0/27 | 8.07/96.0/253 | 0.2463 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 29 | 8.14/11.0/11 | 4.52/8.0/8 |
| normal_contact_contingency | 294 | 8.82/14.1/19 | 7.57/13.0/18 |
| contact_release_contingency | 26 | 7.96/10.0/10 | 6.73/9.0/9 |
| precontact_transition | 52 | 9.42/13.0/13 | 7.79/12.5/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.59/5.0/6 | 6.07/20.0/28 | 0.89/5.0/6 | 270 |
| viability | 1.44/5.0/7 | 8.51/30.0/42 | 1.35/5.0/7 | 363 |
| intent | 1.56/5.0/8 | 7.77/28.0/48 | 1.19/5.0/8 | 377 |
| preference | 1.55/6.0/7 | 15.10/57.0/77 | 1.15/6.0/7 | 373 |
| style | 1.06/3.0/6 | 8.23/20.0/38 | 0.31/3.0/5 | 158 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 280 | 320 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `323` ticks.
Precontact sole-center tangential speed: p50 `3.2899 m/s`, p95 `8.4439 m/s`, max `9.1415 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.276 | 5.274 | 5.274 | 1.000 | 1.000 | 47.820 | 48.078 | 0.258 | 48.230 | 0.001 | 0 | 43 | 0 | 0 | 101 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2275.6 | 2413.0 | 4.00 | 29.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 60–119 | 2273.5 | 2444.3 | 4.00 | 29.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 120–179 | 2271.7 | 2412.9 | 4.00 | 29.08 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.39e-09 | 5.45e-11 | 0 |
| 180–239 | 2259.0 | 3079.4 | 7.20 | 49.35 | 3.95 | 1.00 | 225.80 | 0.209 | 0.314 | 1.13e-09 | 4.97e-11 | 12 |
| 240–299 | 1991.7 | 94608.7 | 8.98 | 56.20 | 7.28 | 162.13 | 35315.50 | 13.954 | 11.337 | 6.38e-10 | 1.34e-11 | 60 |
| 300–359 | 2668.9 | 3321.6 | 8.58 | 53.60 | 7.30 | 5.67 | 1224.00 | 50.565 | 25.905 | 4.36e-10 | 4.75e-12 | 60 |
| 360–419 | 1815.1 | 3630.2 | 8.62 | 51.70 | 7.10 | 3.45 | 745.20 | 121.817 | 93.716 | 9.70e-10 | 1.08e-11 | 60 |
| 420–479 | 1937.3 | 7175.8 | 8.55 | 52.43 | 7.08 | 17.75 | 3834.00 | 205.205 | 174.078 | 5.91e-10 | 4.54e-12 | 60 |
| 480–539 | 25966.0 | 202140.3 | 9.03 | 51.58 | 7.85 | 423.37 | 91440.70 | 322.768 | 312.032 | 2.82e-09 | 7.52e-11 | 60 |
| 540–599 | 5517.1 | 100642.9 | 9.07 | 54.90 | 8.38 | 5.43 | 1173.20 | 475.377 | 482.734 | 1.43e-09 | 5.97e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 197.450 | 192.427 | 204.152 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
