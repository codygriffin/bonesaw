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
5 ms p99 deadline: **PASS**  
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
| `p99_tick_le_5ms` | PASS |

Failed checks: `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.000 s | 23.670 cm | 26.328 cm | 28.588 cm | 46.347 cm | 9.339° | 8.000 rad/s | 4711.6 µs |

Nominal hard residual maxima: dynamics `1.622e-09`, contact acceleration `7.067e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 23.670 cm |
| stance foot RMS | 26.328 cm |
| swing foot RMS | 28.588 cm |
| hand RMS | 46.347 cm |
| maximum root rotation | 9.339° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.622e-09 |
| contact acceleration residual | 7.067e-11 |
| raw max dynamics residual, including rejected ticks | 1.622e-09 |
| raw max contact residual, including rejected ticks | 7.067e-11 |
| active normal force range | 0.000–337.178 N |
| centroidal momentum-rate residual RMS / max | 21.260 / 95.078 N·m |
| point-task acceleration RMS max | 96.291 m/s² |
| frame-angular acceleration RMS max | 118.844 rad/s² |
| longest pre-contact / touchdown transition | 200 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2508.8 µs | 3710.8 µs | 4711.6 µs | 6066.5 µs | 141 | 87 | 200 | 172 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2584.7 | 658.0 | 511.5 | 3296.2 | 5611.8 | 6021.1 | 2030.0 | 600 | 3 | 0 | 386.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2477.2 | 2579.2 | 2598.0 | 2603.2 |
| solved_with_slack | 87 | 2756.7 | 4898.1 | 5413.7 | 6066.5 |
| touchdown_transition | 172 | 2750.2 | 3496.5 | 3936.3 | 4419.3 |
| precontact_transition | 200 | 2369.9 | 3712.9 | 4064.6 | 4491.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.53 | 11.0 | 12.0 | 16 | 5.01 | 11.0 | 13 | 0.2449 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 3.42/8.0/8.0/8 | 755.22/1776.0/1872.0/1872 | 0.73/3.0/3 | 0.38/2.0/2 | 3.08/16.0/18 | 0.7183 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 87 | 8.18/12.6/16 | 5.79/11.3/13 |
| touchdown_transition | 172 | 8.07/12.0/13 | 6.16/11.0/12 |
| precontact_transition | 200 | 8.56/12.0/13 | 7.21/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.55/4.0/7 | 5.93/16.0/28 | 0.81/4.0/7 | 250 |
| viability | 2.37/7.0/12 | 14.10/45.0/49 | 2.12/7.0/12 | 452 |
| intent | 1.42/4.0/5 | 7.76/24.0/29 | 1.14/4.0/5 | 434 |
| preference | 1.16/4.0/7 | 11.57/40.0/75 | 0.65/4.0/7 | 306 |
| style | 1.04/2.0/3 | 9.62/25.0/41 | 0.29/2.0/3 | 164 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 172 | 199 | 0 |
| right_ankle_roll_link | 168 | 0 | 0 | 432 | 0 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `172` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `1.5683 m/s`, p95 `2.0164 m/s`, max `2.1804 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `1.0920 m/s`, p95 `3.3762 m/s`, max `3.4308 m/s` over 171 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.551 | 1.551 | 1.551 | 1.000 | 1.000 | 47.586 | 47.828 | 0.242 | 47.992 | 0.001 | 0 | 41 | 0 | 0 | 9 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2505.5 | 2893.1 | 6.07 | 36.82 | 1.67 | 1.00 | 234.00 | 0.172 | 0.000 | 1.62e-09 | 5.63e-11 | 0 |
| 60–119 | 2472.8 | 3555.9 | 5.10 | 34.98 | 0.17 | 1.00 | 234.00 | 0.032 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–179 | 2519.0 | 5259.8 | 6.12 | 39.43 | 1.62 | 1.12 | 261.30 | 0.002 | 0.000 | 1.46e-09 | 6.30e-11 | 0 |
| 180–239 | 2789.0 | 5415.4 | 7.90 | 50.87 | 6.07 | 3.92 | 883.10 | 3.281 | 3.526 | 1.47e-09 | 5.34e-11 | 12 |
| 240–299 | 3090.6 | 3865.4 | 8.45 | 55.45 | 7.15 | 5.43 | 1206.20 | 7.626 | 23.100 | 1.61e-09 | 7.07e-11 | 60 |
| 300–359 | 3087.1 | 4180.8 | 8.68 | 54.75 | 7.52 | 5.43 | 1206.20 | 11.865 | 27.214 | 7.87e-10 | 1.32e-11 | 60 |
| 360–419 | 1933.4 | 4116.2 | 8.68 | 57.62 | 7.25 | 2.63 | 584.60 | 15.120 | 21.371 | 1.28e-09 | 1.62e-11 | 60 |
| 420–479 | 1952.8 | 3513.1 | 7.98 | 52.83 | 6.03 | 3.22 | 696.40 | 21.998 | 31.938 | 4.80e-10 | 7.89e-12 | 8 |
| 480–539 | 2859.3 | 4124.2 | 8.07 | 52.78 | 6.40 | 5.08 | 1098.00 | 30.487 | 35.690 | 4.24e-10 | 6.27e-12 | 0 |
| 540–599 | 2766.4 | 3344.1 | 8.23 | 54.25 | 6.22 | 5.32 | 1148.40 | 61.245 | 57.489 | 5.15e-10 | 5.85e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 23.670 | 27.096 | 46.347 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
