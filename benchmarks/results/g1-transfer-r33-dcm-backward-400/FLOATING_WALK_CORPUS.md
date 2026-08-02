# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `dcm-backward-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 400 | 2.000 s | 10.810 cm | 2.592 cm | 29.558 cm | 43.447 cm | 18.821° | 8.000 rad/s | 24749.2 µs |

Nominal hard residual maxima: dynamics `1.270e-09`, contact acceleration `5.554e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 10.810 cm |
| CoM RMS / p95 | 7.208 / 17.578 cm |
| stance foot RMS | 2.592 cm |
| swing foot RMS | 29.558 cm |
| hand RMS | 43.447 cm |
| maximum root rotation | 18.821° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.270e-09 |
| contact acceleration residual | 5.554e-11 |
| raw max dynamics residual, including rejected ticks | 1.270e-09 |
| raw max contact residual, including rejected ticks | 5.554e-11 |
| active normal force range | 0.000–325.398 N |
| centroidal momentum-rate residual RMS / max | 17.040 / 62.577 N·m |
| point-task acceleration RMS max | 142.565 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `12.404` / `29.848 cm`.
- Virtual ZMP clipped on `41.25%` of ticks; clip-distance RMS / max `21.408` / `95.522 cm`.
- Measured-height natural frequency min / p50 / max: `3.665` / `3.734` / `4.054 rad/s`.
- CoM command acceleration p95 / max: `20.719` / `24.805 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 400 | 2.0 s | 2520.2 µs | 4434.3 µs | 24749.2 µs | 44515.2 µs | 94 | 306 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3175.0 | 3738.0 | 524.0 | 4189.3 | 42790.2 | 44342.7 | 8514.2 | 400 | 6 | 5 | 315.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 94 | 2440.7 | 2534.5 | 2636.1 | 2969.5 |
| solved_with_slack | 306 | 2938.7 | 4545.8 | 31369.7 | 44515.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.22 | 10.0 | 12.0 | 20 | 4.76 | 12.0 | 17 | 0.1191 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 29.01/8.0/1349.2/2681 | 6452.77/1872.0/299520.2/595182 | 0.80/3.0/3 | 0.43/2.0/2 | 3.57/18.0/19 | 0.9815 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 94 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 306 | 7.90/12.9/20 | 6.22/12.0/17 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.72/5.0/7 | 6.83/20.0/28 | 1.10/5.0/7 | 192 |
| viability | 1.61/6.0/8 | 6.45/30.0/40 | 1.12/6.0/8 | 219 |
| intent | 1.77/5.0/6 | 9.65/30.0/30 | 1.54/5.0/6 | 306 |
| preference | 1.10/3.0/12 | 11.15/29.0/96 | 0.66/3.0/12 | 231 |
| style | 1.01/2.0/2 | 9.81/16.0/24 | 0.33/1.0/2 | 128 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 201 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 400 | 0 |
| left_wrist_roll_rubber_hand | 400 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 400 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.270 | 1.270 | 1.270 | 1.000 | 1.000 | 47.020 | 47.215 | 0.195 | 48.180 | 0.001 | 0 | 48 | 0 | 0 | 22 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–39 | 2436.7 | 2829.7 | 5.00 | 33.27 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| 40–79 | 2439.9 | 2515.8 | 5.00 | 34.17 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 7.81e-10 | 4.71e-11 | 0 |
| 80–119 | 2484.7 | 3006.7 | 6.20 | 40.77 | 2.75 | 1.00 | 234.00 | 0.129 | 0.000 | 1.23e-09 | 4.25e-11 | 0 |
| 120–159 | 4336.9 | 4841.9 | 7.47 | 44.55 | 5.95 | 5.55 | 1298.70 | 1.013 | 0.000 | 6.45e-10 | 3.77e-11 | 0 |
| 160–199 | 2491.4 | 5204.9 | 7.50 | 46.55 | 5.00 | 2.40 | 561.30 | 2.490 | 0.000 | 1.17e-09 | 4.96e-11 | 0 |
| 200–239 | 2990.6 | 3609.9 | 7.70 | 45.17 | 5.33 | 5.90 | 1309.80 | 6.548 | 0.617 | 8.43e-10 | 5.03e-12 | 0 |
| 240–279 | 1854.6 | 3332.0 | 7.92 | 46.33 | 6.55 | 2.75 | 610.50 | 8.853 | 12.972 | 8.48e-10 | 1.02e-11 | 0 |
| 280–319 | 3162.0 | 4501.6 | 8.82 | 51.90 | 7.38 | 6.95 | 1542.90 | 10.399 | 14.067 | 5.11e-10 | 1.95e-11 | 0 |
| 320–359 | 3241.1 | 4002.9 | 8.15 | 47.15 | 7.08 | 8.00 | 1776.00 | 17.739 | 19.649 | 4.74e-10 | 1.67e-11 | 0 |
| 360–399 | 1837.6 | 42829.1 | 8.40 | 49.10 | 7.55 | 255.53 | 56726.55 | 24.844 | 38.637 | 1.27e-09 | 3.13e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 400 | 10.810 | 14.985 | 43.447 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
