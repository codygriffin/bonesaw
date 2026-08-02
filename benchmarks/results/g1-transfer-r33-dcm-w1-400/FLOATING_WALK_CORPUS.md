# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-centroid-preview` reference at `viability` priority with weight `1.000` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 400 | 2.000 s | 12.714 cm | 5.120 cm | 22.435 cm | 33.104 cm | 27.778° | 8.000 rad/s | 5233.7 µs |

Nominal hard residual maxima: dynamics `1.878e-09`, contact acceleration `5.698e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 12.714 cm |
| CoM RMS / p95 | 8.688 / 21.746 cm |
| stance foot RMS | 5.120 cm |
| swing foot RMS | 22.435 cm |
| hand RMS | 33.104 cm |
| maximum root rotation | 27.778° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.878e-09 |
| contact acceleration residual | 5.698e-11 |
| raw max dynamics residual, including rejected ticks | 1.878e-09 |
| raw max contact residual, including rejected ticks | 5.698e-11 |
| active normal force range | 0.000–302.749 N |
| centroidal momentum-rate residual RMS / max | 21.702 / 97.955 N·m |
| point-task acceleration RMS max | 136.915 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `14.918` / `39.531 cm`.
- Virtual ZMP clipped on `45.75%` of ticks; clip-distance RMS / max `26.758` / `108.568 cm`.
- Measured-height natural frequency min / p50 / max: `3.699` / `3.776` / `4.394 rad/s`.
- CoM command acceleration p95 / max: `38.559` / `51.052 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 400 | 2.0 s | 2423.6 µs | 3544.5 µs | 5233.7 µs | 50072.4 µs | 158 | 242 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2740.4 | 3386.1 | 274.0 | 3117.7 | 48215.4 | 49886.7 | 13252.4 | 400 | 6 | 2 | 364.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 158 | 2414.6 | 2516.0 | 2565.0 | 2713.5 |
| solved_with_slack | 242 | 2458.9 | 3802.7 | 16366.0 | 50072.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.09 | 12.0 | 14.0 | 17 | 4.06 | 13.0 | 14 | 0.1407 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 20.45/8.0/15.5/2944 | 4546.98/1776.0/3452.1/653568 | 0.46/3.0/3 | 0.26/2.0/2 | 2.06/16.0/18 | 0.9876 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 158 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 242 | 8.46/15.0/17 | 6.71/13.6/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.53/5.0/5 | 5.75/20.0/24 | 0.82/5.0/5 | 131 |
| viability | 2.06/9.0/12 | 7.62/35.0/51 | 1.55/9.0/12 | 211 |
| intent | 1.40/4.0/5 | 7.05/24.0/27 | 1.00/4.0/5 | 242 |
| preference | 1.07/2.0/4 | 11.41/25.1/45 | 0.46/2.0/4 | 162 |
| style | 1.03/2.0/3 | 9.68/19.0/24 | 0.23/2.0/3 | 82 |

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
| 1.096 | 1.096 | 1.096 | 0.999 | 0.999 | 46.906 | 47.168 | 0.262 | 48.012 | 0.001 | 0 | 46 | 0 | 0 | 11 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–39 | 2415.3 | 2553.5 | 5.00 | 32.45 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.88e-09 | 5.29e-11 | 0 |
| 40–79 | 2442.5 | 2659.5 | 5.00 | 32.33 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 9.55e-10 | 3.23e-11 | 0 |
| 80–119 | 2426.3 | 2525.3 | 5.00 | 31.77 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.44e-09 | 5.70e-11 | 0 |
| 120–159 | 2397.9 | 2633.2 | 5.33 | 31.95 | 0.40 | 1.00 | 234.00 | 0.001 | 0.000 | 1.18e-09 | 4.22e-11 | 0 |
| 160–199 | 2502.0 | 3327.9 | 10.30 | 44.85 | 7.70 | 1.18 | 272.55 | 1.601 | 0.000 | 1.32e-09 | 4.81e-11 | 0 |
| 200–239 | 1662.7 | 4056.0 | 7.00 | 42.48 | 4.12 | 3.80 | 843.60 | 3.647 | 0.171 | 7.48e-10 | 1.28e-11 | 0 |
| 240–279 | 2930.4 | 4028.5 | 8.28 | 48.17 | 6.62 | 5.20 | 1154.40 | 6.068 | 2.402 | 6.32e-10 | 1.34e-11 | 0 |
| 280–319 | 1870.5 | 48257.3 | 8.88 | 54.60 | 7.42 | 181.55 | 40304.10 | 9.422 | 6.693 | 9.70e-10 | 2.13e-11 | 0 |
| 320–359 | 1875.5 | 3052.5 | 8.18 | 49.73 | 7.20 | 1.35 | 299.70 | 20.616 | 24.774 | 9.97e-10 | 1.93e-11 | 0 |
| 360–399 | 3118.7 | 5121.1 | 8.00 | 46.75 | 7.15 | 7.47 | 1659.45 | 32.402 | 28.222 | 1.40e-09 | 1.71e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 400 | 12.714 | 12.087 | 33.104 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
