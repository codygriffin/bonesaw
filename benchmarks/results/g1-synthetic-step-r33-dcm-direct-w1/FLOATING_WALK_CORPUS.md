# Bonesaw floating G1 synthetic-step acceptance

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: deterministic, predeclared G1-sized left step; no mocap or learned policy contributes contact labels or target motion.
- Contact schedule: liftoff tick `53`, touchdown tick `93`, step `0.010 m`, clearance `0.010 m`.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.010 m` forward per `0.800 s` source cycle.
- Applied mean forward speed: `0.012 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `constant synthetic phase rate` in `0.8 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-centroid-preview` reference at `viability` priority with weight `1.000` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `40` ticks (`0.200 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
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
| `touchdown_admission_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.800 s | 9.714 cm | 14.717 cm | 3.502 cm | 34.447 cm | 8.504° | 8.000 rad/s | 32410.4 µs |

Nominal hard residual maxima: dynamics `1.403e-09`, contact acceleration `5.317e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 9.714 cm |
| CoM RMS / p95 | 9.477 / 17.210 cm |
| stance foot RMS | 14.717 cm |
| swing foot RMS | 3.502 cm |
| hand RMS | 34.447 cm |
| maximum root rotation | 8.504° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.403e-09 |
| contact acceleration residual | 5.317e-11 |
| raw max dynamics residual, including rejected ticks | 1.403e-09 |
| raw max contact residual, including rejected ticks | 5.317e-11 |
| active normal force range | 0.000–317.352 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 65.403 m/s² |
| frame-angular acceleration RMS max | 100.087 rad/s² |
| longest pre-contact / touchdown transition | 107 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 67 / 67 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `14.601` / `26.654 cm`.
- Virtual ZMP clipped on `76.25%` of ticks; clip-distance RMS / max `21.119` / `43.516 cm`.
- Measured-height natural frequency min / p50 / max: `3.750` / `3.774` / `3.839 rad/s`.
- CoM command acceleration p95 / max: `7.070` / `8.438 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 2479.5 µs | 3994.3 µs | 32410.4 µs | 42088.7 µs | 12 | 41 | 107 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3318.0 | 4761.0 | 614.3 | 3788.5 | 40638.9 | 41943.7 | 30342.6 | 160 | 5 | 3 | 301.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 12 | 2415.8 | 2503.6 | 2507.9 | 2509.0 |
| solved_with_slack | 41 | 2551.9 | 3261.3 | 3692.6 | 3906.9 |
| precontact_transition | 107 | 2204.3 | 4150.4 | 32913.7 | 42088.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 9.45 | 15.0 | 18.0 | 23 | 7.36 | 16.6 | 20 | 0.0409 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 49.33/8.0/1856.1/2456 | 10955.51/1776.0/412063.1/545232 | 0.69/3.0/3 | 0.42/2.0/2 | 3.41/16.0/18 | 0.9903 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 12 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 41 | 12.22/22.2/23 | 9.76/19.6/20 |
| precontact_transition | 107 | 8.89/13.9/15 | 7.27/12.9/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.64/4.4/5 | 6.44/17.6/20 | 0.93/4.4/5 | 66 |
| viability | 3.34/11.2/14 | 16.62/50.6/64 | 3.14/11.2/13 | 145 |
| intent | 1.88/6.0/6 | 3.89/12.0/12 | 1.77/5.4/6 | 146 |
| preference | 1.56/5.4/7 | 14.45/48.8/63 | 1.18/5.4/7 | 121 |
| style | 1.02/2.0/2 | 8.55/20.0/22 | 0.34/1.0/2 | 54 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 107 | 0 | 53 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 160 | 0 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `107` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.9293 m/s`, p95 `2.9077 m/s`, max `3.1520 m/s` over 107 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.531 | 0.531 | 0.531 | 1.000 | 1.000 | 42.461 | 42.703 | 0.242 | 42.703 | 0.001 | 0 | 61 | 0 | 0 | 6 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2470.0 | 3187.4 | 7.19 | 37.38 | 2.62 | 1.00 | 234.00 | 0.004 | 0.000 | 1.40e-09 | 3.77e-11 | 0 |
| 16–31 | 2490.0 | 3321.0 | 13.50 | 58.88 | 11.12 | 1.00 | 234.00 | 0.231 | 0.000 | 7.92e-10 | 4.84e-11 | 0 |
| 32–47 | 2565.0 | 2960.9 | 11.62 | 56.62 | 9.62 | 1.00 | 234.00 | 1.218 | 0.000 | 9.31e-10 | 5.32e-11 | 0 |
| 48–63 | 2445.4 | 3902.2 | 8.81 | 45.81 | 6.06 | 2.75 | 614.25 | 2.983 | 0.183 | 8.12e-10 | 2.80e-11 | 11 |
| 64–79 | 3763.6 | 4153.9 | 10.00 | 56.06 | 8.62 | 8.00 | 1776.00 | 4.774 | 1.604 | 7.90e-12 | 2.64e-13 | 16 |
| 80–95 | 3187.9 | 3755.4 | 9.25 | 50.50 | 7.50 | 8.00 | 1776.00 | 7.916 | 4.372 | 7.32e-13 | 8.79e-14 | 16 |
| 96–111 | 1844.2 | 3267.1 | 8.62 | 47.50 | 6.31 | 3.62 | 804.75 | 11.120 | 8.684 | 4.64e-10 | 4.42e-12 | 16 |
| 112–127 | 1960.9 | 30731.5 | 10.12 | 57.50 | 9.00 | 197.94 | 43942.12 | 14.111 | 12.844 | 2.79e-10 | 6.74e-12 | 16 |
| 128–143 | 1826.4 | 2156.5 | 7.75 | 44.75 | 6.25 | 1.00 | 222.00 | 15.616 | 22.959 | 1.40e-09 | 1.18e-11 | 16 |
| 144–159 | 1775.0 | 40578.6 | 7.62 | 44.56 | 6.50 | 269.00 | 59718.00 | 16.767 | 33.486 | 4.84e-10 | 4.25e-12 | 16 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 9.714 | 13.822 | 34.447 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
