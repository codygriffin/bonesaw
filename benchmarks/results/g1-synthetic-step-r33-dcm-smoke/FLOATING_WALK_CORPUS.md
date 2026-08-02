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
- CoM task: `rooted` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 160 | 0.800 s | 9.422 cm | 7.343 cm | 1.057 cm | 28.572 cm | 10.842° | 8.000 rad/s | 104723.0 µs |

Nominal hard residual maxima: dynamics `1.341e-09`, contact acceleration `3.026e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 9.422 cm |
| CoM RMS / p95 | 8.013 / 19.889 cm |
| stance foot RMS | 7.343 cm |
| swing foot RMS | 1.057 cm |
| hand RMS | 28.572 cm |
| maximum root rotation | 10.842° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.341e-09 |
| contact acceleration residual | 3.026e-11 |
| raw max dynamics residual, including rejected ticks | 1.341e-09 |
| raw max contact residual, including rejected ticks | 3.026e-11 |
| active normal force range | 0.000–353.097 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 59.549 m/s² |
| frame-angular acceleration RMS max | 71.547 rad/s² |
| longest pre-contact / touchdown transition | 107 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 67 / 67 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `20.576` / `49.483 cm`.
- Virtual ZMP clipped on `66.88%` of ticks; clip-distance RMS / max `45.620` / `117.136 cm`.
- Measured-height natural frequency min / p50 / max: `3.769` / `3.770` / `3.925 rad/s`.
- CoM command acceleration p95 / max: `18.917` / `25.820 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 2424.1 µs | 73428.4 µs | 104723.0 µs | 132999.8 µs | 53 | 0 | 107 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9165.4 | 23186.6 | 636.5 | 4409.1 | 129057.9 | 132605.6 | 63476.7 | 160 | 15 | 12 | 109.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 53 | 2406.1 | 2494.8 | 2514.6 | 2520.1 |
| precontact_transition | 107 | 3036.7 | 87938.1 | 107853.5 | 132999.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.28 | 12.0 | 13.8 | 19 | 4.24 | 12.4 | 18 | 0.4132 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 366.66/4468.8/5459.6/5483 | 81401.66/992073.6/1212022.3/1217226 | 0.94/6.3/12 | 0.63/5.3/11 | 5.21/45.1/91 | 0.9814 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 53 | 5.00/5.0/5 | 0.00/0.0/0 |
| precontact_transition | 107 | 8.40/14.9/19 | 6.34/12.9/18 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.59/4.0/4 | 6.02/16.0/16 | 0.92/4.0/4 | 68 |
| viability | 1.73/6.0/8 | 9.64/36.0/48 | 1.26/6.0/8 | 85 |
| intent | 1.44/5.6/13 | 3.05/14.6/37 | 0.98/5.6/13 | 88 |
| preference | 1.44/5.4/7 | 13.27/47.1/58 | 0.86/5.0/6 | 81 |
| style | 1.07/3.4/4 | 8.51/20.0/22 | 0.21/3.4/4 | 24 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 107 | 0 | 53 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 160 | 0 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `107` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.5601 m/s`, p95 `2.9485 m/s`, max `4.2136 m/s` over 107 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.467 | 1.466 | 1.466 | 1.000 | 1.000 | 42.355 | 42.625 | 0.270 | 42.625 | 0.001 | 0 | 68 | 0 | 0 | 16 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2404.7 | 2495.5 | 5.00 | 30.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 16–31 | 2405.8 | 2484.1 | 5.00 | 30.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 32–47 | 2417.1 | 2507.2 | 5.00 | 30.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 48–63 | 1563.7 | 2512.3 | 5.00 | 27.94 | 0.00 | 1.00 | 225.75 | 0.000 | 0.176 | 1.21e-09 | 3.03e-11 | 11 |
| 64–79 | 1647.5 | 3745.4 | 6.38 | 33.94 | 2.75 | 2.75 | 610.50 | 0.038 | 0.648 | 3.76e-10 | 7.29e-12 | 16 |
| 80–95 | 3701.8 | 4436.0 | 9.25 | 49.19 | 7.56 | 8.00 | 1776.00 | 0.986 | 1.315 | 2.73e-12 | 9.31e-14 | 16 |
| 96–111 | 3304.8 | 4388.6 | 8.69 | 48.25 | 7.44 | 8.00 | 1776.00 | 3.472 | 4.608 | 1.02e-12 | 4.76e-14 | 16 |
| 112–127 | 3020.6 | 3570.3 | 8.94 | 49.94 | 6.81 | 5.81 | 1290.38 | 7.781 | 10.011 | 3.38e-10 | 4.95e-12 | 16 |
| 128–143 | 7484.6 | 129281.0 | 10.00 | 53.75 | 8.94 | 1847.06 | 410047.88 | 14.872 | 14.397 | 3.03e-10 | 7.87e-12 | 16 |
| 144–159 | 2207.1 | 91763.4 | 9.50 | 51.88 | 8.88 | 1790.94 | 397588.12 | 24.351 | 11.930 | 1.34e-09 | 2.32e-11 | 16 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 9.422 | 6.879 | 28.572 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
