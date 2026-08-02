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
- CoM task: `support-centroid-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 160 | 0.800 s | 9.097 cm | 16.298 cm | 3.650 cm | 33.673 cm | 5.488° | 8.000 rad/s | 5064.2 µs |

Nominal hard residual maxima: dynamics `1.647e-09`, contact acceleration `3.768e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 9.097 cm |
| CoM RMS / p95 | 9.308 / 16.448 cm |
| stance foot RMS | 16.298 cm |
| swing foot RMS | 3.650 cm |
| hand RMS | 33.673 cm |
| maximum root rotation | 5.488° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.647e-09 |
| contact acceleration residual | 3.768e-11 |
| raw max dynamics residual, including rejected ticks | 1.647e-09 |
| raw max contact residual, including rejected ticks | 3.768e-11 |
| active normal force range | 0.000–276.954 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 78.573 m/s² |
| frame-angular acceleration RMS max | 106.270 rad/s² |
| longest pre-contact / touchdown transition | 107 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 67 / 67 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `13.948` / `24.426 cm`.
- Virtual ZMP clipped on `76.25%` of ticks; clip-distance RMS / max `20.043` / `36.254 cm`.
- Measured-height natural frequency min / p50 / max: `3.754` / `3.769` / `3.812 rad/s`.
- CoM command acceleration p95 / max: `5.532` / `6.932 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 2446.3 µs | 4092.6 µs | 5064.2 µs | 5588.4 µs | 12 | 41 | 107 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2573.7 | 823.5 | 601.3 | 3729.8 | 5530.7 | 5582.6 | 1732.2 | 160 | 2 | 0 | 388.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 12 | 2441.0 | 2502.9 | 2543.0 | 2553.0 |
| solved_with_slack | 41 | 2588.7 | 3409.2 | 3754.5 | 3800.3 |
| precontact_transition | 107 | 2008.4 | 4656.5 | 5209.4 | 5588.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 9.34 | 16.0 | 18.2 | 20 | 7.31 | 15.4 | 16 | 0.2571 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 2.88/8.0/8.0/8 | 643.61/1776.0/1776.0/1776 | 0.65/3.0/3 | 0.38/2.0/2 | 3.08/17.0/18 | 0.7520 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 12 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 41 | 12.39/20.0/20 | 9.71/16.0/16 |
| precontact_transition | 107 | 8.65/13.0/13 | 7.21/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.70/4.0/5 | 6.67/16.0/20 | 1.01/4.0/5 | 77 |
| viability | 3.23/12.4/13 | 15.57/46.8/49 | 3.03/11.8/13 | 145 |
| intent | 1.90/6.4/8 | 3.99/14.8/18 | 1.79/6.4/7 | 146 |
| preference | 1.48/5.0/6 | 13.48/47.8/53 | 1.12/5.0/6 | 118 |
| style | 1.04/2.0/2 | 8.66/18.4/22 | 0.37/1.4/2 | 57 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 107 | 0 | 53 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 160 | 0 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `107` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `1.0350 m/s`, p95 `2.9057 m/s`, max `3.1597 m/s` over 107 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.412 | 0.411 | 0.411 | 0.998 | 0.998 | 42.387 | 42.652 | 0.266 | 42.652 | 0.001 | 0 | 52 | 0 | 0 | 67 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2453.8 | 3054.1 | 7.31 | 38.69 | 2.62 | 1.00 | 234.00 | 0.002 | 0.000 | 1.40e-09 | 3.77e-11 | 0 |
| 16–31 | 2580.0 | 3644.3 | 13.50 | 58.06 | 11.50 | 1.00 | 234.00 | 0.235 | 0.000 | 1.26e-09 | 2.67e-11 | 0 |
| 32–47 | 2582.8 | 2991.6 | 12.19 | 57.50 | 9.38 | 1.00 | 234.00 | 1.232 | 0.000 | 1.06e-09 | 3.48e-11 | 0 |
| 48–63 | 2467.8 | 3841.4 | 8.19 | 42.88 | 5.19 | 2.75 | 614.25 | 2.995 | 0.204 | 9.74e-10 | 1.77e-11 | 11 |
| 64–79 | 4117.5 | 5534.0 | 9.94 | 54.38 | 8.75 | 8.00 | 1776.00 | 4.943 | 1.862 | 3.69e-12 | 1.42e-13 | 16 |
| 80–95 | 3090.0 | 3840.6 | 8.50 | 46.31 | 6.38 | 8.00 | 1776.00 | 7.832 | 4.306 | 7.44e-12 | 3.98e-13 | 16 |
| 96–111 | 2115.5 | 3504.7 | 8.25 | 45.94 | 6.56 | 4.06 | 901.88 | 11.211 | 7.895 | 5.39e-10 | 3.89e-12 | 16 |
| 112–127 | 1762.4 | 2381.1 | 9.00 | 46.69 | 7.94 | 1.00 | 222.00 | 13.423 | 15.575 | 7.93e-10 | 7.25e-12 | 16 |
| 128–143 | 1766.9 | 2343.6 | 8.88 | 48.56 | 7.62 | 1.00 | 222.00 | 13.723 | 28.833 | 1.65e-09 | 1.69e-11 | 16 |
| 144–159 | 1847.3 | 2255.7 | 7.62 | 44.81 | 7.19 | 1.00 | 222.00 | 15.398 | 34.387 | 7.04e-10 | 1.22e-11 | 16 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 9.097 | 15.300 | 33.673 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
