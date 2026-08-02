# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.010` and `2.000 Hz` response. Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| `no_infeasible_or_failed_ticks` | FAIL |
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

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 331 | 1.655 s | 28.650 cm | 26.007 cm | 92.316 cm | 39.525 cm | 101.698° | 8.000 rad/s | 57500.8 µs |

Nominal hard residual maxima: dynamics `1.846e-09`, contact acceleration `6.421e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 196.556 cm |
| CoM RMS / p95 | 204.125 / 450.407 cm |
| stance foot RMS | 171.635 cm |
| swing foot RMS | 220.165 cm |
| hand RMS | 222.489 cm |
| maximum root rotation | 179.764° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.769e-09 |
| contact acceleration residual | 2.960e-10 |
| raw max dynamics residual, including rejected ticks | 1.380e-08 |
| raw max contact residual, including rejected ticks | 5.649e-10 |
| active normal force range | 0.000–887.879 N |
| centroidal momentum-rate residual RMS / max | 95.943 / 346.338 N·m |
| point-task acceleration RMS max | 147.857 m/s² |
| frame-angular acceleration RMS max | 284.045 rad/s² |
| longest pre-contact / touchdown transition | 103 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3262.5 µs | 101880.2 µs | 190484.4 µs | 214588.7 µs | 0 | 228 | 103 | 0 | 207 | 31 | 0 | 31 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 13763.1 | 36035.1 | 850.6 | 14295.3 | 212125.5 | 214342.4 | 134279.1 | 600 | 149 | 47 | 72.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved_with_slack | 228 | 3253.0 | 5241.3 | 6453.5 | 6946.0 |
| failed | 31 | 13683.5 | 13750.0 | 13763.3 | 13765.6 |
| normal_contact_contingency | 207 | 2993.6 | 14700.1 | 19645.5 | 214588.7 |
| contact_release_contingency | 31 | 162585.5 | 206280.5 | 209701.9 | 210476.4 |
| precontact_transition | 103 | 2659.6 | 56410.5 | 60526.0 | 62717.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.98 | 13.0 | 16.0 | 23 | 7.76 | 16.0 | 21 | -0.0824 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 75.15/8.0/3430.3/3885 | 16674.62/1872.0/761531.0/862470 | 3.07/16.0/22 | 2.46/15.0/21 | 20.41/125.0/176 | 0.1603 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved_with_slack | 228 | 9.32/18.5/23 | 7.72/17.5/21 |
| failed | 31 | 9.48/10.0/10 | 9.48/10.0/10 |
| normal_contact_contingency | 207 | 8.84/13.0/14 | 7.66/12.0/14 |
| contact_release_contingency | 31 | 7.90/10.0/10 | 6.58/9.0/9 |
| precontact_transition | 103 | 8.69/13.0/13 | 7.91/13.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.78/7.0/9 | 12.33/32.0/40 | 2.55/7.0/9 | 517 |
| viability | 2.27/9.0/14 | 12.79/42.0/56 | 2.21/9.0/13 | 591 |
| intent | 1.45/4.0/7 | 7.55/24.0/35 | 1.42/4.0/7 | 587 |
| preference | 1.33/6.0/15 | 12.91/55.0/150 | 1.04/6.0/14 | 441 |
| style | 1.15/4.0/5 | 10.13/28.0/54 | 0.53/3.0/5 | 266 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 331 | 269 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `272` ticks.
Precontact sole-center tangential speed: p50 `4.6137 m/s`, p95 `6.9304 m/s`, max `8.1698 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.258 | 8.253 | 8.253 | 0.999 | 0.999 | 47.727 | 47.957 | 0.230 | 47.957 | 0.001 | 0 | 43 | 0 | 0 | 444 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 4425.0 | 5839.9 | 9.77 | 55.42 | 7.98 | 5.90 | 1380.60 | 0.941 | 0.000 | 1.21e-09 | 3.81e-11 | 0 |
| 60–119 | 3334.8 | 5640.5 | 8.50 | 54.32 | 6.38 | 1.93 | 452.40 | 4.660 | 0.001 | 1.19e-09 | 5.12e-11 | 0 |
| 120–179 | 2776.9 | 6659.0 | 10.00 | 61.82 | 8.95 | 2.75 | 643.50 | 10.524 | 0.513 | 1.40e-09 | 4.70e-11 | 0 |
| 180–239 | 3159.9 | 4632.5 | 8.78 | 54.13 | 7.52 | 5.67 | 1261.80 | 16.744 | 5.570 | 7.31e-10 | 3.70e-11 | 12 |
| 240–299 | 2170.4 | 52395.8 | 8.83 | 57.48 | 8.13 | 208.35 | 46253.70 | 36.867 | 63.232 | 1.85e-09 | 2.02e-11 | 60 |
| 300–359 | 12402.7 | 124984.4 | 9.80 | 64.48 | 8.98 | 498.05 | 110543.90 | 91.518 | 134.229 | 8.77e-09 | 2.74e-10 | 60 |
| 360–419 | 2907.8 | 201678.2 | 8.35 | 51.90 | 7.20 | 6.72 | 1446.30 | 177.398 | 127.273 | 8.46e-09 | 1.51e-10 | 60 |
| 420–479 | 2951.8 | 196268.8 | 8.82 | 51.93 | 7.47 | 7.42 | 1592.40 | 245.149 | 211.177 | 7.22e-09 | 2.96e-10 | 60 |
| 480–539 | 3514.8 | 205800.6 | 8.07 | 49.02 | 6.70 | 7.30 | 1569.60 | 328.595 | 313.124 | 5.02e-09 | 1.47e-10 | 60 |
| 540–599 | 13584.9 | 13761.0 | 8.87 | 56.58 | 8.33 | 7.42 | 1602.00 | 420.299 | 420.243 | 1.38e-08 | 5.65e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 196.556 | 189.075 | 222.489 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
