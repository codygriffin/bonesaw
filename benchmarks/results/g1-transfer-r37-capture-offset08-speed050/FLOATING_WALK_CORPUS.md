# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `dcm-backward-preview` `DCM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `enabled`; authored offset ≤ `0.080 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
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
| 482 | 2.410 s | 13.701 cm | 14.651 cm | 25.114 cm | 41.378 cm | 55.460° | 8.000 rad/s | 11436.2 µs |

Nominal hard residual maxima: dynamics `5.970e-09`, contact acceleration `4.408e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 40.336 cm |
| authored reference vs measured CoM RMS / p95 | 37.172 / 91.388 cm |
| stance foot RMS | 31.038 cm |
| swing foot RMS | 45.598 cm |
| hand RMS | 72.221 cm |
| maximum root rotation | 179.422° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.345e-09 |
| contact acceleration residual | 4.408e-10 |
| raw max dynamics residual, including rejected ticks | 7.345e-09 |
| raw max contact residual, including rejected ticks | 4.408e-10 |
| active normal force range | 0.000–761.281 N |
| centroidal momentum-rate residual RMS / max | 52.361 / 246.030 N·m |
| point-task acceleration RMS max | 120.657 m/s² |
| frame-angular acceleration RMS max | 246.025 rad/s² |
| longest pre-contact / touchdown transition | 254 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `18.292` / `41.112 cm`.
- Virtual ZMP clipped on `58.17%` of ticks; clip-distance RMS / max `37.481` / `102.044 cm`.
- Measured-height natural frequency min / p50 / max: `3.692` / `3.762` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.257 m`; height-floor ticks: `89`.
- CoM command acceleration p95 / max: `24.374` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-69.708` / `-64.341 cm`; inside on `48.33%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`.
- Maximum applied offset / root reach: `0.0800 / 0.8652 m`.
- Authored-offset / reach / slew limited ticks: `372 / 0 / 112`.
- Authored geometry outside configured reach: `0` target-ticks.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `451` ticks; maximum active coordinates `8`; mean target/applied scale `0.665` / `0.745`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2642.9 µs | 15631.6 µs | 221327.8 µs | 281838.4 µs | 129 | 99 | 254 | 0 | 99 | 19 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10253.3 | 37661.3 | 480.0 | 5404.4 | 262933.1 | 279947.9 | 63175.7 | 600 | 64 | 21 | 97.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2596.2 | 3876.7 | 3934.2 | 4046.4 |
| solved_with_slack | 99 | 2634.6 | 3082.6 | 3610.4 | 3660.3 |
| normal_contact_contingency | 99 | 3360.5 | 15956.8 | 29688.4 | 198272.2 |
| contact_release_contingency | 19 | 212725.5 | 253433.1 | 276157.4 | 281838.4 |
| precontact_transition | 254 | 2616.2 | 7252.6 | 14275.8 | 19736.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.58 | 12.0 | 13.0 | 17 | 5.33 | 12.0 | 17 | 0.0694 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 5.39/8.0/8.0/1538 | 1176.33/1776.0/1776.0/332208 | 1.02/15.0/18 | 0.81/14.0/17 | 6.82/125.0/147 | 0.0195 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| normal_contact_contingency | 99 | 9.00/14.1/17 | 8.10/13.1/17 |
| contact_release_contingency | 19 | 7.95/10.0/10 | 6.68/8.8/9 |
| precontact_transition | 254 | 8.61/13.0/17 | 7.47/12.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.86/5.0/7 | 8.05/25.0/35 | 1.31/5.0/7 | 283 |
| viability | 1.75/7.0/9 | 11.19/48.0/72 | 1.35/7.0/9 | 368 |
| intent | 1.59/5.0/8 | 8.24/27.0/37 | 1.36/5.0/8 | 470 |
| preference | 1.33/6.0/8 | 11.31/56.0/86 | 0.91/6.0/7 | 366 |
| style | 1.04/2.0/4 | 9.01/19.0/30 | 0.40/2.0/4 | 226 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 482 | 118 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `121` ticks.
Precontact sole-center tangential speed: p50 `2.2934 m/s`, p95 `7.5028 m/s`, max `8.3545 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.152 | 6.151 | 6.151 | 1.000 | 1.000 | 47.863 | 48.074 | 0.211 | 48.914 | 0.001 | 0 | 38 | 0 | 0 | 42 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2626.5 | 3983.5 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2645.9 | 3891.5 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2615.1 | 3630.3 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2536.0 | 3287.7 | 7.13 | 46.18 | 4.57 | 1.00 | 225.80 | 6.405 | 0.736 | 7.54e-10 | 4.18e-11 | 12 |
| 240–299 | 2571.0 | 3963.5 | 8.30 | 50.87 | 7.03 | 1.00 | 222.00 | 6.440 | 20.801 | 9.62e-10 | 1.00e-11 | 60 |
| 300–359 | 2575.3 | 4764.0 | 8.42 | 49.38 | 6.83 | 2.05 | 455.10 | 7.161 | 24.581 | 8.83e-10 | 1.13e-11 | 60 |
| 360–419 | 2355.6 | 4273.8 | 8.48 | 51.23 | 7.33 | 2.75 | 610.50 | 16.792 | 18.606 | 1.11e-09 | 1.65e-11 | 60 |
| 420–479 | 3634.9 | 14067.3 | 9.25 | 60.80 | 8.70 | 5.55 | 1232.10 | 32.035 | 35.225 | 1.04e-09 | 1.16e-10 | 60 |
| 480–539 | 15619.9 | 263217.2 | 8.90 | 51.80 | 7.95 | 34.02 | 7343.80 | 73.881 | 52.910 | 7.35e-09 | 4.41e-10 | 60 |
| 540–599 | 2666.7 | 5835.1 | 8.80 | 57.12 | 7.87 | 4.50 | 972.00 | 96.786 | 88.887 | 2.08e-09 | 8.47e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 40.336 | 36.503 | 72.221 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
