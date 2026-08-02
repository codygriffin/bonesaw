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
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.250` over 11 waist/arm coordinates.
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
| 468 | 2.340 s | 15.454 cm | 9.727 cm | 24.200 cm | 23.371 cm | 56.740° | 8.000 rad/s | 78054.8 µs |

Nominal hard residual maxima: dynamics `5.365e-09`, contact acceleration `1.460e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 63.086 cm |
| authored reference vs measured CoM RMS / p95 | 58.776 / 138.746 cm |
| stance foot RMS | 41.590 cm |
| swing foot RMS | 25.285 cm |
| hand RMS | 74.509 cm |
| maximum root rotation | 160.836° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.287e-09 |
| contact acceleration residual | 2.718e-10 |
| raw max dynamics residual, including rejected ticks | 6.287e-09 |
| raw max contact residual, including rejected ticks | 2.718e-10 |
| active normal force range | 0.000–684.225 N |
| centroidal momentum-rate residual RMS / max | 57.085 / 342.561 N·m |
| point-task acceleration RMS max | 187.650 m/s² |
| frame-angular acceleration RMS max | 387.830 rad/s² |
| longest pre-contact / touchdown transition | 240 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `31.935` / `92.252 cm`.
- Virtual ZMP clipped on `59.67%` of ticks; clip-distance RMS / max `39.552` / `149.023 cm`.
- Measured-height natural frequency min / p50 / max: `3.717` / `3.769` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.596 m`; height-floor ticks: `136`.
- CoM command acceleration p95 / max: `22.545` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2456.8 µs | 73028.2 µs | 160807.3 µs | 207978.8 µs | 199 | 29 | 240 | 0 | 115 | 17 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10478.3 | 27648.7 | 469.8 | 14219.9 | 206681.9 | 207849.1 | 90409.9 | 600 | 96 | 56 | 95.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 2482.0 | 2559.0 | 2591.8 | 2610.0 |
| solved_with_slack | 29 | 1945.2 | 3267.2 | 3608.5 | 3718.9 |
| normal_contact_contingency | 115 | 3782.0 | 37727.2 | 41826.6 | 119500.4 |
| contact_release_contingency | 17 | 150217.1 | 206246.7 | 207632.4 | 207978.8 |
| precontact_transition | 240 | 1982.0 | 73028.2 | 81102.0 | 86143.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.22 | 11.0 | 13.0 | 16 | 4.15 | 11.0 | 15 | 0.1567 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 229.25/2158.9/4802.6/5365 | 50635.61/467009.1/1066188.3/1191030 | 1.09/14.0/15 | 0.90/13.0/14 | 7.44/111.0/128 | 0.4763 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 29 | 6.31/8.4/9 | 2.38/6.9/8 |
| normal_contact_contingency | 115 | 8.63/12.0/14 | 6.80/11.0/13 |
| contact_release_contingency | 17 | 7.65/9.0/9 | 5.53/7.0/7 |
| precontact_transition | 240 | 8.48/13.6/16 | 6.43/12.0/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.94/6.0/7 | 8.73/26.0/36 | 1.44/6.0/7 | 300 |
| viability | 1.67/6.0/10 | 9.17/37.0/66 | 1.27/6.0/10 | 365 |
| intent | 1.47/5.0/8 | 10.50/37.0/61 | 1.06/5.0/7 | 377 |
| preference | 1.01/1.0/3 | 4.35/6.0/18 | 0.02/1.0/3 | 7 |
| style | 1.13/4.0/7 | 9.29/32.0/51 | 0.36/4.0/6 | 155 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 468 | 132 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `135` ticks.
Precontact sole-center tangential speed: p50 `2.2414 m/s`, p95 `5.0142 m/s`, max `6.0224 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.287 | 6.285 | 6.285 | 1.000 | 1.000 | 47.293 | 47.527 | 0.234 | 48.508 | 0.001 | 0 | 59 | 0 | 0 | 106 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2501.7 | 2599.5 | 5.00 | 28.05 | 0.00 | 1.00 | 234.00 | 0.270 | 0.000 | 1.31e-09 | 5.73e-11 | 0 |
| 60–119 | 2473.9 | 2538.3 | 5.00 | 28.70 | 0.00 | 1.00 | 234.00 | 1.120 | 0.000 | 1.84e-09 | 5.18e-11 | 0 |
| 120–179 | 2478.3 | 2551.8 | 5.00 | 28.03 | 0.00 | 1.00 | 234.00 | 3.189 | 0.000 | 1.47e-09 | 3.94e-11 | 0 |
| 180–239 | 2081.4 | 3486.2 | 6.37 | 36.72 | 2.38 | 1.55 | 347.90 | 7.937 | 0.654 | 8.66e-10 | 4.32e-11 | 12 |
| 240–299 | 1905.7 | 2454.6 | 7.62 | 44.35 | 5.00 | 1.00 | 222.00 | 6.495 | 18.408 | 1.37e-09 | 1.71e-11 | 60 |
| 300–359 | 1962.0 | 3018.5 | 8.38 | 51.17 | 6.18 | 1.00 | 222.00 | 5.043 | 23.385 | 1.04e-09 | 1.78e-11 | 60 |
| 360–419 | 1996.5 | 36677.3 | 8.97 | 52.02 | 7.43 | 88.07 | 19550.80 | 13.112 | 19.386 | 1.05e-09 | 1.16e-11 | 60 |
| 420–479 | 13415.6 | 99819.6 | 9.22 | 54.07 | 7.57 | 1766.50 | 392153.40 | 53.269 | 29.594 | 5.37e-09 | 1.46e-10 | 60 |
| 480–539 | 12748.9 | 206701.4 | 8.15 | 46.88 | 6.28 | 425.22 | 91833.20 | 127.476 | 55.560 | 6.29e-09 | 2.72e-10 | 60 |
| 540–599 | 3374.2 | 5819.6 | 8.55 | 50.45 | 6.62 | 6.13 | 1324.80 | 142.818 | 92.007 | 1.72e-09 | 2.07e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 63.086 | 37.000 | 74.509 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
