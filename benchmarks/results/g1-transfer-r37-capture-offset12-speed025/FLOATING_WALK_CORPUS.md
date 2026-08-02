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
- Capture landing retarget: `enabled`; authored offset ≤ `0.120 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.250 m/s`.
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
| 427 | 2.135 s | 9.328 cm | 0.905 cm | 28.903 cm | 40.565 cm | 16.351° | 8.000 rad/s | 9463.8 µs |

Nominal hard residual maxima: dynamics `1.309e-09`, contact acceleration `5.938e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 85.712 cm |
| authored reference vs measured CoM RMS / p95 | 75.808 / 215.672 cm |
| stance foot RMS | 52.811 cm |
| swing foot RMS | 81.021 cm |
| hand RMS | 101.585 cm |
| maximum root rotation | 134.992° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.162e-09 |
| contact acceleration residual | 2.011e-10 |
| raw max dynamics residual, including rejected ticks | 6.162e-09 |
| raw max contact residual, including rejected ticks | 2.011e-10 |
| active normal force range | 0.000–875.014 N |
| centroidal momentum-rate residual RMS / max | 43.545 / 386.139 N·m |
| point-task acceleration RMS max | 102.510 m/s² |
| frame-angular acceleration RMS max | 188.187 rad/s² |
| longest pre-contact / touchdown transition | 199 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `77.339` / `187.675 cm`.
- Virtual ZMP clipped on `58.50%` of ticks; clip-distance RMS / max `136.012` / `321.400 cm`.
- Measured-height natural frequency min / p50 / max: `3.685` / `3.756` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.908 m`; height-floor ticks: `81`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-222.891` / `-216.962 cm`; inside on `48.33%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`.
- Maximum applied offset / root reach: `0.1200 / 0.8885 m`.
- Authored-offset / reach / slew limited ticks: `372 / 0 / 238`.
- Authored geometry outside configured reach: `0` target-ticks.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `453` ticks; maximum active coordinates `8`; mean target/applied scale `0.663` / `0.747`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2636.1 µs | 5358.4 µs | 144895.1 µs | 211852.6 µs | 129 | 99 | 199 | 0 | 163 | 10 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5640.6 | 19538.0 | 331.7 | 4056.2 | 194051.6 | 210072.5 | 99577.3 | 600 | 33 | 13 | 177.3 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2572.1 | 3482.9 | 3962.6 | 3998.8 |
| solved_with_slack | 99 | 2649.5 | 3065.6 | 4099.0 | 4537.4 |
| normal_contact_contingency | 163 | 3034.7 | 6412.1 | 38705.2 | 182134.7 |
| contact_release_contingency | 10 | 145855.1 | 183732.6 | 206228.6 | 211852.6 |
| precontact_transition | 199 | 2707.2 | 4190.3 | 11933.4 | 12058.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.58 | 11.0 | 13.0 | 15 | 5.25 | 12.0 | 14 | 0.0105 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 17.50/8.0/8.0/4113 | 3791.24/1776.0/1776.0/888408 | 0.73/10.0/12 | 0.51/9.0/11 | 4.10/72.1/92 | 0.3480 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| normal_contact_contingency | 163 | 8.98/13.4/15 | 7.69/12.0/13 |
| contact_release_contingency | 10 | 7.50/8.9/9 | 5.90/7.0/7 |
| precontact_transition | 199 | 8.52/13.0/15 | 7.34/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.92/6.0/8 | 7.82/25.0/32 | 1.36/6.0/8 | 293 |
| viability | 1.75/6.0/9 | 11.34/42.0/56 | 1.35/6.0/9 | 365 |
| intent | 1.55/4.0/6 | 8.20/23.0/30 | 1.32/4.0/6 | 468 |
| preference | 1.35/5.0/6 | 11.72/51.0/61 | 0.90/5.0/6 | 357 |
| style | 1.02/2.0/2 | 8.72/15.0/21 | 0.32/1.0/2 | 187 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 427 | 173 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `176` ticks.
Precontact sole-center tangential speed: p50 `2.2132 m/s`, p95 `4.2114 m/s`, max `5.2446 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.385 | 3.376 | 3.376 | 0.997 | 0.997 | 47.820 | 48.031 | 0.211 | 48.707 | 0.001 | 0 | 38 | 0 | 0 | 471 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2541.4 | 2677.8 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2659.4 | 3989.8 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2643.4 | 3892.3 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2415.8 | 4273.5 | 7.13 | 46.18 | 4.57 | 1.00 | 225.80 | 6.405 | 0.736 | 7.54e-10 | 4.18e-11 | 12 |
| 240–299 | 2601.4 | 3233.6 | 8.20 | 50.93 | 6.88 | 1.00 | 222.00 | 6.340 | 20.836 | 1.03e-09 | 1.36e-11 | 60 |
| 300–359 | 2684.9 | 4670.7 | 8.55 | 51.30 | 7.05 | 2.87 | 636.40 | 6.408 | 24.729 | 9.29e-10 | 8.22e-12 | 60 |
| 360–419 | 3143.4 | 4446.9 | 8.58 | 51.82 | 7.82 | 4.03 | 895.40 | 19.898 | 19.376 | 6.55e-10 | 1.38e-11 | 60 |
| 420–479 | 3187.0 | 98609.4 | 8.62 | 52.93 | 7.37 | 116.18 | 25099.80 | 53.122 | 38.867 | 1.06e-09 | 5.94e-11 | 60 |
| 480–539 | 2338.6 | 3986.4 | 9.33 | 59.57 | 7.80 | 2.05 | 442.80 | 131.090 | 67.070 | 6.18e-10 | 4.39e-12 | 60 |
| 540–599 | 4723.7 | 174984.1 | 8.97 | 54.77 | 7.92 | 44.87 | 9688.20 | 230.085 | 181.521 | 6.16e-09 | 2.01e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 85.712 | 63.546 | 101.585 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
