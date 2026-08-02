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
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/horizontal task weights `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `multi-support` measured-phase policy with immediate engagement and bounded release over `200` ticks.
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
| 448 | 2.240 s | 13.328 cm | 10.668 cm | 27.968 cm | 37.174 cm | 54.978° | 8.000 rad/s | 29491.2 µs |

Nominal hard residual maxima: dynamics `7.488e-09`, contact acceleration `3.035e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 61.046 cm |
| authored reference vs measured CoM RMS / p95 | 55.000 / 136.751 cm |
| stance foot RMS | 36.725 cm |
| swing foot RMS | 43.769 cm |
| hand RMS | 85.831 cm |
| maximum root rotation | 179.345° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.747e-09 |
| contact acceleration residual | 3.035e-10 |
| raw max dynamics residual, including rejected ticks | 7.747e-09 |
| raw max contact residual, including rejected ticks | 3.035e-10 |
| active normal force range | 0.000–849.350 N |
| centroidal momentum-rate residual RMS / max | 54.327 / 222.270 N·m |
| point-task acceleration RMS max | 178.876 m/s² |
| frame-angular acceleration RMS max | 215.973 rad/s² |
| longest pre-contact / touchdown transition | 220 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `21.970` / `45.639 cm`.
- Virtual ZMP clipped on `64.33%` of ticks; clip-distance RMS / max `49.953` / `106.768 cm`.
- Measured-height natural frequency min / p50 / max: `3.677` / `3.770` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.698 m`; height-floor ticks: `124`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `320` ticks; maximum active coordinates `8`; mean phase scale `0.497`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2719.8 µs | 15605.7 µs | 196521.2 µs | 212794.2 µs | 129 | 99 | 220 | 0 | 138 | 14 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8999.4 | 29648.9 | 435.3 | 9084.2 | 211161.2 | 212630.9 | 38743.4 | 600 | 91 | 24 | 111.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2478.5 | 2608.4 | 2675.9 | 2771.3 |
| solved_with_slack | 99 | 2545.7 | 2888.5 | 3455.2 | 4801.6 |
| normal_contact_contingency | 138 | 3578.2 | 16147.2 | 57301.9 | 115576.2 |
| contact_release_contingency | 14 | 196211.8 | 211022.2 | 212439.8 | 212794.2 |
| precontact_transition | 220 | 3266.2 | 9714.9 | 52551.4 | 55581.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.75 | 12.0 | 13.0 | 20 | 5.53 | 13.0 | 17 | 0.0639 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 51.38/8.0/2829.7/4082 | 11269.80/1776.0/611269.2/881712 | 1.91/16.0/16 | 1.47/15.0/15 | 12.21/126.0/135 | 0.1743 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.87/11.2/20 | 3.68/9.2/17 |
| normal_contact_contingency | 138 | 9.43/13.6/15 | 8.61/13.0/14 |
| contact_release_contingency | 14 | 7.71/9.0/9 | 6.50/8.0/8 |
| precontact_transition | 220 | 8.72/13.0/14 | 7.60/12.8/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.00/5.0/7 | 8.23/24.0/30 | 1.50/5.0/7 | 329 |
| viability | 1.73/6.0/8 | 10.71/36.0/56 | 1.36/6.0/8 | 384 |
| intent | 1.53/4.0/7 | 8.11/24.0/36 | 1.30/4.0/7 | 471 |
| preference | 1.39/6.0/15 | 12.52/60.0/135 | 0.94/6.0/15 | 358 |
| style | 1.11/4.0/6 | 9.49/27.0/43 | 0.43/3.0/6 | 210 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 448 | 152 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `155` ticks.
Precontact sole-center tangential speed: p50 `2.5599 m/s`, p95 `7.3975 m/s`, max `9.5349 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.400 | 5.399 | 5.399 | 1.000 | 1.000 | 48.438 | 48.738 | 0.301 | 48.812 | 0.001 | 0 | 61 | 0 | 0 | 75 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2454.0 | 2724.1 | 5.00 | 33.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2519.4 | 2859.7 | 5.33 | 36.08 | 0.62 | 1.00 | 234.00 | 0.157 | 0.000 | 9.54e-10 | 4.41e-11 | 0 |
| 120–179 | 2550.6 | 3972.5 | 5.98 | 39.33 | 2.03 | 1.00 | 234.00 | 1.465 | 0.000 | 1.60e-09 | 5.13e-11 | 0 |
| 180–239 | 2451.2 | 3496.7 | 7.43 | 45.98 | 4.82 | 1.00 | 225.80 | 6.403 | 1.831 | 1.14e-09 | 3.66e-11 | 12 |
| 240–299 | 2619.5 | 5432.8 | 8.20 | 53.07 | 6.53 | 2.67 | 592.00 | 6.575 | 23.330 | 1.40e-09 | 1.63e-11 | 60 |
| 300–359 | 3335.6 | 4580.1 | 8.68 | 54.12 | 7.57 | 5.20 | 1154.40 | 6.069 | 25.214 | 7.89e-10 | 1.39e-11 | 60 |
| 360–419 | 3431.4 | 54088.4 | 8.77 | 51.33 | 7.88 | 115.92 | 25733.50 | 20.737 | 15.943 | 1.71e-10 | 4.81e-12 | 60 |
| 420–479 | 13658.4 | 211185.8 | 9.90 | 63.47 | 9.30 | 254.13 | 55811.10 | 51.760 | 44.008 | 7.49e-09 | 3.03e-10 | 60 |
| 480–539 | 3932.3 | 194653.0 | 9.42 | 58.47 | 8.43 | 123.87 | 26751.20 | 117.965 | 60.685 | 7.75e-09 | 1.42e-10 | 60 |
| 540–599 | 3295.9 | 6591.2 | 8.82 | 55.77 | 8.07 | 8.00 | 1728.00 | 141.839 | 91.137 | 1.33e-10 | 3.26e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 61.046 | 39.195 | 85.831 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
