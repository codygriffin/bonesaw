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
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root horizontal task weight `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.010`, activating at `75.0%` of each URDF limit with `2.000 Hz` response.
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
| 472 | 2.360 s | 13.034 cm | 6.934 cm | 26.631 cm | 35.866 cm | 60.928° | 8.000 rad/s | 45695.4 µs |

Nominal hard residual maxima: dynamics `3.206e-09`, contact acceleration `1.697e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 70.314 cm |
| authored reference vs measured CoM RMS / p95 | 63.866 / 183.395 cm |
| stance foot RMS | 40.401 cm |
| swing foot RMS | 56.071 cm |
| hand RMS | 91.229 cm |
| maximum root rotation | 153.466° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.201e-09 |
| contact acceleration residual | 2.483e-10 |
| raw max dynamics residual, including rejected ticks | 7.201e-09 |
| raw max contact residual, including rejected ticks | 2.483e-10 |
| active normal force range | 0.000–750.181 N |
| centroidal momentum-rate residual RMS / max | 71.210 / 480.545 N·m |
| point-task acceleration RMS max | 184.053 m/s² |
| frame-angular acceleration RMS max | 290.724 rad/s² |
| longest pre-contact / touchdown transition | 244 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `26.507` / `54.341 cm`.
- Virtual ZMP clipped on `64.83%` of ticks; clip-distance RMS / max `55.905` / `162.906 cm`.
- Measured-height natural frequency min / p50 / max: `3.668` / `3.770` / `7.004 rad/s`.
- Minimum measured CoM height: `-1.303 m`; height-floor ticks: `104`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2627.9 µs | 12547.4 µs | 113974.3 µs | 207498.9 µs | 130 | 98 | 244 | 0 | 113 | 15 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7650.9 | 22340.5 | 481.4 | 5430.2 | 199014.9 | 206650.5 | 91504.7 | 600 | 67 | 28 | 130.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 130 | 2458.0 | 2569.9 | 2608.9 | 2639.0 |
| solved_with_slack | 98 | 2511.3 | 2660.0 | 2751.0 | 2975.7 |
| normal_contact_contingency | 113 | 4024.8 | 63640.3 | 72100.5 | 113703.2 |
| contact_release_contingency | 15 | 107906.4 | 197584.4 | 205516.0 | 207498.9 |
| precontact_transition | 244 | 3177.0 | 6368.2 | 48366.4 | 109889.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.54 | 12.0 | 13.0 | 15 | 5.22 | 12.0 | 15 | 0.1320 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 82.65/8.0/4088.1/6834 | 18078.49/1776.0/883025.3/1517148 | 1.30/11.0/16 | 0.89/10.0/15 | 7.25/84.0/135 | 0.3710 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 130 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 98 | 6.59/9.0/9 | 3.39/6.0/6 |
| normal_contact_contingency | 113 | 9.00/13.0/14 | 8.03/13.0/13 |
| contact_release_contingency | 15 | 7.80/9.0/9 | 6.33/8.0/8 |
| precontact_transition | 244 | 8.58/13.0/15 | 7.37/11.6/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.89/5.0/8 | 8.16/25.0/40 | 1.35/5.0/8 | 309 |
| viability | 1.74/6.0/7 | 10.96/42.0/49 | 1.34/6.0/7 | 364 |
| intent | 1.62/5.0/8 | 8.67/26.0/41 | 1.38/5.0/8 | 466 |
| preference | 1.23/5.0/6 | 10.10/47.0/57 | 0.76/5.0/6 | 331 |
| style | 1.06/2.0/5 | 8.97/20.0/47 | 0.39/2.0/4 | 214 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 472 | 128 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `131` ticks.
Precontact sole-center tangential speed: p50 `2.3246 m/s`, p95 `6.0209 m/s`, max `11.6799 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.591 | 4.590 | 4.590 | 1.000 | 1.000 | 48.445 | 48.676 | 0.230 | 48.730 | 0.001 | 0 | 58 | 0 | 0 | 64 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2445.5 | 2565.0 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2489.4 | 2635.9 | 5.30 | 35.92 | 0.63 | 1.00 | 234.00 | 0.159 | 0.000 | 9.54e-10 | 3.76e-11 | 0 |
| 120–179 | 2523.9 | 2712.3 | 5.82 | 37.57 | 1.83 | 1.00 | 234.00 | 1.481 | 0.000 | 9.27e-10 | 4.89e-11 | 0 |
| 180–239 | 1987.4 | 2960.6 | 7.13 | 42.92 | 4.43 | 1.00 | 225.80 | 6.134 | 1.168 | 1.32e-09 | 4.08e-11 | 12 |
| 240–299 | 2504.3 | 3437.4 | 8.45 | 54.40 | 6.87 | 1.00 | 222.00 | 5.557 | 17.556 | 8.31e-10 | 2.61e-11 | 60 |
| 300–359 | 3401.2 | 4232.6 | 8.43 | 49.53 | 7.08 | 6.72 | 1491.10 | 9.265 | 21.804 | 5.83e-10 | 1.98e-11 | 60 |
| 360–419 | 3336.7 | 4627.4 | 8.35 | 46.82 | 7.25 | 8.00 | 1776.00 | 15.971 | 26.925 | 4.22e-12 | 1.63e-13 | 60 |
| 420–479 | 3738.2 | 111453.2 | 9.32 | 58.68 | 8.68 | 792.15 | 173226.40 | 35.046 | 21.757 | 3.21e-09 | 1.70e-10 | 60 |
| 480–539 | 3340.2 | 140111.5 | 9.42 | 59.55 | 8.05 | 6.72 | 1449.20 | 99.568 | 33.243 | 8.81e-10 | 6.67e-12 | 60 |
| 540–599 | 5208.4 | 179977.7 | 8.17 | 49.70 | 7.38 | 7.88 | 1692.40 | 194.645 | 135.038 | 7.20e-09 | 2.48e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 70.314 | 46.178 | 91.229 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
