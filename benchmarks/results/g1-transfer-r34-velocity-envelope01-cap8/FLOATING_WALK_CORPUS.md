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
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each URDF limit with `2.000 Hz` response.
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
| 467 | 2.335 s | 14.159 cm | 9.556 cm | 23.553 cm | 42.235 cm | 75.084° | 8.000 rad/s | 100573.8 µs |

Nominal hard residual maxima: dynamics `1.398e-09`, contact acceleration `7.060e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 61.756 cm |
| authored reference vs measured CoM RMS / p95 | 58.003 / 150.809 cm |
| stance foot RMS | 32.338 cm |
| swing foot RMS | 33.385 cm |
| hand RMS | 90.334 cm |
| maximum root rotation | 179.895° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.589e-09 |
| contact acceleration residual | 3.642e-10 |
| raw max dynamics residual, including rejected ticks | 6.589e-09 |
| raw max contact residual, including rejected ticks | 3.642e-10 |
| active normal force range | 0.000–716.494 N |
| centroidal momentum-rate residual RMS / max | 66.034 / 333.015 N·m |
| point-task acceleration RMS max | 112.810 m/s² |
| frame-angular acceleration RMS max | 303.320 rad/s² |
| longest pre-contact / touchdown transition | 239 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `17.235` / `39.191 cm`.
- Virtual ZMP clipped on `57.50%` of ticks; clip-distance RMS / max `30.233` / `78.843 cm`.
- Measured-height natural frequency min / p50 / max: `3.675` / `3.739` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.842 m`; height-floor ticks: `117`.
- CoM command acceleration p95 / max: `22.055` / `22.363 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2542.6 µs | 98279.0 µs | 102161.9 µs | 185518.7 µs | 129 | 99 | 239 | 0 | 117 | 16 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10153.1 | 25990.2 | 367.4 | 7057.2 | 179570.6 | 184923.9 | 93352.5 | 600 | 87 | 43 | 98.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2430.3 | 2540.3 | 2561.9 | 2587.4 |
| solved_with_slack | 99 | 2511.5 | 2815.0 | 2918.4 | 3070.3 |
| normal_contact_contingency | 117 | 4538.2 | 93617.3 | 101401.7 | 185518.7 |
| contact_release_contingency | 16 | 101303.8 | 125612.9 | 165593.5 | 175588.7 |
| precontact_transition | 239 | 2568.0 | 94620.1 | 102624.5 | 109971.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.72 | 12.0 | 13.0 | 17 | 5.56 | 13.0 | 17 | 0.1831 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 273.75/8.0/6442.5/6794 | 60265.15/1776.0/1402122.8/1508268 | 1.10/10.0/18 | 0.84/9.0/17 | 6.79/76.1/144 | 0.7415 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| normal_contact_contingency | 117 | 9.40/14.0/17 | 8.79/13.0/17 |
| contact_release_contingency | 16 | 6.88/8.8/9 | 5.31/7.0/7 |
| precontact_transition | 239 | 8.82/13.6/14 | 7.73/13.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.93/6.0/12 | 8.06/25.0/52 | 1.40/6.0/12 | 285 |
| viability | 1.82/6.0/8 | 11.91/42.1/65 | 1.43/6.0/8 | 372 |
| intent | 1.56/5.0/8 | 8.26/24.0/46 | 1.32/5.0/8 | 467 |
| preference | 1.36/6.0/8 | 11.15/53.0/71 | 0.98/5.0/8 | 392 |
| style | 1.05/2.0/5 | 8.99/19.0/45 | 0.43/2.0/5 | 230 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 467 | 133 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `136` ticks.
Precontact sole-center tangential speed: p50 `1.9913 m/s`, p95 `8.0190 m/s`, max `9.4543 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.092 | 6.090 | 6.090 | 1.000 | 1.000 | 48.441 | 48.762 | 0.320 | 48.762 | 0.001 | 0 | 59 | 0 | 0 | 88 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2414.1 | 2534.8 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2501.0 | 2695.7 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2495.3 | 2978.8 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2208.5 | 2823.0 | 7.15 | 45.20 | 4.62 | 1.00 | 225.80 | 6.402 | 0.788 | 1.12e-09 | 4.18e-11 | 12 |
| 240–299 | 2439.3 | 3363.9 | 8.38 | 51.82 | 6.92 | 1.00 | 222.00 | 6.212 | 21.733 | 1.04e-09 | 2.33e-11 | 60 |
| 300–359 | 2376.4 | 3771.4 | 8.42 | 52.87 | 6.75 | 1.00 | 222.00 | 6.858 | 24.846 | 1.12e-09 | 1.88e-11 | 60 |
| 360–419 | 3329.1 | 4279.5 | 8.82 | 51.77 | 8.23 | 6.25 | 1387.50 | 20.042 | 9.130 | 9.23e-10 | 1.15e-11 | 60 |
| 420–479 | 80313.0 | 140946.0 | 9.72 | 58.13 | 9.17 | 2482.73 | 547519.50 | 41.380 | 31.486 | 1.40e-09 | 7.06e-11 | 60 |
| 480–539 | 4052.1 | 136274.4 | 9.48 | 58.67 | 8.52 | 234.48 | 50644.70 | 108.846 | 51.608 | 2.28e-09 | 1.37e-10 | 60 |
| 540–599 | 4949.2 | 15862.0 | 8.80 | 54.75 | 8.33 | 8.00 | 1728.00 | 155.073 | 76.529 | 6.59e-09 | 3.64e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 61.756 | 32.688 | 90.334 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
