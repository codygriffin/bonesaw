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
| 393 | 1.965 s | 13.833 cm | 14.744 cm | 51.559 cm | 30.465 cm | 72.961° | 8.000 rad/s | 82741.0 µs |

Nominal hard residual maxima: dynamics `1.560e-09`, contact acceleration `5.480e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 124.937 cm |
| authored reference vs measured CoM RMS / p95 | 129.948 / 283.224 cm |
| stance foot RMS | 110.329 cm |
| swing foot RMS | 145.627 cm |
| hand RMS | 137.256 cm |
| maximum root rotation | 179.870° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.382e-09 |
| contact acceleration residual | 1.395e-10 |
| raw max dynamics residual, including rejected ticks | 8.382e-09 |
| raw max contact residual, including rejected ticks | 1.395e-10 |
| active normal force range | 0.000–554.757 N |
| centroidal momentum-rate residual RMS / max | 63.999 / 299.160 N·m |
| point-task acceleration RMS max | 130.615 m/s² |
| frame-angular acceleration RMS max | 243.592 rad/s² |
| longest pre-contact / touchdown transition | 145 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 152 / 152 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `129.045` / `292.695 cm`.
- Virtual ZMP clipped on `57.50%` of ticks; clip-distance RMS / max `180.169` / `502.195 cm`.
- Measured-height natural frequency min / p50 / max: `3.652` / `3.802` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.742 m`; height-floor ticks: `189`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2933.9 µs | 48118.0 µs | 101282.2 µs | 209214.3 µs | 110 | 138 | 145 | 0 | 196 | 11 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8575.2 | 22068.5 | 513.0 | 7990.6 | 191770.1 | 207469.9 | 42053.9 | 600 | 78 | 43 | 116.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 110 | 2450.7 | 2573.3 | 3751.1 | 3778.3 |
| solved_with_slack | 138 | 2600.1 | 4665.1 | 5971.8 | 8019.4 |
| normal_contact_contingency | 196 | 3165.9 | 35567.4 | 48118.0 | 81117.6 |
| contact_release_contingency | 11 | 101226.5 | 194653.2 | 206302.1 | 209214.3 |
| precontact_transition | 145 | 3168.7 | 73885.7 | 98261.3 | 117634.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.81 | 12.0 | 13.0 | 15 | 5.71 | 13.0 | 14 | 0.1426 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 213.82/2058.7/5183.1/7023 | 47062.62/448223.4/1150637.1/1559106 | 1.46/13.0/14 | 1.00/12.0/13 | 8.19/100.1/114 | 0.6044 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 110 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 138 | 7.31/13.0/15 | 4.87/10.0/13 |
| normal_contact_contingency | 196 | 9.18/14.0/15 | 8.20/13.0/14 |
| contact_release_contingency | 11 | 7.64/9.9/10 | 6.27/8.0/8 |
| precontact_transition | 145 | 8.58/13.6/14 | 7.46/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.11/6.0/7 | 9.09/25.0/35 | 1.69/6.0/7 | 378 |
| viability | 1.68/7.0/9 | 9.13/42.0/54 | 1.32/6.0/9 | 414 |
| intent | 1.57/5.0/6 | 8.16/25.0/35 | 1.39/5.0/6 | 490 |
| preference | 1.41/6.0/7 | 13.94/58.0/81 | 0.99/6.0/7 | 383 |
| style | 1.04/2.0/4 | 9.16/27.0/58 | 0.33/2.0/4 | 185 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 352 | 0 | 219 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 393 | 207 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `352` ticks, planned normal touchdown `0` ticks, normal fallback `210` ticks.
Precontact sole-center tangential speed: p50 `3.0829 m/s`, p95 `6.2482 m/s`, max `6.6589 m/s` over 352 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.145 | 5.139 | 5.139 | 0.999 | 0.999 | 48.125 | 48.625 | 0.500 | 48.625 | 0.001 | 0 | 112 | 0 | 0 | 568 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2424.0 | 2548.5 | 5.00 | 33.12 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.27e-09 | 5.48e-11 | 0 |
| 60–119 | 2483.9 | 3767.9 | 5.30 | 35.90 | 0.68 | 1.00 | 234.00 | 0.016 | 0.000 | 1.36e-09 | 4.95e-11 | 0 |
| 120–179 | 2610.4 | 6829.8 | 7.28 | 48.33 | 4.87 | 1.93 | 452.40 | 1.056 | 0.000 | 1.12e-09 | 4.16e-11 | 0 |
| 180–239 | 2535.6 | 4732.8 | 7.48 | 43.63 | 5.13 | 4.03 | 935.40 | 4.300 | 0.050 | 1.54e-09 | 2.70e-11 | 0 |
| 240–299 | 3169.9 | 4111.5 | 8.28 | 48.78 | 6.68 | 8.00 | 1776.00 | 8.412 | 4.867 | 3.13e-11 | 9.50e-13 | 52 |
| 300–359 | 3131.8 | 4761.6 | 8.43 | 52.00 | 7.50 | 6.83 | 1517.00 | 16.882 | 35.896 | 1.56e-09 | 4.70e-11 | 60 |
| 360–419 | 12851.9 | 155181.9 | 9.55 | 61.60 | 8.55 | 1476.87 | 327558.40 | 55.452 | 103.907 | 8.37e-10 | 1.50e-11 | 60 |
| 420–479 | 4425.9 | 168919.1 | 8.90 | 55.03 | 7.85 | 627.53 | 135546.60 | 151.148 | 155.377 | 8.38e-09 | 1.37e-10 | 60 |
| 480–539 | 3165.9 | 5237.2 | 8.95 | 58.22 | 8.08 | 7.77 | 1677.60 | 230.921 | 199.232 | 3.50e-09 | 1.39e-10 | 60 |
| 540–599 | 1855.9 | 3654.2 | 8.93 | 58.20 | 7.80 | 3.22 | 694.80 | 276.537 | 272.331 | 6.18e-10 | 6.92e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 124.937 | 122.519 | 137.256 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
