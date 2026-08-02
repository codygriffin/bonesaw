# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-centroid-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 398 | 1.990 s | 28.065 cm | 19.026 cm | 44.750 cm | 39.761 cm | 62.595° | 8.000 rad/s | 12131.0 µs |

Nominal hard residual maxima: dynamics `3.166e-09`, contact acceleration `1.258e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 28.793 cm |
| CoM RMS / p95 | 25.881 / 69.552 cm |
| stance foot RMS | 19.936 cm |
| swing foot RMS | 45.728 cm |
| hand RMS | 40.321 cm |
| maximum root rotation | 63.760° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.166e-09 |
| contact acceleration residual | 1.258e-10 |
| raw max dynamics residual, including rejected ticks | 3.166e-09 |
| raw max contact residual, including rejected ticks | 1.258e-10 |
| active normal force range | 0.000–383.755 N |
| centroidal momentum-rate residual RMS / max | 27.878 / 117.943 N·m |
| point-task acceleration RMS max | 121.687 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `35.778` / `91.053 cm`.
- Virtual ZMP clipped on `50.00%` of ticks; clip-distance RMS / max `58.725` / `151.896 cm`.
- Measured-height natural frequency min / p50 / max: `3.687` / `3.783` / `7.004 rad/s`.
- CoM command acceleration p95 / max: `59.828` / `64.329 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 400 | 2.0 s | 2532.0 µs | 4607.3 µs | 30734.4 µs | 207558.2 µs | 119 | 279 | 0 | 0 | 0 | 2 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4018.8 | 12163.1 | 450.3 | 3734.4 | 164968.2 | 203299.2 | 35616.8 | 400 | 18 | 5 | 248.8 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 119 | 2422.5 | 2567.2 | 2909.4 | 2965.4 |
| solved_with_slack | 279 | 3000.7 | 5407.8 | 19305.6 | 76849.4 |
| contact_release_contingency | 2 | 154187.3 | 202221.1 | 206490.8 | 207558.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.07 | 11.0 | 12.0 | 14 | 4.49 | 12.0 | 13 | 0.0696 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 27.56/8.0/15.9/4613 | 6127.73/1776.0/3635.9/1024086 | 1.02/8.0/11 | 0.63/7.0/10 | 5.22/59.2/86 | 0.3536 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 119 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 279 | 7.95/13.0/14 | 6.39/12.0/13 |
| contact_release_contingency | 2 | 7.00/8.0/8 | 5.50/7.0/7 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.84/5.0/7 | 7.41/24.0/35 | 1.29/5.0/7 | 203 |
| viability | 1.50/5.0/7 | 6.14/30.0/42 | 0.99/5.0/7 | 213 |
| intent | 1.59/4.0/6 | 8.05/24.0/30 | 1.29/4.0/6 | 281 |
| preference | 1.11/3.0/6 | 11.52/34.0/63 | 0.63/3.0/5 | 213 |
| style | 1.02/2.0/3 | 9.97/16.0/20 | 0.28/2.0/3 | 103 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 201 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 398 | 2 |
| left_wrist_roll_rubber_hand | 400 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 400 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `5` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.608 | 1.607 | 1.607 | 1.000 | 1.000 | 48.027 | 48.203 | 0.176 | 48.324 | 0.001 | 0 | 44 | 0 | 0 | 24 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–39 | 2415.2 | 2938.4 | 5.00 | 32.40 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.23e-09 | 5.29e-11 | 0 |
| 40–79 | 2442.6 | 2905.3 | 5.00 | 32.30 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.14e-09 | 5.86e-11 | 0 |
| 80–119 | 2420.2 | 2501.6 | 5.08 | 31.80 | 0.15 | 1.00 | 234.00 | 0.000 | 0.000 | 1.44e-09 | 5.70e-11 | 0 |
| 120–159 | 2599.9 | 4566.2 | 7.15 | 44.45 | 4.53 | 2.40 | 561.60 | 0.912 | 0.000 | 1.59e-09 | 4.77e-11 | 0 |
| 160–199 | 2500.3 | 5651.1 | 7.35 | 44.80 | 4.95 | 2.05 | 479.40 | 4.101 | 0.001 | 1.18e-09 | 6.73e-11 | 0 |
| 200–239 | 3110.7 | 4314.4 | 8.15 | 49.48 | 6.60 | 5.38 | 1193.25 | 10.629 | 3.048 | 4.01e-10 | 1.59e-11 | 0 |
| 240–279 | 1962.0 | 4780.7 | 7.97 | 46.85 | 6.88 | 3.90 | 865.80 | 17.169 | 22.617 | 8.30e-10 | 1.66e-11 | 0 |
| 280–319 | 3077.1 | 4797.4 | 8.55 | 51.42 | 7.35 | 7.12 | 1581.75 | 26.186 | 23.134 | 1.36e-09 | 1.31e-11 | 0 |
| 320–359 | 3030.5 | 35460.6 | 8.07 | 46.15 | 7.00 | 109.12 | 24225.75 | 42.211 | 35.204 | 8.80e-10 | 9.73e-12 | 0 |
| 360–399 | 3626.2 | 165928.9 | 8.32 | 51.27 | 7.40 | 142.65 | 31667.70 | 73.466 | 77.033 | 3.17e-09 | 1.26e-10 | 2 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 400 | 28.793 | 28.687 | 40.321 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
