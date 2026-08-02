# Bonesaw floating CMU walking corpus

This is the first moving-root, contact-aware retarget through the Rust floating inverse-dynamics WBC. Python owns source reconstruction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target stride: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.

## Acceptance

Overall: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | FAIL |
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 532 | 2.660 s | 17.178 cm | 29.447 cm | 34.918 cm | 35.661 cm | 17.719° | 8.000 rad/s | 73581.9 µs |

Nominal hard residual maxima: dynamics `9.547e-10`, contact acceleration `5.231e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 45.616 cm |
| stance foot RMS | 46.114 cm |
| swing foot RMS | 70.212 cm |
| hand RMS | 58.569 cm |
| maximum root rotation | 17.719° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.743e-10 |
| contact acceleration residual | 5.231e-11 |
| raw max dynamics residual, including rejected ticks | 9.743e-10 |
| raw max contact residual, including rejected ticks | 5.231e-11 |
| active normal force range | 0.000–570.952 N |
| point-task acceleration RMS max | 140.235 m/s² |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 11288.9 µs | 73316.0 µs | 136103.4 µs | 264145.5 µs | 15 | 410 | 107 | 51 | 17 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 21623.4 | 29147.1 | 9368.9 | 54793.9 | 187620.0 | 256493.0 | 71035.5 | 600 | 389 | 227 | 46.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 15 | 2569.3 | 2629.2 | 2632.6 | 2633.4 |
| solved_with_slack | 410 | 8361.3 | 24826.5 | 38129.6 | 43130.4 |
| normal_contact_contingency | 51 | 50994.4 | 91325.0 | 103342.6 | 107049.3 |
| contact_release_contingency | 17 | 135967.2 | 161941.2 | 243704.6 | 264145.5 |
| touchdown_transition | 107 | 38866.3 | 73586.1 | 75083.1 | 75293.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.02 | 11.0 | 14.0 | 17 | 6.44 | 13.0 | 16 | 0.0467 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 15 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 410 | 8.04/15.0/17 | 6.49/13.9/16 |
| normal_contact_contingency | 51 | 8.67/14.0/14 | 7.65/12.5/13 |
| contact_release_contingency | 17 | 7.53/8.8/9 | 6.35/7.8/8 |
| touchdown_transition | 107 | 8.11/11.0/12 | 6.58/10.0/10 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.02/5.0/7 | 8.05/20.0/28 | 1.56/5.0/7 | 442 |
| viability | 2.07/8.0/12 | 11.60/40.0/63 | 1.90/8.0/12 | 506 |
| intent | 1.64/4.0/10 | 8.39/23.0/50 | 1.61/4.0/10 | 583 |
| preference | 1.20/4.0/7 | 11.92/44.1/77 | 0.90/4.0/7 | 432 |
| style | 1.09/3.0/4 | 9.84/28.0/37 | 0.47/2.0/3 | 251 |

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12.974 | 12.971 | 12.971 | 1.000 | 1.000 | 46.312 | 46.551 | 0.238 | 46.953 | 0.001 | 0 | 41 | 0 | 0 | 136 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 3077.9 | 21978.2 | 7.80 | 45.37 | 5.68 | 0.348 | 0.012 | 8.79e-10 | 5.23e-11 | 0 |
| 60–119 | 2743.9 | 38705.5 | 6.67 | 43.68 | 4.27 | 0.614 | 0.000 | 9.30e-10 | 4.66e-11 | 0 |
| 120–179 | 21108.3 | 27229.9 | 8.08 | 51.32 | 6.78 | 0.786 | 0.000 | 9.09e-10 | 4.76e-11 | 0 |
| 180–239 | 12979.8 | 33206.9 | 8.12 | 50.77 | 6.43 | 2.836 | 4.537 | 1.29e-10 | 5.32e-12 | 0 |
| 240–299 | 8951.2 | 10494.9 | 8.25 | 48.90 | 6.80 | 6.469 | 17.682 | 4.32e-10 | 4.07e-12 | 0 |
| 300–359 | 1966.6 | 42107.1 | 7.98 | 47.83 | 6.28 | 14.524 | 16.702 | 5.55e-10 | 7.25e-12 | 0 |
| 360–419 | 1934.6 | 15007.2 | 8.73 | 54.23 | 7.57 | 18.512 | 34.698 | 9.55e-10 | 2.99e-11 | 0 |
| 420–479 | 30850.6 | 75188.5 | 8.10 | 52.48 | 6.95 | 30.446 | 71.367 | 2.06e-10 | 9.11e-13 | 0 |
| 480–539 | 42429.3 | 188125.0 | 8.07 | 51.83 | 6.37 | 42.076 | 46.904 | 2.51e-10 | 2.58e-14 | 8 |
| 540–599 | 54572.3 | 136388.2 | 8.35 | 51.53 | 7.27 | 132.311 | 146.389 | 9.74e-10 | 1.38e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 45.616 | 55.262 | 58.569 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
