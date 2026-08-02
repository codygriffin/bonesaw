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
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.

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
| 452 | 2.260 s | 11.697 cm | 14.928 cm | 48.971 cm | 27.224 cm | 24.880° | 8.000 rad/s | 107521.2 µs |

Nominal hard residual maxima: dynamics `1.738e-09`, contact acceleration `6.732e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 48.630 cm |
| stance foot RMS | 28.913 cm |
| swing foot RMS | 68.286 cm |
| hand RMS | 54.928 cm |
| maximum root rotation | 179.230° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.738e-09 |
| contact acceleration residual | 6.732e-11 |
| raw max dynamics residual, including rejected ticks | 1.738e-09 |
| raw max contact residual, including rejected ticks | 6.732e-11 |
| active normal force range | 0.000–527.467 N |
| point-task acceleration RMS max | 184.051 m/s² |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2669.4 µs | 135209.1 µs | 163292.4 µs | 265801.1 µs | 147 | 278 | 27 | 99 | 49 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 25984.0 | 42602.5 | 885.3 | 107405.3 | 213364.9 | 260557.5 | 116455.9 | 600 | 259 | 202 | 38.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 147 | 2528.5 | 2607.9 | 2622.6 | 2644.5 |
| solved_with_slack | 278 | 2730.1 | 37215.7 | 39173.6 | 39622.2 |
| normal_contact_contingency | 99 | 16046.1 | 108624.9 | 124079.5 | 129566.4 |
| contact_release_contingency | 49 | 135305.0 | 164365.9 | 223782.1 | 265801.1 |
| touchdown_transition | 27 | 87299.8 | 166493.7 | 172639.8 | 174131.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.16 | 13.0 | 14.0 | 20 | 6.03 | 13.0 | 20 | 0.0907 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 147 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 278 | 9.07/14.2/18 | 7.79/13.2/17 |
| normal_contact_contingency | 99 | 10.09/14.0/15 | 9.22/13.0/14 |
| contact_release_contingency | 49 | 7.80/11.6/13 | 6.55/10.6/12 |
| touchdown_transition | 27 | 9.59/19.5/20 | 8.00/19.2/20 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.06/5.0/7 | 8.20/21.0/29 | 1.62/5.0/7 | 370 |
| viability | 1.93/7.0/13 | 10.67/40.0/48 | 1.68/7.0/13 | 452 |
| intent | 1.52/5.0/9 | 3.21/10.0/18 | 1.23/5.0/9 | 439 |
| preference | 1.60/6.0/10 | 15.07/62.1/96 | 1.21/6.0/10 | 407 |
| style | 1.05/2.0/4 | 8.43/23.0/29 | 0.30/2.0/3 | 164 |

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 15.591 | 15.587 | 15.587 | 1.000 | 1.000 | 46.234 | 46.449 | 0.215 | 46.777 | 0.001 | 0 | 36 | 0 | 0 | 190 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2531.9 | 3517.0 | 6.32 | 33.58 | 1.72 | 0.087 | 0.013 | 1.31e-09 | 4.57e-11 | 0 |
| 60–119 | 2528.2 | 2618.2 | 5.00 | 29.43 | 0.00 | 0.016 | 0.000 | 1.51e-09 | 5.79e-11 | 0 |
| 120–179 | 2551.9 | 3971.2 | 6.70 | 36.73 | 2.28 | 0.059 | 0.000 | 1.52e-09 | 6.73e-11 | 0 |
| 180–239 | 18745.9 | 38570.6 | 9.15 | 48.58 | 7.43 | 3.299 | 2.286 | 5.96e-10 | 2.45e-11 | 0 |
| 240–299 | 25249.7 | 39558.3 | 8.68 | 46.55 | 7.55 | 10.729 | 6.519 | 1.06e-09 | 1.74e-11 | 0 |
| 300–359 | 2121.7 | 36367.3 | 8.68 | 48.68 | 7.97 | 19.876 | 46.495 | 8.21e-10 | 1.67e-11 | 0 |
| 360–419 | 1874.7 | 34688.4 | 8.98 | 51.33 | 8.30 | 19.183 | 51.150 | 1.74e-09 | 6.21e-11 | 0 |
| 420–479 | 51221.0 | 211716.0 | 9.48 | 54.40 | 8.40 | 14.678 | 47.506 | 7.23e-10 | 9.69e-12 | 28 |
| 480–539 | 9994.7 | 25726.6 | 10.62 | 65.07 | 9.67 | 59.394 | 74.468 | 2.79e-10 | 1.45e-12 | 60 |
| 540–599 | 133811.3 | 170423.6 | 7.98 | 41.47 | 6.95 | 137.901 | 91.610 | 4.38e-10 | 5.12e-13 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 48.630 | 45.848 | 54.928 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
