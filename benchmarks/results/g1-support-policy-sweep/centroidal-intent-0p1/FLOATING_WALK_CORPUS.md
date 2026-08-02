# Bonesaw floating CMU walking corpus

This is the first moving-root, contact-aware retarget through the Rust floating inverse-dynamics WBC. Python owns source reconstruction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target stride: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `rooted` reference at `viability` priority with weight `0.000` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.100` and `1.000 Hz` response.

## Acceptance

Overall: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | FAIL |
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

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 348 | 1.740 s | 15.919 cm | 0.666 cm | 4.122 cm | 28.545 cm | 43.683° | 8.000 rad/s | 60254.7 µs |

Nominal hard residual maxima: dynamics `1.502e-09`, contact acceleration `5.449e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 171.664 cm |
| stance foot RMS | 90.505 cm |
| swing foot RMS | 173.748 cm |
| hand RMS | 173.178 cm |
| maximum root rotation | 179.434° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.502e-09 |
| contact acceleration residual | 5.449e-11 |
| raw max dynamics residual, including rejected ticks | 3.965e+02 |
| raw max contact residual, including rejected ticks | 5.449e-11 |
| active normal force range | 0.000–779.925 N |
| point-task acceleration RMS max | 138.548 m/s² |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2501.4 µs | 223684.3 µs | 224594.0 µs | 272803.6 µs | 196 | 149 | 19 | 53 | 91 | 92 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 62829.1 | 83388.2 | 808.8 | 223168.1 | 266506.9 | 272174.0 | 129277.9 | 600 | 272 | 255 | 15.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 196 | 2354.2 | 2493.2 | 2530.6 | 2638.4 |
| solved_with_slack | 149 | 2285.4 | 46504.0 | 86591.6 | 95541.8 |
| primal_infeasible | 92 | 223418.6 | 224548.3 | 224999.6 | 225589.4 |
| normal_contact_contingency | 53 | 3468.2 | 105010.6 | 118168.7 | 118365.0 |
| contact_release_contingency | 91 | 131250.4 | 136014.0 | 263342.8 | 272803.6 |
| touchdown_transition | 19 | 55841.2 | 79499.9 | 84586.1 | 85857.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.70 | 11.0 | 13.0 | 14 | 3.66 | 12.0 | 14 | -0.4647 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 196 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 149 | 8.84/13.5/14 | 7.09/12.5/13 |
| primal_infeasible | 92 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 53 | 8.77/13.5/14 | 7.98/13.0/14 |
| contact_release_contingency | 91 | 7.77/11.1/12 | 6.56/10.1/11 |
| touchdown_transition | 19 | 7.68/11.8/12 | 6.26/11.8/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.58/5.0/7 | 6.09/21.0/28 | 1.16/5.0/7 | 269 |
| viability | 0.93/6.0/9 | 4.51/29.0/45 | 0.82/6.0/9 | 261 |
| intent | 1.16/5.0/7 | 5.38/25.0/35 | 0.79/5.0/7 | 285 |
| preference | 1.18/5.0/7 | 12.21/54.0/69 | 0.77/5.0/7 | 273 |
| style | 0.85/1.0/1 | 5.85/11.0/13 | 0.12/1.0/1 | 75 |

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 37.698 | 37.689 | 37.689 | 1.000 | 1.000 | 46.199 | 46.402 | 0.203 | 46.586 | 0.001 | 0 | 36 | 0 | 0 | 415 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2348.3 | 2482.6 | 4.05 | 29.15 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 60–119 | 2359.8 | 2509.4 | 4.00 | 29.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 120–179 | 2352.0 | 2517.2 | 4.00 | 29.10 | 0.00 | 0.000 | 0.000 | 1.50e-09 | 5.45e-11 | 0 |
| 180–239 | 2331.7 | 2762.1 | 6.85 | 46.17 | 3.43 | 0.186 | 0.105 | 1.32e-09 | 4.97e-11 | 0 |
| 240–299 | 11660.0 | 90455.9 | 8.47 | 47.92 | 6.97 | 13.824 | 2.581 | 9.34e-10 | 2.58e-11 | 0 |
| 300–359 | 8779.6 | 189241.4 | 9.42 | 52.33 | 8.48 | 46.346 | 5.221 | 7.90e-10 | 1.67e-11 | 12 |
| 360–419 | 130643.6 | 158156.9 | 8.52 | 44.23 | 7.52 | 129.697 | 27.229 | 7.92e-10 | 9.25e-12 | 60 |
| 420–479 | 76978.1 | 187605.5 | 8.07 | 44.82 | 7.10 | 233.491 | 146.436 | 8.26e-10 | 4.35e-12 | 44 |
| 480–539 | 219931.6 | 224954.3 | 3.62 | 17.75 | 3.08 | 325.854 | 252.646 | 3.97e+02 | 0.00e+00 | 60 |
| 540–599 | 223583.3 | 224919.6 | 0.00 | 0.00 | 0.00 | 338.859 | 261.988 | 3.97e+02 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 171.664 | 124.373 | 173.178 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
