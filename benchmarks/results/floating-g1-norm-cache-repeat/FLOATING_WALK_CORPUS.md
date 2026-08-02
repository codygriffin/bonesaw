# Bonesaw floating CMU walking corpus

This is the first moving-root, contact-aware retarget through the Rust floating inverse-dynamics WBC. Python owns source reconstruction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target stride: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task weight: `0.000`. Hand task weight: `0.000` (hand error is observational when zero).

## Acceptance

Overall: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 260 | 1.300 s | 0.776 cm | 0.000 cm | 0.159 cm | 10.614 cm | 1.767° | 8.000 rad/s | 13115.7 µs |

Nominal hard residual maxima: dynamics `1.218e-09`, contact acceleration `5.151e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 0.776 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 0.159 cm |
| hand RMS | 10.614 cm |
| maximum root rotation | 1.767° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.218e-09 |
| contact acceleration residual | 5.151e-11 |
| raw max dynamics residual, including rejected ticks | 1.218e-09 |
| raw max contact residual, including rejected ticks | 5.151e-11 |
| active normal force range | 0.000–203.049 N |
| point-task acceleration RMS max | 16.528 m/s² |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 260 | 1.3 s | 3269.1 µs | 6707.3 µs | 13115.7 µs | 14236.0 µs | 196 | 61 | 3 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3610.1 | 1918.0 | 44.9 | 3420.2 | 14151.2 | 14227.6 | 3637.7 | 260 | 16 | 0 | 277.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 196 | 3282.6 | 3420.7 | 3508.8 | 3517.8 |
| solved_with_slack | 61 | 2644.2 | 13046.3 | 14039.5 | 14236.0 |
| touchdown_transition | 3 | 3009.4 | 3117.8 | 3127.5 | 3129.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.94 | 9.0 | 10.0 | 11 | 1.13 | 8.4 | 10 | 0.4131 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 196 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 61 | 7.95/11.0/11 | 4.80/9.4/10 |
| touchdown_transition | 3 | 5.00/5.0/5 | 0.00/0.0/0 |

| priority | pseudoinverse mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|
| invariant | 1.08/3.0/3 | 0.12/2.0/3 | 16 |
| viability | 0.28/2.0/3 | 0.07/2.0/3 | 10 |
| intent | 1.18/4.0/5 | 0.30/4.0/5 | 32 |
| preference | 1.40/5.4/7 | 0.63/5.4/7 | 61 |
| style | 1.00/1.0/1 | 0.01/0.4/1 | 3 |

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.939 | 0.938 | 0.938 | 1.000 | 1.000 | 46.340 | 46.340 | 0.000 | 46.340 | 0.000 | 0 | 0 | 0 | 0 | 9 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–25 | 3322.2 | 3515.6 | 4.12 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 26–51 | 3313.9 | 3506.0 | 4.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 52–77 | 3287.1 | 3413.9 | 4.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 78–103 | 3284.1 | 3312.7 | 4.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 104–129 | 3259.2 | 3293.6 | 4.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 130–155 | 3252.8 | 3270.2 | 4.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 156–181 | 3254.1 | 3442.4 | 4.00 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 182–207 | 3200.3 | 3375.4 | 5.15 | 1.15 | 0.007 | 0.001 | 8.52e-10 | 5.15e-11 | 0 |
| 208–233 | 2607.0 | 2994.7 | 7.65 | 3.85 | 0.033 | 0.010 | 5.88e-10 | 1.31e-11 | 0 |
| 234–259 | 6990.1 | 14154.1 | 8.46 | 6.27 | 2.455 | 0.172 | 6.96e-10 | 7.83e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 260 | 0.776 | 0.054 | 10.614 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
