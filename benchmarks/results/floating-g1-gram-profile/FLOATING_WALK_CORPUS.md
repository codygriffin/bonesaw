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
| 260 | 1.300 s | 0.751 cm | 0.000 cm | 0.187 cm | 10.689 cm | 2.683° | 8.000 rad/s | 11702.5 µs |

Nominal hard residual maxima: dynamics `1.220e-09`, contact acceleration `5.175e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 0.751 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 0.187 cm |
| hand RMS | 10.689 cm |
| maximum root rotation | 2.683° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.220e-09 |
| contact acceleration residual | 5.175e-11 |
| raw max dynamics residual, including rejected ticks | 1.220e-09 |
| raw max contact residual, including rejected ticks | 5.175e-11 |
| active normal force range | 0.456–201.461 N |
| point-task acceleration RMS max | 14.346 m/s² |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 260 | 1.3 s | 3692.4 µs | 6024.9 µs | 11702.5 µs | 13135.1 µs | 196 | 61 | 3 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3885.2 | 1641.7 | 41.9 | 3818.9 | 13008.5 | 13122.5 | 805.0 | 260 | 16 | 0 | 257.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 196 | 3703.1 | 3821.4 | 3932.8 | 4053.0 |
| solved_with_slack | 61 | 2791.3 | 11628.8 | 12841.7 | 13135.1 |
| touchdown_transition | 3 | 3115.6 | 3119.9 | 3120.3 | 3120.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.87 | 9.0 | 10.0 | 11 | 1.06 | 8.0 | 8 | 0.3684 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 196 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 61 | 7.67/10.4/11 | 4.51/8.0/8 |
| touchdown_transition | 3 | 5.00/5.0/5 | 0.00/0.0/0 |

| priority | pseudoinverse mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|
| invariant | 1.08/3.0/3 | 0.13/2.4/3 | 16 |
| viability | 0.25/1.0/2 | 0.03/1.0/1 | 8 |
| intent | 1.19/4.0/5 | 0.31/4.0/5 | 32 |
| preference | 1.34/5.0/6 | 0.58/5.0/6 | 61 |
| style | 1.00/1.0/1 | 0.01/0.4/1 | 3 |

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.010 | 1.010 | 1.010 | 1.000 | 1.000 | 46.359 | 46.359 | 0.000 | 46.359 | 0.000 | 0 | 0 | 0 | 0 | 13 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–25 | 3692.4 | 3881.3 | 4.12 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.04e-11 | 0 |
| 26–51 | 3682.6 | 3759.6 | 4.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.04e-11 | 0 |
| 52–77 | 3697.5 | 3766.8 | 4.00 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 78–103 | 3706.6 | 3777.6 | 4.00 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 104–129 | 3696.2 | 3755.7 | 4.00 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 130–155 | 3713.9 | 3816.9 | 4.00 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 156–181 | 3703.7 | 3921.7 | 4.00 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.04e-11 | 0 |
| 182–207 | 3688.3 | 4038.1 | 5.19 | 1.23 | 0.007 | 0.001 | 8.50e-10 | 5.18e-11 | 0 |
| 208–233 | 2731.2 | 2999.3 | 7.31 | 3.50 | 0.035 | 0.010 | 6.21e-10 | 7.43e-12 | 0 |
| 234–259 | 6090.4 | 13012.9 | 8.12 | 5.85 | 2.375 | 0.203 | 4.11e-10 | 1.01e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 260 | 0.751 | 0.064 | 10.689 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
