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
| 260 | 1.300 s | 0.726 cm | 0.000 cm | 0.232 cm | 10.544 cm | 2.369° | 8.000 rad/s | 10980.7 µs |

Nominal hard residual maxima: dynamics `1.218e-09`, contact acceleration `5.151e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 0.726 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 0.232 cm |
| hand RMS | 10.544 cm |
| maximum root rotation | 2.369° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.218e-09 |
| contact acceleration residual | 5.151e-11 |
| raw max dynamics residual, including rejected ticks | 1.218e-09 |
| raw max contact residual, including rejected ticks | 5.151e-11 |
| active normal force range | 0.000–234.755 N |
| point-task acceleration RMS max | 16.607 m/s² |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 260 | 1.3 s | 2394.9 µs | 6604.4 µs | 10980.7 µs | 12814.8 µs | 196 | 61 | 3 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2742.4 | 1734.9 | 15.7 | 2429.8 | 12595.9 | 12792.9 | 4996.4 | 260 | 16 | 0 | 364.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 196 | 2397.1 | 2430.2 | 2485.6 | 2509.2 |
| solved_with_slack | 61 | 2099.9 | 10912.5 | 12307.8 | 12814.8 |
| touchdown_transition | 3 | 2274.5 | 2287.7 | 2288.8 | 2289.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.02 | 9.0 | 11.4 | 13 | 1.24 | 9.4 | 11 | 0.4038 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 196 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 61 | 8.30/12.4/13 | 5.28/10.4/11 |
| touchdown_transition | 3 | 5.00/5.0/5 | 0.00/0.0/0 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.09/3.0/3 | 3.51/10.0/10 | 0.13/3.0/3 | 16 |
| viability | 0.25/1.0/2 | 0.90/4.0/7 | 0.04/1.0/2 | 9 |
| intent | 1.20/5.0/8 | 2.41/10.0/16 | 0.32/5.0/8 | 32 |
| preference | 1.47/6.0/6 | 9.22/30.4/32 | 0.70/6.0/6 | 61 |
| style | 1.00/1.0/1 | 10.67/12.0/12 | 0.04/1.0/1 | 10 |

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.713 | 0.713 | 0.713 | 1.000 | 1.000 | 46.426 | 46.426 | 0.000 | 46.426 | 0.000 | 0 | 0 | 0 | 0 | 8 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–25 | 2398.7 | 2411.3 | 4.12 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 26–51 | 2401.8 | 2428.7 | 4.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 52–77 | 2395.6 | 2409.1 | 4.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 78–103 | 2397.0 | 2472.1 | 4.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 104–129 | 2392.1 | 2407.7 | 4.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 130–155 | 2399.5 | 2485.2 | 4.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 156–181 | 2392.8 | 2485.2 | 4.00 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 182–207 | 2328.6 | 2447.6 | 5.31 | 1.35 | 0.007 | 0.001 | 8.52e-10 | 5.15e-11 | 0 |
| 208–233 | 1995.0 | 2365.8 | 7.81 | 4.23 | 0.040 | 0.009 | 9.48e-10 | 1.39e-11 | 0 |
| 234–259 | 6656.9 | 12603.5 | 8.96 | 6.81 | 2.295 | 0.251 | 3.69e-10 | 6.85e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 260 | 0.726 | 0.079 | 10.544 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
