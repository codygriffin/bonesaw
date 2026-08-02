# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `rooted` reference at `viability` priority with weight `0.100` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `support-preview`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; contact forces remain off until the authored contact edge.
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
| 368 | 1.840 s | 9.082 cm | 7.930 cm | 46.751 cm | 22.394 cm | 59.026° | 8.000 rad/s | 88000.2 µs |

Nominal hard residual maxima: dynamics `4.086e-09`, contact acceleration `1.432e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 107.866 cm |
| stance foot RMS | 57.541 cm |
| swing foot RMS | 125.785 cm |
| hand RMS | 116.809 cm |
| maximum root rotation | 179.386° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.145e-09 |
| contact acceleration residual | 1.432e-10 |
| raw max dynamics residual, including rejected ticks | 8.145e-09 |
| raw max contact residual, including rejected ticks | 1.432e-10 |
| active normal force range | 0.000–794.036 N |
| centroidal momentum-rate residual RMS / max | 81.930 / 365.928 N·m |
| point-task acceleration RMS max | 169.756 m/s² |
| frame-angular acceleration RMS max | 241.896 rad/s² |
| longest pre-contact / touchdown transition | 140 / 159 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2702.2 µs | 86728.8 µs | 181018.6 µs | 211734.7 µs | 148 | 80 | 140 | 159 | 55 | 18 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11107.5 | 30173.6 | 678.3 | 7535.0 | 208111.0 | 211372.3 | 91619.9 | 600 | 85 | 53 | 90.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 148 | 2411.3 | 2500.0 | 2537.3 | 2557.6 |
| solved_with_slack | 80 | 2613.7 | 3851.8 | 5652.9 | 9697.5 |
| normal_contact_contingency | 55 | 5728.2 | 94937.1 | 139955.9 | 186052.3 |
| contact_release_contingency | 18 | 122454.3 | 206592.6 | 210706.2 | 211734.7 |
| touchdown_transition | 159 | 3309.7 | 9059.9 | 22400.1 | 23027.4 |
| precontact_transition | 140 | 3131.9 | 76221.5 | 100913.5 | 109703.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.67 | 11.0 | 13.0 | 16 | 5.43 | 12.0 | 15 | 0.1079 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 262.25/1360.6/6179.5/6972 | 57311.82/293900.4/1356308.8/1547784 | 1.29/6.0/35 | 0.93/5.0/34 | 7.53/41.0/300 | 0.5376 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 148 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 80 | 8.71/14.2/15 | 6.42/13.0/13 |
| normal_contact_contingency | 55 | 8.93/14.4/16 | 7.36/13.9/15 |
| contact_release_contingency | 18 | 7.56/9.8/10 | 6.28/8.8/9 |
| touchdown_transition | 159 | 8.41/13.3/15 | 7.31/12.8/15 |
| precontact_transition | 140 | 8.59/12.0/12 | 7.61/11.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.15/6.0/12 | 8.96/25.0/58 | 1.72/6.0/11 | 387 |
| viability | 1.96/7.0/10 | 11.48/42.0/60 | 1.71/7.0/10 | 452 |
| intent | 1.38/4.0/8 | 7.06/23.0/43 | 1.13/4.0/8 | 450 |
| preference | 1.10/3.0/5 | 11.12/30.0/50 | 0.58/3.0/5 | 301 |
| style | 1.08/3.0/5 | 9.51/24.0/66 | 0.29/2.0/4 | 150 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 163 | 199 | 9 |
| right_ankle_roll_link | 168 | 0 | 0 | 368 | 64 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `163` ticks, normal fallback `67` ticks.
Precontact sole-center tangential speed: p50 `2.3539 m/s`, p95 `3.0567 m/s`, max `3.1915 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `3.9106 m/s`, p95 `6.5877 m/s`, max `8.4446 m/s` over 162 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.665 | 6.664 | 6.664 | 1.000 | 1.000 | 47.527 | 47.789 | 0.262 | 47.789 | 0.001 | 0 | 44 | 0 | 0 | 31 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2441.1 | 2801.0 | 6.07 | 36.48 | 1.48 | 1.00 | 234.00 | 0.651 | 0.000 | 1.22e-09 | 5.23e-11 | 0 |
| 60–119 | 2385.4 | 2492.6 | 5.00 | 33.43 | 0.00 | 1.00 | 234.00 | 0.051 | 0.000 | 9.40e-10 | 3.47e-11 | 0 |
| 120–179 | 2476.6 | 5562.9 | 6.28 | 40.13 | 2.25 | 1.12 | 261.30 | 1.818 | 0.000 | 1.88e-09 | 5.93e-11 | 0 |
| 180–239 | 2537.1 | 4222.4 | 8.23 | 49.97 | 6.10 | 3.45 | 771.10 | 5.446 | 4.643 | 4.09e-09 | 1.43e-10 | 12 |
| 240–299 | 2070.1 | 4024.1 | 8.60 | 56.35 | 7.43 | 3.80 | 843.60 | 5.245 | 28.606 | 5.92e-10 | 2.18e-11 | 60 |
| 300–359 | 3605.7 | 64763.2 | 8.62 | 53.23 | 7.95 | 331.02 | 73485.70 | 17.196 | 43.383 | 6.16e-10 | 2.00e-11 | 60 |
| 360–419 | 7819.6 | 141006.3 | 8.73 | 54.52 | 7.23 | 2093.17 | 456723.40 | 73.426 | 101.855 | 2.32e-10 | 1.04e-12 | 60 |
| 420–479 | 3321.1 | 170704.2 | 8.05 | 50.35 | 6.98 | 6.13 | 1324.90 | 191.952 | 116.814 | 2.91e-09 | 4.99e-11 | 12 |
| 480–539 | 3387.5 | 4866.3 | 8.42 | 52.77 | 7.58 | 6.25 | 1350.00 | 226.069 | 172.231 | 1.53e-09 | 5.78e-11 | 0 |
| 540–599 | 2954.6 | 208165.5 | 8.72 | 54.17 | 7.30 | 175.57 | 37890.20 | 150.500 | 134.515 | 8.15e-09 | 5.13e-11 | 9 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 107.866 | 86.313 | 116.809 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
