# Bonesaw floating G1 synthetic-step acceptance

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: deterministic, predeclared G1-sized left step; no mocap or learned policy contributes contact labels or target motion.
- Contact schedule: liftoff tick `53`, touchdown tick `93`, step `0.010 m`, clearance `0.010 m`.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.010 m` forward per `0.800 s` source cycle.
- Applied mean forward speed: `0.012 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `constant synthetic phase rate` in `0.8 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `rooted` reference at `viability` priority with weight `0.000` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `40` ticks (`0.200 s`) with a receding cubic landing law capped at `25.000 m/s²`; contact forces remain off until the authored contact edge.
- Reference touchdown blend: `0` ticks (`0.000 s`) with a quintic zero-velocity endpoint.
- Touchdown stabilization point: `ankle origin`.

## Acceptance

Functional: **PASS**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
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
| 160 | 0.800 s | 4.185 cm | 0.099 cm | 0.639 cm | 25.909 cm | 1.369° | 8.000 rad/s | 6096.9 µs |

Nominal hard residual maxima: dynamics `1.212e-09`, contact acceleration `4.625e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 4.185 cm |
| stance foot RMS | 0.099 cm |
| swing foot RMS | 0.639 cm |
| hand RMS | 25.909 cm |
| maximum root rotation | 1.369° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.212e-09 |
| contact acceleration residual | 4.625e-11 |
| raw max dynamics residual, including rejected ticks | 1.212e-09 |
| raw max contact residual, including rejected ticks | 4.625e-11 |
| active normal force range | 0.000–250.557 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 0.044 m/s² |
| frame-angular acceleration RMS max | 0.016 rad/s² |
| longest pre-contact / touchdown transition | 40 / 3 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 2534.7 µs | 4893.5 µs | 6096.9 µs | 7298.4 µs | 53 | 64 | 40 | 3 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2945.5 | 991.3 | 208.1 | 4530.1 | 7122.8 | 7280.8 | 2450.2 | 160 | 8 | 0 | 339.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 53 | 2496.7 | 2887.0 | 2923.6 | 2950.2 |
| solved_with_slack | 64 | 3625.4 | 5726.2 | 6602.8 | 7298.4 |
| touchdown_transition | 3 | 2607.3 | 2907.3 | 2934.0 | 2940.6 |
| precontact_transition | 40 | 2155.3 | 2573.2 | 2667.9 | 2680.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.76 | 12.0 | 13.0 | 14 | 3.84 | 11.4 | 12 | 0.4366 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.41/4.0/4.0/4 | 327.41/936.0/936.0/936 | 0.28/2.0/2 | 0.14/1.0/1 | 1.19/9.0/9 | 0.8438 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 53 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 64 | 8.31/13.4/14 | 6.72/12.0/12 |
| touchdown_transition | 3 | 9.33/12.9/13 | 6.67/10.9/11 |
| precontact_transition | 40 | 7.75/10.6/11 | 4.10/7.0/7 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.33/5.4/7 | 4.99/21.6/28 | 0.44/5.4/6 | 32 |
| viability | 0.27/1.0/1 | 1.31/5.0/5 | 0.00/0.0/0 | 0 |
| intent | 1.92/7.4/8 | 3.85/14.8/16 | 1.39/7.4/8 | 76 |
| preference | 2.12/6.4/10 | 14.32/44.5/70 | 1.74/6.0/10 | 106 |
| style | 1.12/2.4/3 | 11.59/27.1/34 | 0.26/2.0/3 | 33 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 40 | 3 | 117 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 160 | 0 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `40` ticks, planned normal touchdown `3` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.0563 m/s`, p95 `0.0752 m/s`, max `0.0755 m/s` over 40 samples.
Touchdown Normal sole-center tangential speed: p50 `0.0000 m/s`, p95 `0.0000 m/s`, max `0.0000 m/s` over 2 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.471 | 0.471 | 0.471 | 1.000 | 1.000 | 41.555 | 41.785 | 0.230 | 41.785 | 0.001 | 0 | 58 | 0 | 0 | 3 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2510.4 | 2755.0 | 4.00 | 23.62 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 16–31 | 2488.4 | 2889.5 | 4.00 | 23.81 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 32–47 | 2496.5 | 2893.4 | 4.00 | 23.75 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 48–63 | 2406.4 | 2907.8 | 6.81 | 39.00 | 2.94 | 1.00 | 225.75 | 0.000 | 0.176 | 1.21e-09 | 3.03e-11 | 11 |
| 64–79 | 2275.8 | 2675.9 | 7.62 | 43.88 | 3.62 | 1.00 | 222.00 | 0.001 | 0.635 | 1.01e-09 | 9.22e-12 | 16 |
| 80–95 | 1959.5 | 2890.6 | 7.94 | 36.62 | 4.94 | 1.00 | 223.12 | 0.223 | 0.277 | 6.70e-10 | 1.91e-11 | 13 |
| 96–111 | 2994.7 | 4060.6 | 8.75 | 43.31 | 7.00 | 1.00 | 234.00 | 2.035 | 0.001 | 8.94e-10 | 4.62e-11 | 0 |
| 112–127 | 4539.3 | 7132.8 | 7.94 | 44.06 | 6.62 | 3.06 | 716.62 | 4.847 | 0.001 | 4.57e-10 | 8.99e-12 | 0 |
| 128–143 | 4513.3 | 5987.3 | 8.69 | 42.31 | 6.94 | 3.06 | 716.62 | 7.362 | 0.000 | 5.39e-10 | 2.31e-11 | 0 |
| 144–159 | 2791.0 | 3810.2 | 7.88 | 40.25 | 6.31 | 1.00 | 234.00 | 9.657 | 0.293 | 6.68e-10 | 3.21e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 4.185 | 0.244 | 25.909 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
