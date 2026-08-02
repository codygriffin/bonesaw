# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response. Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Touchdown admission limits: `0.025 m` position, `0.200 m/s` tangential, and `0.200 m/s` normal; the outgoing support is retained until its replacement locks.
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
| `touchdown_admission_completes_within_8_ticks` | PASS |
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
| 250 | 1.250 s | 2.269 cm | 0.000 cm | 5.235 cm | 18.934 cm | 2.531° | 8.000 rad/s | 5131.1 µs |

Nominal hard residual maxima: dynamics `1.610e-09`, contact acceleration `6.563e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2.269 cm |
| CoM RMS / p95 | 3.476 / 8.034 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 5.235 cm |
| hand RMS | 18.934 cm |
| maximum root rotation | 2.531° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.610e-09 |
| contact acceleration residual | 6.563e-11 |
| raw max dynamics residual, including rejected ticks | 1.610e-09 |
| raw max contact residual, including rejected ticks | 6.563e-11 |
| active normal force range | 0.000–359.863 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 52.568 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.2 s | 2442.3 µs | 3868.8 µs | 5131.1 µs | 5249.6 µs | 147 | 103 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2676.0 | 641.7 | 81.6 | 3596.3 | 5233.3 | 5248.0 | 1520.7 | 250 | 6 | 0 | 373.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 147 | 2403.9 | 2485.6 | 2518.2 | 2529.1 |
| solved_with_slack | 103 | 2920.1 | 5058.7 | 5183.1 | 5249.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.01 | 13.0 | 16.0 | 17 | 3.26 | 14.0 | 14 | 0.2975 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 2.32/8.0/8.0/8 | 526.73/1776.0/1872.0/1872 | 0.50/3.0/3 | 0.31/2.0/2 | 2.53/18.0/18 | 0.8150 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 147 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 103 | 9.88/17.0/17 | 7.91/14.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.22/3.0/3 | 4.23/12.0/12 | 0.32/3.0/3 | 47 |
| viability | 2.02/8.0/9 | 8.14/33.1/36 | 1.42/8.0/9 | 102 |
| intent | 1.56/6.0/7 | 3.12/12.0/14 | 0.90/5.5/6 | 99 |
| preference | 1.19/4.0/5 | 11.26/39.0/44 | 0.47/4.0/4 | 87 |
| style | 1.02/2.0/2 | 10.64/24.0/24 | 0.15/1.0/1 | 37 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 51 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 250 | 0 |
| left_wrist_roll_rubber_hand | 250 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 250 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.669 | 0.669 | 0.669 | 1.000 | 1.000 | 47.949 | 47.949 | 0.000 | 47.949 | 0.001 | 0 | 0 | 0 | 0 | 10 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–24 | 2439.8 | 2548.5 | 8.20 | 38.36 | 4.28 | 1.00 | 234.00 | 0.159 | 0.000 | 1.61e-09 | 6.56e-11 | 0 |
| 25–49 | 2401.5 | 2520.9 | 5.00 | 28.96 | 0.00 | 1.00 | 234.00 | 0.154 | 0.000 | 1.39e-09 | 4.57e-11 | 0 |
| 50–74 | 2409.3 | 2487.2 | 5.00 | 29.36 | 0.00 | 1.00 | 234.00 | 0.059 | 0.000 | 6.81e-10 | 3.34e-11 | 0 |
| 75–99 | 2407.4 | 2460.6 | 5.00 | 29.08 | 0.00 | 1.00 | 234.00 | 0.018 | 0.000 | 1.03e-09 | 3.34e-11 | 0 |
| 100–124 | 2389.7 | 2502.5 | 5.00 | 28.68 | 0.00 | 1.00 | 234.00 | 0.005 | 0.000 | 8.04e-10 | 3.73e-11 | 0 |
| 125–149 | 2380.6 | 2510.0 | 5.00 | 28.36 | 0.00 | 1.00 | 234.00 | 0.001 | 0.000 | 1.27e-09 | 2.58e-11 | 0 |
| 150–174 | 2518.8 | 3679.8 | 9.60 | 46.52 | 6.00 | 1.00 | 234.00 | 0.092 | 0.000 | 1.11e-09 | 1.71e-11 | 0 |
| 175–199 | 2814.8 | 5233.9 | 8.88 | 44.60 | 6.80 | 3.80 | 885.36 | 0.412 | 0.001 | 1.03e-09 | 4.11e-11 | 0 |
| 200–224 | 3131.7 | 4077.9 | 9.36 | 52.00 | 8.00 | 7.44 | 1651.68 | 2.586 | 1.142 | 5.14e-10 | 1.55e-11 | 0 |
| 225–249 | 3501.0 | 4057.6 | 9.08 | 48.00 | 7.52 | 4.92 | 1092.24 | 6.676 | 5.162 | 7.24e-10 | 1.02e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 250 | 2.269 | 1.672 | 18.934 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
