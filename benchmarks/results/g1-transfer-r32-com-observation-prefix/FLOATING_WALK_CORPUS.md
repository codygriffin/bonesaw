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
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Touchdown admission limits: `0.025 m` position, `0.200 m/s` tangential, and `0.200 m/s` normal; the outgoing support is retained until its replacement locks.
- Reference touchdown blend: `0` ticks (`0.000 s`) with a quintic zero-velocity endpoint.
- Touchdown stabilization point: `ankle origin`.

## Acceptance

Functional: **FAIL**  
5 ms p99 deadline: **PASS**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | PASS |

Failed checks: `swing_foot_tracking_rms_le_8cm`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.250 s | 2.323 cm | 0.000 cm | 8.921 cm | 24.062 cm | 2.117° | 8.000 rad/s | 4736.9 µs |

Nominal hard residual maxima: dynamics `1.622e-09`, contact acceleration `5.634e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2.323 cm |
| CoM RMS / p95 | 3.550 / 8.158 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 8.921 cm |
| hand RMS | 24.062 cm |
| maximum root rotation | 2.117° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.622e-09 |
| contact acceleration residual | 5.634e-11 |
| raw max dynamics residual, including rejected ticks | 1.622e-09 |
| raw max contact residual, including rejected ticks | 5.634e-11 |
| active normal force range | 0.000–204.475 N |
| centroidal momentum-rate residual RMS / max | 8.213 / 30.873 N·m |
| point-task acceleration RMS max | 89.945 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.2 s | 2459.3 µs | 3741.7 µs | 4736.9 µs | 5615.5 µs | 141 | 109 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2649.1 | 554.0 | 82.8 | 3108.7 | 5409.4 | 5594.9 | 1609.6 | 250 | 1 | 0 | 377.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2419.3 | 2519.9 | 2559.8 | 2976.4 |
| solved_with_slack | 109 | 2849.1 | 4512.6 | 4785.7 | 5615.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.39 | 10.0 | 12.0 | 16 | 2.56 | 10.5 | 13 | 0.4672 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 2.32/8.0/8.0/8 | 526.39/1776.0/1872.0/1872 | 0.42/3.0/3 | 0.23/2.0/2 | 1.88/16.0/17 | 0.6964 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 109 | 8.19/12.0/16 | 5.88/11.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.14/2.0/3 | 3.90/8.0/12 | 0.20/2.0/3 | 32 |
| viability | 1.93/8.0/12 | 8.19/29.0/37 | 1.34/8.0/12 | 102 |
| intent | 1.26/3.0/4 | 6.57/18.0/24 | 0.60/3.0/4 | 85 |
| preference | 1.05/2.5/4 | 10.94/26.1/48 | 0.30/2.5/4 | 63 |
| style | 1.02/2.0/2 | 11.03/26.0/29 | 0.12/1.0/2 | 30 |

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
| 0.662 | 0.662 | 0.662 | 1.000 | 1.000 | 46.941 | 46.941 | 0.000 | 47.715 | 0.001 | 0 | 0 | 0 | 0 | 2 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–24 | 2468.1 | 2866.6 | 7.56 | 42.72 | 4.00 | 1.00 | 234.00 | 0.183 | 0.000 | 1.62e-09 | 4.47e-11 | 0 |
| 25–49 | 2431.8 | 2552.4 | 5.00 | 32.64 | 0.00 | 1.00 | 234.00 | 0.185 | 0.000 | 1.53e-09 | 5.63e-11 | 0 |
| 50–74 | 2451.2 | 4011.9 | 5.24 | 36.08 | 0.40 | 1.00 | 234.00 | 0.072 | 0.000 | 9.73e-10 | 2.96e-11 | 0 |
| 75–99 | 2403.9 | 2517.4 | 5.00 | 34.00 | 0.00 | 1.00 | 234.00 | 0.024 | 0.000 | 1.23e-09 | 2.77e-11 | 0 |
| 100–124 | 2412.6 | 2516.8 | 5.00 | 33.76 | 0.00 | 1.00 | 234.00 | 0.007 | 0.000 | 1.12e-09 | 4.79e-11 | 0 |
| 125–149 | 2418.3 | 2541.1 | 5.00 | 33.76 | 0.00 | 1.00 | 234.00 | 0.002 | 0.000 | 9.48e-10 | 4.12e-11 | 0 |
| 150–174 | 2499.3 | 2916.1 | 7.04 | 41.60 | 2.76 | 1.00 | 234.00 | 0.001 | 0.000 | 1.46e-09 | 4.70e-11 | 0 |
| 175–199 | 2865.9 | 5416.8 | 8.48 | 55.00 | 6.84 | 3.24 | 757.68 | 0.473 | 0.001 | 1.41e-09 | 5.57e-11 | 0 |
| 200–224 | 2862.9 | 3609.5 | 7.72 | 49.36 | 5.48 | 4.92 | 1092.24 | 2.813 | 1.625 | 1.44e-09 | 1.27e-11 | 0 |
| 225–249 | 3029.8 | 3752.0 | 7.88 | 47.36 | 6.16 | 8.00 | 1776.00 | 6.763 | 8.862 | 6.54e-12 | 2.20e-13 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 250 | 2.323 | 2.849 | 24.062 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
