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
| 250 | 1.250 s | 2.099 cm | 0.000 cm | 9.227 cm | 23.884 cm | 1.218° | 8.000 rad/s | 4881.1 µs |

Nominal hard residual maxima: dynamics `1.622e-09`, contact acceleration `5.634e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2.099 cm |
| CoM RMS / p95 | 3.624 / 8.218 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 9.227 cm |
| hand RMS | 23.884 cm |
| maximum root rotation | 1.218° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.622e-09 |
| contact acceleration residual | 5.634e-11 |
| raw max dynamics residual, including rejected ticks | 1.622e-09 |
| raw max contact residual, including rejected ticks | 5.634e-11 |
| active normal force range | 0.000–207.612 N |
| centroidal momentum-rate residual RMS / max | 8.122 / 33.780 N·m |
| point-task acceleration RMS max | 89.291 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.2 s | 2496.5 µs | 4266.3 µs | 4881.1 µs | 5696.5 µs | 141 | 109 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2660.9 | 607.6 | 87.1 | 3089.7 | 5580.3 | 5684.9 | 1878.1 | 250 | 2 | 0 | 375.8 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2461.0 | 2564.7 | 2595.6 | 2601.2 |
| solved_with_slack | 109 | 2833.1 | 4653.6 | 5202.3 | 5696.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.47 | 11.0 | 13.5 | 20 | 2.65 | 12.0 | 18 | 0.4245 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 2.12/8.0/8.0/8 | 483.22/1776.0/1872.0/1872 | 0.33/2.5/3 | 0.17/1.5/2 | 1.40/13.1/16 | 0.5958 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 109 | 8.37/15.9/20 | 6.08/13.0/18 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.10/2.0/3 | 3.75/8.0/12 | 0.16/2.0/3 | 24 |
| viability | 1.98/8.0/12 | 8.53/33.1/37 | 1.38/8.0/12 | 102 |
| intent | 1.24/3.5/6 | 6.40/21.1/31 | 0.58/3.5/6 | 85 |
| preference | 1.13/3.5/13 | 11.77/40.6/117 | 0.39/3.5/13 | 65 |
| style | 1.02/2.0/3 | 11.10/27.0/41 | 0.14/1.0/2 | 34 |

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
| 0.665 | 0.665 | 0.665 | 1.000 | 1.000 | 48.000 | 48.000 | 0.000 | 48.000 | 0.001 | 0 | 0 | 0 | 0 | 7 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–24 | 2530.1 | 2927.7 | 7.56 | 42.72 | 4.00 | 1.00 | 234.00 | 0.183 | 0.000 | 1.62e-09 | 4.47e-11 | 0 |
| 25–49 | 2492.0 | 2589.1 | 5.00 | 32.64 | 0.00 | 1.00 | 234.00 | 0.185 | 0.000 | 1.53e-09 | 5.63e-11 | 0 |
| 50–74 | 2497.9 | 4104.0 | 5.24 | 36.08 | 0.40 | 1.00 | 234.00 | 0.072 | 0.000 | 9.73e-10 | 2.96e-11 | 0 |
| 75–99 | 2440.9 | 2558.9 | 5.00 | 34.00 | 0.00 | 1.00 | 234.00 | 0.024 | 0.000 | 1.23e-09 | 2.77e-11 | 0 |
| 100–124 | 2461.0 | 2592.9 | 5.00 | 33.76 | 0.00 | 1.00 | 234.00 | 0.007 | 0.000 | 1.12e-09 | 4.79e-11 | 0 |
| 125–149 | 2474.9 | 2581.3 | 5.00 | 33.76 | 0.00 | 1.00 | 234.00 | 0.002 | 0.000 | 9.48e-10 | 4.12e-11 | 0 |
| 150–174 | 2538.1 | 2674.9 | 7.04 | 41.60 | 2.76 | 1.00 | 234.00 | 0.001 | 0.000 | 1.46e-09 | 4.70e-11 | 0 |
| 175–199 | 4009.3 | 5584.5 | 9.24 | 64.12 | 7.64 | 3.52 | 823.20 | 0.453 | 0.001 | 4.67e-10 | 2.42e-11 | 0 |
| 200–224 | 1927.9 | 3659.2 | 7.52 | 47.56 | 5.64 | 4.36 | 967.92 | 2.613 | 1.691 | 1.16e-09 | 1.36e-11 | 0 |
| 225–249 | 3018.3 | 3201.2 | 8.08 | 49.36 | 6.08 | 6.32 | 1403.04 | 6.080 | 9.165 | 5.54e-10 | 4.59e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 250 | 2.099 | 2.947 | 23.884 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
