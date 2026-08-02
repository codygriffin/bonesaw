# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `0.300 Hz` response (`position-only`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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

Functional: **PASS**  
5 ms p99 deadline: **PASS**  
Combined: **PASS**

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
| `p99_tick_le_5ms` | PASS |

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.250 s | 1.461 cm | 0.000 cm | 4.321 cm | 14.482 cm | 1.427° | 8.000 rad/s | 4295.8 µs |

Nominal hard residual maxima: dynamics `1.391e-09`, contact acceleration `5.833e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1.461 cm |
| CoM RMS / p95 | 6.052 / 12.034 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 4.321 cm |
| hand RMS | 14.482 cm |
| maximum root rotation | 1.427° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.391e-09 |
| contact acceleration residual | 5.833e-11 |
| raw max dynamics residual, including rejected ticks | 1.391e-09 |
| raw max contact residual, including rejected ticks | 5.833e-11 |
| active normal force range | 0.000–269.456 N |
| centroidal momentum-rate residual RMS / max | 7.715 / 40.979 N·m |
| point-task acceleration RMS max | 56.509 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.2 s | 2453.7 µs | 3223.6 µs | 4295.8 µs | 4685.2 µs | 176 | 74 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2540.8 | 439.5 | 64.7 | 3013.6 | 4641.5 | 4680.8 | 2056.8 | 250 | 0 | 0 | 393.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 176 | 2442.9 | 2534.7 | 2578.7 | 2707.5 |
| solved_with_slack | 74 | 2910.0 | 4170.8 | 4556.9 | 4685.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.94 | 10.0 | 13.0 | 13 | 1.60 | 10.5 | 12 | 0.1282 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.87/8.0/8.0/8 | 424.25/1776.0/1776.0/1776 | 0.26/2.5/3 | 0.14/1.5/2 | 1.08/12.1/16 | 0.4100 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 176 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 74 | 8.16/13.0/13 | 5.39/11.3/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.10/2.5/3 | 4.02/10.0/12 | 0.14/2.0/2 | 23 |
| viability | 1.45/6.5/8 | 6.51/38.0/48 | 0.64/6.5/8 | 51 |
| intent | 1.18/4.5/8 | 6.08/22.5/37 | 0.38/4.5/8 | 51 |
| preference | 1.17/4.0/4 | 12.52/44.0/44 | 0.35/4.0/4 | 46 |
| style | 1.03/2.0/2 | 10.89/27.0/30 | 0.08/2.0/2 | 14 |

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
| 0.635 | 0.635 | 0.635 | 1.000 | 1.000 | 48.062 | 48.062 | 0.000 | 48.062 | 0.001 | 0 | 0 | 0 | 0 | 10 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–24 | 2984.2 | 4462.4 | 6.64 | 52.32 | 2.60 | 1.00 | 234.00 | 0.004 | 0.000 | 1.33e-09 | 5.83e-11 | 0 |
| 25–49 | 2419.4 | 2559.7 | 5.00 | 33.48 | 0.00 | 1.00 | 234.00 | 0.003 | 0.000 | 1.19e-09 | 3.21e-11 | 0 |
| 50–74 | 2458.6 | 2677.0 | 5.00 | 33.88 | 0.00 | 1.00 | 234.00 | 0.001 | 0.000 | 8.94e-10 | 3.26e-11 | 0 |
| 75–99 | 2448.6 | 2530.5 | 5.00 | 34.32 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 9.84e-10 | 5.09e-11 | 0 |
| 100–124 | 2428.5 | 2984.9 | 5.08 | 35.72 | 0.12 | 1.00 | 234.00 | 0.000 | 0.000 | 1.04e-09 | 4.48e-11 | 0 |
| 125–149 | 2448.5 | 2514.9 | 5.00 | 33.92 | 0.00 | 1.00 | 234.00 | 0.001 | 0.000 | 1.39e-09 | 4.40e-11 | 0 |
| 150–174 | 2418.8 | 2508.6 | 5.00 | 33.60 | 0.00 | 1.00 | 234.00 | 0.001 | 0.000 | 8.48e-10 | 3.13e-11 | 0 |
| 175–199 | 2481.2 | 4237.2 | 5.52 | 38.88 | 0.72 | 1.00 | 233.52 | 0.003 | 0.000 | 8.19e-10 | 5.20e-11 | 0 |
| 200–224 | 2897.1 | 3617.0 | 7.96 | 47.92 | 5.60 | 6.04 | 1340.88 | 1.282 | 1.187 | 2.62e-10 | 7.23e-12 | 0 |
| 225–249 | 2829.8 | 3180.4 | 9.16 | 56.24 | 6.92 | 4.64 | 1030.08 | 4.439 | 4.200 | 1.01e-09 | 1.96e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 250 | 1.461 | 1.380 | 14.482 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
