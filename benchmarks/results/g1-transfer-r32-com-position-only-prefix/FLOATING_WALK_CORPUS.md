# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response (`position-only`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.250 s | 1.642 cm | 0.000 cm | 9.237 cm | 22.643 cm | 5.698° | 8.000 rad/s | 8040.4 µs |

Nominal hard residual maxima: dynamics `1.729e-09`, contact acceleration `5.358e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1.642 cm |
| CoM RMS / p95 | 4.381 / 9.337 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 9.237 cm |
| hand RMS | 22.643 cm |
| maximum root rotation | 5.698° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.729e-09 |
| contact acceleration residual | 5.358e-11 |
| raw max dynamics residual, including rejected ticks | 1.729e-09 |
| raw max contact residual, including rejected ticks | 5.358e-11 |
| active normal force range | 0.000–309.361 N |
| centroidal momentum-rate residual RMS / max | 13.407 / 39.728 N·m |
| point-task acceleration RMS max | 90.160 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.2 s | 4236.3 µs | 5805.6 µs | 8040.4 µs | 10012.2 µs | 5 | 245 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4009.7 | 1334.8 | 1119.8 | 5488.8 | 9644.5 | 9975.4 | 2880.1 | 250 | 50 | 0 | 249.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 5 | 2521.0 | 2590.0 | 2600.6 | 2603.2 |
| solved_with_slack | 245 | 4245.9 | 5837.6 | 8083.9 | 10012.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.28 | 12.0 | 16.5 | 25 | 6.55 | 15.5 | 23 | 0.2172 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 5.90/8.0/8.0/8 | 1364.04/1872.0/1872.0/1872 | 1.67/5.0/5 | 0.98/4.0/4 | 8.30/36.0/36 | 0.6783 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 5 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 245 | 8.34/16.6/25 | 6.68/15.6/23 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.79/3.0/4 | 6.84/12.0/12 | 1.18/3.0/4 | 177 |
| viability | 2.09/9.5/11 | 8.72/36.0/42 | 1.70/9.5/11 | 162 |
| intent | 1.99/5.0/6 | 10.17/28.5/36 | 1.94/5.0/6 | 242 |
| preference | 1.35/4.5/19 | 12.43/49.6/171 | 1.16/4.5/18 | 202 |
| style | 1.06/2.0/3 | 11.75/26.5/41 | 0.56/2.0/2 | 131 |

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
| 1.003 | 1.002 | 1.002 | 1.000 | 1.000 | 47.945 | 47.945 | 0.000 | 47.945 | 0.001 | 0 | 0 | 0 | 0 | 16 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–24 | 2850.4 | 5495.7 | 9.36 | 49.60 | 7.32 | 3.52 | 823.68 | 0.050 | 0.000 | 1.73e-09 | 5.36e-11 | 0 |
| 25–49 | 5247.0 | 9657.8 | 8.36 | 48.04 | 7.28 | 8.00 | 1872.00 | 0.649 | 0.001 | 1.35e-10 | 7.16e-12 | 0 |
| 50–74 | 4251.8 | 4831.6 | 7.60 | 47.00 | 5.64 | 5.20 | 1216.80 | 1.302 | 0.000 | 6.03e-10 | 3.96e-11 | 0 |
| 75–99 | 4123.8 | 4620.2 | 7.20 | 43.44 | 5.20 | 5.20 | 1216.80 | 1.313 | 0.000 | 5.67e-10 | 2.52e-11 | 0 |
| 100–124 | 4422.8 | 7189.5 | 7.76 | 50.12 | 6.20 | 5.76 | 1347.84 | 0.597 | 0.000 | 3.31e-10 | 3.07e-11 | 0 |
| 125–149 | 4184.4 | 4681.4 | 7.04 | 45.28 | 4.44 | 4.36 | 1020.24 | 0.618 | 0.000 | 8.61e-10 | 2.96e-11 | 0 |
| 150–174 | 4415.5 | 5723.8 | 8.60 | 53.44 | 7.00 | 5.48 | 1282.32 | 0.468 | 0.000 | 7.65e-10 | 4.42e-11 | 0 |
| 175–199 | 5473.6 | 8081.7 | 9.40 | 56.16 | 8.20 | 8.00 | 1868.16 | 1.300 | 0.001 | 4.42e-10 | 1.67e-11 | 0 |
| 200–224 | 3132.7 | 3799.7 | 8.88 | 52.88 | 7.28 | 8.00 | 1776.00 | 2.632 | 2.042 | 7.05e-12 | 2.63e-13 | 0 |
| 225–249 | 2964.8 | 3686.7 | 8.56 | 53.16 | 6.92 | 5.48 | 1216.56 | 3.682 | 9.103 | 3.33e-10 | 3.76e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 250 | 1.642 | 2.950 | 22.643 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
