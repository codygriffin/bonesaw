# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-centroid-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `root_tracking_rms_le_5cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.500 s | 6.268 cm | 1.178 cm | 4.070 cm | 26.027 cm | 11.264° | 8.000 rad/s | 5155.7 µs |

Nominal hard residual maxima: dynamics `1.517e-09`, contact acceleration `6.783e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 6.268 cm |
| CoM RMS / p95 | 3.884 / 8.283 cm |
| stance foot RMS | 1.178 cm |
| swing foot RMS | 4.070 cm |
| hand RMS | 26.027 cm |
| maximum root rotation | 11.264° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.517e-09 |
| contact acceleration residual | 6.783e-11 |
| raw max dynamics residual, including rejected ticks | 1.517e-09 |
| raw max contact residual, including rejected ticks | 6.783e-11 |
| active normal force range | 0.000–290.021 N |
| centroidal momentum-rate residual RMS / max | 11.725 / 49.942 N·m |
| point-task acceleration RMS max | 99.095 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `6.871` / `17.886 cm`.
- Virtual ZMP clipped on `34.33%` of ticks; clip-distance RMS / max `18.766` / `88.223 cm`.
- Measured-height natural frequency min / p50 / max: `3.698` / `3.776` / `3.868 rad/s`.
- CoM command acceleration p95 / max: `9.877` / `27.465 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.5 s | 2447.2 µs | 3917.2 µs | 5155.7 µs | 39605.1 µs | 158 | 142 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2761.3 | 2289.3 | 96.0 | 3650.1 | 31486.9 | 38793.3 | 2878.2 | 300 | 4 | 1 | 362.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 158 | 2421.1 | 2539.7 | 2560.5 | 2569.6 |
| solved_with_slack | 142 | 2721.8 | 4103.8 | 9510.7 | 39605.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.86 | 11.1 | 15.0 | 17 | 3.46 | 14.0 | 16 | 0.2435 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 12.42/8.0/8.0/2409 | 2765.94/1776.0/1776.0/534798 | 0.50/3.0/3 | 0.30/2.0/2 | 2.52/18.0/18 | 0.9668 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 158 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 142 | 8.92/17.0/17 | 7.30/15.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.38/4.0/6 | 4.97/16.0/24 | 0.58/4.0/6 | 82 |
| viability | 1.88/10.0/13 | 5.92/30.0/35 | 1.26/10.0/13 | 121 |
| intent | 1.43/4.0/5 | 7.16/24.0/30 | 0.90/4.0/5 | 142 |
| preference | 1.14/4.0/7 | 12.14/47.1/77 | 0.51/4.0/7 | 115 |
| style | 1.02/2.0/3 | 10.40/24.0/40 | 0.20/2.0/2 | 56 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 101 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 300 | 0 |
| left_wrist_roll_rubber_hand | 300 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 300 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.829 | 0.828 | 0.828 | 1.000 | 1.000 | 47.297 | 47.379 | 0.082 | 47.891 | 0.001 | 0 | 1 | 0 | 0 | 13 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–29 | 2403.9 | 2549.8 | 5.00 | 32.13 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.37e-09 | 5.07e-11 | 0 |
| 30–59 | 2435.5 | 2555.1 | 5.00 | 32.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.52e-09 | 5.82e-11 | 0 |
| 60–89 | 2424.4 | 2566.9 | 5.00 | 31.60 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.37e-09 | 5.02e-11 | 0 |
| 90–119 | 2421.0 | 2534.5 | 5.00 | 31.37 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.13e-09 | 4.66e-11 | 0 |
| 120–149 | 2429.7 | 2534.7 | 5.00 | 31.27 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 4.53e-10 | 4.27e-11 | 0 |
| 150–179 | 2532.6 | 3777.8 | 9.83 | 45.93 | 7.27 | 1.00 | 234.00 | 0.216 | 0.000 | 1.14e-09 | 5.16e-11 | 0 |
| 180–209 | 2604.6 | 5240.3 | 8.03 | 50.43 | 5.90 | 1.47 | 333.20 | 2.629 | 0.009 | 1.31e-09 | 6.78e-11 | 0 |
| 210–239 | 3551.6 | 4055.5 | 8.27 | 49.23 | 6.27 | 8.00 | 1776.00 | 5.936 | 0.696 | 1.79e-11 | 8.57e-13 | 0 |
| 240–269 | 1702.4 | 31731.3 | 8.70 | 49.70 | 7.20 | 102.17 | 22681.00 | 8.813 | 2.105 | 6.35e-10 | 1.28e-11 | 0 |
| 270–299 | 3780.7 | 4443.3 | 8.73 | 52.27 | 7.93 | 6.60 | 1465.20 | 16.523 | 5.874 | 1.07e-09 | 1.08e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 300 | 6.268 | 1.986 | 26.027 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
