# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-centroid-preview` reference at `viability` priority with weight `1.000` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 300 | 1.500 s | 2.988 cm | 0.401 cm | 7.233 cm | 24.414 cm | 1.962° | 8.000 rad/s | 3355.3 µs |

Nominal hard residual maxima: dynamics `1.878e-09`, contact acceleration `5.922e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2.988 cm |
| CoM RMS / p95 | 3.611 / 7.769 cm |
| stance foot RMS | 0.401 cm |
| swing foot RMS | 7.233 cm |
| hand RMS | 24.414 cm |
| maximum root rotation | 1.962° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.878e-09 |
| contact acceleration residual | 5.922e-11 |
| raw max dynamics residual, including rejected ticks | 1.878e-09 |
| raw max contact residual, including rejected ticks | 5.922e-11 |
| active normal force range | 0.000–335.991 N |
| centroidal momentum-rate residual RMS / max | 10.686 / 75.498 N·m |
| point-task acceleration RMS max | 86.560 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `1.554` / `3.973 cm`.
- Virtual ZMP clipped on `28.67%` of ticks; clip-distance RMS / max `1.300` / `8.405 cm`.
- Measured-height natural frequency min / p50 / max: `3.694` / `3.775` / `3.787 rad/s`.
- CoM command acceleration p95 / max: `5.408` / `5.915 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.5 s | 2411.9 µs | 3078.0 µs | 3355.3 µs | 4672.6 µs | 158 | 142 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2374.0 | 434.0 | 71.9 | 2970.9 | 4467.9 | 4652.1 | 1430.0 | 300 | 0 | 0 | 421.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 158 | 2412.7 | 2500.7 | 2520.3 | 2548.8 |
| solved_with_slack | 142 | 2399.1 | 3216.7 | 3855.6 | 4672.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.75 | 12.0 | 15.0 | 18 | 3.06 | 13.0 | 17 | 0.0588 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.82/8.0/8.0/8 | 411.26/1776.0/1776.0/1776 | 0.23/2.0/2 | 0.12/1.0/1 | 0.95/9.0/9 | 0.5797 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 158 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 142 | 8.70/17.0/18 | 6.46/14.0/17 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.21/3.0/4 | 4.32/12.0/16 | 0.32/3.0/4 | 54 |
| viability | 2.14/9.0/12 | 6.76/30.0/50 | 1.47/9.0/12 | 122 |
| intent | 1.36/4.0/5 | 6.83/23.0/30 | 0.84/4.0/5 | 142 |
| preference | 1.01/1.0/2 | 10.69/13.0/20 | 0.26/1.0/2 | 76 |
| style | 1.03/2.0/2 | 10.26/21.0/26 | 0.17/2.0/2 | 44 |

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
| 0.712 | 0.712 | 0.712 | 1.000 | 1.000 | 47.227 | 47.227 | 0.000 | 48.090 | 0.001 | 0 | 0 | 0 | 0 | 11 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–29 | 2412.7 | 2497.6 | 5.00 | 32.40 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.88e-09 | 4.95e-11 | 0 |
| 30–59 | 2434.1 | 2543.7 | 5.00 | 32.63 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 8.98e-10 | 5.29e-11 | 0 |
| 60–89 | 2427.3 | 2504.8 | 5.00 | 31.77 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.35e-09 | 5.70e-11 | 0 |
| 90–119 | 2415.6 | 2502.2 | 5.00 | 31.93 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.44e-09 | 3.11e-11 | 0 |
| 120–149 | 2387.7 | 2500.9 | 5.00 | 31.30 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 9.38e-10 | 4.22e-11 | 0 |
| 150–179 | 2452.7 | 3803.6 | 10.07 | 43.10 | 6.87 | 1.00 | 234.00 | 0.406 | 0.000 | 1.18e-09 | 5.92e-11 | 0 |
| 180–209 | 2426.6 | 3373.7 | 8.03 | 43.53 | 5.23 | 1.00 | 229.60 | 2.203 | 0.004 | 1.04e-09 | 4.06e-11 | 0 |
| 210–239 | 2994.7 | 4255.8 | 7.83 | 47.03 | 5.13 | 7.77 | 1724.20 | 3.686 | 0.420 | 1.55e-10 | 2.21e-12 | 0 |
| 240–269 | 1681.5 | 3118.8 | 8.07 | 45.60 | 6.30 | 2.40 | 532.80 | 5.180 | 5.184 | 1.39e-09 | 1.64e-11 | 0 |
| 270–299 | 1787.0 | 2334.9 | 8.53 | 49.33 | 7.03 | 1.00 | 222.00 | 6.622 | 7.897 | 1.62e-09 | 1.45e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 300 | 2.988 | 2.990 | 24.414 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
