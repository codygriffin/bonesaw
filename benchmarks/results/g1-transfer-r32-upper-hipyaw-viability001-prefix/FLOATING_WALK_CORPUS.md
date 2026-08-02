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
- Protected upper-body posture: `viability` priority with weight `0.010` over 13 waist/arm coordinates.
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
| 250 | 1.250 s | 2.852 cm | 0.000 cm | 11.681 cm | 5.338 cm | 0.000° | 8.000 rad/s | 4754.2 µs |

Nominal hard residual maxima: dynamics `1.581e-09`, contact acceleration `5.499e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2.852 cm |
| CoM RMS / p95 | 3.730 / 8.256 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 11.681 cm |
| hand RMS | 5.338 cm |
| maximum root rotation | 0.000° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.581e-09 |
| contact acceleration residual | 5.499e-11 |
| raw max dynamics residual, including rejected ticks | 1.581e-09 |
| raw max contact residual, including rejected ticks | 5.499e-11 |
| active normal force range | 0.000–231.497 N |
| centroidal momentum-rate residual RMS / max | 4.490 / 13.245 N·m |
| point-task acceleration RMS max | 87.080 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.2 s | 2457.9 µs | 3561.3 µs | 4754.2 µs | 4851.9 µs | 148 | 102 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2563.7 | 467.4 | 55.2 | 2918.5 | 4841.6 | 4850.9 | 2081.7 | 250 | 0 | 0 | 390.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 148 | 2444.0 | 2539.0 | 2649.9 | 2859.9 |
| solved_with_slack | 102 | 2615.0 | 4060.8 | 4810.3 | 4851.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.13 | 10.0 | 12.0 | 13 | 1.68 | 8.5 | 10 | 0.6730 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.00/1.0/1.0/1 | 231.55/234.0/234.0/234 | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0.3079 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 148 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 102 | 7.76/13.0/13 | 4.11/9.0/10 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.00/1.0/1 | 3.34/4.0/4 | 0.00/0.0/0 | 0 |
| viability | 2.04/8.0/9 | 12.60/49.0/54 | 1.45/8.0/9 | 102 |
| intent | 1.00/1.0/1 | 3.94/4.0/5 | 0.00/0.0/0 | 0 |
| preference | 1.03/1.5/6 | 4.90/6.0/18 | 0.06/1.5/6 | 7 |
| style | 1.06/3.0/3 | 11.15/32.5/35 | 0.17/2.0/3 | 37 |

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
| 0.641 | 0.641 | 0.641 | 1.000 | 1.000 | 47.938 | 47.938 | 0.000 | 47.938 | 0.001 | 0 | 0 | 0 | 0 | 16 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–24 | 2492.8 | 3692.6 | 7.08 | 41.44 | 2.44 | 1.00 | 234.00 | 0.318 | 0.000 | 1.53e-09 | 5.01e-11 | 0 |
| 25–49 | 2458.5 | 2520.1 | 5.00 | 29.48 | 0.00 | 1.00 | 234.00 | 0.941 | 0.000 | 7.60e-10 | 4.41e-11 | 0 |
| 50–74 | 2419.6 | 2537.3 | 5.00 | 29.04 | 0.00 | 1.00 | 234.00 | 1.271 | 0.000 | 1.58e-09 | 4.32e-11 | 0 |
| 75–99 | 2448.6 | 2506.4 | 5.00 | 29.00 | 0.00 | 1.00 | 234.00 | 1.387 | 0.000 | 1.43e-09 | 4.17e-11 | 0 |
| 100–124 | 2423.3 | 2817.0 | 5.00 | 29.12 | 0.00 | 1.00 | 234.00 | 1.425 | 0.000 | 4.67e-10 | 2.61e-11 | 0 |
| 125–149 | 2470.8 | 2599.7 | 5.00 | 29.80 | 0.00 | 1.00 | 234.00 | 1.437 | 0.000 | 7.19e-10 | 5.50e-11 | 0 |
| 150–174 | 2750.4 | 4808.7 | 6.80 | 42.04 | 2.56 | 1.00 | 234.00 | 1.443 | 0.000 | 5.22e-10 | 4.19e-11 | 0 |
| 175–199 | 2822.4 | 4689.0 | 7.64 | 47.08 | 3.76 | 1.00 | 233.52 | 2.380 | 0.002 | 1.34e-09 | 4.00e-11 | 0 |
| 200–224 | 2156.1 | 2764.9 | 7.24 | 40.44 | 3.88 | 1.00 | 222.00 | 4.982 | 2.702 | 1.19e-09 | 1.75e-11 | 0 |
| 225–249 | 2156.2 | 3196.6 | 7.52 | 41.76 | 4.12 | 1.00 | 222.00 | 6.335 | 11.484 | 5.12e-10 | 1.18e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 250 | 2.852 | 3.731 | 5.338 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
