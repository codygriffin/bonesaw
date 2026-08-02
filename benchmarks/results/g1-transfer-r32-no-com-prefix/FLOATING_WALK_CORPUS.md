# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `rooted` reference at `viability` priority with weight `0.000` and `2.000 Hz` response. Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 250 | 1.250 s | 0.404 cm | 0.000 cm | 0.359 cm | 6.666 cm | 1.752° | 8.000 rad/s | 2593.5 µs |

Nominal hard residual maxima: dynamics `1.309e-09`, contact acceleration `3.670e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 0.404 cm |
| CoM RMS / p95 | 1.157 / 3.106 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 0.359 cm |
| hand RMS | 6.666 cm |
| maximum root rotation | 1.752° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.309e-09 |
| contact acceleration residual | 3.670e-11 |
| raw max dynamics residual, including rejected ticks | 1.309e-09 |
| raw max contact residual, including rejected ticks | 3.670e-11 |
| active normal force range | 0.000–331.254 N |
| centroidal momentum-rate residual RMS / max | 4.275 / 18.792 N·m |
| point-task acceleration RMS max | 10.190 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 1.2 s | 2274.4 µs | 2422.2 µs | 2593.5 µs | 2818.7 µs | 199 | 51 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2251.2 | 188.5 | 22.2 | 2403.4 | 2811.2 | 2817.9 | 896.1 | 250 | 0 | 0 | 444.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 2276.4 | 2408.3 | 2417.1 | 2442.1 |
| solved_with_slack | 51 | 1996.8 | 2593.4 | 2803.7 | 2818.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.83 | 9.0 | 12.0 | 13 | 1.06 | 10.0 | 12 | -0.2279 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.00/1.0/1.0/1 | 231.55/234.0/234.0/234 | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0.5232 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 51 | 8.06/13.0/13 | 5.20/11.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.11/2.0/2 | 3.66/8.0/8 | 0.16/2.0/2 | 28 |
| viability | 0.21/1.0/2 | 0.80/4.0/8 | 0.06/1.0/1 | 14 |
| intent | 1.14/4.0/6 | 5.92/24.0/36 | 0.25/4.0/6 | 28 |
| preference | 1.37/6.0/7 | 14.30/58.1/71 | 0.56/5.5/7 | 51 |
| style | 1.00/1.0/1 | 9.55/11.0/11 | 0.03/1.0/1 | 8 |

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
| 0.563 | 0.563 | 0.563 | 1.000 | 1.000 | 46.961 | 46.961 | 0.000 | 47.438 | 0.001 | 0 | 0 | 0 | 0 | 7 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–24 | 2276.9 | 2396.9 | 4.00 | 29.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 25–49 | 2270.9 | 2373.1 | 4.00 | 29.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 50–74 | 2274.5 | 2410.4 | 4.00 | 29.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 75–99 | 2285.7 | 2412.0 | 4.00 | 29.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 100–124 | 2276.8 | 2436.0 | 4.00 | 29.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 125–149 | 2275.5 | 2412.0 | 4.00 | 29.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 150–174 | 2273.3 | 2401.6 | 4.00 | 29.16 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 3.67e-11 | 0 |
| 175–199 | 2328.2 | 2439.6 | 4.20 | 30.36 | 0.20 | 1.00 | 233.52 | 0.004 | 0.000 | 1.23e-09 | 1.62e-11 | 0 |
| 200–224 | 2118.0 | 2763.1 | 7.44 | 53.60 | 3.64 | 1.00 | 222.00 | 0.018 | 0.033 | 1.08e-09 | 1.11e-11 | 0 |
| 225–249 | 1885.6 | 2743.4 | 8.64 | 55.20 | 6.76 | 1.00 | 222.00 | 1.277 | 0.361 | 8.63e-10 | 7.73e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 250 | 0.404 | 0.115 | 6.666 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
