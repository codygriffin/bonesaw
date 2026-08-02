# Bonesaw floating G1 synthetic-step acceptance

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: deterministic, predeclared G1-sized left step; no mocap or learned policy contributes contact labels or target motion.
- Contact schedule: liftoff tick `60`, touchdown tick `100`, step `0.040 m`, clearance `0.030 m`.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.040 m` forward per `0.900 s` source cycle.
- Applied mean forward speed: `0.044 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `constant synthetic phase rate` in `0.9 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `rooted` reference at `viability` priority with weight `0.000` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `40` ticks (`0.200 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
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
| 180 | 0.900 s | 4.247 cm | 0.001 cm | 1.922 cm | 25.869 cm | 0.430° | 8.000 rad/s | 3964.5 µs |

Nominal hard residual maxima: dynamics `1.532e-09`, contact acceleration `5.664e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 4.247 cm |
| stance foot RMS | 0.001 cm |
| swing foot RMS | 1.922 cm |
| hand RMS | 25.869 cm |
| maximum root rotation | 0.430° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.532e-09 |
| contact acceleration residual | 5.664e-11 |
| raw max dynamics residual, including rejected ticks | 1.532e-09 |
| raw max contact residual, including rejected ticks | 5.664e-11 |
| active normal force range | 0.000–277.697 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 0.061 m/s² |
| frame-angular acceleration RMS max | 0.026 rad/s² |
| longest pre-contact / touchdown transition | 40 / 3 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 180 | 0.9 s | 2342.5 µs | 3571.4 µs | 3964.5 µs | 4551.3 µs | 60 | 77 | 40 | 3 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2517.6 | 519.0 | 222.3 | 3375.1 | 4504.0 | 4546.6 | 1824.8 | 180 | 0 | 0 | 397.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 60 | 2333.0 | 2939.4 | 3208.8 | 3370.9 |
| solved_with_slack | 77 | 2726.7 | 3784.7 | 4350.6 | 4551.3 |
| touchdown_transition | 3 | 2303.5 | 2609.6 | 2636.8 | 2643.6 |
| precontact_transition | 40 | 1918.6 | 2306.8 | 2399.9 | 2449.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.47 | 13.0 | 15.2 | 22 | 4.53 | 13.4 | 20 | 0.5625 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.00/1.0/1.0/1 | 231.23/234.0/234.0/234 | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0.5696 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 60 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 77 | 9.84/17.4/22 | 8.13/16.2/20 |
| touchdown_transition | 3 | 9.33/11.0/11 | 6.67/8.0/8 |
| precontact_transition | 40 | 7.95/10.6/11 | 4.22/7.6/8 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.24/5.2/9 | 4.64/20.8/36 | 0.31/5.2/9 | 15 |
| viability | 0.24/1.0/1 | 1.16/5.0/5 | 0.00/0.0/0 | 0 |
| intent | 2.14/7.2/9 | 4.28/14.4/18 | 1.62/7.2/9 | 89 |
| preference | 2.76/9.2/16 | 18.15/64.7/107 | 2.37/9.2/15 | 120 |
| style | 1.08/2.0/3 | 10.95/22.0/33 | 0.23/2.0/2 | 37 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 40 | 3 | 137 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 180 | 0 |
| left_wrist_roll_rubber_hand | 180 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 180 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `40` ticks, planned normal touchdown `3` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.2251 m/s`, p95 `0.3005 m/s`, max `0.3018 m/s` over 40 samples.
Touchdown Normal sole-center tangential speed: p50 `0.0000 m/s`, p95 `0.0000 m/s`, max `0.0000 m/s` over 3 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.453 | 0.453 | 0.453 | 1.000 | 1.000 | 41.895 | 42.109 | 0.215 | 42.109 | 0.001 | 0 | 54 | 0 | 0 | 4 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–17 | 2340.1 | 3086.1 | 4.00 | 23.89 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 18–35 | 2326.8 | 2344.3 | 4.00 | 23.89 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 36–53 | 2335.5 | 3296.7 | 4.00 | 23.89 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 54–71 | 2229.7 | 2429.6 | 6.89 | 40.06 | 3.00 | 1.00 | 226.00 | 0.000 | 0.599 | 1.21e-09 | 3.03e-11 | 12 |
| 72–89 | 1918.6 | 2291.1 | 7.67 | 43.61 | 3.72 | 1.00 | 222.00 | 0.001 | 1.872 | 1.14e-09 | 1.13e-11 | 18 |
| 90–107 | 2055.2 | 2974.3 | 8.39 | 39.94 | 5.67 | 1.00 | 226.33 | 0.559 | 0.491 | 8.13e-10 | 3.79e-11 | 10 |
| 108–125 | 2739.4 | 3485.1 | 8.83 | 44.50 | 6.94 | 1.00 | 234.00 | 3.007 | 0.001 | 6.34e-10 | 5.20e-11 | 0 |
| 126–143 | 2735.6 | 3845.8 | 9.50 | 48.33 | 7.72 | 1.00 | 234.00 | 5.595 | 0.000 | 5.52e-10 | 2.72e-11 | 0 |
| 144–161 | 2940.8 | 4506.4 | 11.06 | 54.39 | 8.89 | 1.00 | 234.00 | 7.328 | 0.001 | 7.04e-10 | 5.66e-11 | 0 |
| 162–179 | 2697.2 | 3598.3 | 10.33 | 49.39 | 9.33 | 1.00 | 234.00 | 9.274 | 0.004 | 1.53e-09 | 3.15e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 180 | 4.247 | 0.641 | 25.869 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
