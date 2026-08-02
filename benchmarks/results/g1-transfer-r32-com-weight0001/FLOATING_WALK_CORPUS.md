# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.001` and `2.000 Hz` response. Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| `no_contact_contingency_ticks` | FAIL |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 422 | 2.110 s | 17.059 cm | 13.770 cm | 59.674 cm | 34.788 cm | 46.214° | 8.000 rad/s | 65735.7 µs |

Nominal hard residual maxima: dynamics `1.720e-09`, contact acceleration `9.123e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 41.177 cm |
| CoM RMS / p95 | 49.435 / 96.970 cm |
| stance foot RMS | 39.088 cm |
| swing foot RMS | 63.045 cm |
| hand RMS | 58.828 cm |
| maximum root rotation | 46.214° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.720e-09 |
| contact acceleration residual | 9.123e-11 |
| raw max dynamics residual, including rejected ticks | 1.720e-09 |
| raw max contact residual, including rejected ticks | 9.123e-11 |
| active normal force range | 0.000–401.514 N |
| centroidal momentum-rate residual RMS / max | 32.261 / 114.200 N·m |
| point-task acceleration RMS max | 65.695 m/s² |
| frame-angular acceleration RMS max | 136.694 rad/s² |
| longest pre-contact / touchdown transition | 194 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2588.2 µs | 34767.3 µs | 66149.5 µs | 190263.2 µs | 129 | 99 | 194 | 0 | 178 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6043.7 | 13969.9 | 541.7 | 5000.9 | 120660.1 | 183302.9 | 43980.1 | 600 | 60 | 40 | 165.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2432.1 | 2534.7 | 2564.4 | 2565.5 |
| solved_with_slack | 99 | 2823.8 | 4807.4 | 5729.6 | 6043.1 |
| normal_contact_contingency | 178 | 2757.2 | 31820.5 | 56753.7 | 190263.2 |
| precontact_transition | 194 | 3081.3 | 53279.9 | 72576.2 | 74064.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.66 | 11.0 | 14.0 | 24 | 5.39 | 12.0 | 21 | 0.1185 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 201.83/1995.3/3988.2/4616 | 44394.38/430995.6/885384.8/1024752 | 0.94/6.0/10 | 0.61/5.0/9 | 4.95/41.0/79 | 0.8339 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 8.72/18.1/24 | 6.22/15.1/21 |
| normal_contact_contingency | 178 | 8.33/12.0/14 | 6.70/12.0/13 |
| precontact_transition | 194 | 8.27/12.0/14 | 7.35/11.1/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.75/5.0/7 | 6.87/20.0/28 | 1.16/5.0/7 | 308 |
| viability | 2.21/7.0/13 | 13.57/48.0/64 | 1.96/7.0/13 | 452 |
| intent | 1.41/4.0/4 | 7.48/20.0/24 | 1.15/4.0/4 | 439 |
| preference | 1.22/4.0/17 | 11.81/40.0/187 | 0.76/4.0/16 | 336 |
| style | 1.07/2.0/6 | 10.19/27.0/45 | 0.37/2.0/6 | 194 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 422 | 178 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `181` ticks.
Precontact sole-center tangential speed: p50 `2.0594 m/s`, p95 `5.4403 m/s`, max `5.6369 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.626 | 3.626 | 3.626 | 1.000 | 1.000 | 46.695 | 46.945 | 0.250 | 47.598 | 0.001 | 0 | 43 | 0 | 0 | 44 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2512.7 | 5147.6 | 6.68 | 42.37 | 2.50 | 1.00 | 234.00 | 0.170 | 0.000 | 1.55e-09 | 4.77e-11 | 0 |
| 60–119 | 2421.0 | 4519.6 | 5.23 | 37.87 | 0.35 | 1.00 | 234.00 | 0.026 | 0.000 | 1.09e-09 | 4.13e-11 | 0 |
| 120–179 | 2497.0 | 5037.8 | 6.22 | 41.37 | 1.88 | 1.12 | 261.30 | 0.002 | 0.000 | 1.72e-09 | 5.29e-11 | 0 |
| 180–239 | 3025.7 | 5203.9 | 8.53 | 55.55 | 6.75 | 4.70 | 1058.00 | 3.796 | 3.377 | 1.41e-09 | 3.81e-11 | 12 |
| 240–299 | 1965.7 | 4466.5 | 8.28 | 51.82 | 6.97 | 3.68 | 817.70 | 11.378 | 23.202 | 6.60e-10 | 1.76e-11 | 60 |
| 300–359 | 3102.7 | 3826.7 | 8.05 | 50.83 | 7.33 | 6.48 | 1439.30 | 22.120 | 45.608 | 8.21e-10 | 2.14e-11 | 60 |
| 360–419 | 3253.8 | 73663.5 | 8.50 | 52.88 | 7.85 | 1303.72 | 289425.10 | 36.816 | 69.125 | 1.37e-09 | 9.12e-11 | 60 |
| 420–479 | 2989.3 | 93406.6 | 8.45 | 53.85 | 6.97 | 170.70 | 36872.80 | 53.185 | 61.674 | 1.58e-09 | 7.40e-11 | 60 |
| 480–539 | 1926.5 | 59099.0 | 8.13 | 56.10 | 6.17 | 314.70 | 67975.20 | 71.690 | 58.458 | 4.62e-10 | 7.59e-12 | 60 |
| 540–599 | 2541.6 | 37327.0 | 8.50 | 56.55 | 7.12 | 211.23 | 45626.40 | 83.657 | 93.510 | 5.44e-10 | 1.56e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 41.177 | 48.346 | 58.828 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
