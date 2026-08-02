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
| 313 | 1.565 s | 25.060 cm | 22.516 cm | 87.187 cm | 35.356 cm | 106.804° | 8.000 rad/s | 47251.5 µs |

Nominal hard residual maxima: dynamics `7.829e-09`, contact acceleration `2.321e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 172.494 cm |
| stance foot RMS | 148.106 cm |
| swing foot RMS | 203.034 cm |
| hand RMS | 176.549 cm |
| maximum root rotation | 179.704° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.829e-09 |
| contact acceleration residual | 2.321e-10 |
| raw max dynamics residual, including rejected ticks | 7.829e-09 |
| raw max contact residual, including rejected ticks | 2.321e-10 |
| active normal force range | 0.000–819.987 N |
| centroidal momentum-rate residual RMS / max | 61.275 / 301.020 N·m |
| point-task acceleration RMS max | 148.008 m/s² |
| frame-angular acceleration RMS max | 240.079 rad/s² |
| longest pre-contact / touchdown transition | 85 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3459.3 µs | 161763.3 µs | 205861.3 µs | 278548.2 µs | 36 | 192 | 85 | 0 | 241 | 46 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 18490.0 | 46131.7 | 1011.4 | 17245.6 | 238793.7 | 274572.7 | 182512.5 | 600 | 185 | 57 | 54.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 36 | 2431.4 | 2524.0 | 2544.9 | 2551.2 |
| solved_with_slack | 192 | 2695.0 | 5357.4 | 5983.8 | 6912.3 |
| normal_contact_contingency | 241 | 4384.6 | 16425.1 | 21060.4 | 202661.6 |
| contact_release_contingency | 46 | 176352.7 | 209254.5 | 248682.6 | 278548.2 |
| precontact_transition | 85 | 3056.7 | 46849.6 | 48786.8 | 54527.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 9.06 | 14.0 | 16.0 | 25 | 7.71 | 15.0 | 22 | -0.0821 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 34.00/8.0/35.3/3326 | 7532.03/1872.0/7933.9/738372 | 3.57/16.0/26 | 3.00/15.0/25 | 25.15/139.1/227 | 0.0677 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 36 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 192 | 9.53/18.5/25 | 7.98/16.2/22 |
| normal_contact_contingency | 241 | 9.63/15.0/18 | 8.80/14.0/17 |
| contact_release_contingency | 46 | 7.76/10.5/11 | 6.37/9.5/10 |
| precontact_transition | 85 | 8.78/13.2/14 | 8.05/13.2/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.54/7.0/9 | 11.04/32.0/40 | 2.28/7.0/8 | 461 |
| viability | 2.16/11.0/13 | 12.50/40.0/76 | 2.04/10.0/12 | 536 |
| intent | 1.53/4.0/13 | 7.56/24.0/63 | 1.44/4.0/13 | 543 |
| preference | 1.54/8.0/19 | 15.46/84.0/190 | 1.28/8.0/18 | 464 |
| style | 1.29/5.0/7 | 10.54/35.0/48 | 0.67/4.0/6 | 281 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 313 | 287 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `290` ticks.
Precontact sole-center tangential speed: p50 `3.5767 m/s`, p95 `7.0923 m/s`, max `7.7637 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11.094 | 11.083 | 11.083 | 0.999 | 0.999 | 46.742 | 46.895 | 0.152 | 48.012 | 0.001 | 0 | 38 | 0 | 0 | 832 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2619.2 | 6169.4 | 7.63 | 47.18 | 4.78 | 2.75 | 643.50 | 0.207 | 0.000 | 1.62e-09 | 5.44e-11 | 0 |
| 60–119 | 2548.2 | 4672.8 | 7.95 | 49.95 | 4.87 | 1.23 | 288.60 | 0.860 | 0.000 | 1.53e-09 | 7.18e-11 | 0 |
| 120–179 | 2834.2 | 6174.3 | 10.28 | 62.90 | 8.80 | 2.63 | 616.20 | 8.061 | 1.195 | 1.31e-09 | 4.81e-11 | 0 |
| 180–239 | 1992.3 | 4290.8 | 9.35 | 58.62 | 8.62 | 1.35 | 303.50 | 16.928 | 15.395 | 1.15e-09 | 3.78e-11 | 12 |
| 240–299 | 3044.6 | 50495.3 | 8.85 | 57.05 | 8.18 | 294.05 | 65279.10 | 43.536 | 79.593 | 1.05e-09 | 2.47e-11 | 60 |
| 300–359 | 12081.0 | 203972.3 | 9.55 | 58.17 | 8.60 | 8.00 | 1729.60 | 104.522 | 129.349 | 7.83e-09 | 2.32e-10 | 60 |
| 360–419 | 2883.6 | 4921.1 | 7.98 | 51.10 | 7.08 | 6.72 | 1450.80 | 168.614 | 163.579 | 1.99e-09 | 6.92e-11 | 60 |
| 420–479 | 15813.3 | 209214.1 | 9.77 | 61.78 | 8.82 | 8.00 | 1712.80 | 214.912 | 190.881 | 4.33e-09 | 1.38e-10 | 60 |
| 480–539 | 13826.4 | 239391.1 | 10.07 | 63.93 | 9.18 | 8.00 | 1720.80 | 268.360 | 278.157 | 2.89e-09 | 1.67e-10 | 60 |
| 540–599 | 3958.3 | 171098.3 | 9.15 | 60.32 | 8.22 | 7.30 | 1575.40 | 371.128 | 345.390 | 3.45e-09 | 1.35e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 172.494 | 168.275 | 176.549 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
