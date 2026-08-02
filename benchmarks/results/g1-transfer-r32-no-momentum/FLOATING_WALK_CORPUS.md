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
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
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
| 391 | 1.955 s | 11.754 cm | 12.866 cm | 39.039 cm | 27.668 cm | 88.043° | 8.000 rad/s | 59776.0 µs |

Nominal hard residual maxima: dynamics `1.610e-09`, contact acceleration `6.563e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 107.820 cm |
| CoM RMS / p95 | 110.293 / 237.801 cm |
| stance foot RMS | 103.343 cm |
| swing foot RMS | 109.785 cm |
| hand RMS | 110.873 cm |
| maximum root rotation | 178.790° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.950e-09 |
| contact acceleration residual | 3.172e-10 |
| raw max dynamics residual, including rejected ticks | 8.950e-09 |
| raw max contact residual, including rejected ticks | 3.172e-10 |
| active normal force range | 0.000–930.583 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 104.624 m/s² |
| frame-angular acceleration RMS max | 260.047 rad/s² |
| longest pre-contact / touchdown transition | 163 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2659.6 µs | 111678.6 µs | 188682.0 µs | 209548.5 µs | 147 | 81 | 163 | 0 | 166 | 43 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 14666.2 | 40809.5 | 402.6 | 6180.7 | 209275.6 | 209521.2 | 95051.1 | 600 | 70 | 55 | 68.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 147 | 2429.0 | 2511.2 | 2661.4 | 2842.9 |
| solved_with_slack | 81 | 2823.8 | 5072.9 | 5198.2 | 5212.0 |
| normal_contact_contingency | 166 | 2780.8 | 4364.5 | 6723.2 | 11214.2 |
| contact_release_contingency | 43 | 162664.9 | 206294.4 | 209357.2 | 209548.5 |
| precontact_transition | 163 | 2992.9 | 38995.7 | 79317.0 | 102434.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.00 | 13.0 | 14.0 | 16 | 5.71 | 13.0 | 15 | -0.0340 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 74.23/8.0/2844.3/6505 | 16469.22/1776.0/631441.3/1444110 | 1.12/6.0/13 | 0.68/5.0/12 | 5.47/42.0/99 | 0.1612 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 147 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 81 | 10.59/16.0/16 | 8.70/14.2/15 |
| normal_contact_contingency | 166 | 8.89/14.0/14 | 7.72/12.3/14 |
| contact_release_contingency | 43 | 7.51/9.6/10 | 6.33/8.6/9 |
| precontact_transition | 163 | 8.64/13.4/14 | 7.18/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.12/7.0/10 | 8.83/32.0/45 | 1.69/7.0/10 | 390 |
| viability | 1.86/7.0/9 | 10.88/35.0/52 | 1.59/7.0/9 | 442 |
| intent | 1.54/6.0/7 | 3.08/12.0/18 | 1.21/6.0/6 | 415 |
| preference | 1.47/6.0/8 | 12.92/48.0/65 | 1.01/5.0/7 | 374 |
| style | 1.01/1.0/2 | 8.51/12.0/23 | 0.21/1.0/2 | 125 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 391 | 209 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `212` ticks.
Precontact sole-center tangential speed: p50 `2.6227 m/s`, p95 `5.8469 m/s`, max `8.7264 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.800 | 8.798 | 8.798 | 1.000 | 1.000 | 47.656 | 47.887 | 0.230 | 47.961 | 0.001 | 0 | 43 | 0 | 0 | 143 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2449.5 | 2792.5 | 6.33 | 32.98 | 1.78 | 1.00 | 234.00 | 0.146 | 0.000 | 1.61e-09 | 6.56e-11 | 0 |
| 60–119 | 2422.6 | 2511.7 | 5.00 | 28.97 | 0.00 | 1.00 | 234.00 | 0.025 | 0.000 | 1.03e-09 | 3.73e-11 | 0 |
| 120–179 | 2473.1 | 3275.9 | 7.47 | 37.65 | 3.40 | 1.00 | 234.00 | 0.096 | 0.000 | 1.27e-09 | 3.95e-11 | 0 |
| 180–239 | 3601.7 | 5201.8 | 9.32 | 49.33 | 7.70 | 6.48 | 1451.50 | 3.475 | 2.627 | 1.16e-09 | 2.11e-11 | 12 |
| 240–299 | 3090.7 | 3663.2 | 8.93 | 49.37 | 7.12 | 6.48 | 1439.30 | 9.102 | 12.627 | 7.11e-10 | 8.47e-12 | 60 |
| 300–359 | 1998.1 | 12939.5 | 8.43 | 47.80 | 7.40 | 27.67 | 6142.00 | 15.679 | 28.716 | 1.40e-09 | 2.31e-11 | 60 |
| 360–419 | 40837.3 | 198579.7 | 8.38 | 44.92 | 7.17 | 681.23 | 151200.00 | 46.134 | 90.552 | 7.49e-09 | 1.92e-10 | 60 |
| 420–479 | 2881.9 | 207565.0 | 8.28 | 47.37 | 7.02 | 4.73 | 1007.00 | 124.331 | 121.771 | 8.16e-09 | 1.11e-10 | 60 |
| 480–539 | 2886.4 | 5984.1 | 8.75 | 51.83 | 7.57 | 7.18 | 1551.60 | 207.629 | 166.242 | 8.95e-09 | 3.17e-10 | 60 |
| 540–599 | 2700.2 | 3298.7 | 9.10 | 52.05 | 8.00 | 5.55 | 1198.80 | 234.977 | 244.305 | 4.03e-10 | 6.07e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 107.820 | 105.518 | 110.873 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
