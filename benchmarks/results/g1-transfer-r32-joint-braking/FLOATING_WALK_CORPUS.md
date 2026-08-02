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
| `no_infeasible_or_failed_ticks` | FAIL |
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

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 284 | 1.420 s | 4.149 cm | 0.239 cm | 19.327 cm | 27.279 cm | 9.535° | 8.000 rad/s | 5146.3 µs |

Nominal hard residual maxima: dynamics `1.622e-09`, contact acceleration `6.301e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 148.296 cm |
| CoM RMS / p95 | 141.822 / 372.244 cm |
| stance foot RMS | 104.530 cm |
| swing foot RMS | 188.165 cm |
| hand RMS | 161.719 cm |
| maximum root rotation | 179.936° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 5.140e-09 |
| contact acceleration residual | 2.578e-10 |
| raw max dynamics residual, including rejected ticks | 7.497e+02 |
| raw max contact residual, including rejected ticks | 2.578e-10 |
| active normal force range | 0.000–892.541 N |
| centroidal momentum-rate residual RMS / max | 62.702 / 528.621 N·m |
| point-task acceleration RMS max | 189.225 m/s² |
| frame-angular acceleration RMS max | 394.004 rad/s² |
| longest pre-contact / touchdown transition | 56 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3018.5 µs | 210562.6 µs | 265664.9 µs | 271095.2 µs | 141 | 87 | 56 | 0 | 220 | 72 | 24 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 34001.6 | 72142.3 | 618.8 | 169548.3 | 270598.3 | 271045.5 | 181500.8 | 600 | 167 | 99 | 29.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2460.2 | 2690.6 | 2787.3 | 2820.9 |
| solved_with_slack | 87 | 2770.5 | 4901.3 | 5457.6 | 6051.5 |
| primal_infeasible | 24 | 265166.7 | 269744.9 | 270904.4 | 271095.2 |
| normal_contact_contingency | 220 | 3557.7 | 14782.0 | 26744.5 | 202001.7 |
| contact_release_contingency | 72 | 166267.5 | 214459.2 | 227845.7 | 258303.4 |
| precontact_transition | 56 | 3277.8 | 4435.2 | 4879.6 | 5122.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.18 | 11.0 | 14.0 | 16 | 5.08 | 13.0 | 14 | -0.3463 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 280.83/8.0/6720.0/6720 | 59054.77/1872.0/1411200.0/1411200 | 3.74/36.0/36 | 3.17/35.0/35 | 27.03/312.0/312 | 0.6531 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 87 | 8.21/12.6/16 | 5.89/11.3/13 |
| primal_infeasible | 24 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 220 | 8.64/14.0/15 | 7.76/13.0/14 |
| contact_release_contingency | 72 | 7.69/11.0/11 | 6.50/9.3/10 |
| precontact_transition | 56 | 7.80/12.4/14 | 6.48/11.8/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.95/5.0/6 | 7.70/23.0/29 | 1.50/5.0/6 | 342 |
| viability | 1.80/7.0/12 | 10.94/37.0/49 | 1.55/7.0/12 | 428 |
| intent | 1.28/4.0/7 | 5.94/24.0/35 | 1.00/4.0/7 | 409 |
| preference | 1.14/6.0/9 | 11.01/48.0/88 | 0.73/6.0/8 | 340 |
| style | 1.02/3.0/5 | 8.69/27.0/41 | 0.31/2.0/4 | 161 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 284 | 316 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `319` ticks.
Precontact sole-center tangential speed: p50 `3.3398 m/s`, p95 `6.2899 m/s`, max `6.4699 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20.401 | 20.396 | 20.396 | 1.000 | 1.000 | 47.684 | 47.918 | 0.234 | 47.973 | 0.001 | 0 | 44 | 0 | 0 | 335 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2482.1 | 2933.6 | 6.07 | 36.82 | 1.67 | 1.00 | 234.00 | 0.172 | 0.000 | 1.62e-09 | 5.63e-11 | 0 |
| 60–119 | 2454.0 | 3519.4 | 5.10 | 34.98 | 0.17 | 1.00 | 234.00 | 0.032 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–179 | 2520.6 | 5302.1 | 6.12 | 39.43 | 1.62 | 1.12 | 261.30 | 0.002 | 0.000 | 1.46e-09 | 6.30e-11 | 0 |
| 180–239 | 2779.7 | 5383.2 | 7.92 | 51.18 | 6.22 | 3.92 | 883.10 | 3.395 | 3.486 | 1.47e-09 | 5.34e-11 | 12 |
| 240–299 | 3344.0 | 85843.1 | 7.82 | 51.03 | 6.63 | 6.25 | 1374.70 | 10.816 | 21.571 | 3.87e-09 | 1.25e-10 | 60 |
| 300–359 | 2846.3 | 4485.4 | 7.93 | 50.48 | 6.88 | 6.95 | 1501.20 | 17.392 | 40.971 | 2.70e-09 | 5.36e-11 | 60 |
| 360–419 | 3407.4 | 8662.0 | 8.33 | 52.25 | 7.42 | 7.88 | 1702.80 | 42.541 | 36.147 | 1.10e-09 | 1.89e-11 | 60 |
| 420–479 | 5077.4 | 191581.9 | 8.83 | 52.32 | 8.15 | 8.00 | 1725.60 | 125.517 | 84.527 | 5.14e-09 | 2.58e-10 | 60 |
| 480–539 | 142139.6 | 214873.6 | 8.92 | 49.62 | 7.97 | 81.87 | 17655.10 | 250.206 | 210.195 | 4.76e-09 | 1.88e-10 | 60 |
| 540–599 | 209319.6 | 270605.8 | 4.82 | 24.68 | 4.13 | 2690.35 | 564975.90 | 373.254 | 368.052 | 7.50e+02 | 2.40e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 148.296 | 137.932 | 161.719 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
