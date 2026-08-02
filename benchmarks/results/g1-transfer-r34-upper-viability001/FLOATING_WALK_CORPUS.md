# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `dcm-backward-preview` `DCM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root horizontal task weight `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `viability` priority with weight `0.010` over 11 waist/arm coordinates.
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
| 490 | 2.450 s | 12.408 cm | 7.969 cm | 26.043 cm | 22.264 cm | 48.695° | 8.000 rad/s | 52911.5 µs |

Nominal hard residual maxima: dynamics `1.474e-09`, contact acceleration `8.021e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 55.543 cm |
| authored reference vs measured CoM RMS / p95 | 51.542 / 158.837 cm |
| stance foot RMS | 35.037 cm |
| swing foot RMS | 53.338 cm |
| hand RMS | 71.900 cm |
| maximum root rotation | 178.940° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.676e-09 |
| contact acceleration residual | 1.272e-10 |
| raw max dynamics residual, including rejected ticks | 6.887e+02 |
| raw max contact residual, including rejected ticks | 1.272e-10 |
| active normal force range | 0.000–668.815 N |
| centroidal momentum-rate residual RMS / max | 56.334 / 528.923 N·m |
| point-task acceleration RMS max | 173.601 m/s² |
| frame-angular acceleration RMS max | 383.066 rad/s² |
| longest pre-contact / touchdown transition | 262 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `12.219` / `25.877 cm`.
- Virtual ZMP clipped on `61.00%` of ticks; clip-distance RMS / max `38.419` / `87.219 cm`.
- Measured-height natural frequency min / p50 / max: `3.686` / `3.768` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.895 m`; height-floor ticks: `96`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2478.8 µs | 237141.8 µs | 255671.8 µs | 259060.5 µs | 201 | 27 | 262 | 0 | 45 | 35 | 30 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 27101.5 | 66876.4 | 370.3 | 104916.1 | 258366.6 | 258991.1 | 100411.8 | 600 | 135 | 82 | 36.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 201 | 2469.8 | 2563.8 | 2596.1 | 2686.2 |
| solved_with_slack | 27 | 1773.5 | 2208.7 | 2238.8 | 2241.1 |
| primal_infeasible | 30 | 255136.6 | 257876.7 | 258724.6 | 259060.5 |
| normal_contact_contingency | 45 | 13528.4 | 40363.2 | 150412.8 | 236255.9 |
| contact_release_contingency | 35 | 182307.3 | 215134.7 | 227937.8 | 233490.9 |
| precontact_transition | 262 | 2338.1 | 15839.6 | 55068.7 | 59048.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.58 | 11.0 | 12.0 | 14 | 3.10 | 10.0 | 12 | -0.4253 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 420.96/3717.0/6720.0/6720 | 89319.38/821153.1/1411200.0/1411200 | 3.00/37.0/37 | 2.80/36.0/36 | 24.18/312.0/312 | 0.7745 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 201 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 27 | 6.93/11.0/11 | 2.81/6.7/7 |
| primal_infeasible | 30 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 45 | 9.67/12.6/13 | 7.93/10.6/11 |
| contact_release_contingency | 35 | 7.43/10.0/10 | 4.46/7.7/8 |
| precontact_transition | 262 | 7.87/12.4/14 | 4.85/10.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.77/6.0/7 | 7.90/27.0/35 | 1.22/6.0/7 | 243 |
| viability | 1.66/6.0/9 | 11.73/48.0/64 | 1.26/6.0/9 | 341 |
| intent | 1.03/3.0/6 | 4.07/12.0/24 | 0.15/3.0/5 | 52 |
| preference | 1.01/3.0/7 | 4.44/15.0/35 | 0.09/3.0/6 | 20 |
| style | 1.10/4.0/7 | 8.70/28.0/41 | 0.39/4.0/6 | 162 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 490 | 110 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `113` ticks.
Precontact sole-center tangential speed: p50 `2.2489 m/s`, p95 `7.5684 m/s`, max `12.3368 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 16.261 | 16.252 | 16.251 | 0.999 | 0.999 | 47.215 | 47.512 | 0.297 | 48.504 | 0.001 | 0 | 59 | 0 | 0 | 805 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2479.9 | 2642.6 | 5.00 | 29.67 | 0.00 | 1.00 | 234.00 | 0.271 | 0.000 | 1.47e-09 | 4.76e-11 | 0 |
| 60–119 | 2474.7 | 2563.1 | 5.00 | 30.28 | 0.00 | 1.00 | 234.00 | 1.124 | 0.000 | 1.26e-09 | 4.54e-11 | 0 |
| 120–179 | 2460.4 | 2567.4 | 5.00 | 30.07 | 0.00 | 1.00 | 234.00 | 3.203 | 0.000 | 1.39e-09 | 8.02e-11 | 0 |
| 180–239 | 2109.8 | 3014.1 | 6.32 | 35.05 | 1.95 | 1.00 | 225.80 | 7.964 | 0.802 | 6.98e-10 | 3.33e-11 | 12 |
| 240–299 | 2494.8 | 3465.9 | 7.57 | 43.40 | 3.87 | 1.00 | 222.00 | 6.105 | 15.874 | 1.32e-09 | 1.67e-11 | 60 |
| 300–359 | 2448.1 | 3971.3 | 7.97 | 45.20 | 4.82 | 1.00 | 222.00 | 7.548 | 30.562 | 8.36e-10 | 1.45e-11 | 60 |
| 360–419 | 1948.6 | 8663.9 | 7.82 | 41.82 | 5.23 | 14.43 | 3204.20 | 14.157 | 20.449 | 9.16e-10 | 1.29e-11 | 60 |
| 420–479 | 2163.5 | 57834.7 | 8.10 | 44.00 | 5.48 | 655.95 | 145620.90 | 23.772 | 15.723 | 7.07e-10 | 1.63e-11 | 60 |
| 480–539 | 13619.1 | 223143.5 | 9.22 | 51.47 | 7.28 | 169.22 | 36554.50 | 78.754 | 40.942 | 2.92e-09 | 6.31e-11 | 60 |
| 540–599 | 243732.9 | 258377.1 | 3.82 | 17.48 | 2.38 | 3364.00 | 706442.40 | 153.993 | 118.737 | 6.89e+02 | 1.27e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 55.543 | 41.984 | 71.900 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
