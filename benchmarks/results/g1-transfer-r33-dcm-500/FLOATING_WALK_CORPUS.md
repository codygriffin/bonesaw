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
| 406 | 2.030 s | 13.950 cm | 13.882 cm | 50.606 cm | 29.663 cm | 63.110° | 8.000 rad/s | 47476.4 µs |

Nominal hard residual maxima: dynamics `3.493e-09`, contact acceleration `2.692e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 30.226 cm |
| CoM RMS / p95 | 35.204 / 79.882 cm |
| stance foot RMS | 39.488 cm |
| swing foot RMS | 64.412 cm |
| hand RMS | 43.081 cm |
| maximum root rotation | 130.254° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.493e-09 |
| contact acceleration residual | 2.692e-10 |
| raw max dynamics residual, including rejected ticks | 3.493e-09 |
| raw max contact residual, including rejected ticks | 2.692e-10 |
| active normal force range | 0.000–607.773 N |
| centroidal momentum-rate residual RMS / max | 24.838 / 71.269 N·m |
| point-task acceleration RMS max | 84.575 m/s² |
| frame-angular acceleration RMS max | 193.534 rad/s² |
| longest pre-contact / touchdown transition | 178 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 72 / 72 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `31.784` / `69.094 cm`.
- Virtual ZMP clipped on `57.20%` of ticks; clip-distance RMS / max `54.076` / `167.839 cm`.
- Measured-height natural frequency min / p50 / max: `3.706` / `3.784` / `7.004 rad/s`.
- CoM command acceleration p95 / max: `80.405` / `93.362 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 500 | 2.5 s | 2613.9 µs | 47501.5 µs | 79592.9 µs | 179474.1 µs | 158 | 70 | 178 | 0 | 94 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7388.0 | 16945.4 | 463.4 | 10327.9 | 130149.7 | 174541.6 | 35499.6 | 500 | 76 | 32 | 135.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 158 | 2452.7 | 2533.7 | 2562.6 | 2590.6 |
| solved_with_slack | 70 | 2606.5 | 3685.8 | 4345.6 | 4879.6 |
| normal_contact_contingency | 94 | 9826.4 | 79772.2 | 87546.9 | 179474.1 |
| precontact_transition | 178 | 3090.6 | 6308.2 | 48119.9 | 49163.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.86 | 12.0 | 14.0 | 20 | 5.41 | 13.0 | 16 | 0.2800 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 253.16/2865.2/5118.3/5232 | 54938.18/636045.3/1105555.0/1130112 | 1.66/13.0/14 | 1.26/12.0/13 | 10.51/100.0/114 | 0.8771 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 158 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 70 | 9.03/17.9/20 | 6.56/14.6/16 |
| normal_contact_contingency | 94 | 10.50/13.1/15 | 9.81/13.1/14 |
| precontact_transition | 178 | 8.54/13.2/14 | 7.45/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.93/6.0/8 | 7.99/25.0/35 | 1.40/6.0/8 | 262 |
| viability | 1.94/8.0/16 | 9.14/36.0/48 | 1.54/8.0/15 | 325 |
| intent | 1.49/6.0/7 | 7.53/30.0/36 | 1.17/6.0/7 | 342 |
| preference | 1.41/6.0/6 | 14.18/57.0/66 | 0.94/6.0/6 | 281 |
| style | 1.10/3.0/6 | 9.70/24.0/44 | 0.37/3.0/5 | 150 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 272 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 406 | 94 |
| left_wrist_roll_rubber_hand | 500 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 500 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `272` ticks, planned normal touchdown `0` ticks, normal fallback `97` ticks.
Precontact sole-center tangential speed: p50 `2.0532 m/s`, p95 `5.9121 m/s`, max `7.7296 m/s` over 272 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.694 | 3.693 | 3.693 | 1.000 | 1.000 | 47.871 | 48.246 | 0.375 | 48.289 | 0.001 | 0 | 80 | 0 | 0 | 63 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–49 | 2462.5 | 2581.6 | 5.00 | 32.56 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.88e-09 | 5.86e-11 | 0 |
| 50–99 | 2462.7 | 2551.6 | 5.00 | 31.86 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.43e-09 | 5.70e-11 | 0 |
| 100–149 | 2425.6 | 2528.0 | 5.00 | 31.36 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.25e-09 | 4.22e-11 | 0 |
| 150–199 | 2546.9 | 4500.4 | 9.36 | 45.96 | 6.52 | 1.00 | 233.76 | 0.981 | 0.000 | 1.90e-09 | 5.65e-11 | 0 |
| 200–249 | 3029.6 | 3789.3 | 7.50 | 44.72 | 5.38 | 6.74 | 1496.28 | 4.438 | 1.479 | 1.41e-09 | 1.30e-11 | 22 |
| 250–299 | 3033.0 | 21891.7 | 8.50 | 52.88 | 7.04 | 50.06 | 11113.32 | 9.356 | 13.884 | 7.09e-10 | 2.05e-11 | 50 |
| 300–349 | 3133.3 | 4392.4 | 8.62 | 52.72 | 7.90 | 6.60 | 1465.20 | 14.885 | 34.101 | 7.70e-10 | 1.01e-11 | 50 |
| 350–399 | 3009.6 | 48920.8 | 8.66 | 52.94 | 7.78 | 349.00 | 77478.00 | 31.094 | 63.712 | 2.18e-09 | 1.01e-11 | 50 |
| 400–449 | 8681.8 | 101446.8 | 10.46 | 68.52 | 9.68 | 54.02 | 11674.08 | 51.794 | 106.610 | 3.49e-09 | 2.69e-10 | 50 |
| 450–499 | 12131.5 | 80624.9 | 10.48 | 71.80 | 9.84 | 2061.20 | 445219.20 | 71.810 | 80.634 | 1.81e-09 | 2.32e-11 | 50 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 500 | 30.226 | 48.254 | 43.081 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
