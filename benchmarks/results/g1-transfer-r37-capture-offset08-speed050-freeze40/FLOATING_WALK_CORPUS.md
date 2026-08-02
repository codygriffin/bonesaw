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
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `enabled`; authored offset ≤ `0.080 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `40` scheduled ticks and throughout delayed admission.
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
| 482 | 2.410 s | 14.095 cm | 13.861 cm | 25.241 cm | 40.660 cm | 47.149° | 8.000 rad/s | 50226.5 µs |

Nominal hard residual maxima: dynamics `6.065e-09`, contact acceleration `2.658e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 65.038 cm |
| authored reference vs measured CoM RMS / p95 | 57.720 / 158.397 cm |
| stance foot RMS | 33.073 cm |
| swing foot RMS | 67.268 cm |
| hand RMS | 78.150 cm |
| maximum root rotation | 179.425° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.065e-09 |
| contact acceleration residual | 2.658e-10 |
| raw max dynamics residual, including rejected ticks | 6.494e+02 |
| raw max contact residual, including rejected ticks | 2.658e-10 |
| active normal force range | 0.000–342.314 N |
| centroidal momentum-rate residual RMS / max | 52.705 / 160.730 N·m |
| point-task acceleration RMS max | 105.513 m/s² |
| frame-angular acceleration RMS max | 185.667 rad/s² |
| longest pre-contact / touchdown transition | 254 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `24.861` / `48.710 cm`.
- Virtual ZMP clipped on `58.17%` of ticks; clip-distance RMS / max `50.619` / `111.973 cm`.
- Measured-height natural frequency min / p50 / max: `3.692` / `3.762` / `7.004 rad/s`.
- Minimum measured CoM height: `-1.500 m`; height-floor ticks: `86`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-75.828` / `-72.847 cm`; inside on `48.33%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `160`, frozen `212`.
- Maximum applied offset / root reach: `0.0800 / 0.8466 m`.
- Authored-offset / reach / slew limited ticks: `160 / 0 / 26`.
- Authored geometry outside configured reach: `0` target-ticks.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `451` ticks; maximum active coordinates `8`; mean target/applied scale `0.664` / `0.745`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2656.3 µs | 210468.2 µs | 220189.8 µs | 237185.3 µs | 129 | 99 | 254 | 0 | 18 | 94 | 6 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 33794.3 | 68573.6 | 363.6 | 187462.5 | 228982.1 | 236365.0 | 102062.3 | 600 | 133 | 112 | 29.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2554.9 | 2689.1 | 2725.5 | 2766.0 |
| solved_with_slack | 99 | 2623.2 | 2937.7 | 3200.2 | 3322.3 |
| primal_infeasible | 6 | 219706.4 | 220597.5 | 220808.7 | 220861.5 |
| normal_contact_contingency | 18 | 15839.4 | 54345.3 | 200617.3 | 237185.3 |
| contact_release_contingency | 94 | 192566.0 | 220179.6 | 223194.3 | 223490.4 |
| precontact_transition | 254 | 2638.1 | 9113.8 | 65707.1 | 73731.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.44 | 11.0 | 13.0 | 14 | 5.12 | 12.0 | 14 | 0.0267 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 107.41/8.0/4476.7/6720 | 23026.26/1776.0/993012.1/1411200 | 2.01/21.3/54 | 1.70/20.3/53 | 14.14/181.7/451 | 0.2608 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| primal_infeasible | 6 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 18 | 10.56/14.0/14 | 9.89/13.8/14 |
| contact_release_contingency | 94 | 8.06/13.0/13 | 6.53/12.0/12 |
| precontact_transition | 254 | 8.67/13.5/14 | 7.50/12.5/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.86/5.0/7 | 7.89/24.0/30 | 1.30/5.0/7 | 278 |
| viability | 1.68/7.0/8 | 10.67/42.0/56 | 1.29/7.0/8 | 362 |
| intent | 1.57/5.0/6 | 7.71/25.0/36 | 1.29/5.0/6 | 433 |
| preference | 1.26/5.0/7 | 10.45/42.0/72 | 0.87/5.0/7 | 367 |
| style | 1.07/3.0/6 | 8.30/24.0/46 | 0.37/2.0/6 | 187 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 482 | 118 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `121` ticks.
Precontact sole-center tangential speed: p50 `2.5711 m/s`, p95 `6.4724 m/s`, max `8.2872 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20.277 | 20.273 | 20.273 | 1.000 | 1.000 | 48.840 | 49.055 | 0.215 | 49.832 | 0.001 | 0 | 36 | 0 | 0 | 270 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2534.9 | 2684.5 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2608.2 | 2835.2 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2607.7 | 3248.8 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2478.9 | 3159.9 | 7.13 | 46.18 | 4.57 | 1.00 | 225.80 | 6.405 | 0.736 | 7.54e-10 | 4.18e-11 | 12 |
| 240–299 | 2571.1 | 3286.1 | 8.30 | 50.87 | 7.03 | 1.00 | 222.00 | 6.440 | 20.801 | 9.62e-10 | 1.00e-11 | 60 |
| 300–359 | 2505.6 | 4630.7 | 8.42 | 49.38 | 6.83 | 2.05 | 455.10 | 7.161 | 24.581 | 8.83e-10 | 1.13e-11 | 60 |
| 360–419 | 2398.0 | 4249.4 | 8.55 | 50.97 | 7.28 | 2.75 | 610.50 | 16.749 | 18.880 | 1.20e-09 | 3.21e-11 | 60 |
| 420–479 | 4004.5 | 71946.5 | 9.45 | 59.68 | 8.92 | 377.22 | 83742.10 | 33.373 | 33.443 | 6.07e-09 | 2.66e-10 | 60 |
| 480–539 | 179223.1 | 229105.3 | 8.85 | 47.73 | 7.47 | 7.88 | 1673.10 | 75.849 | 48.612 | 4.41e-09 | 1.87e-10 | 60 |
| 540–599 | 190508.4 | 221808.8 | 7.27 | 34.77 | 6.05 | 679.20 | 142632.00 | 187.129 | 131.962 | 6.49e+02 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 65.038 | 47.212 | 78.150 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
