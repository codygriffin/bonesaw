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
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `enabled`; authored offset ≤ `0.080 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `0` scheduled ticks and throughout delayed admission.
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
| 386 | 1.930 s | 14.048 cm | 14.613 cm | 49.075 cm | 33.214 cm | 80.047° | 8.000 rad/s | 14390.7 µs |

Nominal hard residual maxima: dynamics `1.362e-09`, contact acceleration `5.993e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 100.533 cm |
| authored reference vs measured CoM RMS / p95 | 104.768 / 211.122 cm |
| stance foot RMS | 84.185 cm |
| swing foot RMS | 94.113 cm |
| hand RMS | 117.118 cm |
| maximum root rotation | 179.362° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.100e-09 |
| contact acceleration residual | 1.306e-10 |
| raw max dynamics residual, including rejected ticks | 4.100e-09 |
| raw max contact residual, including rejected ticks | 1.306e-10 |
| active normal force range | 0.000–668.610 N |
| centroidal momentum-rate residual RMS / max | 51.845 / 309.364 N·m |
| point-task acceleration RMS max | 128.140 m/s² |
| frame-angular acceleration RMS max | 298.804 rad/s² |
| longest pre-contact / touchdown transition | 158 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `90.169` / `168.281 cm`.
- Virtual ZMP clipped on `61.50%` of ticks; clip-distance RMS / max `126.054` / `241.779 cm`.
- Measured-height natural frequency min / p50 / max: `3.658` / `3.866` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.730 m`; height-floor ticks: `192`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-140.619` / `-129.696 cm`; inside on `42.00%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8562 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 32`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `1.2332 m / 0.8320 / 0.0116 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 5`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3192.3 µs | 103415.4 µs | 115220.1 µs | 214534.7 µs | 97 | 131 | 158 | 0 | 180 | 34 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12450.9 | 29865.4 | 722.0 | 21170.0 | 202472.3 | 213328.5 | 101307.2 | 600 | 128 | 71 | 80.3 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 97 | 2503.9 | 2621.4 | 2647.1 | 2686.6 |
| solved_with_slack | 131 | 2607.9 | 4778.4 | 5474.1 | 5833.6 |
| normal_contact_contingency | 180 | 3879.5 | 43778.1 | 87197.5 | 95842.1 |
| contact_release_contingency | 34 | 105220.4 | 194213.9 | 207889.3 | 214534.7 |
| precontact_transition | 158 | 3659.8 | 7623.7 | 87850.1 | 105695.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.72 | 11.0 | 13.0 | 14 | 5.74 | 12.0 | 14 | 0.1047 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 163.70/1185.0/5053.1/6573 | 35571.89/255970.8/1091775.6/1459206 | 1.85/10.0/12 | 1.34/9.0/11 | 10.89/77.0/90 | 0.3326 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 97 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 131 | 6.85/10.0/11 | 4.37/8.7/9 |
| normal_contact_contingency | 180 | 8.96/13.2/14 | 7.93/12.0/13 |
| contact_release_contingency | 34 | 7.82/10.0/10 | 6.44/9.0/9 |
| precontact_transition | 158 | 8.68/14.0/14 | 7.74/12.4/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.18/6.0/6 | 9.44/25.0/30 | 1.78/6.0/6 | 397 |
| viability | 1.58/5.0/7 | 8.73/35.0/42 | 1.24/5.0/7 | 419 |
| intent | 1.61/5.0/7 | 8.27/25.0/42 | 1.45/5.0/7 | 503 |
| preference | 1.29/5.0/7 | 12.79/47.0/71 | 0.88/5.0/6 | 385 |
| style | 1.06/3.0/4 | 8.99/22.0/27 | 0.39/2.0/4 | 213 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 386 | 214 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `217` ticks.
Precontact sole-center tangential speed: p50 `2.4279 m/s`, p95 `6.4411 m/s`, max `9.8877 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.471 | 7.459 | 7.459 | 0.998 | 0.998 | 48.211 | 48.570 | 0.359 | 49.312 | 0.001 | 0 | 91 | 0 | 0 | 874 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2493.3 | 2637.6 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2524.9 | 2730.4 | 5.73 | 38.37 | 1.63 | 1.00 | 234.00 | 0.087 | 0.000 | 1.36e-09 | 5.46e-11 | 0 |
| 120–179 | 2693.5 | 5632.7 | 6.95 | 42.42 | 4.78 | 3.10 | 725.40 | 1.481 | 0.000 | 1.36e-09 | 5.99e-11 | 0 |
| 180–239 | 3035.2 | 4445.6 | 6.85 | 40.98 | 4.25 | 5.20 | 1163.80 | 4.562 | 0.481 | 1.16e-09 | 5.83e-11 | 12 |
| 240–299 | 3424.8 | 15505.2 | 8.43 | 51.12 | 7.47 | 36.42 | 8084.50 | 10.814 | 14.354 | 2.33e-10 | 1.15e-11 | 60 |
| 300–359 | 3499.4 | 5319.0 | 8.82 | 55.22 | 8.07 | 6.13 | 1361.60 | 21.622 | 48.048 | 7.43e-10 | 1.74e-11 | 60 |
| 360–419 | 99543.5 | 202653.5 | 8.82 | 51.68 | 7.67 | 657.43 | 143741.60 | 55.960 | 97.807 | 3.42e-10 | 8.88e-12 | 60 |
| 420–479 | 6393.0 | 69313.9 | 8.97 | 57.20 | 8.18 | 477.60 | 103161.60 | 147.078 | 132.498 | 2.90e-09 | 7.79e-11 | 60 |
| 480–539 | 3129.8 | 110040.1 | 8.43 | 51.80 | 7.43 | 7.18 | 1547.60 | 202.970 | 166.078 | 4.10e-09 | 1.31e-10 | 60 |
| 540–599 | 2019.4 | 24294.6 | 9.23 | 59.87 | 7.90 | 441.97 | 95464.80 | 185.747 | 139.675 | 9.25e-10 | 1.35e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 100.533 | 87.594 | 117.118 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
