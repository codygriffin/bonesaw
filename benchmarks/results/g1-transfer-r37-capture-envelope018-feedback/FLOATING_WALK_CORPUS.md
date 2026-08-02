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
- Joint-velocity envelope: `viability` priority with weight `0.180`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
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
| 445 | 2.225 s | 15.144 cm | 10.398 cm | 29.658 cm | 39.400 cm | 55.227° | 8.000 rad/s | 15155.2 µs |

Nominal hard residual maxima: dynamics `5.619e-09`, contact acceleration `2.758e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 55.840 cm |
| authored reference vs measured CoM RMS / p95 | 49.099 / 120.306 cm |
| stance foot RMS | 42.507 cm |
| swing foot RMS | 53.576 cm |
| hand RMS | 77.271 cm |
| maximum root rotation | 179.364° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 5.619e-09 |
| contact acceleration residual | 2.758e-10 |
| raw max dynamics residual, including rejected ticks | 5.619e-09 |
| raw max contact residual, including rejected ticks | 2.758e-10 |
| active normal force range | 0.000–713.414 N |
| centroidal momentum-rate residual RMS / max | 60.276 / 334.211 N·m |
| point-task acceleration RMS max | 143.420 m/s² |
| frame-angular acceleration RMS max | 268.097 rad/s² |
| longest pre-contact / touchdown transition | 217 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `24.401` / `46.113 cm`.
- Virtual ZMP clipped on `62.17%` of ticks; clip-distance RMS / max `51.049` / `120.812 cm`.
- Measured-height natural frequency min / p50 / max: `3.662` / `3.758` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.619 m`; height-floor ticks: `119`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-88.573` / `-71.351 cm`; inside on `38.00%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8410 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 40`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.5917 m / 1.2927 / 0.0383 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 2`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `451` ticks; maximum active coordinates `8`; mean target/applied scale `0.670` / `0.757`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2682.5 µs | 18051.5 µs | 218302.9 µs | 230833.5 µs | 129 | 99 | 217 | 0 | 138 | 17 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9864.3 | 32901.8 | 448.7 | 5016.4 | 230791.0 | 230829.2 | 80581.6 | 600 | 61 | 29 | 101.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2530.1 | 3604.7 | 4132.0 | 4150.8 |
| solved_with_slack | 99 | 2614.8 | 4146.5 | 4309.3 | 4414.9 |
| normal_contact_contingency | 138 | 3373.3 | 31160.4 | 112701.9 | 223509.0 |
| contact_release_contingency | 17 | 201693.3 | 230776.8 | 230822.1 | 230833.5 |
| precontact_transition | 217 | 2960.6 | 9577.3 | 40463.9 | 69313.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.61 | 11.0 | 13.0 | 15 | 5.26 | 13.0 | 15 | 0.0565 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 80.02/8.0/4287.3/6644 | 17405.13/1776.0/951470.3/1435104 | 1.20/15.0/21 | 0.84/14.0/20 | 6.97/119.0/170 | 0.2940 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.83/13.0/14 | 3.59/9.0/10 |
| normal_contact_contingency | 138 | 8.74/13.6/15 | 7.64/13.0/14 |
| contact_release_contingency | 17 | 7.41/9.8/10 | 6.00/8.0/8 |
| precontact_transition | 217 | 8.81/13.8/15 | 7.58/13.0/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.95/6.0/10 | 8.54/28.0/50 | 1.41/6.0/10 | 301 |
| viability | 1.66/6.0/10 | 10.40/42.0/70 | 1.27/6.0/10 | 369 |
| intent | 1.61/5.0/5 | 8.49/25.0/29 | 1.38/5.0/5 | 470 |
| preference | 1.32/5.0/7 | 11.02/47.0/66 | 0.84/5.0/6 | 339 |
| style | 1.07/3.0/8 | 8.93/22.0/75 | 0.37/3.0/8 | 187 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 445 | 155 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `158` ticks.
Precontact sole-center tangential speed: p50 `2.1917 m/s`, p95 `6.0753 m/s`, max `8.8534 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.919 | 5.918 | 5.918 | 1.000 | 1.000 | 47.070 | 47.434 | 0.363 | 49.133 | 0.001 | 0 | 92 | 0 | 0 | 32 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2513.3 | 3837.3 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2603.5 | 2793.7 | 5.35 | 36.42 | 0.83 | 1.00 | 234.00 | 0.112 | 0.000 | 1.01e-09 | 4.05e-11 | 0 |
| 120–179 | 2604.3 | 4351.3 | 5.93 | 38.93 | 1.83 | 1.00 | 234.00 | 1.250 | 0.000 | 1.11e-09 | 4.68e-11 | 0 |
| 180–239 | 2337.9 | 3545.6 | 7.48 | 46.23 | 4.72 | 1.00 | 225.80 | 6.213 | 0.899 | 8.64e-10 | 3.09e-11 | 12 |
| 240–299 | 2500.1 | 4227.7 | 8.35 | 52.53 | 6.48 | 1.93 | 429.20 | 5.444 | 19.872 | 1.07e-09 | 2.34e-11 | 60 |
| 300–359 | 2715.1 | 10481.1 | 8.68 | 52.33 | 7.45 | 18.73 | 4158.80 | 10.068 | 17.862 | 7.25e-10 | 1.47e-11 | 60 |
| 360–419 | 3503.7 | 67263.5 | 8.88 | 50.57 | 8.07 | 168.72 | 37455.10 | 27.534 | 28.753 | 7.81e-10 | 9.93e-12 | 60 |
| 420–479 | 14647.6 | 230791.7 | 9.43 | 57.22 | 8.65 | 593.88 | 128281.20 | 54.310 | 51.462 | 5.62e-09 | 2.76e-10 | 60 |
| 480–539 | 3600.4 | 114511.3 | 8.13 | 51.75 | 6.93 | 6.95 | 1499.60 | 95.334 | 88.842 | 2.35e-09 | 6.32e-11 | 60 |
| 540–599 | 2916.6 | 4655.6 | 8.83 | 54.37 | 7.62 | 6.02 | 1299.60 | 134.959 | 97.488 | 1.24e-09 | 1.06e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 55.840 | 46.461 | 77.271 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
