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
- Landing commitment: freeze the retargeted anchor for the final `0` scheduled ticks and throughout delayed admission.
- Coupled touchdown phase retiming: `enabled`; minimum rate `0.000`, guard `0.020 s`, engagement / release `0 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
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
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `retiming_reaches_first_authored_touchdown` | FAIL |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `retiming_reaches_first_authored_touchdown`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 482 | 2.410 s | 23.177 cm | 2.843 cm | 54.944 cm | 41.388 cm | 82.309° | 8.000 rad/s | 59256.7 µs |

Nominal hard residual maxima: dynamics `2.737e-09`, contact acceleration `9.337e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 68.264 cm |
| authored reference vs measured CoM RMS / p95 | 65.080 / 167.154 cm |
| stance foot RMS | 30.203 cm |
| swing foot RMS | 81.212 cm |
| hand RMS | 86.324 cm |
| maximum root rotation | 179.804° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 5.349e-09 |
| contact acceleration residual | 1.156e-10 |
| raw max dynamics residual, including rejected ticks | 5.349e-09 |
| raw max contact residual, including rejected ticks | 1.156e-10 |
| active normal force range | 0.000–866.048 N |
| centroidal momentum-rate residual RMS / max | 55.310 / 331.307 N·m |
| point-task acceleration RMS max | 160.890 m/s² |
| frame-angular acceleration RMS max | 228.980 rad/s² |
| longest pre-contact / touchdown transition | 254 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `47.620` / `108.174 cm`.
- Virtual ZMP clipped on `58.83%` of ticks; clip-distance RMS / max `63.321` / `199.078 cm`.
- Measured-height natural frequency min / p50 / max: `3.664` / `3.721` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.752 m`; height-floor ticks: `112`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-103.177` / `-65.192 cm`; inside on `44.33%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True`; final source tick `420.872`, progress `420.872` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `False`.
- Target / applied minimum rate: `0.0693` / `0.0705`; mean / p50 applied `0.7031` / `1.0000`.
- Limited / zero-rate hold ticks: `253` / `0`; maximum required landing time `0.8738 s`.
- Position / tangential / normal limiting ticks: `372` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `372`, frozen `0`.
- Maximum applied offset / root reach: `0.0800 / 0.8168 m`.
- Authored-offset / reach / slew limited ticks: `372 / 0 / 24`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `457` ticks; maximum active coordinates `8`; mean target/applied scale `0.664` / `0.759`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2505.4 µs | 52167.1 µs | 186555.8 µs | 366029.1 µs | 126 | 102 | 254 | 0 | 97 | 21 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10792.4 | 33344.2 | 247.7 | 13820.0 | 272873.7 | 356713.6 | 57860.0 | 600 | 79 | 46 | 92.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 126 | 2442.7 | 2565.2 | 2590.4 | 2604.4 |
| solved_with_slack | 102 | 2515.6 | 2787.0 | 3979.2 | 4468.1 |
| normal_contact_contingency | 97 | 3376.8 | 16130.0 | 38516.0 | 39975.1 |
| contact_release_contingency | 21 | 166846.1 | 210510.8 | 334925.5 | 366029.1 |
| precontact_transition | 254 | 2504.4 | 46041.0 | 62227.9 | 66214.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.79 | 12.0 | 14.0 | 25 | 5.57 | 13.0 | 21 | 0.0648 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 128.60/8.0/3511.0/4138 | 28499.91/1776.0/779453.1/918636 | 0.97/16.0/17 | 0.81/15.0/16 | 6.91/126.0/142 | 0.2480 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 126 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 102 | 7.22/14.0/25 | 4.25/11.0/21 |
| normal_contact_contingency | 97 | 9.01/14.0/14 | 8.21/13.0/13 |
| contact_release_contingency | 21 | 7.48/9.8/10 | 6.10/8.8/9 |
| precontact_transition | 254 | 8.96/13.0/15 | 7.82/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.89/6.0/8 | 8.53/30.0/40 | 1.33/6.0/8 | 279 |
| viability | 1.82/6.0/8 | 11.34/42.0/56 | 1.46/6.0/8 | 386 |
| intent | 1.57/5.0/8 | 8.36/26.0/38 | 1.34/5.0/8 | 471 |
| preference | 1.40/6.0/20 | 12.01/51.1/140 | 1.03/5.0/19 | 392 |
| style | 1.11/4.0/5 | 9.39/26.0/45 | 0.41/3.0/5 | 196 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 482 | 118 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `121` ticks.
Precontact sole-center tangential speed: p50 `2.5607 m/s`, p95 `7.7513 m/s`, max `11.7597 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.476 | 6.474 | 6.474 | 1.000 | 1.000 | 49.398 | 49.895 | 0.496 | 50.473 | 0.001 | 0 | 111 | 0 | 0 | 85 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2412.8 | 2525.6 | 5.00 | 33.53 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| 60–119 | 2509.5 | 3195.8 | 5.47 | 37.28 | 1.05 | 1.00 | 234.00 | 0.151 | 0.000 | 1.27e-09 | 3.93e-11 | 0 |
| 120–179 | 2489.6 | 2767.9 | 5.98 | 39.17 | 2.03 | 1.00 | 234.00 | 1.369 | 0.000 | 1.03e-09 | 5.03e-11 | 0 |
| 180–239 | 2465.6 | 3574.8 | 8.05 | 51.20 | 5.53 | 1.00 | 225.80 | 6.458 | 1.270 | 1.35e-09 | 5.23e-11 | 12 |
| 240–299 | 2441.5 | 3953.4 | 8.35 | 53.00 | 6.87 | 1.58 | 351.50 | 7.203 | 23.318 | 8.57e-10 | 2.38e-11 | 60 |
| 300–359 | 2461.2 | 3422.9 | 8.92 | 57.08 | 7.53 | 1.00 | 222.00 | 13.816 | 24.948 | 1.12e-09 | 2.65e-11 | 60 |
| 360–419 | 2356.7 | 3770.7 | 8.63 | 55.58 | 7.87 | 1.82 | 403.30 | 27.710 | 57.824 | 1.03e-09 | 3.46e-11 | 60 |
| 420–479 | 7976.2 | 65081.9 | 9.93 | 62.60 | 9.10 | 1189.50 | 264069.00 | 55.092 | 50.660 | 2.74e-09 | 9.34e-11 | 60 |
| 480–539 | 15310.2 | 274273.3 | 9.15 | 54.30 | 8.22 | 83.00 | 17927.50 | 120.120 | 73.860 | 5.35e-09 | 1.16e-10 | 60 |
| 540–599 | 2773.5 | 3913.1 | 8.40 | 52.55 | 7.52 | 5.08 | 1098.00 | 167.574 | 124.822 | 3.51e-09 | 6.01e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 68.264 | 53.022 | 86.324 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
