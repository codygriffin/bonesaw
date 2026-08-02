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
- Whole-body posture: `intent` priority with weight `0.250`.
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
| 452 | 2.260 s | 15.769 cm | 14.625 cm | 46.038 cm | 26.571 cm | 61.200° | 8.000 rad/s | 78557.0 µs |

Nominal hard residual maxima: dynamics `1.534e-09`, contact acceleration `4.886e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 98.746 cm |
| authored reference vs measured CoM RMS / p95 | 103.782 / 254.373 cm |
| stance foot RMS | 76.709 cm |
| swing foot RMS | 130.648 cm |
| hand RMS | 109.742 cm |
| maximum root rotation | 179.657° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.744e-09 |
| contact acceleration residual | 2.352e-10 |
| raw max dynamics residual, including rejected ticks | 9.744e-09 |
| raw max contact residual, including rejected ticks | 2.352e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 94.839 / 589.030 N·m |
| point-task acceleration RMS max | 249.861 m/s² |
| frame-angular acceleration RMS max | 350.508 rad/s² |
| longest pre-contact / touchdown transition | 224 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `102.029` / `250.938 cm`.
- Virtual ZMP clipped on `62.00%` of ticks; clip-distance RMS / max `143.357` / `426.705 cm`.
- Measured-height natural frequency min / p50 / max: `3.711` / `3.769` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.959 m`; height-floor ticks: `135`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-274.907` / `-223.671 cm`; inside on `43.00%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True`; final source tick `420.545`, progress `420.545` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `False`.
- Target / applied minimum rate: `0.0358` / `0.0358`; mean / p50 applied `0.7026` / `1.0000`.
- Limited / zero-rate hold ticks: `237` / `0`; maximum required landing time `1.0713 s`.
- Position / tangential / normal limiting ticks: `372` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `372`, frozen `0`.
- Maximum applied offset / root reach: `0.0800 / 0.8586 m`.
- Authored-offset / reach / slew limited ticks: `372 / 0 / 31`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `216` ticks; maximum active coordinates `8`; mean target/applied scale `0.670` / `0.755`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2481.2 µs | 100855.0 µs | 152386.3 µs | 207311.6 µs | 219 | 9 | 224 | 0 | 111 | 37 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11883.3 | 31122.8 | 510.3 | 8390.6 | 203799.7 | 206960.4 | 106536.0 | 600 | 115 | 52 | 84.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 219 | 2431.6 | 2538.8 | 2582.4 | 2715.6 |
| solved_with_slack | 9 | 2193.4 | 2597.8 | 2639.3 | 2649.7 |
| normal_contact_contingency | 111 | 5187.5 | 8393.9 | 10484.8 | 12571.9 |
| contact_release_contingency | 37 | 102269.9 | 188441.3 | 205200.9 | 207311.6 |
| precontact_transition | 224 | 2559.2 | 34613.4 | 92145.8 | 109385.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.22 | 10.0 | 11.0 | 13 | 4.09 | 10.0 | 11 | 0.0834 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 96.62/8.0/3218.3/6999 | 21441.60/1776.0/714455.9/1553778 | 1.22/9.0/12 | 0.93/8.0/11 | 7.58/68.0/91 | 0.2815 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 219 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 9 | 7.78/9.9/10 | 5.44/8.0/8 |
| normal_contact_contingency | 111 | 7.32/10.0/10 | 6.64/10.0/10 |
| contact_release_contingency | 37 | 6.57/9.3/10 | 5.24/8.3/9 |
| precontact_transition | 224 | 7.73/12.0/13 | 6.58/11.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.98/6.0/7 | 8.61/25.0/30 | 1.49/6.0/7 | 315 |
| viability | 1.70/6.0/9 | 9.67/42.0/55 | 1.29/6.0/9 | 371 |
| intent | 1.51/6.0/7 | 12.93/48.0/63 | 1.06/6.0/6 | 360 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.03/2.0/5 | 8.46/18.0/40 | 0.26/2.0/5 | 139 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 452 | 148 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `151` ticks.
Precontact sole-center tangential speed: p50 `2.1410 m/s`, p95 `8.4144 m/s`, max `11.8263 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.130 | 7.129 | 7.129 | 1.000 | 1.000 | 49.383 | 49.883 | 0.500 | 50.562 | 0.001 | 0 | 112 | 0 | 0 | 105 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2463.8 | 2586.1 | 4.00 | 25.03 | 0.00 | 1.00 | 234.00 | 0.277 | 0.000 | 1.32e-09 | 4.77e-11 | 0 |
| 60–119 | 2429.6 | 2521.6 | 4.00 | 25.70 | 0.00 | 1.00 | 234.00 | 1.161 | 0.000 | 1.53e-09 | 4.89e-11 | 0 |
| 120–179 | 2440.0 | 2627.0 | 4.00 | 25.20 | 0.00 | 1.00 | 234.00 | 3.309 | 0.000 | 1.44e-09 | 4.63e-11 | 0 |
| 180–239 | 2129.1 | 2844.7 | 5.42 | 33.73 | 2.07 | 1.00 | 225.80 | 8.097 | 0.812 | 8.63e-10 | 4.77e-11 | 12 |
| 240–299 | 3141.2 | 4521.4 | 7.68 | 49.90 | 6.00 | 3.92 | 869.50 | 7.920 | 14.812 | 9.13e-10 | 1.94e-11 | 60 |
| 300–359 | 2214.2 | 36266.7 | 7.72 | 49.15 | 6.72 | 85.43 | 18966.20 | 11.358 | 34.422 | 6.78e-10 | 1.83e-11 | 60 |
| 360–419 | 2667.2 | 37432.0 | 7.70 | 48.78 | 6.83 | 148.80 | 33033.60 | 17.066 | 39.797 | 1.31e-09 | 2.13e-11 | 60 |
| 420–479 | 47046.4 | 203852.5 | 7.42 | 45.72 | 6.55 | 709.55 | 157501.00 | 69.032 | 99.244 | 1.51e-09 | 4.92e-11 | 60 |
| 480–539 | 5409.5 | 106471.8 | 7.48 | 49.52 | 6.72 | 8.00 | 1724.00 | 179.138 | 168.138 | 9.74e-09 | 2.35e-10 | 60 |
| 540–599 | 3967.3 | 170369.0 | 6.78 | 43.90 | 6.02 | 6.48 | 1393.90 | 245.133 | 234.754 | 5.07e-09 | 2.17e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 98.746 | 98.091 | 109.742 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
