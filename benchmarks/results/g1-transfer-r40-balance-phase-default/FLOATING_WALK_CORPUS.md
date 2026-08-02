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
- Coupled balance phase retiming: `enabled`; hold at signed DCM margin ≤ `0.000 m`, recover nominal rate at `0.020 m`.
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
| 345 | 1.725 s | 4.725 cm | 7.480 cm | 21.899 cm | 34.668 cm | 14.815° | 8.000 rad/s | 7536.7 µs |

Nominal hard residual maxima: dynamics `1.267e-09`, contact acceleration `5.554e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 103.153 cm |
| authored reference vs measured CoM RMS / p95 | 105.718 / 259.075 cm |
| stance foot RMS | 85.406 cm |
| swing foot RMS | 126.149 cm |
| hand RMS | 104.482 cm |
| maximum root rotation | 17.222° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.267e-09 |
| contact acceleration residual | 5.554e-11 |
| raw max dynamics residual, including rejected ticks | 1.267e-09 |
| raw max contact residual, including rejected ticks | 5.554e-11 |
| active normal force range | 0.000–298.885 N |
| centroidal momentum-rate residual RMS / max | 34.075 / 123.254 N·m |
| point-task acceleration RMS max | 81.546 m/s² |
| frame-angular acceleration RMS max | 130.972 rad/s² |
| longest pre-contact / touchdown transition | 110 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `145.758` / `336.460 cm`.
- Virtual ZMP clipped on `60.17%` of ticks; clip-distance RMS / max `276.183` / `720.734 cm`.
- Measured-height natural frequency min / p50 / max: `3.665` / `3.754` / `4.160 rad/s`.
- Minimum measured CoM height: `0.567 m`; height-floor ticks: `0`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-379.295` / `-323.337 cm`; inside on `43.00%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True` (touchdown `True`, balance `True`); final source tick `243.161`, progress `243.161` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `False`.
- Target / applied minimum rate: `0.0000` / `0.0000`; mean / p50 applied `0.4069` / `0.0000`.
- Limited / zero-rate hold ticks: `400` / `341`; maximum required landing time `0.7975 s`.
- Position / tangential / normal limiting ticks: `365` / `0` / `0`; unsafe-edge ticks `0`.
- Balance-margin limited ticks: `400`.

## Capture-aware landing

- Active target-ticks: `365`; policy updates `365`, frozen `0`.
- Maximum applied offset / root reach: `0.0800 / 0.8510 m`.
- Authored-offset / reach / slew limited ticks: `365 / 0 / 83`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `36`, precontact `364`, multi-support `199`.
- Joint-velocity envelope active on `460` ticks; maximum active coordinates `10`; mean target/applied scale `0.680` / `0.748`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2585.3 µs | 32852.5 µs | 46165.2 µs | 114352.4 µs | 126 | 109 | 110 | 0 | 255 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5949.0 | 10328.2 | 300.4 | 10808.4 | 76241.0 | 110541.2 | 33893.3 | 600 | 92 | 46 | 168.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 126 | 2440.2 | 2548.7 | 2579.2 | 2592.4 |
| solved_with_slack | 109 | 2510.9 | 2841.6 | 3947.7 | 4439.3 |
| normal_contact_contingency | 255 | 3264.5 | 43345.5 | 48792.4 | 114352.4 |
| precontact_transition | 110 | 2806.1 | 7359.2 | 7891.0 | 8009.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.58 | 11.0 | 13.0 | 25 | 5.02 | 13.0 | 21 | 0.1806 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 181.76/1986.9/2884.3/3128 | 39270.90/429170.4/623002.3/675648 | 1.10/12.0/12 | 0.83/11.0/11 | 6.91/93.0/97 | 0.8847 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 126 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 109 | 7.17/12.8/25 | 4.24/10.8/21 |
| normal_contact_contingency | 255 | 8.64/13.0/16 | 7.04/13.0/15 |
| precontact_transition | 110 | 8.48/12.0/13 | 6.85/10.9/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.48/4.0/5 | 6.04/16.0/25 | 0.71/4.0/5 | 213 |
| viability | 2.15/7.0/8 | 14.12/49.0/60 | 1.79/7.0/8 | 390 |
| intent | 1.54/5.0/9 | 8.28/25.0/50 | 1.32/5.0/9 | 473 |
| preference | 1.33/6.0/20 | 10.79/49.0/140 | 0.83/6.0/19 | 320 |
| style | 1.07/3.0/4 | 9.29/25.0/37 | 0.36/3.0/4 | 191 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 36 | 365 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 345 | 255 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `365` ticks, planned normal touchdown `0` ticks, normal fallback `258` ticks.
Precontact sole-center tangential speed: p50 `3.3489 m/s`, p95 `6.5445 m/s`, max `6.9582 m/s` over 365 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.570 | 3.569 | 3.569 | 1.000 | 1.000 | 49.426 | 49.934 | 0.508 | 50.625 | 0.001 | 0 | 113 | 0 | 0 | 52 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2416.4 | 2552.4 | 5.00 | 33.53 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| 60–119 | 2512.5 | 3207.0 | 5.47 | 37.28 | 1.05 | 1.00 | 234.00 | 0.151 | 0.000 | 1.27e-09 | 3.93e-11 | 0 |
| 120–179 | 2486.3 | 2763.3 | 5.98 | 39.17 | 2.03 | 1.00 | 234.00 | 1.369 | 0.000 | 1.03e-09 | 5.03e-11 | 0 |
| 180–239 | 2357.5 | 3628.5 | 7.73 | 48.55 | 5.10 | 1.00 | 225.80 | 6.521 | 1.023 | 1.09e-09 | 5.23e-11 | 5 |
| 240–299 | 2690.3 | 4441.3 | 8.60 | 55.93 | 6.88 | 1.82 | 403.30 | 8.004 | 17.200 | 9.11e-10 | 2.28e-11 | 60 |
| 300–359 | 3765.6 | 71855.3 | 8.55 | 53.90 | 7.10 | 359.48 | 77673.90 | 6.153 | 32.667 | 1.18e-09 | 2.15e-11 | 60 |
| 360–419 | 11638.1 | 49463.2 | 8.82 | 54.10 | 7.13 | 738.77 | 159573.60 | 30.526 | 41.476 | 5.38e-10 | 1.21e-11 | 60 |
| 420–479 | 3678.6 | 48368.3 | 8.25 | 50.35 | 6.62 | 704.87 | 152251.20 | 92.304 | 116.483 | 5.47e-10 | 9.53e-12 | 60 |
| 480–539 | 2644.4 | 4125.0 | 8.00 | 52.45 | 6.00 | 2.63 | 568.80 | 172.687 | 175.443 | 9.19e-10 | 1.02e-11 | 60 |
| 540–599 | 2888.3 | 3979.7 | 9.37 | 60.02 | 8.25 | 6.07 | 1310.40 | 258.819 | 233.050 | 2.14e-10 | 2.93e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 103.153 | 100.869 | 104.482 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
