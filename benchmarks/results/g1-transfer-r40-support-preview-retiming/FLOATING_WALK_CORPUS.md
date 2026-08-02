# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `support-preview` `CoM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `disabled`; authored offset ≤ `0.120 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `40` scheduled ticks and throughout delayed admission.
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
| 542 | 2.710 s | 16.480 cm | 5.665 cm | 38.661 cm | 42.676 cm | 108.566° | 8.000 rad/s | 65102.8 µs |

Nominal hard residual maxima: dynamics `1.622e-09`, contact acceleration `6.370e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 39.688 cm |
| authored reference vs measured CoM RMS / p95 | 35.501 / 98.067 cm |
| stance foot RMS | 9.871 cm |
| swing foot RMS | 45.365 cm |
| hand RMS | 63.033 cm |
| maximum root rotation | 179.642° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.724e-09 |
| contact acceleration residual | 1.148e-10 |
| raw max dynamics residual, including rejected ticks | 3.724e-09 |
| raw max contact residual, including rejected ticks | 1.148e-10 |
| active normal force range | 0.000–426.776 N |
| centroidal momentum-rate residual RMS / max | 48.853 / 330.005 N·m |
| point-task acceleration RMS max | 122.568 m/s² |
| frame-angular acceleration RMS max | 244.378 rad/s² |
| longest pre-contact / touchdown transition | 314 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `True`; final source tick `423.928`, progress `423.928` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `False`.
- Target / applied minimum rate: `0.0302` / `0.0313`; mean / p50 applied `0.7082` / `1.0000`.
- Limited / zero-rate hold ticks: `237` / `0`; maximum required landing time `0.6746 s`.
- Position / tangential / normal limiting ticks: `372` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `0`, frozen `0`.
- Maximum applied offset / root reach: `0.0000 / 0.8397 m`.
- Authored-offset / reach / slew limited ticks: `0 / 0 / 0`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2441.0 µs | 22259.7 µs | 103577.8 µs | 213460.8 µs | 141 | 87 | 314 | 0 | 43 | 15 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7774.5 | 22866.5 | 602.4 | 9423.2 | 207257.9 | 212840.5 | 90361.6 | 600 | 96 | 32 | 128.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2390.6 | 2492.9 | 2515.2 | 2526.2 |
| solved_with_slack | 87 | 2677.0 | 4797.7 | 6263.8 | 6685.7 |
| normal_contact_contingency | 43 | 4391.1 | 12473.6 | 14713.2 | 14812.5 |
| contact_release_contingency | 15 | 102864.5 | 206212.0 | 212011.0 | 213460.8 |
| precontact_transition | 314 | 2629.2 | 21936.8 | 69090.4 | 78649.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.72 | 12.0 | 13.0 | 23 | 5.48 | 13.0 | 20 | 0.0662 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 92.70/8.0/3995.9/5054 | 20579.70/1776.0/887080.9/1121988 | 1.47/13.0/15 | 1.14/12.0/14 | 9.37/96.0/125 | 0.3620 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 87 | 8.55/17.0/23 | 6.09/14.0/20 |
| normal_contact_contingency | 43 | 9.09/13.2/14 | 8.26/12.6/13 |
| contact_release_contingency | 15 | 8.00/10.0/10 | 6.87/9.0/9 |
| precontact_transition | 314 | 8.52/13.0/14 | 7.32/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.96/6.0/8 | 8.09/25.0/36 | 1.43/6.0/8 | 331 |
| viability | 2.01/8.0/12 | 11.99/42.0/55 | 1.76/8.0/12 | 452 |
| intent | 1.39/4.0/6 | 7.17/24.0/30 | 1.12/4.0/6 | 434 |
| preference | 1.25/6.0/16 | 12.34/60.0/192 | 0.80/5.0/15 | 346 |
| style | 1.11/4.0/5 | 9.74/28.0/32 | 0.36/3.0/4 | 177 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 542 | 58 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `61` ticks.
Precontact sole-center tangential speed: p50 `3.0210 m/s`, p95 `6.3305 m/s`, max `8.3672 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.665 | 4.664 | 4.664 | 1.000 | 1.000 | 49.441 | 49.883 | 0.441 | 50.547 | 0.001 | 0 | 112 | 0 | 0 | 77 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2419.0 | 2812.5 | 6.07 | 36.82 | 1.67 | 1.00 | 234.00 | 0.172 | 0.000 | 1.62e-09 | 5.63e-11 | 0 |
| 60–119 | 2389.3 | 3435.6 | 5.10 | 34.98 | 0.17 | 1.00 | 234.00 | 0.032 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–179 | 2432.6 | 4324.7 | 6.05 | 38.67 | 1.60 | 1.12 | 261.30 | 0.002 | 0.000 | 1.46e-09 | 6.37e-11 | 0 |
| 180–239 | 2882.4 | 6396.3 | 8.68 | 58.33 | 6.77 | 3.87 | 871.40 | 3.130 | 3.554 | 1.15e-09 | 4.19e-11 | 12 |
| 240–299 | 2986.4 | 3803.9 | 7.97 | 50.90 | 6.32 | 4.97 | 1102.60 | 7.208 | 21.159 | 6.78e-10 | 1.62e-11 | 60 |
| 300–359 | 1878.9 | 3986.9 | 8.52 | 54.83 | 7.45 | 2.40 | 532.80 | 11.156 | 26.760 | 9.19e-10 | 2.22e-11 | 60 |
| 360–419 | 1870.2 | 3277.9 | 8.40 | 54.67 | 7.13 | 1.47 | 325.60 | 14.315 | 40.348 | 1.38e-09 | 1.21e-11 | 60 |
| 420–479 | 2977.3 | 4158.2 | 8.32 | 51.45 | 7.47 | 5.55 | 1232.10 | 20.983 | 32.493 | 9.68e-10 | 1.48e-11 | 60 |
| 480–539 | 9795.9 | 76765.4 | 9.27 | 60.12 | 8.27 | 899.25 | 199633.50 | 38.553 | 23.537 | 7.60e-10 | 3.30e-11 | 60 |
| 540–599 | 9046.9 | 207351.1 | 8.87 | 52.63 | 7.95 | 6.37 | 1369.70 | 115.903 | 55.781 | 3.72e-09 | 1.15e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 39.688 | 27.433 | 63.033 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
