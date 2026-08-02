# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.103 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.079 m/s` (`0.10×` forward, `0.10×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `dcm-backward-preview` `DCM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `1.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
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
| 471 | 2.355 s | 11.071 cm | 0.661 cm | 30.028 cm | 26.583 cm | 45.613° | 8.000 rad/s | 15584.7 µs |

Nominal hard residual maxima: dynamics `8.766e-09`, contact acceleration `5.480e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 53.844 cm |
| authored reference vs measured CoM RMS / p95 | 46.455 / 124.022 cm |
| stance foot RMS | 31.210 cm |
| swing foot RMS | 43.650 cm |
| hand RMS | 71.440 cm |
| maximum root rotation | 179.939° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.766e-09 |
| contact acceleration residual | 5.480e-10 |
| raw max dynamics residual, including rejected ticks | 8.766e-09 |
| raw max contact residual, including rejected ticks | 5.480e-10 |
| active normal force range | 0.000–735.534 N |
| centroidal momentum-rate residual RMS / max | 52.108 / 201.300 N·m |
| point-task acceleration RMS max | 152.906 m/s² |
| frame-angular acceleration RMS max | 229.862 rad/s² |
| longest pre-contact / touchdown transition | 243 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `27.553` / `52.217 cm`.
- Virtual ZMP clipped on `58.17%` of ticks; clip-distance RMS / max `53.602` / `114.581 cm`.
- Measured-height natural frequency min / p50 / max: `3.705` / `3.768` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.567 m`; height-floor ticks: `94`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-75.474` / `-69.375 cm`; inside on `42.67%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True`; final source tick `425.098`, progress `425.098` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `False`.
- Target / applied minimum rate: `0.0282` / `0.0282`; mean / p50 applied `0.7102` / `1.0000`.
- Limited / zero-rate hold ticks: `229` / `0`; maximum required landing time `0.8615 s`.
- Position / tangential / normal limiting ticks: `372` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `372`, frozen `0`.
- Maximum applied offset / root reach: `0.0800 / 0.8268 m`.
- Authored-offset / reach / slew limited ticks: `330 / 0 / 127`.
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
| 600 | 3.0 s | 2457.2 µs | 20417.9 µs | 199551.9 µs | 211935.5 µs | 199 | 29 | 243 | 0 | 108 | 21 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9422.8 | 29836.9 | 452.7 | 9491.3 | 211061.0 | 211848.0 | 90147.6 | 600 | 74 | 31 | 106.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 2461.7 | 2611.4 | 3178.1 | 3521.7 |
| solved_with_slack | 29 | 1903.4 | 3292.6 | 3716.1 | 3846.2 |
| normal_contact_contingency | 108 | 3948.0 | 64418.2 | 100592.5 | 188485.4 |
| contact_release_contingency | 21 | 102919.4 | 210475.5 | 211643.5 | 211935.5 |
| precontact_transition | 243 | 2044.0 | 4548.5 | 20416.3 | 67386.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.26 | 11.0 | 14.0 | 16 | 4.25 | 12.0 | 14 | 0.0722 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 67.42/8.0/3729.5/6617 | 14653.58/1776.0/827957.9/1429272 | 1.41/16.0/20 | 1.16/15.0/19 | 9.74/130.0/168 | 0.2844 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 29 | 6.45/10.7/11 | 2.28/7.7/8 |
| normal_contact_contingency | 108 | 8.89/14.0/14 | 7.17/12.0/13 |
| contact_release_contingency | 21 | 7.43/9.0/9 | 5.76/8.0/8 |
| precontact_transition | 243 | 8.48/14.0/16 | 6.55/12.6/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.91/6.0/9 | 8.14/28.0/40 | 1.37/6.0/9 | 287 |
| viability | 1.67/6.0/11 | 9.67/42.0/77 | 1.25/6.0/11 | 365 |
| intent | 1.54/6.0/9 | 10.18/42.0/59 | 1.15/6.0/9 | 393 |
| preference | 1.05/2.0/6 | 4.74/16.0/34 | 0.12/2.0/5 | 43 |
| style | 1.09/4.0/6 | 9.05/28.0/36 | 0.36/3.0/6 | 175 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 471 | 129 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `132` ticks.
Precontact sole-center tangential speed: p50 `2.2726 m/s`, p95 `8.7024 m/s`, max `13.2364 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.654 | 5.652 | 5.652 | 1.000 | 1.000 | 49.312 | 49.758 | 0.445 | 50.242 | 0.001 | 0 | 113 | 0 | 0 | 94 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2500.4 | 3111.7 | 5.00 | 27.57 | 0.00 | 1.00 | 234.00 | 0.279 | 0.000 | 1.49e-09 | 4.80e-11 | 0 |
| 60–119 | 2450.6 | 3005.1 | 5.00 | 28.48 | 0.00 | 1.00 | 234.00 | 1.170 | 0.000 | 1.77e-09 | 4.63e-11 | 0 |
| 120–179 | 2459.1 | 3013.3 | 5.00 | 27.93 | 0.00 | 1.00 | 234.00 | 3.338 | 0.000 | 1.40e-09 | 5.19e-11 | 0 |
| 180–239 | 2126.0 | 3572.1 | 6.47 | 36.72 | 2.33 | 1.55 | 347.90 | 8.327 | 1.204 | 9.00e-10 | 3.60e-11 | 12 |
| 240–299 | 1919.8 | 2563.9 | 8.33 | 49.72 | 5.67 | 1.00 | 222.00 | 8.435 | 19.843 | 1.42e-09 | 2.12e-11 | 60 |
| 300–359 | 2041.2 | 3370.9 | 8.20 | 49.93 | 6.23 | 1.70 | 377.40 | 4.384 | 24.941 | 1.09e-09 | 5.06e-11 | 60 |
| 360–419 | 2306.2 | 4036.0 | 8.55 | 49.47 | 7.03 | 2.87 | 636.40 | 8.073 | 19.866 | 7.62e-10 | 1.91e-11 | 60 |
| 420–479 | 2119.1 | 129290.8 | 8.85 | 50.67 | 7.48 | 231.83 | 50888.10 | 32.958 | 28.946 | 8.77e-09 | 5.48e-10 | 60 |
| 480–539 | 14896.3 | 211074.1 | 8.90 | 49.57 | 7.33 | 424.28 | 91634.80 | 89.594 | 53.919 | 4.62e-09 | 2.02e-10 | 60 |
| 540–599 | 3561.1 | 43970.2 | 8.35 | 47.82 | 6.47 | 8.00 | 1727.20 | 140.146 | 87.724 | 3.47e-09 | 6.86e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 53.844 | 35.850 | 71.440 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
