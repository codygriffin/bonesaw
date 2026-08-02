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
| 465 | 2.325 s | 9.678 cm | 1.039 cm | 39.709 cm | 34.446 cm | 49.187° | 8.000 rad/s | 4517.1 µs |

Nominal hard residual maxima: dynamics `1.445e-09`, contact acceleration `5.554e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 59.969 cm |
| authored reference vs measured CoM RMS / p95 | 53.867 / 153.054 cm |
| stance foot RMS | 23.107 cm |
| swing foot RMS | 45.294 cm |
| hand RMS | 83.949 cm |
| maximum root rotation | 179.889° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.902e-09 |
| contact acceleration residual | 7.989e-11 |
| raw max dynamics residual, including rejected ticks | 2.902e-09 |
| raw max contact residual, including rejected ticks | 7.989e-11 |
| active normal force range | 0.000–600.654 N |
| centroidal momentum-rate residual RMS / max | 60.136 / 389.444 N·m |
| point-task acceleration RMS max | 167.327 m/s² |
| frame-angular acceleration RMS max | 241.553 rad/s² |
| longest pre-contact / touchdown transition | 237 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `28.231` / `61.689 cm`.
- Virtual ZMP clipped on `57.67%` of ticks; clip-distance RMS / max `48.911` / `123.957 cm`.
- Measured-height natural frequency min / p50 / max: `3.682` / `3.748` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.859 m`; height-floor ticks: `97`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-68.664` / `-60.508 cm`; inside on `50.33%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True`; final source tick `423.758`, progress `423.758` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `False`.
- Target / applied minimum rate: `0.0388` / `0.0388`; mean / p50 applied `0.7079` / `1.0000`.
- Limited / zero-rate hold ticks: `252` / `0`; maximum required landing time `0.5895 s`.
- Position / tangential / normal limiting ticks: `372` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `372`, frozen `0`.
- Maximum applied offset / root reach: `0.0800 / 0.7916 m`.
- Authored-offset / reach / slew limited ticks: `372 / 0 / 120`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `444` ticks; maximum active coordinates `8`; mean target/applied scale `0.663` / `0.730`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2536.4 µs | 13976.5 µs | 100411.2 µs | 201888.5 µs | 126 | 102 | 237 | 0 | 124 | 11 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8088.9 | 23839.4 | 421.4 | 5077.1 | 200645.6 | 201764.3 | 11648.3 | 600 | 62 | 27 | 123.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 126 | 2410.3 | 2539.7 | 2809.2 | 3381.9 |
| solved_with_slack | 102 | 2479.7 | 2646.1 | 3859.2 | 3942.9 |
| normal_contact_contingency | 124 | 4116.1 | 95711.2 | 101185.8 | 199813.6 |
| contact_release_contingency | 11 | 99740.4 | 198087.2 | 201128.3 | 201888.5 |
| precontact_transition | 237 | 2856.0 | 4147.2 | 4670.6 | 4998.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.68 | 12.0 | 14.0 | 16 | 5.48 | 13.0 | 15 | 0.2060 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 171.88/8.0/6317.7/6809 | 37140.39/1776.0/1364621.0/1470744 | 1.38/14.0/16 | 1.05/13.0/15 | 8.70/113.0/128 | 0.6504 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 126 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 102 | 7.07/12.0/16 | 4.07/11.0/13 |
| normal_contact_contingency | 124 | 9.21/14.0/15 | 8.32/13.0/14 |
| contact_release_contingency | 11 | 7.36/9.0/9 | 5.91/8.0/8 |
| precontact_transition | 237 | 8.58/13.0/15 | 7.49/13.0/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.92/6.0/7 | 8.07/25.0/35 | 1.39/6.0/7 | 307 |
| viability | 1.77/6.0/8 | 11.33/42.0/56 | 1.40/6.0/8 | 384 |
| intent | 1.57/5.0/8 | 8.31/26.0/40 | 1.35/5.0/8 | 473 |
| preference | 1.33/6.0/11 | 11.02/49.0/77 | 0.92/5.0/11 | 369 |
| style | 1.08/3.0/4 | 9.20/20.0/26 | 0.42/2.0/3 | 217 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 465 | 135 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `138` ticks.
Precontact sole-center tangential speed: p50 `2.8135 m/s`, p95 `6.3362 m/s`, max `7.9185 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.854 | 4.852 | 4.852 | 1.000 | 1.000 | 49.527 | 49.965 | 0.438 | 50.707 | 0.001 | 0 | 111 | 0 | 0 | 73 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2385.9 | 3076.4 | 5.00 | 33.53 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| 60–119 | 2485.1 | 3176.9 | 5.47 | 37.28 | 1.05 | 1.00 | 234.00 | 0.151 | 0.000 | 1.27e-09 | 3.93e-11 | 0 |
| 120–179 | 2448.3 | 2693.3 | 5.98 | 38.98 | 2.07 | 1.00 | 234.00 | 1.374 | 0.000 | 1.44e-09 | 5.03e-11 | 0 |
| 180–239 | 2428.3 | 3616.7 | 7.77 | 49.88 | 5.22 | 1.00 | 225.80 | 6.779 | 1.378 | 9.51e-10 | 3.93e-11 | 12 |
| 240–299 | 2546.4 | 4304.5 | 8.43 | 54.00 | 6.80 | 1.47 | 325.60 | 8.073 | 24.080 | 1.08e-09 | 1.89e-11 | 60 |
| 300–359 | 3000.2 | 4410.7 | 8.23 | 49.28 | 7.20 | 4.50 | 999.00 | 8.917 | 26.289 | 1.33e-09 | 1.45e-11 | 60 |
| 360–419 | 2405.3 | 4131.9 | 8.95 | 54.20 | 8.07 | 3.33 | 740.00 | 8.731 | 39.591 | 1.07e-09 | 1.55e-11 | 60 |
| 420–479 | 4008.3 | 141504.4 | 9.43 | 56.10 | 8.87 | 1577.87 | 340851.70 | 29.312 | 31.466 | 3.87e-10 | 7.27e-12 | 60 |
| 480–539 | 5960.7 | 100750.0 | 9.15 | 55.12 | 8.12 | 119.95 | 25908.50 | 90.109 | 38.283 | 1.65e-09 | 4.71e-11 | 60 |
| 540–599 | 3738.6 | 197402.9 | 8.37 | 50.95 | 7.40 | 7.65 | 1651.30 | 163.447 | 71.517 | 2.90e-09 | 7.99e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 59.969 | 32.266 | 83.949 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
