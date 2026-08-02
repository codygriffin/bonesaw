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
| 482 | 2.410 s | 23.177 cm | 2.843 cm | 54.944 cm | 41.388 cm | 82.309° | 8.000 rad/s | 58992.1 µs |

Nominal hard residual maxima: dynamics `2.737e-09`, contact acceleration `9.337e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 150.373 cm |
| authored reference vs measured CoM RMS / p95 | 145.843 / 324.352 cm |
| stance foot RMS | 123.898 cm |
| swing foot RMS | 159.555 cm |
| hand RMS | 164.107 cm |
| maximum root rotation | 179.966° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 5.968e-09 |
| contact acceleration residual | 1.204e-10 |
| raw max dynamics residual, including rejected ticks | 5.968e-09 |
| raw max contact residual, including rejected ticks | 1.204e-10 |
| active normal force range | 0.000–866.048 N |
| centroidal momentum-rate residual RMS / max | 58.054 / 331.307 N·m |
| point-task acceleration RMS max | 160.890 m/s² |
| frame-angular acceleration RMS max | 240.683 rad/s² |
| longest pre-contact / touchdown transition | 254 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `153.739` / `359.271 cm`.
- Virtual ZMP clipped on `69.12%` of ticks; clip-distance RMS / max `224.376` / `720.196 cm`.
- Measured-height natural frequency min / p50 / max: `3.664` / `3.774` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.752 m`; height-floor ticks: `312`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-339.732` / `-269.960 cm`; inside on `33.25%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True`; final source tick `425.992`, progress `425.992` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `False`.
- Target / applied minimum rate: `0.0089` / `0.0090`; mean / p50 applied `0.5337` / `0.4882`.
- Limited / zero-rate hold ticks: `453` / `0`; maximum required landing time `1.1318 s`.
- Position / tangential / normal limiting ticks: `572` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `572`; policy updates `572`, frozen `0`.
- Maximum applied offset / root reach: `0.0800 / 0.8222 m`.
- Authored-offset / reach / slew limited ticks: `572 / 0 / 24`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `571`, multi-support `199`.
- Joint-velocity envelope active on `457` ticks; maximum active coordinates `8`; mean target/applied scale `0.498` / `0.570`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 800 | 4.0 s | 2627.7 µs | 53644.4 µs | 195824.2 µs | 366958.2 µs | 126 | 102 | 254 | 0 | 286 | 32 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11254.7 | 33462.4 | 376.1 | 12858.8 | 261647.0 | 356427.1 | 107540.5 | 800 | 163 | 58 | 88.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 126 | 2437.7 | 2557.0 | 2598.9 | 2674.4 |
| solved_with_slack | 102 | 2519.9 | 2780.8 | 4007.0 | 4431.3 |
| normal_contact_contingency | 286 | 3794.2 | 14312.8 | 18917.6 | 39845.7 |
| contact_release_contingency | 32 | 164155.8 | 221419.1 | 326099.1 | 366958.2 |
| precontact_transition | 254 | 2511.8 | 46037.4 | 62347.5 | 66290.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.15 | 12.0 | 14.0 | 25 | 6.25 | 13.0 | 21 | 0.0447 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 98.57/8.0/3300.1/4138 | 21832.51/1776.0/732613.3/918636 | 1.85/15.0/17 | 1.52/14.0/16 | 12.46/124.0/142 | 0.2097 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 126 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 102 | 7.22/14.0/25 | 4.25/11.0/21 |
| normal_contact_contingency | 286 | 9.20/14.0/14 | 8.31/13.0/14 |
| contact_release_contingency | 32 | 7.81/10.0/10 | 6.47/9.0/9 |
| precontact_transition | 254 | 8.96/13.0/15 | 7.82/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.16/6.0/8 | 9.88/30.0/40 | 1.73/6.0/8 | 479 |
| viability | 1.80/6.0/8 | 11.66/40.0/56 | 1.52/6.0/8 | 586 |
| intent | 1.55/5.0/8 | 8.13/25.0/38 | 1.38/5.0/8 | 670 |
| preference | 1.50/6.0/20 | 13.39/63.0/140 | 1.15/6.0/19 | 553 |
| style | 1.14/4.0/5 | 9.23/29.0/45 | 0.47/4.0/5 | 293 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 572 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 482 | 318 |
| left_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `572` ticks, planned normal touchdown `0` ticks, normal fallback `321` ticks.
Precontact sole-center tangential speed: p50 `3.1443 m/s`, p95 `8.2954 m/s`, max `11.7597 m/s` over 572 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9.004 | 9.000 | 9.000 | 1.000 | 1.000 | 48.438 | 49.332 | 0.895 | 50.016 | 0.001 | 0 | 209 | 0 | 0 | 242 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–79 | 2418.1 | 2545.5 | 5.00 | 33.77 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| 80–159 | 2498.3 | 2956.4 | 5.65 | 37.64 | 1.35 | 1.00 | 234.00 | 0.607 | 0.000 | 1.27e-09 | 5.03e-11 | 0 |
| 160–239 | 2517.4 | 3308.9 | 7.72 | 49.48 | 5.11 | 1.00 | 227.85 | 5.686 | 1.100 | 1.35e-09 | 5.23e-11 | 12 |
| 240–319 | 2483.3 | 3917.3 | 8.66 | 54.81 | 7.22 | 1.44 | 319.12 | 8.033 | 25.126 | 1.12e-09 | 2.38e-11 | 80 |
| 320–399 | 2399.0 | 3815.5 | 8.47 | 54.45 | 7.33 | 1.61 | 357.98 | 20.287 | 40.890 | 1.02e-09 | 2.65e-11 | 80 |
| 400–479 | 2780.1 | 64735.8 | 9.74 | 61.94 | 8.97 | 892.38 | 198107.25 | 50.579 | 54.807 | 2.74e-09 | 9.34e-11 | 80 |
| 480–559 | 12471.6 | 243104.2 | 8.89 | 53.15 | 8.00 | 63.64 | 13745.33 | 131.607 | 86.686 | 5.35e-09 | 1.16e-10 | 80 |
| 560–639 | 2750.5 | 3375.6 | 8.47 | 53.89 | 7.39 | 6.34 | 1368.90 | 185.231 | 132.500 | 1.53e-09 | 9.10e-12 | 80 |
| 640–719 | 5309.5 | 13518.7 | 9.49 | 63.73 | 8.91 | 8.00 | 1728.00 | 236.163 | 215.658 | 3.64e-09 | 6.63e-11 | 80 |
| 720–799 | 4983.9 | 211708.3 | 9.43 | 60.09 | 8.26 | 9.29 | 2002.72 | 340.073 | 338.502 | 5.97e-09 | 1.20e-10 | 80 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 800 | 150.373 | 138.373 | 164.107 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
