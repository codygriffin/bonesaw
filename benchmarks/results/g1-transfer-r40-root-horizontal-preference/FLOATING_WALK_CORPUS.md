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
| 390 | 1.950 s | 14.837 cm | 11.209 cm | 48.339 cm | 36.860 cm | 61.498° | 8.000 rad/s | 31514.1 µs |

Nominal hard residual maxima: dynamics `1.609e-09`, contact acceleration `5.558e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 103.753 cm |
| authored reference vs measured CoM RMS / p95 | 106.839 / 235.672 cm |
| stance foot RMS | 62.264 cm |
| swing foot RMS | 126.300 cm |
| hand RMS | 124.877 cm |
| maximum root rotation | 179.953° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.967e-09 |
| contact acceleration residual | 1.449e-10 |
| raw max dynamics residual, including rejected ticks | 7.967e-09 |
| raw max contact residual, including rejected ticks | 1.449e-10 |
| active normal force range | 0.000–944.008 N |
| centroidal momentum-rate residual RMS / max | 65.020 / 316.992 N·m |
| point-task acceleration RMS max | 106.770 m/s² |
| frame-angular acceleration RMS max | 222.869 rad/s² |
| longest pre-contact / touchdown transition | 162 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `94.996` / `222.313 cm`.
- Virtual ZMP clipped on `66.00%` of ticks; clip-distance RMS / max `124.817` / `346.003 cm`.
- Measured-height natural frequency min / p50 / max: `3.699` / `3.770` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.864 m`; height-floor ticks: `174`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-193.049` / `-161.856 cm`; inside on `34.33%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True`; final source tick `417.608`, progress `417.608` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `False`.
- Target / applied minimum rate: `0.0596` / `0.0603`; mean / p50 applied `0.6977` / `1.0000`.
- Limited / zero-rate hold ticks: `240` / `0`; maximum required landing time `0.8715 s`.
- Position / tangential / normal limiting ticks: `372` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `372`, frozen `0`.
- Maximum applied offset / root reach: `0.0800 / 0.8606 m`.
- Authored-offset / reach / slew limited ticks: `372 / 0 / 38`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `325` ticks; maximum active coordinates `10`; mean target/applied scale `0.579` / `0.663`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2674.6 µs | 41596.1 µs | 105759.9 µs | 180555.3 µs | 149 | 79 | 162 | 0 | 195 | 15 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8822.2 | 20620.9 | 559.0 | 10020.9 | 166226.5 | 179122.5 | 98660.3 | 600 | 130 | 57 | 113.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 149 | 2461.1 | 2556.7 | 2590.7 | 2632.9 |
| solved_with_slack | 79 | 2474.9 | 3067.7 | 3189.1 | 3235.6 |
| normal_contact_contingency | 195 | 5263.9 | 46881.4 | 51920.7 | 148743.2 |
| contact_release_contingency | 15 | 100496.3 | 163810.4 | 177206.4 | 180555.3 |
| precontact_transition | 162 | 3205.9 | 8853.7 | 44751.7 | 49912.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.80 | 11.0 | 13.0 | 14 | 5.46 | 12.0 | 13 | 0.1110 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 182.41/2148.3/3243.2/3395 | 39576.44/464032.8/700522.6/733320 | 1.48/10.0/11 | 1.13/9.0/10 | 9.19/78.0/89 | 0.4555 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 149 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 79 | 8.09/13.2/14 | 5.51/12.0/12 |
| normal_contact_contingency | 195 | 9.13/13.0/13 | 8.09/11.1/13 |
| contact_release_contingency | 15 | 7.60/9.9/10 | 5.47/7.9/8 |
| precontact_transition | 162 | 8.66/13.0/14 | 7.28/12.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.15/6.0/6 | 9.29/25.0/30 | 1.73/6.0/6 | 349 |
| viability | 1.75/6.0/8 | 10.34/40.0/48 | 1.42/6.0/8 | 407 |
| intent | 1.49/5.0/7 | 5.60/20.0/26 | 1.19/5.0/7 | 435 |
| preference | 1.38/6.0/6 | 13.12/55.0/68 | 0.86/5.0/6 | 337 |
| style | 1.03/2.0/4 | 8.47/18.0/32 | 0.27/2.0/3 | 151 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 390 | 210 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `213` ticks.
Precontact sole-center tangential speed: p50 `2.9682 m/s`, p95 `6.0760 m/s`, max `7.5388 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.294 | 5.292 | 5.292 | 1.000 | 1.000 | 49.457 | 49.961 | 0.504 | 50.531 | 0.001 | 0 | 113 | 0 | 0 | 73 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2452.0 | 2579.8 | 5.00 | 30.43 | 0.00 | 1.00 | 234.00 | 0.517 | 0.000 | 1.61e-09 | 5.25e-11 | 0 |
| 60–119 | 2474.5 | 2586.8 | 5.00 | 31.53 | 0.00 | 1.00 | 234.00 | 1.905 | 0.000 | 1.38e-09 | 5.56e-11 | 0 |
| 120–179 | 2483.4 | 3200.4 | 6.18 | 36.35 | 2.28 | 1.00 | 234.00 | 4.917 | 0.000 | 1.33e-09 | 4.86e-11 | 0 |
| 180–239 | 2373.7 | 3488.3 | 8.72 | 47.17 | 6.48 | 1.12 | 251.70 | 9.650 | 10.949 | 1.45e-09 | 4.49e-11 | 12 |
| 240–299 | 2275.4 | 31741.8 | 8.78 | 49.58 | 6.88 | 103.18 | 22906.70 | 11.528 | 34.685 | 9.32e-10 | 1.29e-11 | 60 |
| 300–359 | 3498.9 | 4482.0 | 8.27 | 47.22 | 7.27 | 7.65 | 1698.30 | 21.009 | 34.507 | 4.01e-10 | 9.95e-12 | 60 |
| 360–419 | 5460.6 | 90432.8 | 9.63 | 59.50 | 8.40 | 735.83 | 159972.70 | 47.164 | 87.506 | 9.47e-10 | 1.23e-11 | 60 |
| 420–479 | 6215.0 | 166441.7 | 9.35 | 60.42 | 7.88 | 296.15 | 63960.30 | 115.358 | 124.118 | 2.42e-09 | 3.37e-11 | 60 |
| 480–539 | 5286.9 | 107665.3 | 8.72 | 54.05 | 7.80 | 7.88 | 1700.30 | 199.145 | 130.289 | 7.97e-09 | 1.45e-10 | 60 |
| 540–599 | 3207.1 | 51998.8 | 8.37 | 51.88 | 7.58 | 669.32 | 144572.40 | 227.510 | 191.226 | 1.44e-09 | 1.08e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 103.753 | 88.949 | 124.877 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
