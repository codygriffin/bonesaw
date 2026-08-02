# Bonesaw floating G1 admitted-reference tracking

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: immutable open-loop-admitted artifact `benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz` from `Python-authored alternating sequence of Rust Bonesaw LIPM boundary plans`.
- Admission contract: authored root, CoM, foot jets, and contact schedule are consumed unchanged; eval-side reconstruction, projection, and retiming are rejected.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.186 m` forward per `11.585 s` source cycle.
- Applied mean forward speed: `0.016 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `immutable authored tick sequence` in `11.6 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Finite-support CoP constraint: disabled; no loaded finite patch was declared.
- Balance task: `standalone-authored` `CoM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `standalone-authored`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`; morphology jet `disabled`.
- Protected coordinate posture: `intent` priority with weight `0.000` over 11 `upper-body` coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `0` ticks (`0.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `disabled`; authored offset ≤ `0.120 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `40` scheduled ticks and throughout delayed admission.
- Coupled touchdown phase retiming: `disabled`; minimum rate `0.000`, guard `0.020 s`, engagement / release `0 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
- Coupled balance phase retiming: `disabled`; hold at signed DCM margin ≤ `-0.020 m`, recover nominal rate at `0.000 m`.
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
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 511 | 2.555 s | 4.794 cm | 0.404 cm | 1.223 cm | 27.932 cm | 5.564° | 8.000 rad/s | 7116.2 µs |

Nominal hard residual maxima: dynamics `1.473e-09`, contact acceleration `5.795e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2242.282 cm |
| authored reference vs measured CoM RMS / p95 | 2233.138 / 4390.133 cm |
| stance foot RMS | 2133.999 cm |
| swing foot RMS | 2463.404 cm |
| hand RMS | 2251.061 cm |
| maximum root rotation | 84.934° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.982e-09 |
| contact acceleration residual | 1.637e-10 |
| raw max dynamics residual, including rejected ticks | 4.676e+02 |
| raw max contact residual, including rejected ticks | 1.014e+02 |
| active normal force range | 0.000–605.270 N |
| centroidal momentum-rate residual RMS / max | 22.139 / 207.350 N·m |
| point-task acceleration RMS max | 121.683 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 49 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `False` (touchdown `False`, balance `False`); final source tick `2316.000`, progress `2316.000` ticks.
- First post-liftoff authored touchdown source tick: `529`; reached before trace end: `True`.
- Target / applied minimum rate: `1.0000` / `1.0000`; mean / p50 applied `1.0000` / `1.0000`.
- Limited / zero-rate hold ticks: `0` / `0`; maximum required landing time `0.0000 s`.
- Position / tangential / normal limiting ticks: `0` / `0` / `0`; unsafe-edge ticks `0`.
- Balance-margin limited ticks: `0`.

## Capture-aware landing

- Active target-ticks: `0`; policy updates `0`, frozen `0`.
- Maximum applied offset / root reach: `0.0000 / 0.0000 m`.
- Authored-offset / reach / slew limited ticks: `0 / 0 / 0`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1593`, single support `424`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 16.0 µs | 4535.8 µs | 6097.3 µs | 82909.5 µs | 16 | 495 | 0 | 148 | 1,595 | 59 | 1 | 3 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1083.1 | 2400.1 | 1.7 | 3606.9 | 7794.7 | 65693.5 | 2346.0 | 694 | 75 | 1 | 923.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 16 | 3063.8 | 3142.6 | 3184.3 | 3194.8 |
| solved_with_slack | 495 | 3433.3 | 5463.2 | 7123.4 | 8574.3 |
| normal_contact_contingency | 1,595 | 15.1 | 18.3 | 21.9 | 6201.6 |
| contact_release_contingency | 3 | 1245.6 | 1350.1 | 1359.4 | 1361.7 |
| touchdown_transition | 148 | 2792.4 | 6071.8 | 7332.9 | 7910.7 |
| contact_solve_hold | 59 | 742.5 | 1373.9 | 35923.9 | 82909.5 |
| localized_contact_handoff | 1 | 3067.3 | 3067.3 | 3067.3 | 3067.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.20 | 9.0 | 12.0 | 15 | 1.79 | 11.0 | 14 | 0.6172 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 3.06/8.0/80.0/192 | 669.83/1776.0/17280.0/41472 | 0.61/6.8/60 | 0.47/5.8/59 | 4.12/47.2/551 | 0.1091 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 16 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 495 | 7.51/13.1/15 | 5.85/12.0/14 |
| normal_contact_contingency | 1,595 | 0.03/0.0/14 | 0.03/0.0/13 |
| contact_release_contingency | 3 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 148 | 8.41/13.0/13 | 8.14/12.0/13 |
| contact_solve_hold | 59 | 0.00/0.0/0 | 0.00/0.0/0 |
| localized_contact_handoff | 1 | 7.00/7.0/7 | 5.00/5.0/5 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.48/4.0/7 | 1.97/16.0/35 | 0.29/4.0/7 | 323 |
| viability | 0.21/3.0/9 | 1.03/15.0/48 | 0.14/3.0/9 | 199 |
| intent | 0.69/6.8/12 | 3.75/36.0/60 | 0.63/6.8/12 | 521 |
| preference | 0.50/4.0/10 | 6.46/57.0/134 | 0.49/4.0/10 | 650 |
| style | 0.31/2.0/3 | 4.18/32.0/37 | 0.25/2.0/3 | 528 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,901 | 0 | 116 | 300 | 0 |
| right_ankle_roll_link | 1,708 | 0 | 57 | 511 | 41 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `61` ticks, normal fallback `21` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.8600 m/s`, p95 `6.9954 m/s`, max `9.0904 m/s` over 169 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.510 | 2.510 | 2.510 | 1.000 | 1.000 | 52.891 | 56.375 | 3.484 | 56.375 | 0.002 | 0 | 906 | 0 | 0 | 21 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3836.2 | 7406.3 | 6.26 | 58.94 | 4.30 | 2.48 | 579.96 | 0.711 | 0.404 | 1.47e-09 | 5.79e-11 | 0 |
| 232–463 | 3201.9 | 5686.2 | 8.37 | 63.20 | 6.90 | 4.59 | 1022.61 | 3.936 | 0.882 | 7.73e-10 | 4.50e-11 | 0 |
| 464–695 | 738.9 | 5659.8 | 3.42 | 25.33 | 2.89 | 10.66 | 2308.89 | 82.448 | 68.711 | 3.70e+02 | 3.77e+01 | 143 |
| 696–927 | 15.0 | 21.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 530.104 | 521.983 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 14.8 | 1903.5 | 0.54 | 3.66 | 0.52 | 3.21 | 692.69 | 1096.498 | 1089.995 | 4.68e+02 | 1.34e+01 | 215 |
| 1160–1391 | 15.1 | 22.3 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1610.421 | 1604.629 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 15.0 | 21.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2264.519 | 2256.633 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 15.9 | 3080.7 | 1.90 | 12.90 | 1.87 | 5.65 | 1219.32 | 2907.484 | 2848.735 | 3.38e+02 | 3.55e+01 | 182 |
| 1855–2085 | 15.1 | 21.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3635.980 | 3552.130 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 14.4 | 7379.1 | 1.45 | 9.79 | 1.43 | 4.05 | 875.22 | 4416.484 | 4333.030 | 1.43e+02 | 1.01e+02 | 191 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2242.282 | 2203.021 | 2251.061 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
