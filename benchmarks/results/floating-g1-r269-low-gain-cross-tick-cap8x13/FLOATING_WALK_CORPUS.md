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
- Whole-body posture: `preference` priority with weight `0.050`; morphology jet `enabled`.
- Protected coordinate posture: `intent` priority with weight `0.000` over 11 `upper-body` coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
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
5 ms p99 deadline: **PASS**
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
| `p99_tick_le_5ms` | PASS |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 863 | 4.315 s | 1.845 cm | 0.356 cm | 0.110 cm | 34.873 cm | 0.015° | 8.000 rad/s | 5346.0 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `5.972e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1702.591 cm |
| authored reference vs measured CoM RMS / p95 | 1693.140 / 3557.789 cm |
| stance foot RMS | 1580.529 cm |
| swing foot RMS | 1873.596 cm |
| hand RMS | 1721.142 cm |
| maximum root rotation | 120.383° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 5.286e-09 |
| contact acceleration residual | 7.424e-11 |
| raw max dynamics residual, including rejected ticks | 3.950e+02 |
| raw max contact residual, including rejected ticks | 4.131e+01 |
| active normal force range | 0.000–835.137 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 109.961 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 76 ticks |
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

- Phase ticks: unsupported `1255`, single support `422`, precontact `0`, multi-support `640`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 9.1 µs | 3698.5 µs | 4843.8 µs | 102348.0 µs | 312 | 554 | 0 | 141 | 1,257 | 48 | 1 | 4 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1348.8 | 2597.0 | 3.3 | 3278.2 | 6891.5 | 80272.4 | 2058.6 | 1,042 | 20 | 1 | 741.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 312 | 1881.4 | 3050.9 | 3099.4 | 3157.0 |
| solved_with_slack | 554 | 3119.4 | 4683.3 | 5760.8 | 7030.0 |
| normal_contact_contingency | 1,257 | 6.6 | 9.8 | 13.1 | 4597.5 |
| contact_release_contingency | 4 | 989.3 | 1515.6 | 1543.7 | 1550.7 |
| touchdown_transition | 141 | 2813.3 | 4421.9 | 4696.5 | 5038.5 |
| contact_solve_hold | 48 | 1305.5 | 1568.4 | 56747.1 | 102348.0 |
| localized_contact_handoff | 1 | 3199.8 | 3199.8 | 3199.8 | 3199.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.10 | 10.0 | 15.0 | 23 | 2.04 | 14.0 | 23 | 0.4970 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 2.40/8.0/78.7/192 | 528.09/1728.0/17003.5/42624 | 0.49/7.7/64 | 0.43/6.7/63 | 3.89/53.4/586 | 0.0466 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 312 | 4.55/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 554 | 7.90/18.5/23 | 6.12/16.9/21 |
| normal_contact_contingency | 1,257 | 0.04/0.0/12 | 0.04/0.0/11 |
| contact_release_contingency | 4 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 141 | 9.37/19.2/23 | 9.13/19.2/23 |
| contact_solve_hold | 48 | 0.00/0.0/0 | 0.00/0.0/0 |
| localized_contact_handoff | 1 | 8.00/8.0/8 | 7.00/7.0/7 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.58/5.0/7 | 2.36/20.0/34 | 0.20/5.0/7 | 144 |
| viability | 0.23/3.0/15 | 1.10/14.7/76 | 0.13/3.0/15 | 150 |
| intent | 1.03/9.0/15 | 2.06/18.0/30 | 0.79/9.0/14 | 505 |
| preference | 0.81/8.0/19 | 6.75/71.7/157 | 0.65/8.0/18 | 695 |
| style | 0.45/2.0/3 | 6.45/31.0/49 | 0.26/1.0/3 | 596 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,618 | 0 | 43 | 656 | 0 |
| right_ankle_roll_link | 1,314 | 0 | 110 | 863 | 30 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `76` ticks, normal fallback `15` ticks.
Touchdown Normal sole-center tangential speed: p50 `3.8834 m/s`, p95 `6.0646 m/s`, max `7.0425 m/s` over 149 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.125 | 3.125 | 3.125 | 1.000 | 1.000 | 52.379 | 55.871 | 3.492 | 55.871 | 0.002 | 0 | 818 | 0 | 0 | 23 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3017.3 | 6448.8 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 2084.3 | 4687.1 | 6.72 | 37.97 | 3.28 | 1.57 | 352.78 | 0.665 | 0.334 | 1.29e-09 | 4.22e-11 | 0 |
| 464–695 | 3050.9 | 4844.1 | 6.98 | 41.75 | 4.09 | 1.00 | 230.56 | 0.221 | 0.286 | 1.11e-09 | 5.97e-11 | 0 |
| 696–927 | 2994.3 | 5853.6 | 6.22 | 33.81 | 5.34 | 6.22 | 1389.00 | 9.834 | 6.045 | 3.66e+02 | 1.35e+01 | 59 |
| 928–1159 | 7.3 | 3430.1 | 1.94 | 11.92 | 1.88 | 0.86 | 185.28 | 310.639 | 303.274 | 1.60e-09 | 2.24e-11 | 180 |
| 1160–1391 | 6.7 | 3455.6 | 1.31 | 8.64 | 1.29 | 3.21 | 692.69 | 773.688 | 686.464 | 3.23e+02 | 3.85e+00 | 208 |
| 1392–1623 | 6.3 | 12.7 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1506.129 | 1409.457 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.6 | 4699.9 | 1.05 | 6.58 | 1.05 | 6.37 | 1376.42 | 2220.101 | 2120.965 | 3.95e+02 | 3.78e+01 | 203 |
| 1855–2085 | 6.6 | 11.0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2891.040 | 2800.527 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.9 | 4380.2 | 1.33 | 8.53 | 1.31 | 3.81 | 822.86 | 3579.468 | 3487.289 | 3.06e+02 | 4.13e+01 | 197 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1702.591 | 1642.611 | 1721.142 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
