# Bonesaw floating G1 admitted-reference tracking

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: immutable open-loop-admitted artifact `benchmarks/results/g1-bonesaw-lipm-r43/reference-inputs.npz` from `Bonesaw two-stage support-constrained LIPM`.
- Admission contract: authored root, CoM, foot jets, and contact schedule are consumed unchanged; eval-side reconstruction, projection, and retiming are rejected.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.228 m` forward per `3.000 s` source cycle.
- Applied mean forward speed: `0.076 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `immutable authored tick sequence` in `3.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `rooted` `CoM` reference at `viability` priority with weight `1.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `viability` priority with weight `1.000` and `1.000 Hz` response.
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
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 241 | 1.205 s | 20.860 cm | 1.730 cm | 22.031 cm | 48.972 cm | 117.795° | 8.000 rad/s | 41588.4 µs |

Nominal hard residual maxima: dynamics `2.791e-09`, contact acceleration `1.029e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 245.362 cm |
| authored reference vs measured CoM RMS / p95 | 246.189 / 535.922 cm |
| stance foot RMS | 280.174 cm |
| swing foot RMS | 134.938 cm |
| hand RMS | 266.510 cm |
| maximum root rotation | 179.945° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.730e-09 |
| contact acceleration residual | 3.765e-10 |
| raw max dynamics residual, including rejected ticks | 9.730e-09 |
| raw max contact residual, including rejected ticks | 3.765e-10 |
| active normal force range | 0.000–840.319 N |
| centroidal momentum-rate residual RMS / max | 58.235 / 386.258 N·m |
| point-task acceleration RMS max | 151.218 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `False` (touchdown `False`, balance `False`); final source tick `599.000`, progress `599.000` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `True`.
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

- Phase ticks: unsupported `1`, single support `229`, precontact `0`, multi-support `370`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 5131.2 µs | 250552.8 µs | 285475.6 µs | 300312.4 µs | 0 | 241 | 0 | 0 | 271 | 88 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 38279.8 | 72713.4 | 2443.3 | 135531.5 | 295779.7 | 299859.2 | 204411.5 | 600 | 307 | 171 | 26.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved_with_slack | 241 | 4534.0 | 12915.1 | 41588.4 | 102815.9 |
| normal_contact_contingency | 271 | 5490.4 | 52687.4 | 101401.5 | 120655.4 |
| contact_release_contingency | 88 | 200692.5 | 286688.9 | 293729.0 | 300312.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 9.85 | 19.0 | 22.0 | 25 | 8.89 | 22.0 | 24 | -0.0924 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 127.94/94.0/4137.3/6079 | 28595.35/20314.8/918489.5/1349538 | 4.77/26.0/29 | 4.08/25.0/28 | 35.07/227.1/256 | 0.0962 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved_with_slack | 241 | 9.12/16.6/25 | 8.21/16.2/24 |
| normal_contact_contingency | 271 | 11.15/23.0/25 | 10.37/22.3/24 |
| contact_release_contingency | 88 | 7.81/11.3/13 | 6.24/10.1/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 3.07/11.0/16 | 14.11/51.0/69 | 2.91/11.0/16 | 558 |
| viability | 1.92/7.0/8 | 12.58/42.0/54 | 1.92/7.0/8 | 600 |
| intent | 1.59/6.0/9 | 3.12/12.0/18 | 1.56/6.0/9 | 591 |
| preference | 1.91/14.0/20 | 19.14/140.0/215 | 1.67/13.0/20 | 468 |
| style | 1.36/7.0/9 | 11.83/66.0/89 | 0.83/6.0/8 | 319 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 229 | 0 | 172 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 241 | 359 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `172` ticks, normal fallback `362` ticks.
Touchdown Normal sole-center tangential speed: p50 `5.1721 m/s`, p95 `7.3177 m/s`, max `7.6515 m/s` over 171 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 22.968 | 22.954 | 22.954 | 0.999 | 0.999 | 45.164 | 45.871 | 0.707 | 45.871 | 0.001 | 0 | 180 | 0 | 0 | 1,276 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 4481.4 | 6816.2 | 8.72 | 50.75 | 7.58 | 6.25 | 1462.50 | 0.842 | 0.001 | 1.66e-09 | 5.01e-11 | 0 |
| 60–119 | 4496.6 | 6102.1 | 8.75 | 52.85 | 7.60 | 5.78 | 1353.30 | 3.079 | 2.582 | 1.19e-09 | 4.81e-11 | 0 |
| 120–179 | 4491.2 | 6972.1 | 9.92 | 65.32 | 9.17 | 5.20 | 1216.80 | 14.817 | 2.063 | 2.42e-09 | 6.49e-11 | 0 |
| 180–239 | 6628.8 | 76938.5 | 9.05 | 58.92 | 8.42 | 217.53 | 50837.20 | 38.316 | 12.538 | 2.79e-09 | 1.03e-10 | 0 |
| 240–299 | 2893.1 | 67441.9 | 9.12 | 52.55 | 7.97 | 84.55 | 18263.60 | 94.214 | 51.061 | 2.19e-09 | 6.57e-11 | 59 |
| 300–359 | 3524.3 | 206350.4 | 8.28 | 47.75 | 7.40 | 6.72 | 1445.10 | 169.921 | 106.685 | 5.07e-09 | 1.11e-10 | 60 |
| 360–419 | 4385.3 | 7536.8 | 8.55 | 50.07 | 7.37 | 30.38 | 6562.80 | 176.373 | 183.048 | 3.58e-09 | 1.12e-10 | 60 |
| 420–479 | 26728.7 | 286051.7 | 12.07 | 78.17 | 11.40 | 8.00 | 1739.20 | 277.731 | 318.832 | 4.69e-09 | 2.08e-10 | 60 |
| 480–539 | 51458.6 | 287769.7 | 13.63 | 88.52 | 12.97 | 909.92 | 201979.90 | 412.791 | 445.767 | 2.38e-09 | 1.31e-10 | 60 |
| 540–599 | 133004.4 | 294794.9 | 10.38 | 63.00 | 9.08 | 5.08 | 1093.10 | 532.861 | 567.359 | 9.73e-09 | 3.76e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 600 | 245.362 | 258.829 | 266.510 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
