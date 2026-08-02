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
- Balance task: `rooted` `CoM` reference at `viability` priority with weight `1.000` and `0.500 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `1.000` / `2.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
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
| 324 | 1.620 s | 29.398 cm | 21.917 cm | 84.183 cm | 53.356 cm | 71.128° | 8.000 rad/s | 52729.4 µs |

Nominal hard residual maxima: dynamics `8.481e-09`, contact acceleration `3.368e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 135.950 cm |
| authored reference vs measured CoM RMS / p95 | 132.535 / 260.802 cm |
| stance foot RMS | 119.592 cm |
| swing foot RMS | 109.770 cm |
| hand RMS | 162.838 cm |
| maximum root rotation | 179.655° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.187e-09 |
| contact acceleration residual | 3.368e-10 |
| raw max dynamics residual, including rejected ticks | 9.187e-09 |
| raw max contact residual, including rejected ticks | 3.368e-10 |
| active normal force range | 0.000–689.850 N |
| centroidal momentum-rate residual RMS / max | 102.058 / 452.346 N·m |
| point-task acceleration RMS max | 171.271 m/s² |
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
| 600 | 3.0 s | 4674.0 µs | 141317.0 µs | 265633.9 µs | 278798.5 µs | 1 | 323 | 0 | 0 | 185 | 91 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 30364.8 | 56887.8 | 2080.7 | 130268.2 | 276011.0 | 278519.7 | 137084.3 | 600 | 261 | 130 | 32.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 1 | 2539.6 | 2539.6 | 2539.6 | 2539.6 |
| solved_with_slack | 323 | 3989.0 | 16919.4 | 52760.0 | 56983.1 |
| normal_contact_contingency | 185 | 5006.0 | 39828.8 | 90815.0 | 234752.0 |
| contact_release_contingency | 91 | 133447.5 | 268342.4 | 274610.3 | 278798.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 9.02 | 14.0 | 22.0 | 26 | 7.82 | 22.0 | 25 | -0.0724 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 100.63/8.0/3110.9/7041 | 22398.76/1872.0/690427.8/1563102 | 3.49/26.0/31 | 2.85/25.0/30 | 24.13/226.0/275 | 0.0880 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 1 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 323 | 8.93/17.0/24 | 7.54/15.8/22 |
| normal_contact_contingency | 185 | 9.76/24.2/26 | 8.91/24.0/25 |
| contact_release_contingency | 91 | 7.91/12.0/12 | 6.73/11.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.64/7.0/10 | 12.41/34.0/50 | 2.40/7.0/10 | 502 |
| viability | 2.13/9.0/14 | 13.71/48.1/74 | 2.08/9.0/13 | 599 |
| intent | 1.46/5.0/8 | 3.00/10.0/16 | 1.39/5.0/8 | 561 |
| preference | 1.57/12.0/20 | 16.12/128.0/187 | 1.33/12.0/20 | 476 |
| style | 1.22/4.0/9 | 10.80/42.1/87 | 0.62/4.0/9 | 273 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 229 | 0 | 172 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 324 | 276 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `172` ticks, normal fallback `279` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.8996 m/s`, p95 `5.2002 m/s`, max `9.6117 m/s` over 171 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 18.219 | 18.215 | 18.215 | 1.000 | 1.000 | 45.391 | 46.074 | 0.684 | 46.074 | 0.001 | 0 | 174 | 0 | 0 | 241 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2705.1 | 5146.0 | 7.83 | 48.12 | 5.42 | 2.98 | 698.10 | 0.607 | 0.001 | 1.21e-09 | 3.78e-11 | 0 |
| 60–119 | 4421.6 | 6881.5 | 8.62 | 52.33 | 6.55 | 5.20 | 1216.80 | 1.942 | 2.261 | 1.01e-09 | 5.45e-11 | 0 |
| 120–179 | 4717.9 | 7863.8 | 9.65 | 64.37 | 8.62 | 6.72 | 1571.70 | 5.358 | 11.420 | 8.71e-10 | 5.43e-11 | 0 |
| 180–239 | 2161.7 | 45677.5 | 8.98 | 57.95 | 8.13 | 119.85 | 28028.30 | 18.274 | 23.889 | 9.73e-10 | 1.90e-11 | 0 |
| 240–299 | 3778.3 | 55298.3 | 8.77 | 53.72 | 7.78 | 568.95 | 126306.90 | 48.157 | 78.341 | 1.98e-09 | 6.74e-11 | 0 |
| 300–359 | 7521.0 | 106213.7 | 9.62 | 59.32 | 8.85 | 7.77 | 1696.80 | 88.449 | 69.807 | 8.48e-09 | 3.37e-10 | 36 |
| 360–419 | 5200.0 | 163170.7 | 8.58 | 51.75 | 7.75 | 147.98 | 31958.00 | 156.592 | 120.082 | 9.19e-09 | 3.01e-10 | 60 |
| 420–479 | 128441.4 | 271825.7 | 8.03 | 44.55 | 7.12 | 132.90 | 29475.20 | 192.025 | 164.071 | 5.68e-09 | 9.88e-11 | 60 |
| 480–539 | 34393.7 | 131917.4 | 11.82 | 81.42 | 10.77 | 6.37 | 1398.80 | 207.710 | 179.836 | 4.06e-09 | 2.49e-10 | 60 |
| 540–599 | 73790.7 | 273767.0 | 8.32 | 46.92 | 7.25 | 7.57 | 1637.00 | 264.152 | 230.928 | 3.82e-09 | 1.70e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 600 | 135.950 | 117.781 | 162.838 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
