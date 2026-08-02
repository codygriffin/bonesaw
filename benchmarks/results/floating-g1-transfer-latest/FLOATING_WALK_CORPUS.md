# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Finite-support CoP constraint: disabled; no loaded finite patch was declared.
- Balance task: `rooted` `CoM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
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
| `dynamics_residual_le_1e_8` | FAIL |
| `contact_residual_le_1e_8` | FAIL |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `dynamics_residual_le_1e_8`, `contact_residual_le_1e_8`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 351 | 1.755 s | 18.942 cm | 3.960 cm | 18.184 cm | 32.021 cm | 32.481° | 8.000 rad/s | 97394.2 µs |

Nominal hard residual maxima: dynamics `7.469e-09`, contact acceleration `3.301e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 46.420 cm |
| authored reference vs measured CoM RMS / p95 | 41.734 / 67.443 cm |
| stance foot RMS | 25.665 cm |
| swing foot RMS | 32.730 cm |
| hand RMS | 69.835 cm |
| maximum root rotation | 32.481° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.043e+02 |
| contact acceleration residual | 4.394e+01 |
| raw max dynamics residual, including rejected ticks | 3.043e+02 |
| raw max contact residual, including rejected ticks | 4.394e+01 |
| active normal force range | 0.000–335.136 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 96.964 m/s² |
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
| 600 | 3.0 s | 4883.8 µs | 271820.6 µs | 276773.8 µs | 376164.7 µs | 199 | 152 | 0 | 0 | 249 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 110403.4 | 122175.9 | 3200.8 | 271160.7 | 328433.0 | 371391.6 | 50939.0 | 600 | 300 | 281 | 9.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 3018.5 | 3085.2 | 3110.3 | 3148.6 |
| solved_with_slack | 152 | 3554.0 | 92335.1 | 98998.6 | 100508.8 |
| normal_contact_contingency | 249 | 270424.0 | 273950.9 | 290017.2 | 376164.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.43 | 10.0 | 13.0 | 17 | 1.83 | 11.0 | 17 | -0.7856 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 3250.47/7296.0/7296.0/7296 | 734158.18/1663488.0/1663488.0/1663488 | 27.10/64.0/64 | 26.57/63.0/63 | 249.33/598.0/598 | 0.9786 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 152 | 8.31/14.0/17 | 7.24/14.0/17 |
| normal_contact_contingency | 249 | 0.00/0.0/0 | 0.00/0.0/0 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.93/5.0/10 | 3.58/20.0/50 | 0.51/5.0/10 | 108 |
| viability | 0.36/4.0/5 | 1.47/15.0/24 | 0.27/4.0/5 | 99 |
| intent | 0.75/4.0/10 | 1.54/8.0/30 | 0.37/4.0/10 | 124 |
| preference | 0.78/5.0/8 | 6.75/40.1/82 | 0.45/5.0/8 | 152 |
| style | 0.61/2.0/4 | 8.48/22.0/42 | 0.23/2.0/3 | 125 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 229 | 0 | 172 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 600 | 0 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `172` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `3.7940 m/s`, p95 `3.7940 m/s`, max `3.7940 m/s` over 171 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 66.242 | 66.229 | 66.229 | 1.000 | 1.000 | 52.887 | 53.562 | 0.676 | 53.562 | 0.001 | 0 | 178 | 0 | 2 | 605 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 3020.7 | 3124.5 | 4.00 | 28.25 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 60–119 | 3022.7 | 3124.3 | 4.00 | 28.28 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 120–179 | 3015.2 | 3097.4 | 4.00 | 28.45 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 180–239 | 2381.6 | 3179.6 | 6.12 | 41.40 | 2.73 | 1.00 | 225.80 | 0.145 | 0.009 | 1.14e-09 | 5.87e-11 | 0 |
| 240–299 | 3520.6 | 4843.2 | 8.27 | 45.37 | 7.78 | 6.72 | 1491.10 | 10.474 | 2.662 | 4.12e-10 | 1.14e-11 | 0 |
| 300–359 | 62619.8 | 210358.4 | 7.93 | 46.57 | 7.82 | 3527.62 | 783130.90 | 52.714 | 25.910 | 3.04e+02 | 4.39e+01 | 9 |
| 360–419 | 209507.3 | 212865.3 | 0.00 | 0.00 | 0.00 | 7104.00 | 1577088.00 | 75.038 | 37.404 | 3.04e+02 | 4.39e+01 | 60 |
| 420–479 | 268042.2 | 329150.2 | 0.00 | 0.00 | 0.00 | 7270.40 | 1651968.00 | 69.097 | 39.754 | 3.04e+02 | 4.39e+01 | 60 |
| 480–539 | 271178.2 | 280931.5 | 0.00 | 0.00 | 0.00 | 7296.00 | 1663488.00 | 64.032 | 42.501 | 3.04e+02 | 4.39e+01 | 60 |
| 540–599 | 270715.9 | 276389.0 | 0.00 | 0.00 | 0.00 | 7296.00 | 1663488.00 | 64.454 | 49.877 | 3.04e+02 | 4.39e+01 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 46.420 | 28.199 | 69.835 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
