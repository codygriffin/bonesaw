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
| 265 | 1.325 s | 19.255 cm | 8.172 cm | 49.378 cm | 46.447 cm | 134.028° | 8.000 rad/s | 93984.9 µs |

Nominal hard residual maxima: dynamics `2.745e-09`, contact acceleration `9.739e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 152.584 cm |
| authored reference vs measured CoM RMS / p95 | 146.594 / 261.219 cm |
| stance foot RMS | 141.659 cm |
| swing foot RMS | 129.045 cm |
| hand RMS | 149.565 cm |
| maximum root rotation | 179.747° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.958e-09 |
| contact acceleration residual | 2.349e-10 |
| raw max dynamics residual, including rejected ticks | 8.958e-09 |
| raw max contact residual, including rejected ticks | 2.349e-10 |
| active normal force range | 0.000–857.933 N |
| centroidal momentum-rate residual RMS / max | 64.191 / 233.073 N·m |
| point-task acceleration RMS max | 146.650 m/s² |
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
| 600 | 3.0 s | 4545.2 µs | 239600.9 µs | 289166.6 µs | 316581.0 µs | 0 | 265 | 0 | 0 | 261 | 74 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 35101.1 | 70716.7 | 1770.7 | 134373.8 | 305159.6 | 315438.9 | 223154.2 | 600 | 274 | 137 | 28.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved_with_slack | 265 | 4286.8 | 7907.7 | 93984.9 | 112474.5 |
| normal_contact_contingency | 261 | 3672.2 | 90101.5 | 123837.0 | 162719.4 |
| contact_release_contingency | 74 | 225212.7 | 291843.7 | 302661.8 | 316581.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 9.19 | 15.0 | 22.0 | 24 | 8.25 | 21.0 | 23 | -0.0684 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 268.10/2227.9/5949.3/6909 | 59449.70/494604.9/1320742.4/1533798 | 3.29/25.0/38 | 2.63/24.0/37 | 22.44/207.0/337 | 0.1935 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved_with_slack | 265 | 8.74/13.4/24 | 7.77/13.4/23 |
| normal_contact_contingency | 261 | 10.03/22.0/24 | 9.20/22.0/23 |
| contact_release_contingency | 74 | 7.86/10.0/10 | 6.62/9.0/9 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.82/6.0/8 | 12.81/30.0/40 | 2.70/6.0/8 | 557 |
| viability | 1.88/6.0/8 | 12.27/40.0/56 | 1.88/6.0/8 | 600 |
| intent | 1.54/6.0/11 | 3.21/14.0/33 | 1.52/6.0/11 | 588 |
| preference | 1.56/13.0/18 | 15.21/115.0/180 | 1.32/13.0/18 | 466 |
| style | 1.39/7.0/12 | 13.05/80.0/157 | 0.84/6.0/12 | 309 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 229 | 0 | 172 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 265 | 335 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `172` ticks, normal fallback `338` ticks.
Touchdown Normal sole-center tangential speed: p50 `3.3299 m/s`, p95 `5.0208 m/s`, max `5.7430 m/s` over 171 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 21.061 | 21.053 | 21.052 | 1.000 | 1.000 | 45.422 | 46.109 | 0.688 | 46.109 | 0.001 | 0 | 175 | 0 | 0 | 601 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 5086.0 | 7925.5 | 8.62 | 49.67 | 7.47 | 6.37 | 1489.80 | 1.199 | 0.001 | 2.33e-09 | 5.05e-11 | 0 |
| 60–119 | 4138.1 | 6691.2 | 8.23 | 50.82 | 7.08 | 4.13 | 967.20 | 2.568 | 0.010 | 8.67e-10 | 4.29e-11 | 0 |
| 120–179 | 2931.7 | 13623.7 | 9.77 | 61.28 | 9.12 | 16.38 | 3833.70 | 11.552 | 3.778 | 2.74e-09 | 9.74e-11 | 0 |
| 180–239 | 3430.0 | 6331.7 | 8.13 | 48.63 | 7.23 | 6.60 | 1495.60 | 27.517 | 13.726 | 1.43e-09 | 5.17e-11 | 0 |
| 240–299 | 5097.4 | 167786.7 | 9.23 | 54.52 | 8.13 | 935.22 | 206803.30 | 65.393 | 79.962 | 8.96e-09 | 1.45e-10 | 35 |
| 300–359 | 2939.4 | 148157.4 | 8.55 | 49.38 | 7.65 | 6.95 | 1496.40 | 149.364 | 102.831 | 6.72e-09 | 2.35e-10 | 60 |
| 360–419 | 3274.7 | 4307.6 | 8.45 | 48.00 | 7.42 | 8.00 | 1728.00 | 191.814 | 152.760 | 1.28e-09 | 3.52e-11 | 60 |
| 420–479 | 134891.4 | 290644.8 | 8.37 | 46.35 | 7.27 | 599.13 | 132948.40 | 206.490 | 186.812 | 6.55e-09 | 2.15e-10 | 60 |
| 480–539 | 36540.4 | 305331.2 | 11.23 | 76.05 | 10.45 | 388.85 | 86266.90 | 235.297 | 215.921 | 4.73e-09 | 1.76e-10 | 60 |
| 540–599 | 17205.8 | 135211.6 | 11.32 | 80.83 | 10.68 | 709.32 | 157467.70 | 265.591 | 268.570 | 7.22e-09 | 1.15e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 600 | 152.584 | 139.340 | 149.565 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
