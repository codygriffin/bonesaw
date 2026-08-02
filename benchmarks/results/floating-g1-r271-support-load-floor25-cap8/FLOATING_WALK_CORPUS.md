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
- Finite-support CoP constraint: `disabled`; required `0.000 mm`, measured minimum `0.000 mm` over `393` loaded ticks.
- Balance task: `standalone-authored` `CoM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `standalone-authored`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.050`; morphology jet `enabled`.
- Protected coordinate posture: `intent` priority with weight `0.000` over 11 `upper-body` coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.250`, activating at `50.0%` of each effective limit with `2.000 Hz` response and `multi-support` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Joint-velocity envelope scope: `lower-body allowlist`; hard acceleration-bound intersection `enabled`.
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
| 308 | 1.540 s | 2.421 cm | 0.404 cm | 0.373 cm | 27.543 cm | 0.012° | 7.266 rad/s | 4945.2 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `6.302e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2896.020 cm |
| authored reference vs measured CoM RMS / p95 | 2889.691 / 5290.456 cm |
| stance foot RMS | 2797.939 cm |
| swing foot RMS | 3154.311 cm |
| hand RMS | 2906.162 cm |
| maximum root rotation | 15.134° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.961e-09 |
| contact acceleration residual | 6.302e-11 |
| raw max dynamics residual, including rejected ticks | 2.961e-09 |
| raw max contact residual, including rejected ticks | 6.302e-11 |
| active normal force range | 0.000–932.048 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 139.946 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 25 ticks |
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

- Phase ticks: unsupported `1924`, single support `93`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.129` / `0.129`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 6.7 µs | 3117.1 µs | 3995.6 µs | 6662.2 µs | 137 | 171 | 0 | 85 | 1,919 | 0 | 0 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 511.3 | 1144.1 | 0.9 | 2937.6 | 5127.5 | 6414.0 | 1457.0 | 398 | 3 | 0 | 1955.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 2934.8 | 3033.3 | 3345.0 | 3420.3 |
| solved_with_slack | 171 | 3185.2 | 4646.4 | 5323.9 | 6662.2 |
| normal_contact_contingency | 1,919 | 6.6 | 9.3 | 12.1 | 15.1 |
| contact_release_contingency | 5 | 1344.9 | 1402.4 | 1411.2 | 1413.4 |
| touchdown_transition | 85 | 1834.8 | 3306.2 | 3449.3 | 3488.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.09 | 8.0 | 10.0 | 22 | 0.72 | 9.0 | 21 | 0.8979 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.23/1.0/1.0/8 | 53.84/244.0/244.0/1768 | 0.02/0.0/3 | 0.01/0.0/2 | 0.09/0.0/17 | 0.6385 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 171 | 7.37/14.2/22 | 5.73/13.2/21 |
| normal_contact_contingency | 1,919 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 5 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 85 | 8.31/12.2/13 | 8.05/12.2/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.24/3.0/6 | 0.94/12.0/24 | 0.11/3.0/6 | 87 |
| viability | 0.07/2.0/5 | 0.39/12.0/32 | 0.07/2.0/5 | 89 |
| intent | 0.33/5.0/14 | 0.67/10.0/28 | 0.25/5.0/14 | 195 |
| preference | 0.27/4.0/18 | 2.24/32.0/132 | 0.21/4.0/18 | 251 |
| style | 0.18/1.0/3 | 2.63/17.0/50 | 0.09/1.0/3 | 211 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,972 | 0 | 45 | 300 | 0 |
| right_ankle_roll_link | 1,969 | 0 | 40 | 308 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `25` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.1491 m/s`, p95 `3.7195 m/s`, max `4.0007 m/s` over 81 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.185 | 1.185 | 1.185 | 1.000 | 1.000 | 52.391 | 55.727 | 3.336 | 55.727 | 0.002 | 0 | 793 | 0 | 0 | 5 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2992.6 | 4521.8 | 5.34 | 36.09 | 2.17 | 1.00 | 244.00 | 0.368 | 0.404 | 1.69e-09 | 6.30e-11 | 0 |
| 232–463 | 7.5 | 4930.1 | 2.46 | 13.71 | 2.05 | 0.33 | 79.34 | 106.940 | 107.154 | 9.28e-10 | 5.36e-11 | 156 |
| 464–695 | 6.7 | 3348.4 | 0.89 | 5.12 | 0.87 | 0.38 | 83.83 | 555.454 | 542.679 | 2.96e-09 | 1.77e-11 | 207 |
| 696–927 | 6.7 | 10.9 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1162.048 | 1137.654 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 6.0 | 2891.1 | 0.78 | 4.65 | 0.75 | 0.37 | 80.97 | 1800.074 | 1773.319 | 1.74e-09 | 1.07e-11 | 210 |
| 1160–1391 | 6.7 | 12.3 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2443.151 | 2415.832 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.2 | 13.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3119.356 | 3096.770 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.6 | 2442.8 | 0.73 | 4.76 | 0.71 | 0.12 | 25.83 | 3811.581 | 3783.773 | 1.67e-09 | 2.59e-11 | 211 |
| 1855–2085 | 6.6 | 11.6 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 4554.546 | 4518.631 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.9 | 2569.8 | 0.65 | 4.37 | 0.62 | 0.11 | 23.92 | 5305.868 | 5266.320 | 1.11e-09 | 1.02e-11 | 213 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2896.020 | 2871.891 | 2906.162 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
