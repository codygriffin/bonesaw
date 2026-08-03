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
- Balance task: `standalone-authored` `DCM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 462 | 2.310 s | 6.003 cm | 0.411 cm | 6.124 cm | 31.357 cm | 10.588° | 8.000 rad/s | 5655.1 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `4.840e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2234.776 cm |
| authored reference vs measured CoM RMS / p95 | 2232.733 / 4156.115 cm |
| stance foot RMS | 2178.303 cm |
| swing foot RMS | 2459.605 cm |
| hand RMS | 2234.261 cm |
| maximum root rotation | 86.091° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.242e-09 |
| contact acceleration residual | 6.841e-11 |
| raw max dynamics residual, including rejected ticks | 4.242e-09 |
| raw max contact residual, including rejected ticks | 6.841e-11 |
| active normal force range | 0.000–822.597 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 107.460 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 43 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `10.100` / `21.507 cm`.
- Virtual ZMP clipped on `74.57%` of ticks; clip-distance RMS / max `15.117` / `37.364 cm`.
- Measured-height natural frequency min / p50 / max: `3.747` / `3.812` / `4.071 rad/s`.
- Minimum measured CoM height: `0.592 m`; height-floor ticks: `0`.
- CoM command acceleration p95 / max: `1.018` / `1.192 m/s²`; support hull `4–6` vertices.
- Signed measured DCM support margin min / p05: `-15.778` / `-14.214 cm`; inside on `44.51%` of ticks.

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

- Phase ticks: unsupported `1763`, single support `254`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.129` / `0.129`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `137` / `417` / `5` / `0` / `0` / `0` / `1758`.
- Bound-limited ticks / upper-bound ticks: `2` / `1`; maximum coordinate violation `1.7637156188067045`.
- Named-linear-row-limited ticks / upper-row ticks: `5` / `0`; maximum row violation `262.39057008578993`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 7.6 µs | 3666.9 µs | 4832.4 µs | 211938.1 µs | 137 | 325 | 0 | 92 | 1,758 | 0 | 0 | 0 | 0 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 854.7 | 4607.7 | 1.1 | 3099.2 | 7533.5 | 165096.4 | 2063.3 | 559 | 19 | 1 | 1170.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 2983.8 | 3097.9 | 3169.5 | 3193.2 |
| solved_with_slack | 325 | 3220.0 | 4947.3 | 6652.7 | 7659.1 |
| normal_contact_contingency | 1,758 | 6.8 | 10.2 | 13.2 | 116.2 |
| contact_release_contingency | 5 | 1433.4 | 170018.9 | 203554.2 | 211938.1 |
| touchdown_transition | 92 | 2814.2 | 4089.2 | 6724.0 | 9685.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.75 | 9.0 | 12.0 | 18 | 1.27 | 11.8 | 16 | 0.2643 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.60/8.0/8.0/8 | 133.30/1728.0/1776.0/1776 | 0.13/3.0/12 | 0.08/2.0/11 | 0.62/16.0/86 | 0.1958 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 325 | 8.41/16.0/18 | 6.77/14.0/16 |
| normal_contact_contingency | 1,758 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 5 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 92 | 8.33/12.0/12 | 8.15/11.1/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.37/3.0/6 | 1.48/12.0/24 | 0.21/3.0/6 | 207 |
| viability | 0.19/4.0/9 | 0.98/16.0/46 | 0.15/4.0/8 | 196 |
| intent | 0.52/7.0/13 | 1.06/14.0/26 | 0.42/7.0/13 | 338 |
| preference | 0.39/5.0/14 | 3.58/47.0/112 | 0.32/5.0/14 | 413 |
| style | 0.27/2.0/3 | 3.83/29.8/49 | 0.17/2.0/3 | 336 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,995 | 0 | 22 | 300 | 0 |
| right_ankle_roll_link | 1,785 | 0 | 70 | 462 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `43` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `1.6622 m/s`, p95 `5.1421 m/s`, max `5.5718 m/s` over 89 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.981 | 1.981 | 1.980 | 1.000 | 1.000 | 52.871 | 56.242 | 3.371 | 56.242 | 0.002 | 0 | 800 | 0 | 0 | 3 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3050.4 | 6560.8 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 3103.5 | 5134.9 | 8.74 | 50.13 | 7.32 | 3.07 | 685.78 | 8.705 | 3.681 | 1.09e-09 | 4.22e-11 | 2 |
| 464–695 | 6.8 | 3252.2 | 0.74 | 4.59 | 0.73 | 0.58 | 124.76 | 178.410 | 180.379 | 1.70e-09 | 1.43e-11 | 210 |
| 696–927 | 6.8 | 12.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 698.549 | 697.460 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 7.7 | 2918.5 | 0.92 | 5.90 | 0.89 | 0.33 | 70.76 | 1292.189 | 1291.587 | 4.24e-09 | 6.84e-11 | 205 |
| 1160–1391 | 6.9 | 15.9 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1802.932 | 1806.816 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 7.6 | 13.9 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2386.613 | 2394.788 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.7 | 12.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2977.532 | 2983.592 | 0.00e+00 | 0.00e+00 | 231 |
| 1855–2085 | 6.8 | 11.7 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3571.694 | 3573.857 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 7.0 | 4915.6 | 1.65 | 10.87 | 1.62 | 1.00 | 216.94 | 4157.712 | 4155.844 | 3.78e-09 | 4.76e-11 | 188 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2234.776 | 2236.715 | 2234.261 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
