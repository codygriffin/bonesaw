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
| 449 | 2.245 s | 4.255 cm | 0.404 cm | 2.401 cm | 32.340 cm | 4.637° | 8.000 rad/s | 9964.5 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `4.840e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2448.929 cm |
| authored reference vs measured CoM RMS / p95 | 2441.912 / 4450.521 cm |
| stance foot RMS | 2363.585 cm |
| swing foot RMS | 2668.691 cm |
| hand RMS | 2447.314 cm |
| maximum root rotation | 51.447° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.523e-09 |
| contact acceleration residual | 3.157e-10 |
| raw max dynamics residual, including rejected ticks | 4.523e-09 |
| raw max contact residual, including rejected ticks | 3.157e-10 |
| active normal force range | 0.000–696.956 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 152.569 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 52 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `8.711` / `19.393 cm`.
- Virtual ZMP clipped on `66.45%` of ticks; clip-distance RMS / max `13.141` / `34.495 cm`.
- Measured-height natural frequency min / p50 / max: `3.747` / `3.811` / `3.878 rad/s`.
- Minimum measured CoM height: `0.652 m`; height-floor ticks: `0`.
- CoM command acceleration p95 / max: `0.971` / `1.046 m/s²`; support hull `4–6` vertices.
- Signed measured DCM support margin min / p05: `-16.717` / `-15.192 cm`; inside on `36.77%` of ticks.

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

- Phase ticks: unsupported `1727`, single support `290`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.129` / `0.129`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `137` / `453` / `5` / `0` / `0` / `0` / `1722`.
- Bound-limited ticks / upper-bound ticks: `2` / `0`; maximum coordinate violation `8.17250151218613`.
- Named-linear-row-limited ticks / upper-row ticks: `5` / `2`; maximum row violation `112.48389197090785`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 7.3 µs | 4427.5 µs | 6041.8 µs | 217516.2 µs | 137 | 312 | 0 | 141 | 1,722 | 0 | 0 | 0 | 0 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1124.0 | 6224.5 | 1.3 | 4093.6 | 22084.3 | 210748.7 | 2836.8 | 595 | 54 | 4 | 889.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 4315.6 | 4503.8 | 15330.5 | 22135.9 |
| solved_with_slack | 312 | 3296.7 | 6124.2 | 7938.8 | 10082.3 |
| normal_contact_contingency | 1,722 | 6.8 | 10.0 | 14.0 | 31.2 |
| contact_release_contingency | 5 | 1497.6 | 211672.1 | 216347.4 | 217516.2 |
| touchdown_transition | 141 | 2150.9 | 4518.7 | 5310.0 | 21972.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.94 | 10.0 | 13.0 | 20 | 1.47 | 12.8 | 20 | 0.2068 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.67/8.0/8.0/8 | 148.66/1728.0/1776.0/1776 | 0.15/3.0/22 | 0.09/2.0/21 | 0.74/16.0/184 | 0.1497 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 312 | 8.57/16.0/18 | 6.93/14.9/18 |
| normal_contact_contingency | 1,722 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 5 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 141 | 9.08/16.2/20 | 8.85/16.2/20 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.42/4.0/6 | 1.72/17.8/30 | 0.25/4.0/6 | 242 |
| viability | 0.26/5.0/14 | 1.40/24.0/84 | 0.23/4.0/14 | 244 |
| intent | 0.55/7.0/13 | 1.12/14.0/36 | 0.45/7.0/13 | 373 |
| preference | 0.43/5.0/14 | 3.88/50.8/112 | 0.36/5.0/14 | 447 |
| style | 0.29/2.0/3 | 4.00/29.0/49 | 0.18/2.0/3 | 369 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,961 | 0 | 56 | 300 | 0 |
| right_ankle_roll_link | 1,783 | 0 | 85 | 449 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `52` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.6613 m/s`, p95 `3.8472 m/s`, max `4.0507 m/s` over 137 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.605 | 2.556 | 2.555 | 0.981 | 0.981 | 52.863 | 56.227 | 3.363 | 56.227 | 0.002 | 0 | 800 | 0 | 0 | 50 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 4356.9 | 11464.2 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 3136.1 | 4884.0 | 8.48 | 48.89 | 7.15 | 3.50 | 780.52 | 7.842 | 2.553 | 1.22e-09 | 4.66e-11 | 15 |
| 464–695 | 6.8 | 3149.6 | 1.07 | 6.39 | 1.01 | 0.71 | 152.69 | 218.805 | 208.536 | 2.39e-09 | 2.35e-11 | 201 |
| 696–927 | 6.8 | 12.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 813.708 | 792.670 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 6.2 | 3994.8 | 2.17 | 13.29 | 2.14 | 0.44 | 94.03 | 1485.007 | 1464.608 | 4.52e-09 | 3.16e-10 | 180 |
| 1160–1391 | 6.9 | 18.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2082.803 | 2055.342 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.8 | 14.8 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2682.405 | 2656.616 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 7.0 | 2996.3 | 1.00 | 6.08 | 0.96 | 0.38 | 82.29 | 3279.666 | 3254.587 | 9.05e-10 | 5.18e-12 | 206 |
| 1855–2085 | 6.9 | 14.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3866.936 | 3836.340 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 6.1 | 5000.1 | 1.29 | 8.76 | 1.27 | 0.66 | 142.13 | 4464.709 | 4428.204 | 3.69e-09 | 1.59e-10 | 198 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2448.929 | 2426.939 | 2447.314 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
