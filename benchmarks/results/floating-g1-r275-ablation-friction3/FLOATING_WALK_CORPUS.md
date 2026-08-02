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
| 911 | 4.555 s | 2.546 cm | 0.352 cm | 2.110 cm | 33.089 cm | 6.102° | 8.000 rad/s | 9746.1 µs |

Nominal hard residual maxima: dynamics `1.583e-09`, contact acceleration `6.731e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1877.018 cm |
| authored reference vs measured CoM RMS / p95 | 1870.059 / 4038.828 cm |
| stance foot RMS | 1774.704 cm |
| swing foot RMS | 2122.473 cm |
| hand RMS | 1887.876 cm |
| maximum root rotation | 37.475° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.965e-09 |
| contact acceleration residual | 1.272e-10 |
| raw max dynamics residual, including rejected ticks | 2.965e-09 |
| raw max contact residual, including rejected ticks | 1.272e-10 |
| active normal force range | 0.000–808.008 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 143.307 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 50 ticks |
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

- Phase ticks: unsupported `1284`, single support `383`, precontact `0`, multi-support `650`.
- Joint-velocity envelope active on `13` ticks; maximum active coordinates `1`; mean target/applied scale `0.281` / `0.281`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `451` / `582` / `4` / `0` / `0` / `0` / `1280`.
- Bound-limited ticks / upper-bound ticks: `1` / `0`; maximum coordinate violation `0.40700616329487893`.
- Named-linear-row-limited ticks / upper-row ticks: `4` / `1`; maximum row violation `61.20659573709219`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 8.3 µs | 3752.5 µs | 6979.1 µs | 10121.9 µs | 448 | 460 | 0 | 125 | 1,280 | 0 | 0 | 0 | 0 | 4 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1366.3 | 1737.1 | 2.7 | 3377.8 | 9953.9 | 10112.6 | 5219.6 | 1,037 | 56 | 0 | 731.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 448 | 1868.1 | 2972.5 | 3026.3 | 3143.7 |
| solved_with_slack | 460 | 3339.9 | 6988.7 | 9876.0 | 10121.9 |
| normal_contact_contingency | 1,280 | 6.5 | 9.1 | 11.8 | 64.6 |
| contact_release_contingency | 4 | 1374.1 | 2005.0 | 2088.7 | 2109.6 |
| touchdown_transition | 125 | 2841.1 | 3801.8 | 4729.9 | 5146.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.82 | 9.0 | 14.0 | 24 | 1.50 | 13.0 | 22 | 0.8525 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.78/1.0/8.0/8 | 174.94/234.0/1776.0/1776 | 0.12/3.0/6 | 0.07/2.0/5 | 0.59/16.0/42 | 0.4865 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 448 | 4.51/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 460 | 7.51/22.0/24 | 5.32/20.0/22 |
| normal_contact_contingency | 1,280 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 4 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 125 | 8.44/13.0/14 | 8.17/12.8/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.58/4.0/6 | 2.37/16.8/30 | 0.20/4.0/6 | 164 |
| viability | 0.22/3.0/6 | 1.02/14.8/36 | 0.11/3.0/6 | 151 |
| intent | 0.70/6.8/13 | 1.39/13.7/36 | 0.37/6.0/12 | 309 |
| preference | 0.81/7.0/20 | 6.57/57.7/151 | 0.57/6.8/20 | 579 |
| style | 0.50/3.0/5 | 7.34/50.8/85 | 0.25/2.8/5 | 482 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,585 | 0 | 53 | 679 | 0 |
| right_ankle_roll_link | 1,366 | 0 | 72 | 879 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `50` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `4.6048 m/s`, p95 `7.4181 m/s`, max `7.8726 m/s` over 121 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.166 | 3.166 | 3.166 | 1.000 | 1.000 | 52.703 | 56.078 | 3.375 | 56.078 | 0.002 | 0 | 803 | 0 | 0 | 6 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2943.9 | 9330.8 | 4.83 | 36.82 | 1.36 | 1.00 | 234.00 | 0.345 | 0.404 | 1.58e-09 | 5.06e-11 | 0 |
| 232–463 | 1808.3 | 9879.9 | 5.34 | 37.32 | 0.98 | 1.00 | 225.52 | 0.001 | 0.332 | 1.30e-09 | 5.51e-11 | 0 |
| 464–695 | 2933.3 | 8175.1 | 6.38 | 41.09 | 2.94 | 1.00 | 230.56 | 0.019 | 0.286 | 1.17e-09 | 6.73e-11 | 0 |
| 696–927 | 3186.1 | 6326.0 | 7.12 | 44.13 | 5.28 | 1.74 | 396.05 | 7.677 | 3.633 | 1.27e-09 | 4.76e-11 | 17 |
| 928–1159 | 6.4 | 3199.1 | 1.32 | 7.89 | 1.28 | 1.03 | 223.45 | 286.773 | 277.084 | 2.76e-09 | 1.65e-11 | 195 |
| 1160–1391 | 6.5 | 12.6 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 852.845 | 809.867 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.4 | 11.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1586.993 | 1535.650 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.7 | 3090.4 | 1.93 | 11.76 | 1.90 | 0.82 | 177.66 | 2330.584 | 2282.936 | 8.88e-10 | 1.44e-11 | 181 |
| 1855–2085 | 6.6 | 12.0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3172.036 | 3132.135 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.9 | 3883.3 | 1.24 | 7.74 | 1.23 | 1.21 | 261.82 | 4062.350 | 4018.957 | 2.96e-09 | 1.27e-10 | 196 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1877.018 | 1848.643 | 1887.876 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
