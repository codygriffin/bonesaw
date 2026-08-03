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
| 793 | 3.965 s | 10.635 cm | 0.552 cm | 0.314 cm | 38.930 cm | 62.862° | 8.000 rad/s | 14082.6 µs |

Nominal hard residual maxima: dynamics `7.292e-09`, contact acceleration `2.671e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1834.117 cm |
| authored reference vs measured CoM RMS / p95 | 1825.446 / 3633.278 cm |
| stance foot RMS | 1717.749 cm |
| swing foot RMS | 1977.913 cm |
| hand RMS | 1853.238 cm |
| maximum root rotation | 179.178° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.292e-09 |
| contact acceleration residual | 3.703e-10 |
| raw max dynamics residual, including rejected ticks | 7.292e-09 |
| raw max contact residual, including rejected ticks | 3.703e-10 |
| active normal force range | 0.000–224.986 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 133.683 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 20 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `3.178` / `7.083 cm`.
- Virtual ZMP clipped on `62.82%` of ticks; clip-distance RMS / max `4.867` / `14.657 cm`.
- Measured-height natural frequency min / p50 / max: `3.749` / `3.789` / `3.813 rad/s`.
- Minimum measured CoM height: `0.675 m`; height-floor ticks: `0`.
- CoM command acceleration p95 / max: `0.315` / `0.414 m/s²`; support hull `4–6` vertices.
- Signed measured DCM support margin min / p05: `-6.805` / `-5.333 cm`; inside on `61.54%` of ticks.

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

- Phase ticks: unsupported `1418`, single support `268`, precontact `0`, multi-support `631`.
- Joint-velocity envelope active on `111` ticks; maximum active coordinates `5`; mean target/applied scale `0.272` / `0.272`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `287` / `611` / `5` / `0` / `0` / `0` / `1414`.
- Bound-limited ticks / upper-bound ticks: `5` / `3`; maximum coordinate violation `11.992700928848267`.
- Named-linear-row-limited ticks / upper-row ticks: `5` / `4`; maximum row violation `810.7072549971962`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 8.2 µs | 5180.6 µs | 16940.3 µs | 161895.9 µs | 287 | 486 | 0 | 59 | 1,481 | 0 | 0 | 0 | 0 | 4 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1812.1 | 6712.8 | 2.4 | 3801.2 | 125642.5 | 158750.0 | 4621.5 | 903 | 146 | 11 | 551.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 287 | 2903.1 | 3086.3 | 3240.4 | 4102.5 |
| solved_with_slack | 486 | 3252.3 | 6444.8 | 16552.2 | 24551.6 |
| normal_contact_contingency | 1,481 | 6.6 | 16.0 | 16886.6 | 148312.7 |
| contact_release_contingency | 4 | 124247.2 | 156817.4 | 160880.2 | 161895.9 |
| touchdown_transition | 59 | 3388.4 | 8143.2 | 12998.9 | 16641.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.74 | 10.0 | 15.0 | 22 | 1.79 | 14.0 | 22 | 0.2877 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.87/8.0/8.0/8 | 197.01/1776.0/1872.0/1872 | 0.43/13.0/27 | 0.36/12.0/26 | 3.05/106.0/233 | 0.3294 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 287 | 4.47/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 486 | 8.15/16.1/21 | 6.40/16.0/21 |
| normal_contact_contingency | 1,481 | 0.41/11.0/22 | 0.40/11.0/22 |
| contact_release_contingency | 4 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 59 | 8.59/16.4/17 | 7.29/15.3/17 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.57/4.8/7 | 2.40/20.0/35 | 0.26/4.8/7 | 202 |
| viability | 0.29/5.0/17 | 1.24/16.0/102 | 0.17/5.0/17 | 140 |
| intent | 0.79/7.0/13 | 1.63/14.0/33 | 0.58/7.0/13 | 436 |
| preference | 0.67/6.0/14 | 5.74/56.0/117 | 0.54/6.0/14 | 609 |
| style | 0.42/2.0/5 | 6.34/32.0/84 | 0.24/2.0/5 | 492 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,668 | 0 | 38 | 544 | 67 |
| right_ankle_roll_link | 1,436 | 0 | 21 | 793 | 67 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `20` ticks, normal fallback `70` ticks.
Touchdown Normal sole-center tangential speed: p50 `1.6796 m/s`, p95 `4.3253 m/s`, max `4.8593 m/s` over 55 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.199 | 4.196 | 4.196 | 0.999 | 0.999 | 52.855 | 56.219 | 3.363 | 56.219 | 0.002 | 0 | 800 | 0 | 0 | 135 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3053.1 | 6521.3 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 2539.1 | 5026.7 | 7.41 | 45.74 | 4.24 | 1.60 | 359.48 | 0.797 | 0.392 | 1.20e-09 | 4.45e-11 | 0 |
| 464–695 | 3077.9 | 5255.4 | 6.55 | 41.44 | 3.68 | 1.00 | 230.12 | 2.467 | 0.287 | 1.36e-09 | 6.25e-11 | 0 |
| 696–927 | 4901.4 | 36911.4 | 6.50 | 38.36 | 6.23 | 4.27 | 970.81 | 125.350 | 73.992 | 7.29e-09 | 2.67e-10 | 135 |
| 928–1159 | 6.6 | 3050.2 | 0.52 | 3.56 | 0.52 | 0.09 | 18.62 | 621.652 | 531.219 | 1.79e-09 | 2.94e-11 | 219 |
| 1160–1391 | 6.6 | 12.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1218.398 | 1125.201 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.6 | 12.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1823.401 | 1735.591 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.6 | 9737.4 | 0.71 | 4.32 | 0.71 | 0.62 | 134.65 | 2433.493 | 2348.112 | 4.89e-09 | 3.70e-10 | 213 |
| 1855–2085 | 6.6 | 12.3 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3043.382 | 2954.374 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.8 | 3592.5 | 0.29 | 2.06 | 0.29 | 0.10 | 20.57 | 3650.166 | 3560.140 | 8.21e-10 | 3.38e-11 | 223 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1834.117 | 1772.207 | 1853.238 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
