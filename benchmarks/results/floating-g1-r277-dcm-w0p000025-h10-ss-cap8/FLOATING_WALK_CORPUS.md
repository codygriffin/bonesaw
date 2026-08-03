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
| 431 | 2.155 s | 2.923 cm | 0.404 cm | 2.608 cm | 32.784 cm | 3.568° | 8.000 rad/s | 5512.9 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `4.840e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2529.306 cm |
| authored reference vs measured CoM RMS / p95 | 2519.187 / 4640.093 cm |
| stance foot RMS | 2407.455 cm |
| swing foot RMS | 2738.098 cm |
| hand RMS | 2536.878 cm |
| maximum root rotation | 72.636° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 5.365e-09 |
| contact acceleration residual | 1.878e-10 |
| raw max dynamics residual, including rejected ticks | 5.365e-09 |
| raw max contact residual, including rejected ticks | 1.878e-10 |
| active normal force range | 0.000–495.682 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 157.079 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 47 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `6.285` / `12.688 cm`.
- Virtual ZMP clipped on `69.01%` of ticks; clip-distance RMS / max `8.164` / `23.018 cm`.
- Measured-height natural frequency min / p50 / max: `3.751` / `3.797` / `3.834 rad/s`.
- Minimum measured CoM height: `0.667 m`; height-floor ticks: `0`.
- CoM command acceleration p95 / max: `0.914` / `1.011 m/s²`; support hull `4–6` vertices.
- Signed measured DCM support margin min / p05: `-10.384` / `-8.345 cm`; inside on `41.55%` of ticks.

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

- Phase ticks: unsupported `1739`, single support `278`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.129` / `0.129`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `137` / `441` / `5` / `0` / `0` / `0` / `1734`.
- Bound-limited ticks / upper-bound ticks: `2` / `0`; maximum coordinate violation `9.824913006693468`.
- Named-linear-row-limited ticks / upper-row ticks: `5` / `3`; maximum row violation `119.55254807196766`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 7.6 µs | 3816.7 µs | 5093.4 µs | 186409.9 µs | 137 | 294 | 0 | 147 | 1,734 | 0 | 0 | 0 | 0 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 991.4 | 5342.4 | 1.3 | 3267.3 | 16967.8 | 180743.6 | 2034.2 | 583 | 25 | 2 | 1008.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 3037.2 | 4111.8 | 4192.1 | 4233.3 |
| solved_with_slack | 294 | 3227.8 | 5004.2 | 5742.4 | 7159.9 |
| normal_contact_contingency | 1,734 | 6.8 | 10.3 | 12.9 | 22.2 |
| contact_release_contingency | 5 | 1627.6 | 181516.7 | 185431.3 | 186409.9 |
| touchdown_transition | 147 | 2900.9 | 5324.7 | 13118.6 | 18699.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.85 | 9.0 | 13.0 | 24 | 1.39 | 11.0 | 22 | 0.2350 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.79/8.0/8.0/8 | 174.79/1728.0/1776.0/1776 | 0.22/3.0/19 | 0.14/2.0/18 | 1.16/16.0/155 | 0.1864 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 294 | 8.52/16.1/24 | 6.81/15.1/22 |
| normal_contact_contingency | 1,734 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 5 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 147 | 8.43/13.0/14 | 8.27/13.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.41/4.0/7 | 1.73/18.0/30 | 0.24/4.0/7 | 222 |
| viability | 0.23/4.0/7 | 1.22/22.7/36 | 0.21/4.0/7 | 246 |
| intent | 0.54/7.0/14 | 1.08/14.0/28 | 0.44/7.0/13 | 361 |
| preference | 0.40/4.8/14 | 3.62/44.0/112 | 0.33/4.0/14 | 439 |
| style | 0.27/2.0/3 | 3.84/29.0/49 | 0.17/2.0/3 | 349 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,928 | 0 | 89 | 300 | 0 |
| right_ankle_roll_link | 1,828 | 0 | 58 | 431 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `47` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.6398 m/s`, p95 `4.9808 m/s`, max `7.4726 m/s` over 143 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.298 | 2.298 | 2.297 | 1.000 | 1.000 | 52.957 | 56.258 | 3.301 | 56.258 | 0.002 | 0 | 799 | 0 | 0 | 5 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3188.2 | 6495.5 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 3108.5 | 5348.1 | 7.75 | 44.27 | 6.46 | 3.24 | 723.10 | 8.623 | 3.622 | 1.07e-09 | 4.53e-11 | 33 |
| 464–695 | 7.1 | 3521.5 | 1.51 | 8.74 | 1.48 | 0.88 | 189.00 | 281.948 | 246.989 | 2.25e-09 | 5.25e-11 | 190 |
| 696–927 | 6.8 | 12.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 869.785 | 816.619 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 6.3 | 2896.3 | 0.74 | 4.66 | 0.72 | 0.34 | 72.62 | 1498.455 | 1446.373 | 1.66e-09 | 8.24e-12 | 210 |
| 1160–1391 | 6.9 | 15.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2114.522 | 2076.722 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.8 | 11.9 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2731.394 | 2698.889 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.9 | 3849.1 | 1.74 | 10.39 | 1.71 | 1.20 | 259.95 | 3356.665 | 3288.476 | 3.59e-09 | 2.38e-11 | 184 |
| 1855–2085 | 6.9 | 13.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 4009.257 | 3916.073 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 7.7 | 13154.1 | 1.37 | 9.03 | 1.34 | 1.25 | 269.30 | 4663.498 | 4569.296 | 5.36e-09 | 1.88e-10 | 195 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2529.306 | 2476.316 | 2536.878 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
