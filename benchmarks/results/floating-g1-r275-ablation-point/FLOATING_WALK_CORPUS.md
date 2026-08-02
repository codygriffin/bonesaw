# Bonesaw floating G1 admitted-reference tracking

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: immutable open-loop-admitted artifact `benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz` from `Python-authored alternating sequence of Rust Bonesaw LIPM boundary plans`.
- Admission contract: authored root, CoM, foot jets, and contact schedule are consumed unchanged; eval-side reconstruction, projection, and retiming are rejected.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 1 sole contact points per foot (`x=0.035±0.000 m`, `y=±0.0000 m`, `z=-0.035 m` in the foot frame).
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
| 176 | 0.880 s | 6.674 cm | 3.822 cm | nan cm | 16.701 cm | 3.859° | 8.000 rad/s | 10649.5 µs |

Nominal hard residual maxima: dynamics `1.738e-09`, contact acceleration `5.428e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 4653.651 cm |
| authored reference vs measured CoM RMS / p95 | 4648.825 / 8138.911 cm |
| stance foot RMS | 4506.715 cm |
| swing foot RMS | 5019.332 cm |
| hand RMS | 4682.143 cm |
| maximum root rotation | 132.862° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.677e-09 |
| contact acceleration residual | 2.132e-10 |
| raw max dynamics residual, including rejected ticks | 8.677e-09 |
| raw max contact residual, including rejected ticks | 2.132e-10 |
| active normal force range | 0.000–773.892 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 168.922 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 43 ticks |
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

- Phase ticks: unsupported `1865`, single support `152`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `120` ticks; maximum active coordinates `4`; mean target/applied scale `0.129` / `0.129`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `88` / `363` / `6` / `0` / `0` / `0` / `1860`.
- Bound-limited ticks / upper-bound ticks: `3` / `0`; maximum coordinate violation `15.749998523280489`.
- Named-linear-row-limited ticks / upper-row ticks: `6` / `5`; maximum row violation `153.59824368077545`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 5.4 µs | 2400.6 µs | 4622.3 µs | 99138.1 µs | 88 | 88 | 0 | 115 | 2,021 | 0 | 0 | 0 | 0 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 574.5 | 3346.7 | 0.3 | 2176.5 | 54695.3 | 97035.1 | 3017.4 | 455 | 13 | 3 | 1740.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 88 | 1626.4 | 8174.5 | 10678.1 | 10736.4 |
| solved_with_slack | 88 | 2281.4 | 3826.3 | 3982.9 | 4038.4 |
| normal_contact_contingency | 2,021 | 5.3 | 2016.9 | 2399.6 | 3291.5 |
| contact_release_contingency | 5 | 75003.7 | 97322.0 | 98774.9 | 99138.1 |
| touchdown_transition | 115 | 2155.2 | 4142.3 | 6220.5 | 8358.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.36 | 8.0 | 10.0 | 12 | 1.11 | 10.0 | 12 | 0.2451 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.16/8.0/8.0/8 | 208.95/1424.0/1488.0/1488 | 0.39/5.0/12 | 0.26/4.0/11 | 2.06/32.0/93 | 0.2304 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 88 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 88 | 6.73/10.0/10 | 5.75/9.1/10 |
| normal_contact_contingency | 2,021 | 0.64/10.0/12 | 0.59/9.0/12 |
| contact_release_contingency | 5 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 115 | 7.99/11.9/12 | 7.63/11.7/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.43/4.0/7 | 1.77/16.0/33 | 0.34/4.0/7 | 334 |
| viability | 0.23/3.0/6 | 1.28/18.0/36 | 0.22/3.0/6 | 281 |
| intent | 0.29/3.0/6 | 0.51/6.0/12 | 0.24/3.0/6 | 350 |
| preference | 0.22/2.0/4 | 2.76/25.0/55 | 0.17/2.0/4 | 342 |
| style | 0.20/1.0/2 | 1.82/10.0/20 | 0.14/1.0/2 | 314 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,981 | 0 | 36 | 176 | 124 |
| right_ankle_roll_link | 1,901 | 0 | 79 | 176 | 161 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `43` ticks, normal fallback `161` ticks.
Touchdown Normal sole-center tangential speed: p50 `6.1651 m/s`, p95 `8.2352 m/s`, max `9.7361 m/s` over 111 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.332 | 1.227 | 1.227 | 0.922 | 0.922 | 51.078 | 54.281 | 3.203 | 54.281 | 0.002 | 0 | 774 | 0 | 0 | 32 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2240.3 | 10024.4 | 6.04 | 36.34 | 4.03 | 4.86 | 890.34 | 20.974 | 5.517 | 1.74e-09 | 5.43e-11 | 56 |
| 232–463 | 7.2 | 2759.1 | 3.60 | 21.60 | 3.27 | 3.08 | 546.34 | 405.589 | 379.396 | 2.44e-10 | 9.21e-12 | 232 |
| 464–695 | 5.3 | 2140.6 | 0.48 | 2.74 | 0.45 | 0.27 | 47.79 | 1234.262 | 1202.914 | 2.35e-10 | 6.40e-12 | 218 |
| 696–927 | 5.4 | 13.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2211.477 | 2173.920 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 5.2 | 4129.0 | 1.47 | 8.50 | 1.38 | 1.48 | 260.97 | 3202.774 | 3161.034 | 4.79e-09 | 1.67e-10 | 189 |
| 1160–1391 | 5.4 | 12.7 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 4191.372 | 4135.531 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 5.3 | 10.7 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 5175.919 | 5128.782 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 5.4 | 4966.7 | 0.75 | 4.53 | 0.73 | 0.70 | 123.43 | 6167.837 | 6118.929 | 8.68e-09 | 2.13e-10 | 209 |
| 1855–2085 | 5.3 | 11.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 7161.299 | 7107.348 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.2 | 4554.9 | 1.27 | 7.60 | 1.23 | 1.25 | 219.43 | 8150.777 | 8091.472 | 3.77e-09 | 1.44e-10 | 195 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 4653.651 | 4612.564 | 4682.143 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
