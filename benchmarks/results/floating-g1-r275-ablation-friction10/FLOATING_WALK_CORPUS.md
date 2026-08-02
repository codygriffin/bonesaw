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
| 950 | 4.750 s | 2.671 cm | 0.347 cm | 0.576 cm | 33.232 cm | 7.513° | 8.000 rad/s | 7295.0 µs |

Nominal hard residual maxima: dynamics `1.583e-09`, contact acceleration `6.474e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 982.940 cm |
| authored reference vs measured CoM RMS / p95 | 979.106 / 2285.255 cm |
| stance foot RMS | 927.030 cm |
| swing foot RMS | 1141.800 cm |
| hand RMS | 986.655 cm |
| maximum root rotation | 35.888° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.848e-09 |
| contact acceleration residual | 6.634e-11 |
| raw max dynamics residual, including rejected ticks | 1.848e-09 |
| raw max contact residual, including rejected ticks | 6.634e-11 |
| active normal force range | 0.000–812.128 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 153.370 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 44 ticks |
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

- Phase ticks: unsupported `836`, single support `531`, precontact `0`, multi-support `950`.
- Joint-velocity envelope active on `251` ticks; maximum active coordinates `3`; mean target/applied scale `0.410` / `0.410`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `478` / `1002` / `4` / `0` / `0` / `0` / `833`.
- Bound-limited ticks / upper-bound ticks: `1` / `1`; maximum coordinate violation `0.6515166299469026`.
- Named-linear-row-limited ticks / upper-row ticks: `4` / `1`; maximum row violation `66.06809541158697`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 2705.8 µs | 5281.2 µs | 7297.6 µs | 196329.7 µs | 475 | 472 | 0 | 76 | 1,291 | 0 | 0 | 0 | 0 | 3 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2290.7 | 4569.2 | 1548.0 | 4696.4 | 21542.8 | 158572.9 | 4363.7 | 1,484 | 149 | 3 | 436.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 475 | 2880.1 | 3100.2 | 3251.6 | 3432.0 |
| solved_with_slack | 472 | 3396.0 | 6983.5 | 8432.3 | 8879.7 |
| normal_contact_contingency | 1,291 | 7.5 | 5036.1 | 6700.4 | 33303.8 |
| contact_release_contingency | 3 | 1339.1 | 176830.6 | 192429.9 | 196329.7 |
| touchdown_transition | 76 | 2780.4 | 3488.5 | 3609.2 | 3701.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.60 | 12.0 | 22.0 | 26 | 3.05 | 19.0 | 26 | 0.3395 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.99/8.0/8.0/8 | 442.29/1776.0/1776.0/1776 | 0.49/3.0/29 | 0.30/2.0/28 | 2.49/18.0/252 | 0.2557 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 475 | 4.48/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 472 | 7.66/22.0/24 | 5.35/20.0/21 |
| normal_contact_contingency | 1,291 | 3.33/19.0/26 | 3.08/18.0/26 |
| contact_release_contingency | 3 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 76 | 7.96/11.0/11 | 7.49/11.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.90/3.0/6 | 3.61/12.0/24 | 0.40/3.0/6 | 481 |
| viability | 0.74/9.0/20 | 3.71/45.0/100 | 0.62/9.0/20 | 563 |
| intent | 1.01/6.0/11 | 2.08/12.0/30 | 0.66/5.0/11 | 733 |
| preference | 1.24/13.0/19 | 10.38/103.2/157 | 0.97/12.0/19 | 991 |
| style | 0.71/3.0/5 | 10.23/50.0/81 | 0.40/2.0/5 | 843 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,109 | 0 | 32 | 718 | 458 |
| right_ankle_roll_link | 1,094 | 0 | 344 | 879 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `300` ticks, normal fallback `461` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.7057 m/s`, p95 `4.2577 m/s`, max `5.8652 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.308 | 5.308 | 5.308 | 1.000 | 1.000 | 52.746 | 56.141 | 3.395 | 56.141 | 0.002 | 0 | 803 | 0 | 0 | 19 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3072.4 | 5549.5 | 4.69 | 34.88 | 1.22 | 1.00 | 234.00 | 0.345 | 0.404 | 1.58e-09 | 4.57e-11 | 0 |
| 232–463 | 1855.9 | 8035.5 | 5.34 | 36.81 | 1.02 | 1.00 | 225.52 | 0.001 | 0.332 | 1.03e-09 | 6.45e-11 | 0 |
| 464–695 | 3039.2 | 7266.0 | 6.37 | 43.35 | 2.75 | 1.00 | 230.56 | 0.016 | 0.286 | 1.32e-09 | 6.47e-11 | 0 |
| 696–927 | 3266.8 | 8132.0 | 7.63 | 50.31 | 5.27 | 2.00 | 452.51 | 3.600 | 0.501 | 1.42e-09 | 5.87e-11 | 0 |
| 928–1159 | 2944.0 | 5662.1 | 8.76 | 53.92 | 8.03 | 5.83 | 1270.81 | 67.279 | 70.177 | 1.41e-09 | 5.02e-11 | 210 |
| 1160–1391 | 4404.1 | 9285.0 | 9.70 | 60.13 | 8.88 | 6.61 | 1467.88 | 233.489 | 227.815 | 5.03e-10 | 1.17e-11 | 232 |
| 1392–1623 | 6.5 | 21559.0 | 0.87 | 5.16 | 0.83 | 0.55 | 122.48 | 551.334 | 535.457 | 1.44e-09 | 6.63e-11 | 232 |
| 1624–1854 | 6.6 | 3450.1 | 0.99 | 5.96 | 0.96 | 0.70 | 151.48 | 1100.131 | 1086.868 | 1.72e-09 | 1.70e-11 | 202 |
| 1855–2085 | 6.6 | 11.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1691.190 | 1674.509 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.9 | 3439.9 | 1.56 | 9.23 | 1.51 | 1.22 | 263.69 | 2292.592 | 2273.884 | 1.85e-09 | 5.53e-11 | 187 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 982.940 | 973.249 | 986.655 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
