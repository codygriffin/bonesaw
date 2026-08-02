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
| 978 | 4.890 s | 7.093 cm | 4.036 cm | 31.306 cm | 35.248 cm | 27.396° | 8.000 rad/s | 8277.1 µs |

Nominal hard residual maxima: dynamics `2.028e-09`, contact acceleration `6.993e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1353.171 cm |
| authored reference vs measured CoM RMS / p95 | 1349.220 / 2890.688 cm |
| stance foot RMS | 1278.414 cm |
| swing foot RMS | 1525.461 cm |
| hand RMS | 1358.731 cm |
| maximum root rotation | 41.400° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.337e-09 |
| contact acceleration residual | 6.993e-11 |
| raw max dynamics residual, including rejected ticks | 9.337e-09 |
| raw max contact residual, including rejected ticks | 6.993e-11 |
| active normal force range | 0.000–549.054 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 144.757 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 39 ticks |
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

- Phase ticks: unsupported `1201`, single support `466`, precontact `0`, multi-support `650`.
- Joint-velocity envelope active on `1` ticks; maximum active coordinates `1`; mean target/applied scale `0.281` / `0.281`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `461` / `654` / `5` / `0` / `0` / `0` / `1197`.
- Bound-limited ticks / upper-bound ticks: `3` / `2`; maximum coordinate violation `0.9750232336329364`.
- Named-linear-row-limited ticks / upper-row ticks: `5` / `2`; maximum row violation `204.53387667327422`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 10.1 µs | 4654.1 µs | 6953.5 µs | 102694.0 µs | 458 | 517 | 0 | 80 | 1,258 | 0 | 0 | 0 | 0 | 4 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1550.4 | 3046.4 | 4.3 | 3456.6 | 19526.3 | 93318.1 | 4425.2 | 1,119 | 79 | 3 | 645.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 458 | 2338.6 | 3016.9 | 3423.6 | 4609.7 |
| solved_with_slack | 517 | 3300.4 | 6736.3 | 8993.6 | 10287.0 |
| normal_contact_contingency | 1,258 | 6.7 | 18.6 | 2989.4 | 62210.9 |
| contact_release_contingency | 4 | 12560.2 | 90859.1 | 100327.0 | 102694.0 |
| touchdown_transition | 80 | 2807.0 | 4292.4 | 5163.2 | 7081.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.14 | 10.0 | 16.0 | 23 | 1.78 | 14.0 | 22 | 0.4928 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.84/8.0/8.0/8 | 189.00/1728.0/1776.0/1776 | 0.12/3.0/9 | 0.07/2.0/8 | 0.57/15.0/67 | 0.2933 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 458 | 4.50/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 517 | 7.75/22.0/23 | 5.72/20.0/22 |
| normal_contact_contingency | 1,258 | 0.43/10.0/16 | 0.42/10.0/16 |
| contact_release_contingency | 4 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 80 | 8.31/13.2/14 | 8.00/13.2/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.66/4.0/7 | 2.69/18.0/31 | 0.28/4.0/7 | 228 |
| viability | 0.28/3.0/7 | 1.30/16.0/36 | 0.18/3.0/7 | 234 |
| intent | 0.74/6.0/13 | 1.51/12.0/26 | 0.39/6.0/12 | 380 |
| preference | 0.91/10.0/19 | 7.46/88.0/157 | 0.66/10.0/19 | 649 |
| style | 0.55/3.0/5 | 7.94/50.0/82 | 0.27/2.0/4 | 558 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,482 | 0 | 28 | 746 | 61 |
| right_ankle_roll_link | 1,386 | 0 | 52 | 879 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `39` ticks, normal fallback `64` ticks.
Touchdown Normal sole-center tangential speed: p50 `1.0606 m/s`, p95 `2.5326 m/s`, max `2.7335 m/s` over 76 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.593 | 3.591 | 3.591 | 1.000 | 1.000 | 52.707 | 56.082 | 3.375 | 56.082 | 0.002 | 0 | 803 | 0 | 0 | 32 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2958.8 | 6937.6 | 4.73 | 35.59 | 1.26 | 1.00 | 234.00 | 0.345 | 0.404 | 1.58e-09 | 6.99e-11 | 0 |
| 232–463 | 1798.7 | 7402.9 | 5.33 | 36.35 | 1.03 | 1.00 | 225.52 | 0.001 | 0.332 | 2.03e-09 | 3.33e-11 | 0 |
| 464–695 | 2978.8 | 9575.8 | 6.28 | 43.04 | 2.73 | 1.00 | 230.56 | 0.026 | 0.286 | 1.28e-09 | 6.75e-11 | 0 |
| 696–927 | 3203.0 | 6839.9 | 8.05 | 52.03 | 5.94 | 1.53 | 350.12 | 5.085 | 4.950 | 1.35e-09 | 5.31e-11 | 0 |
| 928–1159 | 1756.6 | 6199.7 | 4.65 | 27.81 | 4.54 | 2.21 | 481.42 | 113.610 | 102.018 | 9.34e-09 | 5.42e-11 | 169 |
| 1160–1391 | 6.7 | 13.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 559.268 | 516.838 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.6 | 11.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1141.439 | 1095.167 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.7 | 3123.8 | 0.91 | 5.20 | 0.89 | 0.41 | 88.83 | 1726.161 | 1687.066 | 3.70e-09 | 8.97e-12 | 206 |
| 1855–2085 | 6.6 | 13.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2311.340 | 2279.489 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 7.5 | 4357.7 | 1.40 | 8.79 | 1.39 | 1.29 | 278.65 | 2899.763 | 2870.669 | 1.65e-09 | 6.68e-11 | 192 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1353.171 | 1330.889 | 1358.731 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
