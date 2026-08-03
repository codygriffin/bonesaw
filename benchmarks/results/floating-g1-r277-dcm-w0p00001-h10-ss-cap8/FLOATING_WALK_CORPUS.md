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
| 462 | 2.310 s | 5.842 cm | 0.406 cm | 7.544 cm | 31.729 cm | 5.950° | 8.000 rad/s | 11718.9 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `4.840e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1999.728 cm |
| authored reference vs measured CoM RMS / p95 | 1995.851 / 3891.517 cm |
| stance foot RMS | 1925.197 cm |
| swing foot RMS | 2232.670 cm |
| hand RMS | 2003.631 cm |
| maximum root rotation | 65.955° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.496e-09 |
| contact acceleration residual | 5.122e-11 |
| raw max dynamics residual, including rejected ticks | 3.496e-09 |
| raw max contact residual, including rejected ticks | 5.122e-11 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 133.997 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 60 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `12.918` / `30.789 cm`.
- Virtual ZMP clipped on `71.10%` of ticks; clip-distance RMS / max `20.057` / `55.721 cm`.
- Measured-height natural frequency min / p50 / max: `3.746` / `3.869` / `3.982 rad/s`.
- Minimum measured CoM height: `0.619 m`; height-floor ticks: `0`.
- CoM command acceleration p95 / max: `1.178` / `1.610 m/s²`; support hull `4–6` vertices.
- Signed measured DCM support margin min / p05: `-20.394` / `-18.114 cm`; inside on `37.57%` of ticks.

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

- Phase ticks: unsupported `1671`, single support `346`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.129` / `0.129`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `137` / `510` / `4` / `0` / `0` / `0` / `1666`.
- Bound-limited ticks / upper-bound ticks: `0` / `0`; maximum coordinate violation `0.0`.
- Named-linear-row-limited ticks / upper-row ticks: `4` / `1`; maximum row violation `276.4064692700755`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 7.6 µs | 3910.5 µs | 7342.4 µs | 26167.3 µs | 137 | 325 | 0 | 185 | 1,666 | 0 | 0 | 0 | 0 | 4 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 971.9 | 1826.8 | 1.6 | 3275.5 | 15910.2 | 24301.7 | 3024.5 | 651 | 51 | 1 | 1028.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 3015.6 | 9237.8 | 17460.3 | 26167.3 |
| solved_with_slack | 325 | 3222.0 | 4970.9 | 6684.3 | 8309.8 |
| normal_contact_contingency | 1,666 | 6.8 | 10.1 | 13.1 | 63.4 |
| contact_release_contingency | 4 | 1377.9 | 1667.8 | 1708.4 | 1718.5 |
| touchdown_transition | 185 | 2813.6 | 7651.8 | 8802.1 | 9296.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.13 | 10.0 | 13.0 | 18 | 1.66 | 12.0 | 16 | 0.7716 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.90/8.0/8.0/8 | 199.34/1728.0/1776.0/1776 | 0.27/6.0/10 | 0.18/5.0/9 | 1.44/40.0/76 | 0.6124 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 325 | 8.62/16.0/18 | 7.08/14.8/16 |
| normal_contact_contingency | 1,666 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 4 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 185 | 8.63/15.0/17 | 8.38/15.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.49/4.0/6 | 1.98/16.0/26 | 0.32/4.0/6 | 304 |
| viability | 0.30/5.0/10 | 1.56/25.0/50 | 0.28/5.0/10 | 310 |
| intent | 0.59/7.0/13 | 1.15/14.0/26 | 0.48/7.0/13 | 422 |
| preference | 0.44/5.0/14 | 4.02/46.7/112 | 0.37/5.0/14 | 507 |
| style | 0.31/2.0/3 | 4.29/28.8/49 | 0.21/2.0/3 | 416 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,926 | 0 | 91 | 300 | 0 |
| right_ankle_roll_link | 1,761 | 0 | 94 | 462 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `60` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `4.2122 m/s`, p95 `7.5158 m/s`, max `8.1686 m/s` over 181 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.252 | 2.148 | 2.148 | 0.954 | 0.954 | 52.969 | 56.270 | 3.301 | 56.270 | 0.002 | 0 | 799 | 0 | 0 | 164 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3133.4 | 15917.6 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 3174.6 | 5139.6 | 9.03 | 51.99 | 7.75 | 4.07 | 906.83 | 8.524 | 4.984 | 1.46e-09 | 4.22e-11 | 2 |
| 464–695 | 6.9 | 3639.1 | 1.16 | 6.80 | 1.14 | 0.65 | 139.66 | 122.426 | 124.151 | 3.50e-09 | 4.37e-11 | 201 |
| 696–927 | 6.9 | 13.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 568.497 | 568.670 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 6.1 | 3823.4 | 1.54 | 9.44 | 1.49 | 0.46 | 98.69 | 1128.888 | 1129.339 | 2.84e-09 | 5.12e-11 | 189 |
| 1160–1391 | 6.9 | 12.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1491.579 | 1483.767 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.8 | 14.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2043.499 | 2033.922 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 7.0 | 8483.9 | 2.34 | 12.94 | 2.29 | 1.59 | 344.10 | 2574.874 | 2559.083 | 1.71e-09 | 2.20e-11 | 171 |
| 1855–2085 | 6.9 | 11.8 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3189.851 | 3170.236 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 6.0 | 3602.3 | 1.86 | 11.04 | 1.78 | 1.25 | 270.23 | 3906.041 | 3889.025 | 1.99e-09 | 1.73e-11 | 180 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1999.728 | 1989.746 | 2003.631 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
