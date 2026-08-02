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
| 887 | 4.435 s | 2.824 cm | 0.381 cm | 0.490 cm | 34.054 cm | 3.871° | 8.000 rad/s | 6690.9 µs |

Nominal hard residual maxima: dynamics `1.593e-09`, contact acceleration `7.250e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1136.499 cm |
| authored reference vs measured CoM RMS / p95 | 1133.343 / 2533.256 cm |
| stance foot RMS | 1082.895 cm |
| swing foot RMS | 1307.787 cm |
| hand RMS | 1138.274 cm |
| maximum root rotation | 23.685° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.199e-09 |
| contact acceleration residual | 7.250e-11 |
| raw max dynamics residual, including rejected ticks | 2.199e-09 |
| raw max contact residual, including rejected ticks | 7.250e-11 |
| active normal force range | 0.000–817.534 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 151.790 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 51 ticks |
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

- Phase ticks: unsupported `1129`, single support `538`, precontact `0`, multi-support `650`.
- Joint-velocity envelope active on `16` ticks; maximum active coordinates `1`; mean target/applied scale `0.281` / `0.281`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `367` / `821` / `4` / `0` / `0` / `0` / `1125`.
- Bound-limited ticks / upper-bound ticks: `1` / `1`; maximum coordinate violation `0.02770205757497291`.
- Named-linear-row-limited ticks / upper-row ticks: `4` / `1`; maximum row violation `5.17082594364723`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 1728.4 µs | 3857.0 µs | 6147.1 µs | 10221.2 µs | 367 | 517 | 0 | 116 | 1,314 | 0 | 0 | 0 | 0 | 3 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1485.9 | 1614.8 | 1720.2 | 3304.7 | 8589.8 | 10014.4 | 3206.1 | 1,191 | 44 | 0 | 673.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 367 | 1811.4 | 3006.5 | 4182.4 | 6168.2 |
| solved_with_slack | 517 | 3196.5 | 5880.2 | 7583.3 | 10221.2 |
| normal_contact_contingency | 1,314 | 6.6 | 2749.7 | 3595.0 | 4519.6 |
| contact_release_contingency | 3 | 1347.2 | 2157.9 | 2230.0 | 2248.0 |
| touchdown_transition | 116 | 2835.3 | 3306.2 | 3568.4 | 3881.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.61 | 10.0 | 15.0 | 22 | 2.36 | 14.0 | 21 | 0.8609 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.98/8.0/8.0/8 | 216.34/1728.0/1728.0/1728 | 0.14/2.0/4 | 0.07/1.0/3 | 0.57/9.0/24 | 0.4749 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 367 | 4.62/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 517 | 7.99/22.0/22 | 6.08/19.8/21 |
| normal_contact_contingency | 1,314 | 1.20/11.0/16 | 1.07/10.0/15 |
| contact_release_contingency | 3 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 116 | 8.23/12.8/14 | 7.84/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.67/4.0/7 | 2.69/16.0/28 | 0.24/4.0/7 | 243 |
| viability | 0.42/4.0/12 | 2.07/24.0/60 | 0.31/4.0/12 | 313 |
| intent | 1.05/8.0/12 | 2.12/16.0/24 | 0.77/7.8/12 | 631 |
| preference | 0.92/10.0/19 | 7.68/77.0/156 | 0.72/10.0/19 | 816 |
| style | 0.55/2.0/5 | 7.76/34.0/80 | 0.31/2.0/4 | 672 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,446 | 0 | 27 | 655 | 189 |
| right_ankle_roll_link | 1,349 | 0 | 89 | 879 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `51` ticks, normal fallback `192` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.2952 m/s`, p95 `3.2659 m/s`, max `3.7890 m/s` over 112 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.443 | 3.439 | 3.439 | 0.999 | 0.999 | 52.832 | 56.145 | 3.312 | 56.145 | 0.002 | 0 | 802 | 0 | 0 | 32 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2997.6 | 6785.9 | 5.15 | 38.50 | 1.76 | 1.00 | 234.00 | 0.345 | 0.404 | 1.58e-09 | 4.57e-11 | 0 |
| 232–463 | 1802.3 | 7546.4 | 6.18 | 35.79 | 2.14 | 1.00 | 225.52 | 0.027 | 0.332 | 1.44e-09 | 4.25e-11 | 0 |
| 464–695 | 3070.0 | 6546.2 | 6.44 | 40.25 | 3.30 | 1.00 | 230.56 | 0.211 | 0.286 | 1.59e-09 | 7.25e-11 | 0 |
| 696–927 | 3137.1 | 6274.9 | 8.91 | 51.28 | 7.62 | 1.06 | 243.44 | 13.100 | 10.675 | 1.07e-09 | 4.88e-11 | 41 |
| 928–1159 | 2161.6 | 4293.3 | 6.72 | 41.45 | 6.07 | 3.79 | 818.38 | 83.026 | 104.269 | 4.96e-10 | 8.57e-12 | 194 |
| 1160–1391 | 6.6 | 13.0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 287.786 | 283.871 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.4 | 12.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 828.312 | 817.760 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.6 | 3032.0 | 0.88 | 5.02 | 0.87 | 0.80 | 172.99 | 1378.913 | 1368.837 | 8.27e-10 | 2.75e-11 | 207 |
| 1855–2085 | 6.5 | 11.3 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1953.373 | 1943.328 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.8 | 3250.8 | 1.79 | 10.65 | 1.76 | 1.10 | 237.51 | 2541.464 | 2532.645 | 2.20e-09 | 1.85e-11 | 180 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1136.499 | 1130.901 | 1138.274 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
