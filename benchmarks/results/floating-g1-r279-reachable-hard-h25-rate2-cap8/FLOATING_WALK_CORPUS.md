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
| 275 | 1.375 s | 0.339 cm | 0.404 cm | nan cm | 25.102 cm | 0.000° | 5.530 rad/s | 5829.8 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `4.840e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2656.198 cm |
| authored reference vs measured CoM RMS / p95 | 2647.342 / 4677.162 cm |
| stance foot RMS | 2510.265 cm |
| swing foot RMS | 2812.643 cm |
| hand RMS | 2695.821 cm |
| maximum root rotation | 148.951° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.762e-09 |
| contact acceleration residual | 1.281e-10 |
| raw max dynamics residual, including rejected ticks | 9.762e-09 |
| raw max contact residual, including rejected ticks | 1.281e-10 |
| active normal force range | 0.000–719.537 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 120.667 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 52 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Support-transfer trajectory tube

- `exact discrete DCM backward-reachable set` telemetry is active on `1016` ticks; hard enforcement is `True`. Optional intent projection is `False` and changed the authored CoM/root request on `0` ticks.
- Measured joint-headroom slew scale minimum / p05: `1.000` / `1.000`; projected target displacement RMS is `1.926 cm`.
- Hard acceleration-tube minimum margin: `inactive` m/s²; violations beyond tolerance: `0` ticks; limiting face counts (+x/-x/+y/-y): `{'0': 0, '1': 0, '2': 0, '3': 0}`.
- First-hard-solve witness margin minimum across all active attempts: `-1.324012256873379` m/s²; witness violations: `5` ticks; limiting witness faces (+x/-x/+y/-y): `{'0': 0, '1': 0, '2': 5, '3': 0}`.
- Hard rows returned an admitted solve on `0` active ticks; `1016` active requests remained unresolved and are not misreported as boundary violations.
- When hard enforcement is enabled, four allocation-free WBC rows bound realized CoM acceleration. The separately switchable preview projector may shape intent; neither layer admits contact or grants actuator authority, and failed solves remain visible.

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

- Phase ticks: unsupported `1920`, single support `122`, precontact `0`, multi-support `275`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.119` / `0.119`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `137` / `260` / `9` / `0` / `0` / `0` / `1911`.
- Bound-limited ticks / upper-bound ticks: `3` / `2`; maximum coordinate violation `10.340906376113699`.
- Named-linear-row-limited ticks / upper-row ticks: `9` / `4`; maximum row violation `1009.682469298356`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 7.1 µs | 3209.0 µs | 4662.3 µs | 228232.7 µs | 137 | 138 | 0 | 122 | 1,911 | 0 | 0 | 0 | 0 | 9 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 795.8 | 7059.5 | 0.4 | 2954.4 | 116728.9 | 217236.6 | 2045.9 | 406 | 18 | 3 | 1256.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 2936.6 | 3085.8 | 3481.6 | 3518.9 |
| solved_with_slack | 138 | 3163.5 | 5205.3 | 6853.0 | 7118.5 |
| normal_contact_contingency | 1,911 | 7.0 | 10.2 | 12.8 | 23.8 |
| contact_release_contingency | 9 | 4273.4 | 209241.2 | 224434.4 | 228232.7 |
| touchdown_transition | 122 | 2754.4 | 4512.8 | 5647.5 | 5878.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.17 | 8.0 | 12.0 | 18 | 0.81 | 11.0 | 16 | 0.1368 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.36/1.0/8.0/8 | 80.91/234.0/1728.0/1728 | 0.08/3.0/8 | 0.05/2.0/7 | 0.42/16.0/51 | 0.1000 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 138 | 8.14/17.3/18 | 6.29/15.6/16 |
| normal_contact_contingency | 1,911 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 9 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 122 | 8.53/15.8/16 | 8.28/14.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.26/3.0/6 | 1.06/14.0/24 | 0.14/3.0/6 | 122 |
| viability | 0.09/2.0/6 | 0.45/11.0/34 | 0.09/2.0/6 | 122 |
| intent | 0.34/6.0/13 | 0.69/12.0/26 | 0.25/6.0/13 | 184 |
| preference | 0.31/5.0/14 | 2.56/35.0/112 | 0.24/5.0/14 | 259 |
| style | 0.18/1.0/3 | 2.61/17.0/49 | 0.10/1.0/3 | 217 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,970 | 0 | 72 | 275 | 0 |
| right_ankle_roll_link | 1,992 | 0 | 50 | 275 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `52` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `1.6962 m/s`, p95 `3.3989 m/s`, max `4.4099 m/s` over 118 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.844 | 1.844 | 1.844 | 1.000 | 1.000 | 53.570 | 57.184 | 3.613 | 57.184 | 0.002 | 0 | 819 | 0 | 0 | 17 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2980.5 | 6419.2 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 7.8 | 4575.6 | 1.80 | 8.40 | 1.57 | 0.19 | 43.37 | 155.307 | 156.643 | 9.69e-10 | 4.22e-11 | 189 |
| 464–695 | 7.8 | 3658.7 | 2.03 | 12.35 | 1.98 | 0.59 | 126.62 | 608.754 | 566.895 | 4.71e-09 | 5.40e-11 | 180 |
| 696–927 | 6.9 | 12.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1173.282 | 1100.206 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 7.1 | 4470.3 | 0.97 | 6.00 | 0.94 | 0.72 | 156.41 | 1778.309 | 1697.960 | 2.67e-09 | 3.42e-11 | 204 |
| 1160–1391 | 6.8 | 13.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2367.086 | 2263.958 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 7.0 | 15.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2951.097 | 2852.177 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 7.0 | 4480.4 | 0.71 | 4.21 | 0.68 | 0.69 | 149.61 | 3535.547 | 3425.467 | 3.09e-09 | 1.28e-10 | 211 |
| 1855–2085 | 7.0 | 12.6 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 4115.203 | 3997.993 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 7.1 | 4462.1 | 0.78 | 4.82 | 0.77 | 0.46 | 99.12 | 4690.803 | 4571.891 | 9.76e-09 | 1.13e-10 | 209 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2656.198 | 2572.855 | 2695.821 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
