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
| 295 | 1.475 s | 0.405 cm | 0.404 cm | nan cm | 26.179 cm | 0.000° | 5.530 rad/s | 6723.9 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `4.840e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2662.901 cm |
| authored reference vs measured CoM RMS / p95 | 2661.402 / 4812.471 cm |
| stance foot RMS | 2588.835 cm |
| swing foot RMS | 2910.603 cm |
| hand RMS | 2677.168 cm |
| maximum root rotation | 71.023° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.192e-09 |
| contact acceleration residual | 5.214e-11 |
| raw max dynamics residual, including rejected ticks | 3.192e-09 |
| raw max contact residual, including rejected ticks | 5.214e-11 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 137.911 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 51 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Support-transfer trajectory tube

- `exact discrete DCM backward-reachable set` telemetry is active on `936` ticks; hard enforcement is `True`. Optional intent projection is `True` and changed the authored CoM/root request on `861` ticks.
- Measured joint-headroom slew scale minimum / p05: `1.000` / `1.000`; projected target displacement RMS is `1.033 cm`.
- Hard acceleration-tube minimum margin: `inactive` m/s²; violations beyond tolerance: `0` ticks; limiting face counts (+x/-x/+y/-y): `{'0': 0, '1': 0, '2': 0, '3': 0}`.
- First-hard-solve witness margin minimum across all active attempts: `-2.836165448276511` m/s²; witness violations: `5` ticks; limiting witness faces (+x/-x/+y/-y): `{'0': 0, '1': 0, '2': 5, '3': 0}`.
- Hard rows returned an admitted solve on `0` active ticks; `936` active requests remained unresolved and are not misreported as boundary violations.
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

- Phase ticks: unsupported `1914`, single support `108`, precontact `0`, multi-support `295`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.127` / `0.127`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `137` / `267` / `8` / `0` / `0` / `0` / `1905`.
- Bound-limited ticks / upper-bound ticks: `0` / `0`; maximum coordinate violation `0.0`.
- Named-linear-row-limited ticks / upper-row ticks: `8` / `1`; maximum row violation `103.83873593115862`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 8.4 µs | 3181.7 µs | 4642.9 µs | 7642.0 µs | 137 | 158 | 0 | 109 | 1,905 | 0 | 0 | 0 | 0 | 8 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 556.6 | 1240.1 | 1.9 | 2947.5 | 6789.5 | 7519.9 | 2063.3 | 412 | 18 | 0 | 1796.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 2943.8 | 3380.0 | 3910.8 | 4118.7 |
| solved_with_slack | 158 | 3301.2 | 5822.6 | 6948.4 | 7642.0 |
| normal_contact_contingency | 1,905 | 7.8 | 11.6 | 14.5 | 62.0 |
| contact_release_contingency | 8 | 4182.4 | 5437.8 | 5509.3 | 5527.2 |
| touchdown_transition | 109 | 2148.7 | 3544.4 | 3863.9 | 4619.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.17 | 8.0 | 11.0 | 18 | 0.80 | 10.0 | 16 | 0.8674 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.26/1.0/8.0/8 | 59.53/234.0/1728.0/1728 | 0.03/2.0/4 | 0.02/1.0/3 | 0.12/8.0/24 | 0.5849 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 158 | 8.11/16.9/18 | 6.32/15.4/16 |
| normal_contact_contingency | 1,905 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 8 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 109 | 8.14/14.7/15 | 7.94/12.8/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.26/3.0/6 | 1.07/14.0/26 | 0.13/3.0/6 | 109 |
| viability | 0.07/2.0/9 | 0.41/11.8/54 | 0.07/2.0/9 | 109 |
| intent | 0.36/6.0/13 | 0.73/14.0/26 | 0.26/6.0/13 | 191 |
| preference | 0.30/4.0/14 | 2.47/35.0/112 | 0.23/4.0/14 | 267 |
| style | 0.18/1.0/3 | 2.69/17.0/49 | 0.10/1.0/3 | 225 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,983 | 0 | 39 | 295 | 0 |
| right_ankle_roll_link | 1,952 | 0 | 70 | 295 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `51` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.6069 m/s`, p95 `6.0162 m/s`, max `7.1924 m/s` over 105 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.290 | 1.290 | 1.290 | 1.000 | 1.000 | 53.602 | 57.219 | 3.617 | 57.219 | 0.002 | 0 | 818 | 0 | 0 | 12 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2989.5 | 6678.9 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 7.7 | 5448.0 | 2.47 | 11.91 | 2.13 | 0.27 | 63.54 | 123.312 | 124.595 | 9.69e-10 | 4.22e-11 | 169 |
| 464–695 | 8.1 | 3118.7 | 0.82 | 5.00 | 0.79 | 0.31 | 67.97 | 553.337 | 547.474 | 2.84e-09 | 5.21e-11 | 208 |
| 696–927 | 6.6 | 12.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1112.527 | 1099.421 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 6.2 | 2561.8 | 0.73 | 4.71 | 0.71 | 0.11 | 24.21 | 1707.692 | 1695.322 | 1.78e-09 | 2.44e-11 | 213 |
| 1160–1391 | 6.5 | 13.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2254.738 | 2251.000 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.4 | 12.0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2899.602 | 2900.276 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 9.2 | 3526.3 | 0.52 | 3.10 | 0.51 | 0.34 | 72.94 | 3535.308 | 3528.965 | 1.74e-09 | 1.71e-11 | 216 |
| 1855–2085 | 9.3 | 16.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 4174.735 | 4161.564 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 9.2 | 3492.7 | 1.77 | 11.22 | 1.72 | 0.61 | 132.78 | 4811.847 | 4796.447 | 3.19e-09 | 2.49e-11 | 180 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2662.901 | 2655.532 | 2677.168 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
