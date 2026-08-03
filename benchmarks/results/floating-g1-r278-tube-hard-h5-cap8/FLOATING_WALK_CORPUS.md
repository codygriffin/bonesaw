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
| 324 | 1.620 s | 0.538 cm | 0.404 cm | 0.265 cm | 27.738 cm | 0.000° | 8.000 rad/s | 5309.5 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `4.840e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1595.775 cm |
| authored reference vs measured CoM RMS / p95 | 1586.088 / 3273.419 cm |
| stance foot RMS | 1473.896 cm |
| swing foot RMS | 1732.655 cm |
| hand RMS | 1612.813 cm |
| maximum root rotation | 140.332° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.604e-09 |
| contact acceleration residual | 1.092e-10 |
| raw max dynamics residual, including rejected ticks | 4.604e-09 |
| raw max contact residual, including rejected ticks | 1.092e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 115.020 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 62 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Support-transfer trajectory tube

- Hard tube active on `936` ticks. Optional intent projection is `False` and changed the authored CoM/root request on `0` ticks.
- Measured joint-headroom slew scale minimum / p05: `1.000` / `1.000`; projected target displacement RMS is `4.799 cm`.
- Hard acceleration-tube minimum margin: `-8.881784197001252e-16` m/s²; violations beyond tolerance: `0` ticks; limiting face counts (+x/-x/+y/-y): `{'0': 0, '1': 0, '2': 171, '3': 62}`.
- First-hard-solve witness margin minimum across all active attempts: `-50.90010882643414` m/s²; witness violations: `1` ticks; limiting witness faces (+x/-x/+y/-y): `{'0': 1, '1': 0, '2': 171, '3': 63}`.
- Hard rows returned an admitted solve on `233` active ticks; `703` active requests remained unresolved and are not misreported as boundary violations.
- Four allocation-free hard WBC rows bound realized CoM acceleration. The separately switchable preview projector may shape intent; neither layer admits contact or grants actuator authority, and failed solves remain visible.

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

- Phase ticks: unsupported `1326`, single support `346`, precontact `0`, multi-support `645`.
- Joint-velocity envelope active on `4` ticks; maximum active coordinates `1`; mean target/applied scale `0.278` / `0.278`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `317` / `674` / `5` / `0` / `0` / `0` / `1321`.
- Bound-limited ticks / upper-bound ticks: `2` / `2`; maximum coordinate violation `11.012049779295092`.
- Named-linear-row-limited ticks / upper-row ticks: `5` / `3`; maximum row violation `2683.4734972055585`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 8.1 µs | 3534.8 µs | 5112.5 µs | 231801.8 µs | 311 | 529 | 0 | 151 | 1,321 | 0 | 0 | 0 | 0 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1435.7 | 6150.5 | 2.5 | 3163.5 | 67321.3 | 211817.8 | 2220.6 | 996 | 26 | 3 | 696.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 311 | 1839.5 | 2954.1 | 3012.7 | 3110.8 |
| solved_with_slack | 529 | 3052.1 | 4582.6 | 6257.5 | 7028.5 |
| normal_contact_contingency | 1,321 | 6.4 | 8.8 | 13.2 | 22.2 |
| contact_release_contingency | 5 | 92618.0 | 214544.4 | 228350.3 | 231801.8 |
| touchdown_transition | 151 | 2717.0 | 5065.1 | 7647.0 | 12565.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.90 | 10.0 | 13.0 | 23 | 1.85 | 12.8 | 22 | 0.1947 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.66/1.0/8.0/8 | 148.02/234.0/1728.0/1808 | 0.11/3.0/15 | 0.08/2.0/14 | 0.63/17.0/118 | 0.1306 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 311 | 4.55/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 529 | 7.61/17.4/23 | 5.94/15.7/22 |
| normal_contact_contingency | 1,321 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 5 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 151 | 8.52/13.5/14 | 7.62/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.53/4.0/7 | 2.20/16.8/28 | 0.15/4.0/7 | 121 |
| viability | 0.19/2.0/7 | 0.96/14.0/45 | 0.09/2.0/7 | 119 |
| intent | 0.98/8.0/13 | 1.97/16.0/26 | 0.75/8.0/13 | 478 |
| preference | 0.76/6.8/18 | 6.34/62.5/130 | 0.59/6.0/18 | 674 |
| style | 0.44/2.0/3 | 6.38/31.0/51 | 0.26/1.0/3 | 583 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,640 | 0 | 35 | 642 | 0 |
| right_ankle_roll_link | 1,358 | 0 | 116 | 843 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `62` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `1.5158 m/s`, p95 `4.2516 m/s`, max `6.0247 m/s` over 147 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.327 | 3.327 | 3.326 | 1.000 | 1.000 | 53.461 | 57.031 | 3.570 | 57.031 | 0.002 | 0 | 819 | 0 | 0 | 25 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2930.9 | 6295.0 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 2018.4 | 4502.8 | 6.67 | 39.67 | 3.16 | 1.12 | 253.96 | 0.593 | 0.471 | 1.22e-09 | 4.22e-11 | 1 |
| 464–695 | 3000.1 | 6173.1 | 6.52 | 39.75 | 3.71 | 1.00 | 231.68 | 0.340 | 0.302 | 1.29e-09 | 4.94e-11 | 0 |
| 696–927 | 2938.1 | 5046.2 | 5.82 | 31.87 | 4.94 | 0.77 | 179.53 | 13.457 | 9.353 | 1.06e-09 | 5.99e-11 | 54 |
| 928–1159 | 5.7 | 3262.6 | 2.04 | 12.29 | 2.00 | 0.95 | 204.83 | 340.731 | 332.807 | 1.56e-09 | 5.38e-11 | 180 |
| 1160–1391 | 6.5 | 3188.9 | 0.47 | 3.69 | 0.46 | 0.04 | 9.31 | 855.921 | 769.294 | 2.22e-09 | 1.07e-11 | 222 |
| 1392–1623 | 6.4 | 12.6 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1475.965 | 1384.916 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.5 | 7947.9 | 1.19 | 7.48 | 1.19 | 0.87 | 187.01 | 2094.773 | 1989.079 | 4.60e-09 | 1.09e-10 | 199 |
| 1855–2085 | 6.4 | 13.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2693.839 | 2586.399 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.8 | 4278.1 | 0.89 | 5.76 | 0.87 | 0.83 | 179.53 | 3294.972 | 3186.958 | 3.16e-09 | 9.75e-11 | 207 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1595.775 | 1528.522 | 1612.813 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
