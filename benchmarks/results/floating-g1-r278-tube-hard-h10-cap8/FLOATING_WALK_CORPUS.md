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
| 325 | 1.625 s | 0.539 cm | 0.404 cm | 0.262 cm | 27.809 cm | 0.000° | 8.000 rad/s | 5382.6 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `4.840e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1968.444 cm |
| authored reference vs measured CoM RMS / p95 | 1961.476 / 4136.081 cm |
| stance foot RMS | 1865.086 cm |
| swing foot RMS | 2213.308 cm |
| hand RMS | 1977.644 cm |
| maximum root rotation | 39.863° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.879e-09 |
| contact acceleration residual | 9.197e-11 |
| raw max dynamics residual, including rejected ticks | 3.879e-09 |
| raw max contact residual, including rejected ticks | 9.197e-11 |
| active normal force range | 0.000–511.610 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 154.778 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 43 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Support-transfer trajectory tube

- Hard tube active on `956` ticks. Optional intent projection is `False` and changed the authored CoM/root request on `0` ticks.
- Measured joint-headroom slew scale minimum / p05: `1.000` / `1.000`; projected target displacement RMS is `4.860 cm`.
- Hard acceleration-tube minimum margin: `-3.3306690738754696e-16` m/s²; violations beyond tolerance: `0` ticks; limiting face counts (+x/-x/+y/-y): `{'0': 0, '1': 0, '2': 178, '3': 59}`.
- First-hard-solve witness margin minimum across all active attempts: `-185.11694557321925` m/s²; witness violations: `2` ticks; limiting witness faces (+x/-x/+y/-y): `{'0': 0, '1': 0, '2': 178, '3': 63}`.
- Hard rows returned an admitted solve on `237` active ticks; `719` active requests remained unresolved and are not misreported as boundary violations.
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

- Phase ticks: unsupported `1337`, single support `340`, precontact `0`, multi-support `640`.
- Joint-velocity envelope active on `17` ticks; maximum active coordinates `1`; mean target/applied scale `0.276` / `0.276`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `231` / `748` / `7` / `0` / `0` / `0` / `1331`.
- Bound-limited ticks / upper-bound ticks: `4` / `3`; maximum coordinate violation `0.5888519914919259`.
- Named-linear-row-limited ticks / upper-row ticks: `7` / `6`; maximum row violation `5796.986219495778`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 8.2 µs | 3581.4 µs | 5020.6 µs | 6883.8 µs | 221 | 622 | 0 | 136 | 1,332 | 0 | 0 | 0 | 0 | 6 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1222.5 | 1521.4 | 2.5 | 3137.4 | 6665.3 | 6867.9 | 2012.5 | 986 | 25 | 0 | 818.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 221 | 2882.9 | 2996.1 | 3030.3 | 3056.6 |
| solved_with_slack | 622 | 3027.6 | 4860.9 | 6128.6 | 6883.8 |
| normal_contact_contingency | 1,332 | 6.5 | 9.1 | 12.1 | 4562.2 |
| contact_release_contingency | 6 | 1802.3 | 3143.4 | 3338.0 | 3386.7 |
| touchdown_transition | 136 | 2290.1 | 3281.3 | 3481.5 | 4014.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.90 | 9.0 | 14.0 | 24 | 1.93 | 13.0 | 24 | 0.8774 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.66/1.0/8.0/8 | 150.72/234.0/1808.0/1872 | 0.08/3.0/4 | 0.05/2.0/3 | 0.37/14.0/24 | 0.5766 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 221 | 4.38/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 622 | 7.46/16.0/24 | 5.58/15.0/24 |
| normal_contact_contingency | 1,332 | 0.01/0.0/10 | 0.01/0.0/8 |
| contact_release_contingency | 6 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 136 | 8.12/14.0/15 | 7.24/14.0/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.59/4.8/8 | 2.39/20.0/32 | 0.25/4.8/8 | 186 |
| viability | 0.24/2.0/9 | 1.03/12.0/54 | 0.13/2.0/9 | 217 |
| intent | 0.96/9.0/17 | 1.93/18.0/51 | 0.73/9.0/17 | 492 |
| preference | 0.67/5.0/17 | 5.76/48.0/145 | 0.52/5.0/17 | 649 |
| style | 0.45/2.0/3 | 6.43/32.0/49 | 0.30/2.0/3 | 656 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,643 | 0 | 37 | 637 | 0 |
| right_ankle_roll_link | 1,371 | 0 | 99 | 846 | 1 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `43` ticks, normal fallback `4` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.4858 m/s`, p95 `5.0684 m/s`, max `5.5366 m/s` over 132 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.833 | 2.832 | 2.832 | 1.000 | 1.000 | 53.535 | 57.020 | 3.484 | 57.020 | 0.002 | 0 | 816 | 0 | 0 | 20 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2973.5 | 6374.1 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 1845.1 | 4607.1 | 6.72 | 38.99 | 3.27 | 1.18 | 267.71 | 0.600 | 0.461 | 1.22e-09 | 4.22e-11 | 2 |
| 464–695 | 2924.9 | 4960.5 | 6.90 | 39.50 | 4.52 | 1.00 | 231.68 | 0.795 | 0.303 | 1.33e-09 | 6.53e-11 | 0 |
| 696–927 | 3029.6 | 6075.6 | 5.85 | 33.78 | 5.27 | 1.35 | 315.70 | 41.288 | 17.721 | 2.14e-09 | 9.20e-11 | 59 |
| 928–1159 | 5.9 | 2875.8 | 1.56 | 9.66 | 1.53 | 0.67 | 144.31 | 411.579 | 385.581 | 1.19e-09 | 1.45e-11 | 189 |
| 1160–1391 | 6.5 | 12.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1032.099 | 975.053 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.5 | 11.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1753.666 | 1699.453 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.6 | 3040.3 | 1.24 | 7.31 | 1.20 | 0.63 | 136.52 | 2501.567 | 2457.555 | 1.27e-09 | 9.36e-12 | 197 |
| 1855–2085 | 6.6 | 11.8 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3322.110 | 3279.977 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.9 | 3307.8 | 1.32 | 8.39 | 1.28 | 0.82 | 176.73 | 4152.174 | 4111.412 | 3.88e-09 | 3.44e-11 | 196 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1968.444 | 1938.885 | 1977.644 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
