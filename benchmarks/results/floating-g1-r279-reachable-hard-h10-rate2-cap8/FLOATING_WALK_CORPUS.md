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
| 290 | 1.450 s | 0.380 cm | 0.404 cm | nan cm | 25.940 cm | 0.000° | 5.530 rad/s | 5623.4 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `4.840e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2961.910 cm |
| authored reference vs measured CoM RMS / p95 | 2953.486 / 5429.605 cm |
| stance foot RMS | 2819.560 cm |
| swing foot RMS | 3201.811 cm |
| hand RMS | 2988.852 cm |
| maximum root rotation | 160.620° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.635e-09 |
| contact acceleration residual | 3.312e-10 |
| raw max dynamics residual, including rejected ticks | 6.635e-09 |
| raw max contact residual, including rejected ticks | 3.312e-10 |
| active normal force range | 0.000–735.531 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 122.665 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 49 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Support-transfer trajectory tube

- `exact discrete DCM backward-reachable set` telemetry is active on `956` ticks; hard enforcement is `True`. Optional intent projection is `False` and changed the authored CoM/root request on `0` ticks.
- Measured joint-headroom slew scale minimum / p05: `1.000` / `1.000`; projected target displacement RMS is `1.369 cm`.
- Hard acceleration-tube minimum margin: `inactive` m/s²; violations beyond tolerance: `0` ticks; limiting face counts (+x/-x/+y/-y): `{'0': 0, '1': 0, '2': 0, '3': 0}`.
- First-hard-solve witness margin minimum across all active attempts: `-2.3675873244254886` m/s²; witness violations: `5` ticks; limiting witness faces (+x/-x/+y/-y): `{'0': 0, '1': 0, '2': 5, '3': 0}`.
- Hard rows returned an admitted solve on `0` active ticks; `956` active requests remained unresolved and are not misreported as boundary violations.
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

- Phase ticks: unsupported `1847`, single support `180`, precontact `0`, multi-support `290`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.125` / `0.125`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `137` / `333` / `9` / `0` / `0` / `0` / `1838`.
- Bound-limited ticks / upper-bound ticks: `3` / `0`; maximum coordinate violation `13.477268666116117`.
- Named-linear-row-limited ticks / upper-row ticks: `9` / `3`; maximum row violation `81.88140198836291`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 6.7 µs | 3382.6 µs | 4972.7 µs | 215056.1 µs | 137 | 153 | 0 | 180 | 1,838 | 0 | 0 | 0 | 0 | 9 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 904.3 | 7306.1 | 0.5 | 3002.1 | 129717.5 | 211369.1 | 2011.7 | 479 | 22 | 3 | 1105.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 2945.6 | 3420.3 | 3522.7 | 3532.6 |
| solved_with_slack | 153 | 3151.7 | 5121.8 | 6875.3 | 6988.4 |
| normal_contact_contingency | 1,838 | 6.6 | 9.7 | 12.2 | 20.0 |
| contact_release_contingency | 9 | 4223.0 | 208688.3 | 213782.5 | 215056.1 |
| touchdown_transition | 180 | 2737.3 | 4983.7 | 6604.3 | 7405.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.46 | 9.0 | 13.0 | 22 | 1.09 | 12.0 | 22 | 0.1405 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.48/1.0/8.0/8 | 105.45/234.0/1728.0/1728 | 0.13/4.0/9 | 0.09/3.0/8 | 0.74/24.0/67 | 0.1079 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 153 | 8.09/17.0/18 | 6.29/15.5/16 |
| normal_contact_contingency | 1,838 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 9 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 180 | 8.91/14.4/22 | 8.73/14.4/22 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.36/4.0/6 | 1.58/20.0/30 | 0.24/4.0/6 | 180 |
| viability | 0.13/2.8/15 | 0.68/14.5/75 | 0.13/2.8/15 | 180 |
| intent | 0.41/6.0/13 | 0.81/13.0/26 | 0.31/6.0/13 | 257 |
| preference | 0.36/5.0/14 | 3.23/54.7/112 | 0.29/5.0/14 | 332 |
| style | 0.21/1.0/3 | 2.99/17.0/49 | 0.13/1.0/3 | 284 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,941 | 0 | 86 | 290 | 0 |
| right_ankle_roll_link | 1,933 | 0 | 94 | 290 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `49` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `3.7672 m/s`, p95 `7.9171 m/s`, max `9.4599 m/s` over 176 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.096 | 2.095 | 2.095 | 1.000 | 1.000 | 53.656 | 57.207 | 3.551 | 57.207 | 0.002 | 0 | 818 | 0 | 0 | 9 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3010.8 | 6435.2 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 7.4 | 4739.6 | 2.29 | 10.92 | 1.98 | 0.25 | 58.50 | 131.071 | 132.370 | 9.69e-10 | 4.22e-11 | 174 |
| 464–695 | 6.7 | 2965.6 | 1.34 | 8.21 | 1.32 | 0.49 | 106.14 | 550.677 | 531.251 | 3.07e-09 | 2.95e-11 | 195 |
| 696–927 | 6.6 | 11.8 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1123.293 | 1084.657 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 6.5 | 3901.1 | 1.84 | 12.07 | 1.77 | 0.75 | 162.93 | 1767.762 | 1722.346 | 6.63e-09 | 3.31e-10 | 183 |
| 1160–1391 | 6.6 | 12.0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2458.963 | 2386.311 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.5 | 13.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3182.103 | 3115.235 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 7.1 | 7004.5 | 1.80 | 11.10 | 1.77 | 1.15 | 248.73 | 3922.767 | 3840.406 | 6.55e-09 | 1.20e-10 | 182 |
| 1855–2085 | 6.6 | 11.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 4685.796 | 4588.054 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 6.4 | 4961.5 | 1.95 | 12.86 | 1.93 | 1.13 | 244.99 | 5447.963 | 5345.732 | 4.59e-09 | 1.32e-10 | 186 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2961.910 | 2899.119 | 2988.852 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
