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
- Balance task: `standalone-authored` `CoM` reference at `style` priority with weight `0.010` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 875 | 4.375 s | 2.110 cm | 0.355 cm | 0.110 cm | 34.940 cm | 1.331° | 8.000 rad/s | 5186.8 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `5.972e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1328.838 cm |
| authored reference vs measured CoM RMS / p95 | 1317.612 / 2904.018 cm |
| stance foot RMS | 1235.308 cm |
| swing foot RMS | 1473.108 cm |
| hand RMS | 1333.676 cm |
| maximum root rotation | 34.762° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.123e-09 |
| contact acceleration residual | 1.396e-10 |
| raw max dynamics residual, including rejected ticks | 8.123e-09 |
| raw max contact residual, including rejected ticks | 1.396e-10 |
| active normal force range | 0.000–379.325 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 116.571 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 48 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Support-transfer trajectory tube

- `exact discrete DCM backward-reachable set` telemetry is active on `1832` ticks; hard enforcement is `False`. Optional intent projection is `True` and changed the authored CoM/root request on `1205` ticks.
- Measured joint-headroom slew scale minimum / p05: `1.000` / `1.000`; projected target displacement RMS is `4.990 cm`.
- Hard acceleration rows are observer-only in this profile; no admitted margin or boundary-violation count is claimed.
- The observer retained `1832` active requests without installing hard rows or changing the integrated state.
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

- Phase ticks: unsupported `1046`, single support `544`, precontact `0`, multi-support `727`.
- Joint-velocity envelope active on `76` ticks; maximum active coordinates `4`; mean target/applied scale `0.314` / `0.314`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `312` / `957` / `5` / `0` / `0` / `0` / `1043`.
- Bound-limited ticks / upper-bound ticks: `2` / `1`; maximum coordinate violation `7.699403471924189`.
- Named-linear-row-limited ticks / upper-row ticks: `5` / `1`; maximum row violation `152.81148945447643`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 1776.8 µs | 4047.8 µs | 5283.0 µs | 195172.6 µs | 312 | 560 | 0 | 88 | 1,353 | 0 | 0 | 0 | 1 | 3 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1774.0 | 5751.1 | 1758.9 | 3467.9 | 7233.8 | 192654.3 | 2155.8 | 1,274 | 42 | 2 | 563.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 312 | 1824.8 | 2969.3 | 3012.5 | 3069.7 |
| solved_with_slack | 560 | 3095.2 | 4580.0 | 5332.4 | 6793.9 |
| normal_contact_contingency | 1,353 | 16.0 | 3604.0 | 4993.4 | 6735.0 |
| contact_release_contingency | 3 | 184299.1 | 194085.3 | 194955.2 | 195172.6 |
| touchdown_transition | 88 | 2913.6 | 6140.3 | 6966.6 | 7386.2 |
| localized_contact_handoff | 1 | 3692.0 | 3692.0 | 3692.0 | 3692.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.04 | 11.0 | 16.0 | 26 | 2.87 | 14.0 | 24 | 0.2109 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.25/8.0/8.0/8 | 277.04/1728.0/1776.0/1776 | 0.26/4.0/7 | 0.16/3.0/6 | 1.29/24.0/48 | 0.1491 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 312 | 4.55/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 560 | 8.02/22.0/23 | 6.24/20.4/21 |
| normal_contact_contingency | 1,353 | 2.00/13.0/26 | 1.80/13.0/24 |
| contact_release_contingency | 3 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 88 | 8.35/13.0/13 | 7.98/12.1/13 |
| localized_contact_handoff | 1 | 11.00/11.0/11 | 10.00/10.0/10 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.70/3.8/7 | 2.81/15.8/28 | 0.23/3.8/7 | 248 |
| viability | 0.51/6.0/12 | 2.56/30.0/61 | 0.41/6.0/12 | 398 |
| intent | 1.27/9.0/15 | 2.59/18.0/35 | 1.04/9.0/14 | 759 |
| preference | 1.00/9.0/21 | 8.43/75.0/157 | 0.83/8.8/20 | 956 |
| style | 0.56/2.0/3 | 7.97/31.0/49 | 0.36/1.0/3 | 820 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,354 | 0 | 10 | 643 | 310 |
| right_ankle_roll_link | 1,282 | 0 | 156 | 875 | 4 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `126` ticks, normal fallback `313` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.6098 m/s`, p95 `3.4254 m/s`, max `4.5517 m/s` over 162 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.111 | 4.109 | 4.109 | 1.000 | 1.000 | 53.832 | 57.723 | 3.891 | 57.723 | 0.002 | 0 | 936 | 0 | 0 | 52 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2949.6 | 6341.7 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 2029.9 | 4535.1 | 6.72 | 37.97 | 3.28 | 1.57 | 352.78 | 0.665 | 0.334 | 1.29e-09 | 4.22e-11 | 0 |
| 464–695 | 3007.4 | 4783.8 | 6.98 | 41.75 | 4.09 | 1.00 | 230.56 | 0.221 | 0.286 | 1.11e-09 | 5.97e-11 | 0 |
| 696–927 | 3032.5 | 5253.4 | 8.42 | 47.35 | 7.31 | 1.48 | 334.27 | 11.252 | 6.759 | 1.27e-09 | 5.00e-11 | 53 |
| 928–1159 | 2819.3 | 5602.7 | 8.70 | 53.63 | 7.81 | 4.53 | 986.38 | 141.473 | 137.210 | 7.37e-10 | 1.06e-11 | 232 |
| 1160–1391 | 17.7 | 6592.5 | 2.79 | 16.79 | 2.64 | 2.01 | 437.95 | 446.515 | 401.517 | 3.15e-10 | 5.00e-12 | 184 |
| 1392–1623 | 14.9 | 22.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1004.014 | 939.086 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 15.1 | 3538.6 | 0.24 | 1.61 | 0.24 | 0.21 | 45.82 | 1637.659 | 1567.414 | 9.16e-10 | 6.74e-12 | 224 |
| 1855–2085 | 15.7 | 21.3 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2281.137 | 2212.263 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 12.2 | 3950.5 | 1.13 | 6.39 | 1.12 | 0.68 | 145.87 | 2931.421 | 2857.554 | 8.12e-09 | 1.40e-10 | 201 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1328.838 | 1285.806 | 1333.676 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
