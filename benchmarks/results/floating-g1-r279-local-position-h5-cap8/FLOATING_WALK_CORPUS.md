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
| 874 | 4.370 s | 2.068 cm | 0.355 cm | 0.110 cm | 34.923 cm | 1.065° | 8.000 rad/s | 5238.2 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `5.972e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1742.679 cm |
| authored reference vs measured CoM RMS / p95 | 1735.906 / 3647.024 cm |
| stance foot RMS | 1645.899 cm |
| swing foot RMS | 1960.750 cm |
| hand RMS | 1753.436 cm |
| maximum root rotation | 40.183° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.597e-09 |
| contact acceleration residual | 1.385e-10 |
| raw max dynamics residual, including rejected ticks | 3.597e-09 |
| raw max contact residual, including rejected ticks | 1.385e-10 |
| active normal force range | 0.000–884.612 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 184.523 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 51 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Support-transfer trajectory tube

- `finite-horizon support-intersection position tube` telemetry is active on `936` ticks; hard enforcement is `True`. Optional intent projection is `False` and changed the authored CoM/root request on `0` ticks.
- Measured joint-headroom slew scale minimum / p05: `1.000` / `1.000`; projected target displacement RMS is `4.796 cm`.
- Hard acceleration-tube minimum margin: `14.331236080312564` m/s²; violations beyond tolerance: `0` ticks; limiting face counts (+x/-x/+y/-y): `{'0': 27, '1': 53, '2': 93, '3': 61}`.
- First-hard-solve witness margin minimum across all active attempts: `-40.87435453534264` m/s²; witness violations: `1` ticks; limiting witness faces (+x/-x/+y/-y): `{'0': 27, '1': 53, '2': 94, '3': 61}`.
- Hard rows returned an admitted solve on `234` active ticks; `702` active requests remained unresolved and are not misreported as boundary violations.
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

- Phase ticks: unsupported `1317`, single support `355`, precontact `0`, multi-support `645`.
- Joint-velocity envelope active on `16` ticks; maximum active coordinates `1`; mean target/applied scale `0.278` / `0.278`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `312` / `689` / `3` / `0` / `0` / `0` / `1313`.
- Bound-limited ticks / upper-bound ticks: `0` / `0`; maximum coordinate violation `0.0`.
- Named-linear-row-limited ticks / upper-row ticks: `3` / `0`; maximum row violation `660.4328163225327`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 8.4 µs | 3524.1 µs | 4736.9 µs | 272989.4 µs | 312 | 559 | 0 | 130 | 1,313 | 0 | 0 | 0 | 0 | 3 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1352.4 | 5839.5 | 2.5 | 3195.0 | 6847.2 | 211354.6 | 2035.4 | 1,004 | 19 | 1 | 739.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 312 | 1854.9 | 2983.5 | 3063.9 | 3355.9 |
| solved_with_slack | 559 | 3090.5 | 4561.8 | 5622.2 | 6863.2 |
| normal_contact_contingency | 1,313 | 6.7 | 9.3 | 12.4 | 16.7 |
| contact_release_contingency | 3 | 1368.5 | 245827.4 | 267557.0 | 272989.4 |
| touchdown_transition | 130 | 2525.5 | 3777.7 | 4924.0 | 5960.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.01 | 10.0 | 15.0 | 23 | 1.95 | 13.0 | 21 | 0.2114 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.67/1.0/8.0/8 | 152.10/234.0/1728.0/1808 | 0.08/2.0/7 | 0.04/1.0/6 | 0.35/8.0/47 | 0.1287 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 312 | 4.55/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 559 | 7.97/18.4/23 | 6.19/17.4/21 |
| normal_contact_contingency | 1,313 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 3 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 130 | 8.40/14.0/15 | 8.16/13.7/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.55/4.0/6 | 2.25/19.0/28 | 0.18/4.0/6 | 142 |
| viability | 0.21/2.0/10 | 0.91/12.0/36 | 0.10/2.0/9 | 130 |
| intent | 1.02/9.0/15 | 2.05/18.0/30 | 0.79/9.0/14 | 491 |
| preference | 0.78/8.0/19 | 6.41/64.8/157 | 0.62/8.0/18 | 688 |
| style | 0.44/2.0/3 | 6.39/30.8/49 | 0.26/1.0/3 | 584 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,633 | 0 | 42 | 642 | 0 |
| right_ankle_roll_link | 1,355 | 0 | 88 | 874 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `51` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.0574 m/s`, p95 `4.4648 m/s`, max `5.4924 m/s` over 126 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.134 | 3.132 | 3.132 | 0.999 | 0.999 | 53.531 | 57.082 | 3.551 | 57.082 | 0.002 | 0 | 818 | 0 | 0 | 97 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2958.5 | 6382.6 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 2059.2 | 4535.8 | 6.72 | 37.97 | 3.28 | 1.57 | 357.99 | 0.665 | 0.334 | 1.29e-09 | 4.22e-11 | 0 |
| 464–695 | 3035.2 | 4841.5 | 6.98 | 41.75 | 4.09 | 1.00 | 231.68 | 0.221 | 0.286 | 1.11e-09 | 5.97e-11 | 0 |
| 696–927 | 2984.5 | 5981.6 | 6.30 | 33.87 | 5.41 | 0.77 | 179.53 | 15.457 | 11.815 | 1.27e-09 | 4.09e-11 | 54 |
| 928–1159 | 6.0 | 4091.7 | 1.31 | 7.94 | 1.29 | 0.85 | 184.34 | 360.717 | 351.232 | 3.60e-09 | 1.38e-10 | 195 |
| 1160–1391 | 6.7 | 13.0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 924.898 | 877.646 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.6 | 12.0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1571.003 | 1518.443 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.8 | 3817.5 | 1.42 | 9.19 | 1.39 | 0.81 | 173.92 | 2224.773 | 2174.514 | 1.46e-09 | 7.69e-12 | 192 |
| 1855–2085 | 6.7 | 12.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2935.189 | 2890.757 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 6.1 | 2885.4 | 1.91 | 11.48 | 1.87 | 0.74 | 158.96 | 3664.368 | 3622.859 | 2.78e-09 | 2.89e-11 | 180 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1742.679 | 1712.731 | 1753.436 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
