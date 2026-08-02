# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Finite-support CoP constraint: disabled; no loaded finite patch was declared.
- Balance task: `rooted` `CoM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
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
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | FAIL |
| `contact_residual_le_1e_8` | FAIL |
| `p99_tick_le_5ms` | PASS |

Failed checks: `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `dynamics_residual_le_1e_8`, `contact_residual_le_1e_8`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.500 s | 4.684 cm | 0.003 cm | 2.902 cm | 18.131 cm | 22.242° | 8.000 rad/s | 4300.8 µs |

Nominal hard residual maxima: dynamics `1.219e-09`, contact acceleration `5.865e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 15.418 cm |
| authored reference vs measured CoM RMS / p95 | 14.915 / 40.688 cm |
| stance foot RMS | 4.345 cm |
| swing foot RMS | 8.540 cm |
| hand RMS | 27.563 cm |
| maximum root rotation | 22.242° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.207e+02 |
| contact acceleration residual | 2.509e+01 |
| raw max dynamics residual, including rejected ticks | 4.207e+02 |
| raw max contact residual, including rejected ticks | 2.509e+01 |
| active normal force range | 0.000–335.136 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 105.445 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `False` (touchdown `False`, balance `False`); final source tick `339.000`, progress `339.000` ticks.
- First post-liftoff authored touchdown source tick: `None`; reached before trace end: `True`.
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

- Phase ticks: unsupported `28`, single support `113`, precontact `0`, multi-support `199`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 340 | 1.7 s | 2954.2 µs | 3839.2 µs | 4637.2 µs | 9890.4 µs | 199 | 101 | 0 | 0 | 39 | 1 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2727.4 | 1052.2 | 150.6 | 3433.4 | 9200.8 | 9821.4 | 4360.6 | 313 | 2 | 0 | 366.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 2975.7 | 3199.7 | 3236.8 | 3254.1 |
| solved_with_slack | 101 | 2897.1 | 4104.9 | 4842.4 | 4995.2 |
| normal_contact_contingency | 39 | 4.8 | 2191.9 | 7241.3 | 9890.4 |
| contact_release_contingency | 1 | 7856.1 | 7856.1 | 7856.1 | 7856.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.96 | 9.0 | 13.0 | 14 | 2.14 | 11.0 | 14 | 0.4078 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.93/8.0/8.0/8 | 434.49/1776.0/1776.0/1776 | 0.38/3.0/3 | 0.24/2.0/2 | 1.93/16.6/18 | 0.4164 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 101 | 7.79/13.0/14 | 6.25/11.0/14 |
| normal_contact_contingency | 39 | 2.62/13.0/13 | 2.46/13.0/13 |
| contact_release_contingency | 1 | 0.00/0.0/0 | 0.00/0.0/0 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.25/3.0/5 | 4.41/13.0/20 | 0.51/3.0/5 | 69 |
| viability | 0.43/3.0/8 | 1.82/14.2/46 | 0.27/3.0/8 | 60 |
| intent | 1.15/4.6/9 | 2.31/9.2/18 | 0.48/4.6/9 | 85 |
| preference | 1.21/6.2/8 | 10.21/59.2/82 | 0.62/6.2/8 | 113 |
| style | 0.92/1.0/2 | 13.36/17.0/30 | 0.26/1.0/2 | 86 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 141 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 28 | 0 | 0 | 300 | 12 |
| left_wrist_roll_rubber_hand | 340 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 340 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `15` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.928 | 0.927 | 0.927 | 1.000 | 1.000 | 54.777 | 55.500 | 0.723 | 55.500 | 0.001 | 0 | 93 | 0 | 0 | 9 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–33 | 2961.9 | 3184.0 | 4.00 | 28.24 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 34–67 | 2953.4 | 3209.2 | 4.00 | 28.29 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.02e-11 | 0 |
| 68–101 | 2994.5 | 3197.3 | 4.00 | 28.24 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 102–135 | 2960.2 | 3248.4 | 4.00 | 28.29 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 136–169 | 2969.2 | 3221.6 | 4.00 | 28.26 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 170–203 | 2983.1 | 3232.5 | 4.29 | 30.56 | 0.32 | 1.00 | 232.24 | 0.004 | 0.000 | 1.20e-09 | 5.87e-11 | 0 |
| 204–237 | 2217.0 | 3295.6 | 7.21 | 48.97 | 4.15 | 1.00 | 222.00 | 0.120 | 0.011 | 1.14e-09 | 1.45e-11 | 0 |
| 238–271 | 3605.4 | 4668.8 | 8.32 | 43.88 | 7.50 | 6.97 | 1547.47 | 4.339 | 0.796 | 4.12e-10 | 3.74e-12 | 0 |
| 272–305 | 3021.8 | 8275.0 | 8.24 | 47.47 | 7.94 | 5.12 | 1135.06 | 16.964 | 5.228 | 3.85e-10 | 1.14e-11 | 6 |
| 306–339 | 4.6 | 6226.8 | 1.50 | 8.97 | 1.47 | 0.18 | 38.12 | 45.503 | 16.520 | 4.21e+02 | 2.51e+01 | 34 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 340 | 15.418 | 5.485 | 27.563 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
