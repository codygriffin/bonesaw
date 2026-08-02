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
| 300 | 1.500 s | 4.684 cm | 0.003 cm | 2.902 cm | 18.131 cm | 22.242° | 8.000 rad/s | 4189.7 µs |

Nominal hard residual maxima: dynamics `1.219e-09`, contact acceleration `5.865e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 257.740 cm |
| authored reference vs measured CoM RMS / p95 | 255.016 / 591.801 cm |
| stance foot RMS | 200.276 cm |
| swing foot RMS | 301.690 cm |
| hand RMS | 265.492 cm |
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

- Enabled: `False` (touchdown `False`, balance `False`); final source tick `599.000`, progress `599.000` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `True`.
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

- Phase ticks: unsupported `288`, single support `113`, precontact `0`, multi-support `199`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 1758.2 µs | 3517.9 µs | 3974.2 µs | 5172.9 µs | 199 | 101 | 0 | 0 | 299 | 1 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1516.1 | 1496.6 | 1751.8 | 3054.5 | 4835.6 | 5139.2 | 1227.9 | 312 | 1 | 0 | 659.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 3000.9 | 3092.5 | 3120.2 | 3173.8 |
| solved_with_slack | 101 | 2962.2 | 4006.2 | 4609.8 | 5172.9 |
| normal_contact_contingency | 299 | 4.6 | 10.1 | 1897.5 | 3028.5 |
| contact_release_contingency | 1 | 664.6 | 664.6 | 664.6 | 664.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.81 | 9.0 | 11.0 | 14 | 1.21 | 11.0 | 14 | 0.8352 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.09/8.0/8.0/8 | 246.21/1776.0/1776.0/1776 | 0.22/3.0/3 | 0.14/2.0/2 | 1.09/16.0/18 | 0.6087 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 101 | 7.79/13.0/14 | 6.25/11.0/14 |
| normal_contact_contingency | 299 | 0.34/9.0/13 | 0.32/9.0/13 |
| contact_release_contingency | 1 | 0.00/0.0/0 | 0.00/0.0/0 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.71/3.0/5 | 2.50/12.0/20 | 0.29/3.0/5 | 69 |
| viability | 0.24/3.0/8 | 1.03/12.0/46 | 0.15/2.0/8 | 60 |
| intent | 0.65/4.0/9 | 1.31/8.0/18 | 0.27/4.0/9 | 85 |
| preference | 0.68/5.0/8 | 5.79/40.1/82 | 0.35/5.0/8 | 113 |
| style | 0.52/1.0/2 | 7.57/17.0/30 | 0.14/1.0/2 | 86 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 401 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 288 | 0 | 0 | 300 | 12 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `15` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.910 | 0.910 | 0.910 | 1.000 | 1.000 | 54.812 | 55.879 | 1.066 | 55.879 | 0.001 | 0 | 178 | 0 | 0 | 8 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2998.4 | 3081.6 | 4.00 | 28.25 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 60–119 | 2997.1 | 3098.2 | 4.00 | 28.28 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 120–179 | 3006.5 | 3136.5 | 4.00 | 28.45 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 180–239 | 2380.7 | 3216.8 | 6.12 | 41.40 | 2.73 | 1.00 | 225.80 | 0.145 | 0.009 | 1.14e-09 | 5.87e-11 | 0 |
| 240–299 | 3528.1 | 4840.7 | 8.27 | 45.37 | 7.78 | 6.72 | 1491.10 | 10.474 | 2.662 | 4.12e-10 | 1.14e-11 | 0 |
| 300–359 | 4.8 | 2629.5 | 1.70 | 10.25 | 1.60 | 0.20 | 43.20 | 56.360 | 23.709 | 4.21e+02 | 2.51e+01 | 60 |
| 360–419 | 4.6 | 10.3 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 150.833 | 107.886 | 4.21e+02 | 2.51e+01 | 60 |
| 420–479 | 4.6 | 7.0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 288.814 | 252.505 | 4.21e+02 | 2.51e+01 | 60 |
| 480–539 | 4.6 | 11.9 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 441.953 | 411.194 | 4.21e+02 | 2.51e+01 | 60 |
| 540–599 | 4.6 | 6.3 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 599.603 | 569.625 | 4.21e+02 | 2.51e+01 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 257.740 | 238.646 | 265.492 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
