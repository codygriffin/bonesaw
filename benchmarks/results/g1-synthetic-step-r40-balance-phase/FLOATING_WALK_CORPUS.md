# Bonesaw floating G1 synthetic-step acceptance

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: deterministic, predeclared G1-sized left step; no mocap or learned policy contributes contact labels or target motion.
- Contact schedule: liftoff tick `53`, touchdown tick `93`, step `0.010 m`, clearance `0.010 m`.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.010 m` forward per `0.800 s` source cycle.
- Applied mean forward speed: `0.012 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `constant synthetic phase rate` in `0.8 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `dcm-backward-preview` `DCM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `40` ticks (`0.200 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `disabled`; authored offset ≤ `0.120 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `40` scheduled ticks and throughout delayed admission.
- Coupled touchdown phase retiming: `enabled`; minimum rate `0.000`, guard `0.020 s`, engagement / release `50 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
- Coupled balance phase retiming: `enabled`; hold at signed DCM margin ≤ `-0.020 m`, recover nominal rate at `0.000 m`.
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
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `retiming_reaches_first_authored_touchdown` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.800 s | 7.756 cm | 11.438 cm | 1.879 cm | 35.821 cm | 7.879° | 8.000 rad/s | 23035.7 µs |

Nominal hard residual maxima: dynamics `1.361e-09`, contact acceleration `5.379e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 7.756 cm |
| authored reference vs measured CoM RMS / p95 | 7.515 / 13.480 cm |
| stance foot RMS | 11.438 cm |
| swing foot RMS | 1.879 cm |
| hand RMS | 35.821 cm |
| maximum root rotation | 7.879° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.361e-09 |
| contact acceleration residual | 5.379e-11 |
| raw max dynamics residual, including rejected ticks | 1.361e-09 |
| raw max contact residual, including rejected ticks | 5.379e-11 |
| active normal force range | 0.000–336.041 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 75.599 m/s² |
| frame-angular acceleration RMS max | 124.828 rad/s² |
| longest pre-contact / touchdown transition | 107 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 47 / 47 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `11.479` / `23.674 cm`.
- Virtual ZMP clipped on `66.88%` of ticks; clip-distance RMS / max `13.899` / `34.716 cm`.
- Measured-height natural frequency min / p50 / max: `3.712` / `3.768` / `3.785 rad/s`.
- Minimum measured CoM height: `0.685 m`; height-floor ticks: `0`.
- CoM command acceleration p95 / max: `1.494` / `1.567 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-11.451` / `-9.862 cm`; inside on `45.00%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True` (touchdown `True`, balance `True`); final source tick `102.290`, progress `102.290` ticks.
- First post-liftoff authored touchdown source tick: `93`; reached before trace end: `True`.
- Target / applied minimum rate: `0.0000` / `0.0000`; mean / p50 applied `0.6456` / `0.6800`.
- Limited / zero-rate hold ticks: `97` / `17`; maximum required landing time `0.4672 s`.
- Position / tangential / normal limiting ticks: `48` / `0` / `0`; unsafe-edge ticks `47`.
- Balance-margin limited ticks: `41`.

## Capture-aware landing

- Active target-ticks: `107`; policy updates `0`, frozen `0`.
- Maximum applied offset / root reach: `0.0000 / 0.7933 m`.
- Authored-offset / reach / slew limited ticks: `0 / 0 / 0`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `47 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0263 m / 0.2700 / 0.0120 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 27`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `0`, precontact `106`, multi-support `53`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 2500.3 µs | 4082.8 µs | 23035.7 µs | 29615.9 µs | 13 | 40 | 107 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3138.1 | 3505.2 | 595.7 | 3636.2 | 29611.5 | 29615.4 | 27713.3 | 160 | 4 | 2 | 318.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 13 | 2401.2 | 2554.1 | 2608.1 | 2621.6 |
| solved_with_slack | 40 | 2597.5 | 3949.7 | 4508.5 | 4781.2 |
| precontact_transition | 107 | 2415.4 | 4374.0 | 28922.0 | 29615.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.93 | 14.0 | 16.4 | 18 | 6.62 | 13.4 | 14 | 0.0038 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 35.86/8.0/1262.0/1723 | 7964.06/1776.0/280166.2/382506 | 0.68/3.0/3 | 0.38/2.0/2 | 3.10/16.0/16 | 0.9818 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 13 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 40 | 10.75/17.6/18 | 7.67/14.0/14 |
| precontact_transition | 107 | 8.73/12.9/14 | 7.03/11.9/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.57/4.4/5 | 6.41/19.2/22 | 0.85/4.4/5 | 68 |
| viability | 2.61/9.0/10 | 11.72/38.9/52 | 2.21/8.4/10 | 125 |
| intent | 2.04/9.4/11 | 4.14/18.8/22 | 1.88/9.4/10 | 145 |
| preference | 1.66/5.0/7 | 14.59/41.4/63 | 1.32/5.0/6 | 132 |
| style | 1.05/2.0/3 | 8.80/22.4/32 | 0.35/2.0/2 | 53 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 107 | 0 | 53 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 160 | 0 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `107` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.2434 m/s`, p95 `4.6500 m/s`, max `5.0108 m/s` over 107 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.502 | 0.501 | 0.501 | 0.998 | 0.998 | 44.488 | 44.703 | 0.215 | 44.703 | 0.001 | 0 | 38 | 0 | 0 | 72 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2418.0 | 2611.5 | 6.06 | 29.62 | 1.62 | 1.00 | 234.00 | 0.006 | 0.000 | 1.36e-09 | 3.85e-11 | 0 |
| 16–31 | 2558.3 | 3088.9 | 12.44 | 46.50 | 9.69 | 1.00 | 234.00 | 0.320 | 0.000 | 7.80e-10 | 3.93e-11 | 0 |
| 32–47 | 2656.9 | 4676.3 | 10.00 | 48.38 | 6.25 | 1.00 | 234.00 | 0.915 | 0.000 | 1.28e-09 | 5.38e-11 | 0 |
| 48–63 | 1990.2 | 2643.7 | 7.94 | 41.19 | 5.62 | 1.00 | 225.75 | 1.298 | 0.239 | 8.31e-10 | 2.09e-11 | 11 |
| 64–79 | 2536.5 | 4766.0 | 9.44 | 49.94 | 7.06 | 4.50 | 999.00 | 3.006 | 1.021 | 4.44e-10 | 2.58e-12 | 16 |
| 80–95 | 3176.5 | 4433.5 | 8.06 | 43.88 | 6.19 | 8.00 | 1776.00 | 5.196 | 1.599 | 5.00e-12 | 2.88e-13 | 16 |
| 96–111 | 3188.9 | 3501.5 | 8.50 | 45.06 | 6.94 | 8.00 | 1776.00 | 7.919 | 1.657 | 8.67e-13 | 9.92e-14 | 16 |
| 112–127 | 2032.6 | 3496.9 | 10.00 | 56.50 | 8.56 | 2.75 | 610.50 | 11.395 | 2.554 | 7.61e-10 | 6.16e-12 | 16 |
| 128–143 | 1937.4 | 29611.8 | 8.25 | 45.25 | 7.31 | 329.88 | 73232.25 | 13.642 | 11.135 | 8.67e-10 | 1.01e-11 | 16 |
| 144–159 | 1850.1 | 2931.3 | 8.62 | 50.19 | 6.94 | 1.44 | 319.12 | 13.571 | 30.539 | 6.47e-10 | 1.12e-11 | 16 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 7.756 | 10.342 | 35.821 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
