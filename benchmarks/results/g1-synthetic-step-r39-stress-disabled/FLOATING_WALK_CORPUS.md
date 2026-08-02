# Bonesaw floating G1 synthetic-step acceptance

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: deterministic, predeclared G1-sized left step; no mocap or learned policy contributes contact labels or target motion.
- Contact schedule: liftoff tick `80`, touchdown tick `100`, step `0.100 m`, clearance `0.050 m`.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.100 m` forward per `1.200 s` source cycle.
- Applied mean forward speed: `0.083 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `constant synthetic phase rate` in `1.2 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `rooted` `CoM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `40` ticks (`0.200 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `disabled`; authored offset ≤ `0.120 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `40` scheduled ticks and throughout delayed admission.
- Coupled touchdown phase retiming: `disabled`; minimum rate `0.000`, guard `0.020 s`, engagement / release `0 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
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
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 239 | 1.195 s | 11.312 cm | 1.069 cm | 3.288 cm | 32.147 cm | 10.579° | 8.000 rad/s | 29538.5 µs |

Nominal hard residual maxima: dynamics `8.649e-09`, contact acceleration `2.485e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 11.481 cm |
| authored reference vs measured CoM RMS / p95 | 10.581 / 23.728 cm |
| stance foot RMS | 1.074 cm |
| swing foot RMS | 3.288 cm |
| hand RMS | 32.289 cm |
| maximum root rotation | 11.918° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.649e-09 |
| contact acceleration residual | 2.485e-10 |
| raw max dynamics residual, including rejected ticks | 8.649e-09 |
| raw max contact residual, including rejected ticks | 2.485e-10 |
| active normal force range | 0.000–298.163 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 40.652 m/s² |
| frame-angular acceleration RMS max | 26.196 rad/s² |
| longest pre-contact / touchdown transition | 40 / 3 ticks |
| delayed touchdown admission ticks / longest delay | 20 / 20 ticks |

## Coupled touchdown phase retiming

- Enabled: `False`; final source tick `239.000`, progress `239.000` ticks.
- First post-liftoff authored touchdown source tick: `100`; reached before trace end: `True`.
- Target / applied minimum rate: `1.0000` / `1.0000`; mean / p50 applied `1.0000` / `1.0000`.
- Limited / zero-rate hold ticks: `0` / `0`; maximum required landing time `0.0000 s`.
- Position / tangential / normal limiting ticks: `0` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `40`; policy updates `0`, frozen `0`.
- Maximum applied offset / root reach: `0.0000 / 0.8005 m`.
- Authored-offset / reach / slew limited ticks: `0 / 0 / 0`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `20 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0105 m / 0.5542 / 0.0043 m/s`.
- Individually viable position / tangential / normal target-ticks: `12 / 0 / 13`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `0`, precontact `40`, multi-support `199`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 240 | 1.2 s | 2341.4 µs | 6189.2 µs | 42891.4 µs | 207751.3 µs | 80 | 116 | 40 | 3 | 1 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4685.1 | 14331.2 | 331.0 | 4889.7 | 171353.6 | 204111.5 | 19652.0 | 240 | 21 | 6 | 213.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 80 | 2284.4 | 2343.2 | 2377.7 | 2388.9 |
| solved_with_slack | 116 | 3582.2 | 19373.2 | 47482.6 | 55459.9 |
| normal_contact_contingency | 1 | 207751.3 | 207751.3 | 207751.3 | 207751.3 |
| touchdown_transition | 3 | 2509.9 | 2598.3 | 2606.1 | 2608.1 |
| precontact_transition | 40 | 1814.3 | 2217.1 | 2380.4 | 2459.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.36 | 13.0 | 17.0 | 21 | 4.97 | 15.6 | 21 | 0.1272 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 2.52/8.0/8.0/8 | 586.42/1872.0/1872.0/1872 | 1.02/17.6/31 | 0.80/16.6/30 | 7.18/152.1/277 | 0.2920 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 80 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 116 | 9.33/17.8/21 | 8.39/17.7/21 |
| normal_contact_contingency | 1 | 9.00/9.0/9 | 7.00/7.0/7 |
| touchdown_transition | 3 | 12.67/16.9/17 | 9.67/13.9/14 |
| precontact_transition | 40 | 7.95/12.6/13 | 4.62/9.6/10 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.91/7.6/8 | 7.53/32.0/32 | 1.29/7.6/8 | 103 |
| viability | 0.22/2.0/4 | 1.11/10.0/20 | 0.07/2.0/4 | 8 |
| intent | 1.76/7.2/11 | 3.56/14.4/22 | 1.31/7.2/11 | 134 |
| preference | 2.25/10.6/13 | 14.42/63.7/78 | 1.86/10.6/13 | 155 |
| style | 1.22/4.0/11 | 12.78/45.8/98 | 0.45/3.6/11 | 74 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 40 | 3 | 196 | 1 |
| right_ankle_roll_link | 0 | 0 | 0 | 239 | 1 |
| left_wrist_roll_rubber_hand | 240 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 240 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `40` ticks, planned normal touchdown `3` ticks, normal fallback `4` ticks.
Precontact sole-center tangential speed: p50 `1.0249 m/s`, p95 `1.7532 m/s`, max `1.8718 m/s` over 40 samples.
Touchdown Normal sole-center tangential speed: p50 `0.0492 m/s`, p95 `0.0552 m/s`, max `0.0559 m/s` over 3 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.125 | 1.124 | 1.124 | 1.000 | 1.000 | 44.406 | 44.719 | 0.312 | 44.719 | 0.001 | 0 | 58 | 0 | 0 | 12 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–23 | 2282.5 | 2382.2 | 4.00 | 23.92 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 24–47 | 2289.2 | 2366.8 | 4.00 | 23.96 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 48–71 | 2284.3 | 2309.7 | 4.00 | 23.83 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 72–95 | 2073.0 | 2425.2 | 6.54 | 37.00 | 2.67 | 1.00 | 226.00 | 0.000 | 2.083 | 1.21e-09 | 3.03e-11 | 16 |
| 96–119 | 1776.4 | 2191.2 | 8.04 | 39.00 | 5.04 | 1.00 | 222.00 | 0.479 | 1.820 | 8.36e-10 | 1.18e-11 | 24 |
| 120–143 | 3143.5 | 6095.0 | 9.08 | 44.79 | 7.58 | 3.33 | 779.25 | 3.101 | 0.963 | 7.15e-10 | 2.21e-11 | 0 |
| 144–167 | 4294.0 | 6035.2 | 8.50 | 43.62 | 7.58 | 7.42 | 1735.50 | 8.026 | 1.031 | 6.56e-10 | 4.10e-11 | 0 |
| 168–191 | 2959.9 | 3854.9 | 10.92 | 57.62 | 9.54 | 1.00 | 234.00 | 12.774 | 1.745 | 1.00e-09 | 3.86e-11 | 0 |
| 192–215 | 3414.6 | 6050.9 | 9.54 | 48.17 | 8.92 | 3.62 | 848.25 | 18.044 | 1.251 | 8.33e-10 | 3.36e-11 | 0 |
| 216–239 | 4291.8 | 172724.3 | 9.00 | 52.12 | 8.42 | 4.79 | 1117.25 | 27.480 | 1.141 | 8.65e-09 | 2.48e-10 | 1 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 240 | 11.481 | 1.247 | 32.289 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
