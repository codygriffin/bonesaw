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
- Coupled touchdown phase retiming: `enabled`; minimum rate `0.000`, guard `0.020 s`, engagement / release `0 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
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
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `retiming_reaches_first_authored_touchdown` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `root_tracking_rms_le_5cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 240 | 1.200 s | 9.374 cm | 1.122 cm | 3.260 cm | 31.302 cm | 7.670° | 8.000 rad/s | 5394.4 µs |

Nominal hard residual maxima: dynamics `1.559e-09`, contact acceleration `5.812e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 9.374 cm |
| authored reference vs measured CoM RMS / p95 | 9.193 / 19.847 cm |
| stance foot RMS | 1.122 cm |
| swing foot RMS | 3.260 cm |
| hand RMS | 31.302 cm |
| maximum root rotation | 7.670° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.559e-09 |
| contact acceleration residual | 5.812e-11 |
| raw max dynamics residual, including rejected ticks | 1.559e-09 |
| raw max contact residual, including rejected ticks | 5.812e-11 |
| active normal force range | 0.000–235.169 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 0.002 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 40 / 3 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `True`; final source tick `202.847`, progress `202.847` ticks.
- First post-liftoff authored touchdown source tick: `100`; reached before trace end: `True`.
- Target / applied minimum rate: `0.3725` / `0.3725`; mean / p50 applied `0.8494` / `1.0000`.
- Limited / zero-rate hold ticks: `35` / `0`; maximum required landing time `0.1454 s`.
- Position / tangential / normal limiting ticks: `25` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `40`; policy updates `0`, frozen `0`.
- Maximum applied offset / root reach: `0.0000 / 0.8005 m`.
- Authored-offset / reach / slew limited ticks: `0 / 0 / 0`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `0`, precontact `40`, multi-support `199`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 240 | 1.2 s | 2344.9 µs | 4369.4 µs | 5394.4 µs | 6684.0 µs | 80 | 117 | 40 | 3 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2678.1 | 819.4 | 218.0 | 4002.3 | 6382.4 | 6653.8 | 2515.8 | 240 | 6 | 0 | 373.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 80 | 2320.4 | 2353.1 | 2364.6 | 2372.6 |
| solved_with_slack | 117 | 2782.4 | 4998.1 | 5421.5 | 6684.0 |
| touchdown_transition | 3 | 2577.8 | 2743.1 | 2757.7 | 2761.4 |
| precontact_transition | 40 | 1857.8 | 2219.2 | 2253.4 | 2273.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.23 | 13.0 | 16.6 | 23 | 4.67 | 15.6 | 21 | 0.4143 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.70/8.0/8.0/8 | 395.73/1872.0/1872.0/1872 | 0.20/2.0/2 | 0.10/1.0/1 | 0.88/9.0/9 | 0.8100 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 80 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 117 | 9.20/17.8/23 | 7.98/16.0/21 |
| touchdown_transition | 3 | 9.33/10.0/10 | 7.00/7.0/7 |
| precontact_transition | 40 | 7.78/12.2/13 | 4.12/10.2/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.90/7.6/9 | 7.25/30.4/36 | 1.22/7.6/9 | 94 |
| viability | 0.18/1.0/1 | 0.87/5.0/5 | 0.00/0.0/0 | 0 |
| intent | 1.65/6.0/8 | 3.36/12.0/16 | 1.19/6.0/8 | 129 |
| preference | 2.41/10.0/15 | 15.62/66.1/97 | 2.03/9.6/14 | 159 |
| style | 1.09/2.0/3 | 11.53/23.0/34 | 0.23/2.0/2 | 51 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 40 | 3 | 197 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 240 | 0 |
| left_wrist_roll_rubber_hand | 240 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 240 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `40` ticks, planned normal touchdown `3` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.5888 m/s`, p95 `0.7467 m/s`, max `0.7497 m/s` over 40 samples.
Touchdown Normal sole-center tangential speed: p50 `0.0342 m/s`, p95 `0.0363 m/s`, max `0.0365 m/s` over 3 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.643 | 0.643 | 0.643 | 1.000 | 1.000 | 44.480 | 44.699 | 0.219 | 44.699 | 0.001 | 0 | 55 | 0 | 0 | 2 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–23 | 2334.8 | 2370.2 | 4.00 | 23.92 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 24–47 | 2302.2 | 2350.4 | 4.00 | 23.96 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 48–71 | 2328.6 | 2352.5 | 4.00 | 23.83 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 72–95 | 2081.7 | 2337.0 | 6.33 | 36.29 | 2.33 | 1.00 | 226.00 | 0.000 | 1.779 | 1.21e-09 | 3.03e-11 | 16 |
| 96–119 | 1787.7 | 2261.8 | 7.96 | 40.38 | 4.54 | 1.00 | 222.00 | 0.081 | 2.386 | 5.67e-10 | 1.33e-11 | 24 |
| 120–143 | 2700.6 | 5161.9 | 9.46 | 47.21 | 7.62 | 1.58 | 369.75 | 2.517 | 0.046 | 4.49e-10 | 4.21e-11 | 0 |
| 144–167 | 4344.7 | 6392.9 | 8.38 | 44.58 | 7.00 | 7.42 | 1735.50 | 6.759 | 0.014 | 2.66e-10 | 2.51e-12 | 0 |
| 168–191 | 2749.7 | 3606.8 | 10.38 | 54.12 | 9.21 | 1.00 | 234.00 | 10.793 | 1.179 | 8.29e-10 | 2.73e-11 | 0 |
| 192–215 | 2631.3 | 4138.3 | 9.54 | 49.58 | 8.67 | 1.00 | 234.00 | 14.989 | 3.141 | 1.37e-09 | 5.81e-11 | 0 |
| 216–239 | 2633.4 | 3609.3 | 8.25 | 42.38 | 7.29 | 1.00 | 234.00 | 22.036 | 0.535 | 1.56e-09 | 3.73e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 240 | 9.374 | 1.428 | 31.302 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
