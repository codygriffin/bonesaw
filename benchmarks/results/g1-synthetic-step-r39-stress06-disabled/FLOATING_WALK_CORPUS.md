# Bonesaw floating G1 synthetic-step acceptance

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: deterministic, predeclared G1-sized left step; no mocap or learned policy contributes contact labels or target motion.
- Contact schedule: liftoff tick `80`, touchdown tick `100`, step `0.060 m`, clearance `0.030 m`.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.060 m` forward per `1.200 s` source cycle.
- Applied mean forward speed: `0.050 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
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
5 ms p99 deadline: **PASS**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | PASS |

Failed checks: `touchdown_transition_completes_within_8_ticks`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 240 | 1.200 s | 0.000 cm | 0.095 cm | 1.933 cm | 5.845 cm | 0.000° | 4.220 rad/s | 3815.4 µs |

Nominal hard residual maxima: dynamics `1.212e-09`, contact acceleration `4.976e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 0.000 cm |
| authored reference vs measured CoM RMS / p95 | 0.752 / 1.293 cm |
| stance foot RMS | 0.095 cm |
| swing foot RMS | 1.933 cm |
| hand RMS | 5.845 cm |
| maximum root rotation | 0.000° |
| maximum joint velocity | 4.220 rad/s |
| dynamics residual | 1.212e-09 |
| contact acceleration residual | 4.976e-11 |
| raw max dynamics residual, including rejected ticks | 1.212e-09 |
| raw max contact residual, including rejected ticks | 4.976e-11 |
| active normal force range | 0.000–238.104 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 0.020 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 20 / 10 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `False`; final source tick `239.000`, progress `239.000` ticks.
- First post-liftoff authored touchdown source tick: `100`; reached before trace end: `True`.
- Target / applied minimum rate: `1.0000` / `1.0000`; mean / p50 applied `1.0000` / `1.0000`.
- Limited / zero-rate hold ticks: `0` / `0`; maximum required landing time `0.0000 s`.
- Position / tangential / normal limiting ticks: `0` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `20`; policy updates `0`, frozen `0`.
- Maximum applied offset / root reach: `0.0000 / 0.7961 m`.
- Authored-offset / reach / slew limited ticks: `0 / 0 / 0`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `0`, precontact `20`, multi-support `219`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 240 | 1.2 s | 2280.2 µs | 3189.4 µs | 3815.4 µs | 4496.7 µs | 193 | 17 | 20 | 10 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2343.8 | 363.0 | 41.2 | 2792.1 | 4350.7 | 4482.1 | 1013.8 | 240 | 0 | 0 | 426.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 193 | 2277.5 | 2324.0 | 2355.1 | 2489.4 |
| solved_with_slack | 17 | 2886.4 | 3803.3 | 3822.2 | 3826.9 |
| touchdown_transition | 10 | 3436.1 | 4221.7 | 4441.7 | 4496.7 |
| precontact_transition | 20 | 1967.4 | 2196.7 | 2301.4 | 2327.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.98 | 10.0 | 13.0 | 23 | 1.04 | 9.6 | 18 | 0.6920 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.00/1.0/1.0/1 | 232.75/234.0/234.0/234 | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0.0707 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 193 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 17 | 7.94/11.8/12 | 4.59/7.8/8 |
| touchdown_transition | 10 | 13.10/22.1/23 | 9.40/17.3/18 |
| precontact_transition | 20 | 7.85/10.8/11 | 3.90/6.8/7 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.00/1.0/1 | 3.67/4.0/4 | 0.00/0.0/0 | 0 |
| viability | 0.12/1.0/1 | 0.54/5.0/5 | 0.00/0.0/0 | 0 |
| intent | 1.00/1.0/1 | 2.00/2.0/2 | 0.00/0.0/0 | 0 |
| preference | 1.82/9.0/19 | 12.07/58.0/134 | 0.96/9.0/18 | 47 |
| style | 1.04/2.0/2 | 11.13/22.0/23 | 0.08/1.0/2 | 17 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 20 | 10 | 210 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 240 | 0 |
| left_wrist_roll_rubber_hand | 240 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 240 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `20` ticks, planned normal touchdown `10` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.6665 m/s`, p95 `0.9451 m/s`, max `0.9490 m/s` over 20 samples.
Touchdown Normal sole-center tangential speed: p50 `0.0715 m/s`, p95 `0.0939 m/s`, max `0.0966 m/s` over 10 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.563 | 0.562 | 0.562 | 0.999 | 0.999 | 44.387 | 44.605 | 0.219 | 44.605 | 0.001 | 0 | 55 | 0 | 0 | 9 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–23 | 2303.6 | 2466.3 | 4.00 | 23.92 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 24–47 | 2286.3 | 2304.5 | 4.00 | 23.96 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 48–71 | 2298.1 | 2323.1 | 4.00 | 23.83 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 72–95 | 2027.3 | 2323.4 | 6.50 | 36.96 | 2.50 | 1.00 | 226.00 | 0.000 | 1.229 | 1.21e-09 | 3.03e-11 | 16 |
| 96–119 | 3131.1 | 4356.2 | 10.58 | 65.62 | 7.12 | 1.00 | 229.50 | 0.000 | 0.320 | 8.61e-10 | 2.84e-11 | 4 |
| 120–143 | 2231.6 | 3360.4 | 4.71 | 28.33 | 0.79 | 1.00 | 234.00 | 0.000 | 0.142 | 1.12e-09 | 4.29e-11 | 0 |
| 144–167 | 2235.2 | 2301.1 | 4.00 | 23.25 | 0.00 | 1.00 | 234.00 | 0.000 | 0.059 | 8.48e-10 | 3.72e-11 | 0 |
| 168–191 | 2210.8 | 2334.2 | 4.00 | 22.17 | 0.00 | 1.00 | 234.00 | 0.000 | 0.051 | 9.84e-10 | 4.98e-11 | 0 |
| 192–215 | 2266.2 | 2351.3 | 4.00 | 22.58 | 0.00 | 1.00 | 234.00 | 0.000 | 0.051 | 8.66e-10 | 2.95e-11 | 0 |
| 216–239 | 2260.6 | 2322.5 | 4.00 | 23.50 | 0.00 | 1.00 | 234.00 | 0.000 | 0.051 | 9.85e-10 | 2.70e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 240 | 0.000 | 0.405 | 5.845 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
