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
- Coupled touchdown phase retiming: `enabled`; minimum rate `0.000`, guard `0.020 s`, engagement / release `0 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
- Touchdown admission limits: `0.025 m` position, `0.200 m/s` tangential, and `0.200 m/s` normal; the outgoing support is retained until its replacement locks.
- Reference touchdown blend: `0` ticks (`0.000 s`) with a quintic zero-velocity endpoint.
- Touchdown stabilization point: `ankle origin`.

## Acceptance

Functional: **PASS**  
5 ms p99 deadline: **PASS**  
Combined: **PASS**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `retiming_reaches_first_authored_touchdown` | PASS |
| `p99_tick_le_5ms` | PASS |

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 240 | 1.200 s | 0.383 cm | 0.013 cm | 1.907 cm | 26.684 cm | 0.299° | 8.000 rad/s | 4688.0 µs |

Nominal hard residual maxima: dynamics `1.212e-09`, contact acceleration `5.499e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 0.383 cm |
| authored reference vs measured CoM RMS / p95 | 2.522 / 4.289 cm |
| stance foot RMS | 0.013 cm |
| swing foot RMS | 1.907 cm |
| hand RMS | 26.684 cm |
| maximum root rotation | 0.299° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.212e-09 |
| contact acceleration residual | 5.499e-11 |
| raw max dynamics residual, including rejected ticks | 1.212e-09 |
| raw max contact residual, including rejected ticks | 5.499e-11 |
| active normal force range | 0.000–273.583 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 0.002 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 30 / 3 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `True`; final source tick `218.607`, progress `218.607` ticks.
- First post-liftoff authored touchdown source tick: `100`; reached before trace end: `True`.
- Target / applied minimum rate: `0.4820` / `0.4820`; mean / p50 applied `0.9150` / `1.0000`.
- Limited / zero-rate hold ticks: `24` / `0`; maximum required landing time `0.1074 s`.
- Position / tangential / normal limiting ticks: `15` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `30`; policy updates `0`, frozen `0`.
- Maximum applied offset / root reach: `0.0000 / 0.7961 m`.
- Authored-offset / reach / slew limited ticks: `0 / 0 / 0`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `0`, precontact `30`, multi-support `209`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 240 | 1.2 s | 2384.0 µs | 4094.4 µs | 4688.0 µs | 5320.7 µs | 80 | 127 | 30 | 3 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2625.6 | 635.3 | 134.4 | 3581.3 | 5265.7 | 5315.2 | 1543.4 | 240 | 2 | 0 | 380.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 80 | 2308.0 | 2395.8 | 3208.7 | 3231.0 |
| solved_with_slack | 127 | 2647.0 | 4457.6 | 4991.3 | 5320.7 |
| touchdown_transition | 3 | 2596.0 | 2824.4 | 2844.7 | 2849.8 |
| precontact_transition | 30 | 1911.8 | 2386.9 | 2431.5 | 2435.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.75 | 12.0 | 14.8 | 16 | 3.63 | 13.2 | 14 | 0.3743 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.38/8.0/8.0/8 | 321.15/1872.0/1872.0/1872 | 0.11/2.0/2 | 0.05/1.0/1 | 0.47/9.0/9 | 0.7004 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 80 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 127 | 8.21/16.0/16 | 5.86/14.0/14 |
| touchdown_transition | 3 | 8.67/10.0/10 | 4.67/6.0/6 |
| precontact_transition | 30 | 7.67/11.7/12 | 3.77/7.0/7 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.06/2.0/2 | 3.90/8.0/8 | 0.08/2.0/2 | 14 |
| viability | 0.14/1.0/1 | 0.66/5.0/5 | 0.00/0.0/0 | 0 |
| intent | 2.04/9.0/13 | 4.08/18.0/26 | 1.24/9.0/13 | 80 |
| preference | 2.42/8.0/11 | 15.50/53.9/77 | 2.08/8.0/11 | 160 |
| style | 1.09/2.0/3 | 11.71/23.0/35 | 0.23/1.0/2 | 53 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 30 | 3 | 207 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 240 | 0 |
| left_wrist_roll_rubber_hand | 240 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 240 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `30` ticks, planned normal touchdown `3` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.4894 m/s`, p95 `0.6218 m/s`, max `0.6240 m/s` over 30 samples.
Touchdown Normal sole-center tangential speed: p50 `0.0285 m/s`, p95 `0.0302 m/s`, max `0.0304 m/s` over 3 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.630 | 0.630 | 0.630 | 1.000 | 1.000 | 44.414 | 44.629 | 0.215 | 44.629 | 0.001 | 0 | 54 | 0 | 0 | 5 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–23 | 2307.3 | 2355.8 | 4.00 | 23.92 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 24–47 | 2307.8 | 2320.0 | 4.00 | 23.96 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 48–71 | 2307.8 | 2317.8 | 4.00 | 23.83 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 72–95 | 2165.6 | 3224.5 | 6.54 | 37.67 | 2.67 | 1.00 | 226.00 | 0.000 | 1.270 | 1.21e-09 | 3.03e-11 | 16 |
| 96–119 | 2134.6 | 2857.8 | 7.71 | 41.83 | 4.25 | 1.00 | 226.25 | 0.001 | 0.812 | 8.25e-10 | 3.79e-11 | 14 |
| 120–143 | 2610.1 | 3535.4 | 9.67 | 44.08 | 7.96 | 1.00 | 234.00 | 0.492 | 0.029 | 7.08e-10 | 5.50e-11 | 0 |
| 144–167 | 2560.7 | 4681.5 | 8.42 | 41.17 | 6.04 | 1.58 | 370.50 | 0.938 | 0.006 | 7.96e-10 | 4.65e-11 | 0 |
| 168–191 | 3169.9 | 5267.8 | 8.75 | 38.50 | 6.88 | 4.21 | 984.75 | 0.526 | 0.004 | 6.87e-10 | 4.70e-11 | 0 |
| 192–215 | 2764.1 | 4023.1 | 7.50 | 42.96 | 4.62 | 1.00 | 234.00 | 0.240 | 0.004 | 5.57e-10 | 2.76e-11 | 0 |
| 216–239 | 2876.7 | 4448.9 | 6.88 | 40.58 | 3.88 | 1.00 | 234.00 | 0.083 | 0.004 | 7.67e-10 | 3.14e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 240 | 0.383 | 0.477 | 26.684 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
