# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `dcm-backward-preview` `DCM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `enabled`; authored offset ≤ `0.080 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `0` scheduled ticks and throughout delayed admission.
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
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 414 | 2.070 s | 13.837 cm | 1.164 cm | 27.940 cm | 37.199 cm | 55.171° | 8.000 rad/s | 11962.7 µs |

Nominal hard residual maxima: dynamics `5.910e-09`, contact acceleration `1.437e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 255.434 cm |
| authored reference vs measured CoM RMS / p95 | 254.352 / 597.623 cm |
| stance foot RMS | 223.140 cm |
| swing foot RMS | 286.261 cm |
| hand RMS | 267.458 cm |
| maximum root rotation | 179.822° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.155e-09 |
| contact acceleration residual | 3.470e-10 |
| raw max dynamics residual, including rejected ticks | 8.155e-09 |
| raw max contact residual, including rejected ticks | 3.470e-10 |
| active normal force range | 0.000–752.763 N |
| centroidal momentum-rate residual RMS / max | 66.554 / 368.626 N·m |
| point-task acceleration RMS max | 184.861 m/s² |
| frame-angular acceleration RMS max | 316.429 rad/s² |
| longest pre-contact / touchdown transition | 86 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 182 / 182 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `281.805` / `670.280 cm`.
- Virtual ZMP clipped on `72.00%` of ticks; clip-distance RMS / max `414.629` / `1095.890 cm`.
- Measured-height natural frequency min / p50 / max: `3.675` / `4.075` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.911 m`; height-floor ticks: `267`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-639.064` / `-567.745 cm`; inside on `28.00%` of ticks.

## Capture-aware landing

- Active target-ticks: `382`; policy updates `200`, frozen `182`.
- Maximum applied offset / root reach: `0.0800 / 0.8610 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 47`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `182 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.6424 m / 2.1867 / 0.0660 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 2`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `193`, precontact `382`, multi-support `224`.
- Joint-velocity envelope active on `517` ticks; maximum active coordinates `9`; mean target/applied scale `0.595` / `0.670`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 800 | 4.0 s | 2722.0 µs | 168560.3 µs | 217371.0 µs | 248896.9 µs | 150 | 178 | 86 | 0 | 311 | 75 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20682.0 | 50107.8 | 553.6 | 97977.2 | 241619.3 | 248169.1 | 112637.3 | 800 | 196 | 99 | 48.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 150 | 2514.0 | 2640.8 | 3715.2 | 3758.4 |
| solved_with_slack | 178 | 2461.9 | 3137.9 | 3757.7 | 4154.7 |
| normal_contact_contingency | 311 | 3674.8 | 31936.9 | 101198.4 | 215118.6 |
| contact_release_contingency | 75 | 169925.7 | 227216.2 | 242156.7 | 248896.9 |
| precontact_transition | 86 | 2206.1 | 11857.2 | 16648.7 | 23642.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.87 | 12.0 | 13.0 | 18 | 5.60 | 13.0 | 17 | 0.0574 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 99.31/8.0/4258.7/6727 | 21455.60/1728.0/919870.6/1453032 | 2.15/18.0/27 | 1.77/17.0/26 | 14.68/147.0/227 | 0.1626 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 150 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 178 | 7.66/12.0/13 | 4.85/10.2/12 |
| normal_contact_contingency | 311 | 9.12/14.9/18 | 8.04/13.9/17 |
| contact_release_contingency | 75 | 7.80/10.3/11 | 6.27/9.3/10 |
| precontact_transition | 86 | 8.86/13.0/13 | 7.50/13.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.15/7.0/13 | 9.51/33.0/63 | 1.71/7.0/13 | 457 |
| viability | 1.78/6.0/8 | 10.97/40.0/59 | 1.42/6.0/8 | 548 |
| intent | 1.52/5.0/6 | 7.76/24.0/39 | 1.28/5.0/6 | 622 |
| preference | 1.34/6.0/8 | 11.88/53.0/73 | 0.85/5.0/7 | 448 |
| style | 1.08/3.0/6 | 8.38/24.0/41 | 0.34/3.0/6 | 223 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 194 | 382 | 0 | 224 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 414 | 386 |
| left_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `382` ticks, planned normal touchdown `0` ticks, normal fallback `389` ticks.
Precontact sole-center tangential speed: p50 `4.1683 m/s`, p95 `10.1952 m/s`, max `11.6381 m/s` over 382 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 16.546 | 16.537 | 16.537 | 0.999 | 0.999 | 47.203 | 47.977 | 0.773 | 49.652 | 0.001 | 0 | 182 | 0 | 0 | 730 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–79 | 2515.4 | 3737.3 | 5.00 | 33.60 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.45e-09 | 4.27e-11 | 0 |
| 80–159 | 2535.6 | 3072.7 | 5.44 | 36.30 | 0.86 | 1.00 | 234.00 | 0.403 | 0.000 | 1.42e-09 | 6.78e-11 | 0 |
| 160–239 | 2563.9 | 4012.4 | 6.95 | 43.48 | 3.74 | 1.00 | 231.60 | 4.491 | 0.009 | 1.25e-09 | 5.77e-11 | 0 |
| 240–319 | 2250.4 | 2788.4 | 8.28 | 49.86 | 5.69 | 1.00 | 222.00 | 5.224 | 1.330 | 1.02e-09 | 1.49e-11 | 0 |
| 320–399 | 2196.1 | 3638.8 | 8.55 | 49.29 | 6.99 | 1.26 | 280.27 | 23.591 | 26.004 | 1.50e-09 | 2.62e-11 | 72 |
| 400–479 | 15512.6 | 227160.5 | 9.44 | 57.39 | 8.36 | 728.49 | 157352.25 | 84.511 | 45.286 | 5.91e-09 | 1.44e-10 | 80 |
| 480–559 | 3092.6 | 7513.7 | 8.96 | 57.61 | 7.83 | 6.78 | 1463.40 | 174.541 | 111.140 | 6.11e-09 | 1.62e-10 | 80 |
| 560–639 | 4599.2 | 78220.2 | 8.40 | 52.05 | 7.31 | 161.10 | 34797.00 | 272.884 | 238.821 | 4.99e-09 | 1.59e-10 | 80 |
| 640–719 | 57594.5 | 241701.2 | 8.74 | 50.45 | 7.36 | 80.06 | 17274.00 | 416.424 | 427.095 | 3.61e-09 | 8.89e-11 | 80 |
| 720–799 | 6432.9 | 211906.5 | 8.97 | 54.94 | 7.84 | 11.45 | 2467.43 | 605.295 | 597.774 | 8.16e-09 | 3.47e-10 | 80 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 800 | 255.434 | 247.358 | 267.458 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
