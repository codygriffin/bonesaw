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
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `100` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Touchdown admission limits: `0.025 m` position, `0.200 m/s` tangential, and `0.200 m/s` normal; the outgoing support is retained until its replacement locks.
- Reference touchdown blend: `0` ticks (`0.000 s`) with a quintic zero-velocity endpoint.
- Touchdown stabilization point: `ankle origin`.

## Acceptance

Functional: **FAIL**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | FAIL |
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

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 449 | 2.245 s | 12.981 cm | 6.862 cm | 23.426 cm | 42.274 cm | 45.878° | 8.000 rad/s | 18364.7 µs |

Nominal hard residual maxima: dynamics `7.983e-09`, contact acceleration `3.595e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 45.129 cm |
| authored reference vs measured CoM RMS / p95 | 33.345 / 70.089 cm |
| stance foot RMS | 33.046 cm |
| swing foot RMS | 47.478 cm |
| hand RMS | 67.358 cm |
| maximum root rotation | 145.858° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.983e-09 |
| contact acceleration residual | 3.595e-10 |
| raw max dynamics residual, including rejected ticks | 8.661e+02 |
| raw max contact residual, including rejected ticks | 3.595e-10 |
| active normal force range | 0.000–339.546 N |
| centroidal momentum-rate residual RMS / max | 66.912 / 279.583 N·m |
| point-task acceleration RMS max | 200.717 m/s² |
| frame-angular acceleration RMS max | 181.097 rad/s² |
| longest pre-contact / touchdown transition | 221 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `26.176` / `50.498 cm`.
- Virtual ZMP clipped on `55.50%` of ticks; clip-distance RMS / max `55.290` / `122.205 cm`.
- Measured-height natural frequency min / p50 / max: `3.687` / `3.766` / `7.004 rad/s`.
- Minimum measured CoM height: `0.158 m`; height-floor ticks: `107`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-83.108` / `-78.518 cm`; inside on `48.50%` of ticks.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `398` ticks; maximum active coordinates `8`; mean target/applied scale `0.592` / `0.681`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2600.4 µs | 213868.5 µs | 216620.6 µs | 322247.3 µs | 129 | 99 | 221 | 0 | 25 | 22 | 104 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 46030.8 | 82069.5 | 466.6 | 213068.7 | 270519.3 | 317074.5 | 86378.3 | 600 | 163 | 139 | 21.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2469.7 | 2585.0 | 2597.5 | 2653.1 |
| solved_with_slack | 99 | 2550.1 | 2898.6 | 3129.0 | 3129.6 |
| primal_infeasible | 104 | 213177.4 | 217970.3 | 235665.1 | 322247.3 |
| normal_contact_contingency | 25 | 10105.3 | 100417.4 | 111826.8 | 115068.2 |
| contact_release_contingency | 22 | 126745.1 | 185467.1 | 196284.6 | 198973.5 |
| precontact_transition | 221 | 2685.8 | 11986.7 | 48217.3 | 64304.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.00 | 11.0 | 13.0 | 15 | 3.87 | 12.0 | 15 | -0.7662 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1252.58/6720.0/6720.0/6720 | 263688.71/1411200.0/1411200.0/1441584 | 7.70/40.0/40 | 7.36/39.0/39 | 63.12/335.0/335 | 0.9379 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.85/9.0/9 | 3.86/6.0/7 |
| primal_infeasible | 104 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 25 | 9.08/12.0/12 | 8.28/11.8/12 |
| contact_release_contingency | 22 | 7.41/9.6/10 | 5.73/7.0/7 |
| precontact_transition | 221 | 8.53/14.0/15 | 7.26/14.0/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.39/5.0/7 | 5.93/24.0/35 | 0.85/5.0/7 | 188 |
| viability | 1.37/6.0/7 | 8.52/42.0/53 | 0.98/6.0/7 | 267 |
| intent | 1.26/4.0/5 | 6.90/21.0/30 | 1.01/4.0/5 | 359 |
| preference | 1.09/5.0/7 | 9.11/44.0/56 | 0.71/4.0/6 | 291 |
| style | 0.90/3.0/8 | 7.74/20.0/55 | 0.30/2.0/8 | 150 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 449 | 151 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `154` ticks.
Precontact sole-center tangential speed: p50 `2.1171 m/s`, p95 `15.8216 m/s`, max `15.8216 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 27.619 | 27.613 | 27.613 | 1.000 | 1.000 | 48.602 | 48.840 | 0.238 | 48.840 | 0.001 | 0 | 60 | 0 | 0 | 337 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2446.4 | 2560.3 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2533.2 | 2725.9 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2543.2 | 3129.2 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2169.9 | 2950.2 | 7.27 | 46.05 | 4.70 | 1.00 | 225.80 | 6.400 | 1.028 | 1.17e-09 | 4.18e-11 | 12 |
| 240–299 | 2540.1 | 3320.4 | 8.38 | 53.90 | 6.68 | 1.00 | 222.00 | 6.115 | 20.042 | 1.21e-09 | 1.39e-11 | 60 |
| 300–359 | 2280.0 | 6693.6 | 8.25 | 50.10 | 6.90 | 2.17 | 481.00 | 6.009 | 22.413 | 6.23e-10 | 1.48e-11 | 60 |
| 360–419 | 3265.9 | 63266.0 | 8.47 | 51.37 | 7.60 | 184.48 | 40955.30 | 20.923 | 12.838 | 9.41e-10 | 1.41e-11 | 60 |
| 420–479 | 14853.5 | 174794.9 | 8.87 | 55.50 | 7.70 | 684.02 | 147765.00 | 51.533 | 34.593 | 7.98e-09 | 3.60e-10 | 60 |
| 480–539 | 212504.9 | 217263.3 | 2.28 | 14.45 | 2.02 | 4930.13 | 1035336.00 | 91.593 | 78.533 | 8.66e+02 | 1.12e-10 | 60 |
| 540–599 | 213211.6 | 271296.5 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 93.632 | 79.568 | 8.66e+02 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 45.129 | 38.425 | 67.358 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
