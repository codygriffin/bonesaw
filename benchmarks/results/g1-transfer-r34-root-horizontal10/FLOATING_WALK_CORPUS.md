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
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root horizontal task weight `10.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
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
| 429 | 2.145 s | 22.401 cm | 13.954 cm | 64.793 cm | 34.619 cm | 41.415° | 8.000 rad/s | 69874.8 µs |

Nominal hard residual maxima: dynamics `4.786e-09`, contact acceleration `1.650e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 109.025 cm |
| authored reference vs measured CoM RMS / p95 | 107.091 / 271.270 cm |
| stance foot RMS | 83.945 cm |
| swing foot RMS | 106.314 cm |
| hand RMS | 117.237 cm |
| maximum root rotation | 179.543° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.632e-09 |
| contact acceleration residual | 1.650e-10 |
| raw max dynamics residual, including rejected ticks | 4.984e+02 |
| raw max contact residual, including rejected ticks | 1.650e-10 |
| active normal force range | 0.000–671.861 N |
| centroidal momentum-rate residual RMS / max | 57.005 / 274.368 N·m |
| point-task acceleration RMS max | 128.637 m/s² |
| frame-angular acceleration RMS max | 261.293 rad/s² |
| longest pre-contact / touchdown transition | 201 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `54.135` / `99.194 cm`.
- Virtual ZMP clipped on `61.50%` of ticks; clip-distance RMS / max `84.326` / `171.288 cm`.
- Measured-height natural frequency min / p50 / max: `3.655` / `3.943` / `7.004 rad/s`.
- Minimum measured CoM height: `-1.830 m`; height-floor ticks: `169`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3158.5 µs | 276422.7 µs | 279115.9 µs | 282159.8 µs | 97 | 131 | 201 | 0 | 102 | 32 | 37 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 33762.6 | 76088.6 | 724.2 | 174526.3 | 282141.0 | 282157.9 | 77147.6 | 600 | 146 | 107 | 29.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 97 | 2437.4 | 2617.3 | 3540.7 | 3631.3 |
| solved_with_slack | 131 | 2479.8 | 4560.7 | 5402.3 | 5656.8 |
| primal_infeasible | 37 | 276870.3 | 281764.2 | 282148.5 | 282159.8 |
| normal_contact_contingency | 102 | 3090.1 | 72971.1 | 94896.0 | 213991.4 |
| contact_release_contingency | 32 | 198334.7 | 212030.3 | 217211.2 | 219319.2 |
| precontact_transition | 201 | 3689.4 | 59209.9 | 72540.9 | 74071.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.11 | 11.0 | 13.0 | 16 | 5.12 | 12.0 | 16 | -0.5134 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 646.37/6720.0/6720.0/6720 | 137954.70/1411200.0/1411200.0/1419984 | 2.95/19.0/25 | 2.40/18.0/24 | 20.09/155.0/202 | 0.7757 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 97 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 131 | 6.87/10.0/11 | 4.19/8.0/9 |
| primal_infeasible | 37 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 102 | 8.81/13.0/13 | 7.81/12.0/12 |
| contact_release_contingency | 32 | 7.75/9.7/10 | 6.56/8.7/9 |
| precontact_transition | 201 | 8.63/13.0/16 | 7.56/13.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.90/5.0/7 | 8.10/25.0/31 | 1.50/5.0/7 | 351 |
| viability | 1.48/6.0/7 | 8.02/36.1/49 | 1.11/6.0/7 | 361 |
| intent | 1.55/5.0/6 | 7.92/25.0/33 | 1.39/5.0/6 | 466 |
| preference | 1.17/5.0/6 | 11.49/53.0/64 | 0.77/5.0/6 | 341 |
| style | 1.01/3.0/5 | 8.61/24.0/38 | 0.35/3.0/4 | 176 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 429 | 171 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `174` ticks.
Precontact sole-center tangential speed: p50 `3.2639 m/s`, p95 `8.5585 m/s`, max `9.4701 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20.258 | 20.253 | 20.253 | 1.000 | 1.000 | 48.328 | 48.562 | 0.234 | 48.562 | 0.001 | 0 | 59 | 0 | 0 | 315 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2424.4 | 3575.6 | 5.00 | 32.97 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.05e-11 | 0 |
| 60–119 | 2474.7 | 2707.6 | 5.62 | 36.73 | 1.45 | 1.00 | 234.00 | 0.080 | 0.000 | 9.91e-10 | 4.75e-11 | 0 |
| 120–179 | 2568.2 | 5507.0 | 7.02 | 42.68 | 4.68 | 3.45 | 807.30 | 1.043 | 0.000 | 1.39e-09 | 7.32e-11 | 0 |
| 180–239 | 2450.6 | 4039.4 | 7.08 | 42.53 | 4.35 | 3.45 | 769.70 | 6.108 | 0.117 | 1.05e-09 | 4.90e-11 | 12 |
| 240–299 | 3656.8 | 4671.0 | 8.37 | 50.92 | 7.38 | 8.00 | 1776.00 | 12.402 | 17.148 | 2.86e-10 | 9.32e-12 | 60 |
| 300–359 | 3654.3 | 57741.4 | 8.22 | 50.83 | 6.97 | 312.45 | 69363.90 | 30.543 | 56.875 | 9.88e-10 | 7.71e-12 | 60 |
| 360–419 | 9397.9 | 73746.5 | 9.12 | 59.80 | 8.12 | 1034.07 | 229562.80 | 42.769 | 71.611 | 1.18e-09 | 3.63e-11 | 60 |
| 420–479 | 3076.3 | 147459.7 | 9.27 | 58.48 | 8.15 | 549.87 | 118777.60 | 93.169 | 69.454 | 4.79e-09 | 1.65e-10 | 60 |
| 480–539 | 16246.5 | 214782.7 | 8.25 | 47.42 | 7.25 | 319.97 | 69104.80 | 180.979 | 154.502 | 1.77e-09 | 4.68e-11 | 60 |
| 540–599 | 276431.5 | 282141.3 | 3.18 | 19.05 | 2.90 | 4230.48 | 888916.90 | 272.903 | 217.246 | 4.98e+02 | 5.10e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 109.025 | 91.950 | 117.237 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
