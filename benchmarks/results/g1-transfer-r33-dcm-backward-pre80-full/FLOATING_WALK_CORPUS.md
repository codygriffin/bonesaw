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
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `80` ticks (`0.400 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
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
| 412 | 2.060 s | 11.638 cm | 11.426 cm | 33.890 cm | 33.989 cm | 72.592° | 8.000 rad/s | 113677.0 µs |

Nominal hard residual maxima: dynamics `1.362e-09`, contact acceleration `5.993e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 107.473 cm |
| authored reference vs measured CoM RMS / p95 | 108.020 / 228.207 cm |
| stance foot RMS | 77.924 cm |
| swing foot RMS | 90.784 cm |
| hand RMS | 125.962 cm |
| maximum root rotation | 173.062° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.998e-09 |
| contact acceleration residual | 2.983e-10 |
| raw max dynamics residual, including rejected ticks | 1.036e-08 |
| raw max contact residual, including rejected ticks | 2.983e-10 |
| active normal force range | 0.000–619.681 N |
| centroidal momentum-rate residual RMS / max | 74.652 / 367.811 N·m |
| point-task acceleration RMS max | 205.765 m/s² |
| frame-angular acceleration RMS max | 220.341 rad/s² |
| longest pre-contact / touchdown transition | 64 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `82.859` / `155.729 cm`.
- Virtual ZMP clipped on `61.50%` of ticks; clip-distance RMS / max `115.299` / `280.840 cm`.
- Measured-height natural frequency min / p50 / max: `3.658` / `3.775` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.964 m`; height-floor ticks: `170`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3131.1 µs | 54249.7 µs | 118775.4 µs | 197225.1 µs | 97 | 251 | 64 | 0 | 90 | 4 | 0 | 94 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8588.1 | 22184.4 | 702.0 | 5899.7 | 191540.9 | 196656.7 | 18595.2 | 600 | 140 | 39 | 116.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 97 | 2455.1 | 2550.6 | 2576.4 | 2580.6 |
| solved_with_slack | 251 | 3100.3 | 4468.8 | 5199.7 | 5682.6 |
| failed | 94 | 5876.2 | 5947.6 | 7229.2 | 7691.3 |
| normal_contact_contingency | 90 | 2085.2 | 60167.0 | 69080.4 | 132232.5 |
| contact_release_contingency | 4 | 183257.2 | 195801.7 | 196940.4 | 197225.1 |
| precontact_transition | 64 | 3340.4 | 115033.8 | 123678.8 | 125791.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.76 | 11.0 | 12.0 | 14 | 5.86 | 11.0 | 14 | 0.1811 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 230.72/2432.5/5615.4/7073 | 50674.11/525884.4/1246627.7/1570206 | 2.05/7.0/11 | 1.52/6.0/10 | 12.35/48.0/84 | 0.7458 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 97 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 251 | 7.56/12.0/13 | 5.63/11.5/12 |
| failed | 94 | 9.00/9.0/9 | 9.00/9.0/9 |
| normal_contact_contingency | 90 | 9.37/12.0/12 | 8.12/11.0/11 |
| contact_release_contingency | 4 | 7.50/9.9/10 | 6.50/8.9/9 |
| precontact_transition | 64 | 8.62/12.7/14 | 7.78/12.1/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.33/5.0/8 | 9.56/25.0/38 | 1.92/5.0/8 | 396 |
| viability | 1.42/5.0/7 | 7.32/30.0/42 | 1.06/5.0/7 | 415 |
| intent | 1.75/5.0/7 | 9.05/30.0/42 | 1.59/5.0/7 | 503 |
| preference | 1.22/5.0/8 | 12.36/54.0/94 | 0.82/4.0/8 | 391 |
| style | 1.04/2.0/4 | 9.15/19.0/31 | 0.47/2.0/3 | 264 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 149 | 252 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 412 | 188 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `252` ticks, planned normal touchdown `0` ticks, normal fallback `191` ticks.
Precontact sole-center tangential speed: p50 `4.9385 m/s`, p95 `14.0061 m/s`, max `14.0061 m/s` over 252 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.153 | 5.152 | 5.152 | 1.000 | 1.000 | 47.266 | 47.500 | 0.234 | 48.477 | 0.001 | 0 | 59 | 0 | 0 | 81 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2446.7 | 2574.2 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2475.3 | 2685.2 | 5.73 | 38.37 | 1.63 | 1.00 | 234.00 | 0.087 | 0.000 | 1.36e-09 | 5.46e-11 | 0 |
| 120–179 | 2617.3 | 5536.1 | 6.95 | 42.42 | 4.78 | 3.10 | 725.40 | 1.481 | 0.000 | 1.36e-09 | 5.99e-11 | 0 |
| 180–239 | 3001.7 | 4398.8 | 6.88 | 40.82 | 4.23 | 5.20 | 1163.80 | 4.583 | 0.341 | 1.16e-09 | 5.83e-11 | 0 |
| 240–299 | 3329.8 | 4204.1 | 8.35 | 48.83 | 6.88 | 7.65 | 1698.30 | 9.323 | 6.438 | 2.67e-10 | 4.94e-12 | 0 |
| 300–359 | 3225.9 | 4784.3 | 8.42 | 50.90 | 7.47 | 8.00 | 1776.00 | 11.336 | 21.046 | 2.17e-10 | 6.55e-12 | 12 |
| 360–419 | 3541.9 | 128432.2 | 8.97 | 56.57 | 8.17 | 1748.38 | 385814.40 | 32.938 | 58.780 | 3.69e-11 | 1.53e-12 | 60 |
| 420–479 | 2042.1 | 191626.3 | 9.32 | 62.12 | 7.87 | 518.37 | 111966.80 | 124.371 | 133.787 | 8.00e-09 | 2.98e-10 | 60 |
| 480–539 | 5852.3 | 5970.0 | 8.95 | 51.75 | 8.55 | 6.48 | 1400.40 | 219.576 | 149.131 | 1.04e-08 | 1.70e-10 | 60 |
| 540–599 | 5875.0 | 7398.1 | 9.00 | 49.00 | 9.00 | 8.00 | 1728.00 | 224.721 | 154.340 | 1.04e-08 | 1.70e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 107.473 | 82.401 | 125.962 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
