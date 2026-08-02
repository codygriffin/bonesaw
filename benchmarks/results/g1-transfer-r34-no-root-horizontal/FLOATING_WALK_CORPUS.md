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
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root horizontal task weight `0.000`.
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
| 409 | 2.045 s | 9.022 cm | 7.210 cm | 38.550 cm | 25.076 cm | 52.477° | 8.000 rad/s | 86464.3 µs |

Nominal hard residual maxima: dynamics `1.629e-09`, contact acceleration `7.208e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 97.312 cm |
| authored reference vs measured CoM RMS / p95 | 99.429 / 221.148 cm |
| stance foot RMS | 81.706 cm |
| swing foot RMS | 90.723 cm |
| hand RMS | 105.661 cm |
| maximum root rotation | 165.493° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 5.228e-09 |
| contact acceleration residual | 1.345e-10 |
| raw max dynamics residual, including rejected ticks | 5.228e-09 |
| raw max contact residual, including rejected ticks | 1.345e-10 |
| active normal force range | 0.000–658.862 N |
| centroidal momentum-rate residual RMS / max | 44.232 / 312.347 N·m |
| point-task acceleration RMS max | 105.120 m/s² |
| frame-angular acceleration RMS max | 271.965 rad/s² |
| longest pre-contact / touchdown transition | 181 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `86.806` / `202.859 cm`.
- Virtual ZMP clipped on `63.67%` of ticks; clip-distance RMS / max `114.096` / `338.036 cm`.
- Measured-height natural frequency min / p50 / max: `3.753` / `3.772` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.783 m`; height-floor ticks: `165`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2687.1 µs | 86473.1 µs | 98335.6 µs | 165105.6 µs | 149 | 79 | 181 | 0 | 187 | 4 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11713.9 | 25928.7 | 530.1 | 63392.2 | 150133.8 | 163608.4 | 50490.5 | 600 | 89 | 67 | 85.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 149 | 2422.5 | 2520.6 | 2568.1 | 4066.6 |
| solved_with_slack | 79 | 2544.2 | 5298.6 | 5622.4 | 6222.5 |
| normal_contact_contingency | 187 | 3373.7 | 89896.2 | 92764.4 | 165105.6 |
| contact_release_contingency | 4 | 103646.3 | 135015.4 | 139091.8 | 140110.9 |
| precontact_transition | 181 | 3016.0 | 67990.1 | 97135.1 | 105576.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.83 | 12.0 | 13.0 | 15 | 5.46 | 12.0 | 14 | 0.2999 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 550.60/5497.3/6145.0/6801 | 119657.10/1187416.8/1348075.4/1509822 | 1.12/7.0/9 | 0.73/6.0/8 | 6.03/49.0/67 | 0.9227 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 149 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 79 | 7.65/12.0/12 | 5.03/10.4/12 |
| normal_contact_contingency | 187 | 9.23/14.0/15 | 8.37/13.1/14 |
| contact_release_contingency | 4 | 7.00/7.0/7 | 4.75/5.0/5 |
| precontact_transition | 181 | 8.81/13.0/14 | 7.14/12.2/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.05/6.0/6 | 8.82/25.0/30 | 1.60/6.0/6 | 370 |
| viability | 1.73/6.0/9 | 9.68/36.0/65 | 1.39/6.0/9 | 413 |
| intent | 1.67/6.0/8 | 6.33/23.0/29 | 1.39/6.0/8 | 440 |
| preference | 1.30/5.0/7 | 13.15/57.0/74 | 0.76/5.0/6 | 313 |
| style | 1.08/4.0/5 | 8.61/22.0/27 | 0.32/3.0/4 | 161 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 409 | 191 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `194` ticks.
Precontact sole-center tangential speed: p50 `2.1857 m/s`, p95 `4.5509 m/s`, max `5.8128 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.029 | 7.027 | 7.027 | 1.000 | 1.000 | 47.270 | 47.578 | 0.309 | 48.559 | 0.001 | 0 | 60 | 0 | 0 | 135 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2431.4 | 2533.0 | 5.00 | 30.52 | 0.00 | 1.00 | 234.00 | 0.503 | 0.000 | 1.51e-09 | 4.04e-11 | 0 |
| 60–119 | 2424.8 | 2566.8 | 5.00 | 31.53 | 0.00 | 1.00 | 234.00 | 1.843 | 0.000 | 1.63e-09 | 4.05e-11 | 0 |
| 120–179 | 2453.0 | 4458.1 | 5.88 | 34.57 | 1.93 | 3.22 | 752.70 | 4.382 | 0.000 | 1.22e-09 | 7.21e-11 | 0 |
| 180–239 | 1786.6 | 5768.6 | 8.30 | 44.90 | 5.85 | 2.87 | 662.60 | 8.862 | 3.870 | 1.35e-09 | 4.10e-11 | 12 |
| 240–299 | 1851.3 | 3185.6 | 8.60 | 47.70 | 6.82 | 3.57 | 791.80 | 8.544 | 22.241 | 1.14e-09 | 1.34e-11 | 60 |
| 300–359 | 3066.5 | 3497.2 | 8.97 | 48.58 | 7.52 | 6.83 | 1517.00 | 9.228 | 17.144 | 4.18e-10 | 1.23e-11 | 60 |
| 360–419 | 3364.0 | 129983.5 | 9.23 | 55.03 | 7.77 | 2022.17 | 443867.30 | 24.941 | 57.578 | 7.41e-10 | 8.50e-12 | 60 |
| 420–479 | 83378.5 | 120068.5 | 9.47 | 59.30 | 8.53 | 3452.95 | 745836.80 | 109.513 | 101.164 | 5.23e-09 | 1.35e-10 | 60 |
| 480–539 | 2795.0 | 5495.0 | 8.33 | 52.78 | 7.52 | 6.72 | 1450.80 | 185.531 | 142.244 | 3.70e-09 | 2.67e-11 | 60 |
| 540–599 | 2873.4 | 4199.1 | 9.50 | 61.00 | 8.65 | 5.67 | 1224.00 | 217.715 | 193.174 | 3.07e-10 | 2.38e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 97.312 | 84.795 | 105.661 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
