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
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root horizontal task weight `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `multi-support` measured-phase policy with immediate engagement and bounded release over `100` ticks.
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
| 453 | 2.265 s | 13.929 cm | 8.240 cm | 22.409 cm | 44.826 cm | 41.903° | 8.000 rad/s | 19253.8 µs |

Nominal hard residual maxima: dynamics `4.339e-09`, contact acceleration `1.459e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 57.316 cm |
| authored reference vs measured CoM RMS / p95 | 50.671 / 121.026 cm |
| stance foot RMS | 31.308 cm |
| swing foot RMS | 78.258 cm |
| hand RMS | 79.046 cm |
| maximum root rotation | 179.445° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.339e-09 |
| contact acceleration residual | 1.459e-10 |
| raw max dynamics residual, including rejected ticks | 4.339e-09 |
| raw max contact residual, including rejected ticks | 1.459e-10 |
| active normal force range | 0.000–539.722 N |
| centroidal momentum-rate residual RMS / max | 43.639 / 133.639 N·m |
| point-task acceleration RMS max | 154.485 m/s² |
| frame-angular acceleration RMS max | 235.073 rad/s² |
| longest pre-contact / touchdown transition | 225 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `40.466` / `88.877 cm`.
- Virtual ZMP clipped on `57.17%` of ticks; clip-distance RMS / max `74.645` / `206.842 cm`.
- Measured-height natural frequency min / p50 / max: `3.669` / `3.732` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.484 m`; height-floor ticks: `103`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `220` ticks; maximum active coordinates `8`; mean phase scale `0.414`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2531.3 µs | 104104.6 µs | 207587.9 µs | 216590.4 µs | 129 | 99 | 225 | 0 | 112 | 35 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 13006.4 | 36992.7 | 488.1 | 9405.4 | 215648.6 | 216496.2 | 82797.2 | 600 | 83 | 49 | 76.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2454.0 | 2606.0 | 2831.6 | 3711.1 |
| solved_with_slack | 99 | 2551.5 | 3071.0 | 3176.7 | 3743.0 |
| normal_contact_contingency | 112 | 3002.0 | 88052.4 | 104097.6 | 189567.1 |
| contact_release_contingency | 35 | 108866.5 | 211543.9 | 216055.8 | 216590.4 |
| precontact_transition | 225 | 2434.7 | 4422.3 | 50262.3 | 73046.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.61 | 12.0 | 14.0 | 15 | 5.29 | 13.0 | 14 | 0.0388 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 106.93/8.0/5467.4/6758 | 23242.12/1776.0/1180967.0/1459728 | 1.01/12.0/17 | 0.76/11.0/16 | 6.24/98.1/141 | 0.2715 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.85/9.0/9 | 3.86/6.0/7 |
| normal_contact_contingency | 112 | 9.04/14.0/14 | 7.86/13.0/13 |
| contact_release_contingency | 35 | 7.51/10.7/11 | 6.29/9.7/10 |
| precontact_transition | 225 | 8.74/14.0/15 | 7.52/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.87/5.0/9 | 7.81/24.0/36 | 1.33/5.0/9 | 281 |
| viability | 1.75/6.0/9 | 11.09/42.0/69 | 1.34/6.0/9 | 363 |
| intent | 1.57/4.0/5 | 8.27/23.0/30 | 1.34/4.0/5 | 467 |
| preference | 1.34/6.0/7 | 12.34/60.0/78 | 0.93/6.0/7 | 371 |
| style | 1.06/2.0/6 | 8.85/18.0/38 | 0.35/2.0/5 | 184 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 453 | 147 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `150` ticks.
Precontact sole-center tangential speed: p50 `2.2870 m/s`, p95 `8.0195 m/s`, max `12.2955 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.804 | 7.802 | 7.802 | 1.000 | 1.000 | 48.582 | 48.906 | 0.324 | 48.906 | 0.001 | 0 | 60 | 0 | 0 | 119 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2429.0 | 2561.6 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2523.5 | 3169.4 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2590.7 | 3402.1 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2421.8 | 3290.4 | 7.43 | 47.53 | 4.85 | 1.00 | 225.80 | 6.406 | 0.920 | 9.06e-10 | 4.18e-11 | 12 |
| 240–299 | 2572.1 | 3742.2 | 8.32 | 51.63 | 6.80 | 1.00 | 222.00 | 5.996 | 20.484 | 9.88e-10 | 2.04e-11 | 60 |
| 300–359 | 1988.7 | 4075.3 | 8.53 | 55.90 | 7.02 | 1.23 | 273.80 | 7.864 | 21.190 | 1.06e-09 | 2.36e-11 | 60 |
| 360–419 | 3075.7 | 4157.1 | 8.78 | 54.15 | 7.95 | 4.62 | 1024.90 | 23.207 | 10.989 | 8.70e-10 | 2.32e-11 | 60 |
| 420–479 | 5706.9 | 141953.8 | 9.05 | 55.30 | 8.12 | 770.78 | 167851.50 | 48.288 | 37.324 | 4.34e-09 | 1.46e-10 | 60 |
| 480–539 | 7075.3 | 209684.6 | 8.67 | 52.85 | 7.63 | 279.90 | 60444.00 | 105.873 | 76.049 | 3.00e-09 | 1.24e-10 | 60 |
| 540–599 | 1896.2 | 215662.7 | 8.83 | 55.58 | 7.47 | 7.78 | 1677.20 | 136.496 | 136.577 | 3.96e-09 | 3.55e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 57.316 | 51.788 | 79.046 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
