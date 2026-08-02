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
- Whole-body posture: `intent` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
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
| 475 | 2.375 s | 8.869 cm | 8.457 cm | 28.730 cm | 22.347 cm | 34.260° | 8.000 rad/s | 91620.0 µs |

Nominal hard residual maxima: dynamics `1.581e-09`, contact acceleration `5.561e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 63.815 cm |
| authored reference vs measured CoM RMS / p95 | 60.861 / 170.385 cm |
| stance foot RMS | 42.373 cm |
| swing foot RMS | 44.004 cm |
| hand RMS | 76.844 cm |
| maximum root rotation | 179.870° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.046e-09 |
| contact acceleration residual | 3.563e-10 |
| raw max dynamics residual, including rejected ticks | 8.046e-09 |
| raw max contact residual, including rejected ticks | 3.563e-10 |
| active normal force range | 0.000–648.624 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 146.035 m/s² |
| frame-angular acceleration RMS max | 290.518 rad/s² |
| longest pre-contact / touchdown transition | 247 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `38.734` / `98.940 cm`.
- Virtual ZMP clipped on `57.17%` of ticks; clip-distance RMS / max `57.552` / `250.660 cm`.
- Measured-height natural frequency min / p50 / max: `3.638` / `3.768` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.911 m`; height-floor ticks: `106`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2305.0 µs | 37884.8 µs | 123450.4 µs | 210735.3 µs | 224 | 4 | 247 | 0 | 114 | 11 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7810.0 | 24019.2 | 443.8 | 5102.6 | 203793.3 | 210041.1 | 91544.4 | 600 | 62 | 37 | 128.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 224 | 2339.8 | 2444.3 | 3359.5 | 3477.6 |
| solved_with_slack | 4 | 2097.5 | 2421.3 | 2438.7 | 2443.0 |
| normal_contact_contingency | 114 | 2752.2 | 7277.3 | 52067.0 | 123152.0 |
| contact_release_contingency | 11 | 152991.7 | 204940.6 | 209576.4 | 210735.3 |
| precontact_transition | 247 | 1889.6 | 66209.6 | 99113.1 | 111174.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.27 | 10.0 | 12.0 | 13 | 4.15 | 11.0 | 12 | 0.1212 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 174.81/8.0/5488.1/7016 | 38757.65/1776.0/1218360.4/1557552 | 0.65/8.0/10 | 0.50/7.0/9 | 4.09/58.0/75 | 0.5525 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 224 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 4 | 7.75/9.0/9 | 5.25/7.0/7 |
| normal_contact_contingency | 114 | 7.60/12.0/13 | 6.84/11.0/12 |
| contact_release_contingency | 11 | 5.82/8.8/9 | 4.36/7.8/8 |
| precontact_transition | 247 | 7.72/12.0/13 | 6.64/11.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.98/6.0/7 | 8.73/28.0/35 | 1.45/6.0/7 | 289 |
| viability | 1.73/7.0/8 | 9.46/42.0/57 | 1.29/7.0/8 | 360 |
| intent | 1.51/6.0/8 | 11.77/44.0/65 | 1.09/5.0/7 | 363 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.05/2.0/5 | 8.74/17.0/35 | 0.32/2.0/4 | 176 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 475 | 125 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `128` ticks.
Precontact sole-center tangential speed: p50 `2.4663 m/s`, p95 `7.7022 m/s`, max `10.9143 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.686 | 4.685 | 4.685 | 1.000 | 1.000 | 48.238 | 48.535 | 0.297 | 48.535 | 0.001 | 0 | 60 | 0 | 0 | 75 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2349.5 | 2492.6 | 4.00 | 24.77 | 0.00 | 1.00 | 234.00 | 0.268 | 0.000 | 1.46e-09 | 5.34e-11 | 0 |
| 60–119 | 2355.1 | 2413.4 | 4.00 | 25.75 | 0.00 | 1.00 | 234.00 | 1.112 | 0.000 | 1.40e-09 | 4.83e-11 | 0 |
| 120–179 | 2351.3 | 3466.6 | 4.00 | 24.88 | 0.00 | 1.00 | 234.00 | 3.165 | 0.000 | 1.58e-09 | 5.56e-11 | 0 |
| 180–239 | 1930.6 | 2584.2 | 5.13 | 31.50 | 1.55 | 1.00 | 225.80 | 7.796 | 0.332 | 1.45e-09 | 5.18e-11 | 12 |
| 240–299 | 1838.4 | 3138.6 | 7.32 | 46.18 | 5.72 | 1.00 | 222.00 | 6.928 | 9.187 | 1.24e-09 | 2.15e-11 | 60 |
| 300–359 | 1752.3 | 2668.7 | 7.43 | 44.97 | 6.40 | 1.00 | 222.00 | 8.327 | 29.604 | 7.10e-10 | 1.82e-11 | 60 |
| 360–419 | 1826.3 | 3550.3 | 8.10 | 48.52 | 7.55 | 1.93 | 429.20 | 8.886 | 29.269 | 1.25e-09 | 1.71e-11 | 60 |
| 420–479 | 5112.6 | 116085.1 | 7.97 | 48.67 | 7.12 | 1664.47 | 369427.40 | 22.603 | 23.798 | 5.50e-10 | 7.38e-12 | 60 |
| 480–539 | 1957.5 | 203897.6 | 7.45 | 45.27 | 6.47 | 68.77 | 14848.50 | 102.632 | 48.858 | 2.13e-09 | 5.94e-11 | 60 |
| 540–599 | 4393.2 | 99232.9 | 7.32 | 46.48 | 6.70 | 6.95 | 1499.60 | 171.495 | 116.832 | 8.05e-09 | 3.56e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 63.815 | 42.919 | 76.844 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
