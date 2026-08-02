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
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `multi-support` measured-phase policy with immediate engagement and bounded release over `200` ticks.
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
| 455 | 2.275 s | 12.910 cm | 8.196 cm | 22.857 cm | 39.515 cm | 73.291° | 8.000 rad/s | 93822.0 µs |

Nominal hard residual maxima: dynamics `1.483e-09`, contact acceleration `5.511e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 68.177 cm |
| authored reference vs measured CoM RMS / p95 | 63.425 / 156.254 cm |
| stance foot RMS | 40.839 cm |
| swing foot RMS | 33.777 cm |
| hand RMS | 92.059 cm |
| maximum root rotation | 179.534° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.332e-09 |
| contact acceleration residual | 3.364e-10 |
| raw max dynamics residual, including rejected ticks | 9.332e-09 |
| raw max contact residual, including rejected ticks | 3.364e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 61.566 / 357.435 N·m |
| point-task acceleration RMS max | 123.878 m/s² |
| frame-angular acceleration RMS max | 391.095 rad/s² |
| longest pre-contact / touchdown transition | 227 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `10.831` / `19.996 cm`.
- Virtual ZMP clipped on `60.33%` of ticks; clip-distance RMS / max `27.847` / `57.051 cm`.
- Measured-height natural frequency min / p50 / max: `3.653` / `3.746` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.933 m`; height-floor ticks: `134`.
- CoM command acceleration p95 / max: `21.107` / `22.247 m/s²`; support hull `4–4` vertices.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `320` ticks; maximum active coordinates `8`; mean phase scale `0.497`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2649.4 µs | 82778.2 µs | 112259.0 µs | 213761.8 µs | 129 | 99 | 227 | 0 | 129 | 16 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 13302.9 | 30023.2 | 406.9 | 74640.6 | 198976.4 | 212283.3 | 75782.5 | 600 | 125 | 74 | 75.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2502.5 | 2885.4 | 3404.4 | 3457.0 |
| solved_with_slack | 99 | 2555.7 | 2969.5 | 3207.9 | 3609.6 |
| normal_contact_contingency | 129 | 6138.9 | 79297.0 | 99928.2 | 148893.4 |
| contact_release_contingency | 16 | 102212.7 | 195249.1 | 210059.3 | 213761.8 |
| precontact_transition | 227 | 2618.5 | 78847.4 | 102271.3 | 111942.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.76 | 12.0 | 13.0 | 14 | 5.54 | 13.0 | 14 | 0.3074 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 445.25/4848.3/6024.8/6887 | 97234.93/1051380.0/1304769.5/1528914 | 1.11/9.0/14 | 0.91/8.0/13 | 7.41/66.0/112 | 0.6963 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.89/9.0/10 | 3.85/7.0/9 |
| normal_contact_contingency | 129 | 9.12/14.0/14 | 8.33/13.0/14 |
| contact_release_contingency | 16 | 7.56/9.0/9 | 6.25/8.0/8 |
| precontact_transition | 227 | 8.94/13.0/14 | 7.80/12.7/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.91/6.0/7 | 8.49/25.0/35 | 1.38/6.0/7 | 291 |
| viability | 1.82/6.0/8 | 11.66/44.0/55 | 1.43/6.0/7 | 376 |
| intent | 1.59/5.0/9 | 8.33/29.0/52 | 1.36/5.0/9 | 471 |
| preference | 1.33/6.0/7 | 12.04/56.0/78 | 0.95/5.0/7 | 393 |
| style | 1.10/3.0/6 | 9.54/28.0/42 | 0.42/3.0/6 | 195 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 455 | 145 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `148` ticks.
Precontact sole-center tangential speed: p50 `2.4346 m/s`, p95 `6.0320 m/s`, max `9.2528 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.982 | 7.980 | 7.980 | 1.000 | 1.000 | 47.453 | 47.750 | 0.297 | 48.625 | 0.001 | 0 | 60 | 0 | 0 | 107 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2489.1 | 3434.8 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2568.1 | 2919.5 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2553.6 | 3367.8 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2312.0 | 3064.3 | 7.32 | 46.13 | 4.77 | 1.00 | 225.80 | 6.343 | 0.652 | 7.07e-10 | 4.18e-11 | 12 |
| 240–299 | 2712.0 | 3707.4 | 8.57 | 56.50 | 6.92 | 1.58 | 351.50 | 5.397 | 19.996 | 9.91e-10 | 1.97e-11 | 60 |
| 300–359 | 2594.1 | 3186.3 | 8.88 | 56.57 | 7.68 | 1.00 | 222.00 | 6.763 | 20.735 | 1.48e-09 | 1.61e-11 | 60 |
| 360–419 | 2783.5 | 4556.3 | 8.50 | 54.63 | 7.63 | 4.03 | 895.40 | 20.245 | 14.598 | 6.81e-10 | 9.34e-12 | 60 |
| 420–479 | 76321.5 | 127092.5 | 10.52 | 70.43 | 9.75 | 3759.00 | 822456.10 | 52.450 | 33.942 | 3.76e-10 | 4.01e-12 | 60 |
| 480–539 | 7246.1 | 199198.5 | 8.88 | 52.72 | 8.13 | 423.75 | 91520.10 | 143.867 | 59.333 | 9.33e-09 | 3.36e-10 | 60 |
| 540–599 | 5117.8 | 24160.7 | 8.45 | 53.10 | 7.50 | 259.15 | 55976.40 | 150.021 | 96.016 | 3.05e-09 | 3.62e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 68.177 | 38.646 | 92.059 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
