# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `0.75×` multiplier.
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
| 447 | 2.235 s | 12.651 cm | 2.501 cm | 35.547 cm | 37.191 cm | 78.710° | 8.000 rad/s | 12094.0 µs |

Nominal hard residual maxima: dynamics `1.902e-09`, contact acceleration `6.609e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 68.668 cm |
| authored reference vs measured CoM RMS / p95 | 60.859 / 147.851 cm |
| stance foot RMS | 50.127 cm |
| swing foot RMS | 49.890 cm |
| hand RMS | 82.625 cm |
| maximum root rotation | 179.797° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.662e-09 |
| contact acceleration residual | 1.832e-10 |
| raw max dynamics residual, including rejected ticks | 4.662e-09 |
| raw max contact residual, including rejected ticks | 1.832e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 47.691 / 227.704 N·m |
| point-task acceleration RMS max | 182.847 m/s² |
| frame-angular acceleration RMS max | 266.551 rad/s² |
| longest pre-contact / touchdown transition | 163 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 116 / 116 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `20.968` / `43.977 cm`.
- Virtual ZMP clipped on `63.17%` of ticks; clip-distance RMS / max `46.127` / `125.566 cm`.
- Measured-height natural frequency min / p50 / max: `3.668` / `3.765` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.772 m`; height-floor ticks: `134`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-81.081` / `-63.200 cm`; inside on `42.83%` of ticks.

## Capture-aware landing

- Active target-ticks: `316`; policy updates `200`, frozen `116`.
- Maximum applied offset / root reach: `0.0800 / 0.8858 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 61`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `116 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.5004 m / 0.5210 / 0.0712 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 1`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `82`, precontact `315`, multi-support `202`.
- Joint-velocity envelope active on `438` ticks; maximum active coordinates `8`; mean target/applied scale `0.665` / `0.734`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2689.4 µs | 19144.2 µs | 101666.8 µs | 163917.2 µs | 129 | 155 | 163 | 0 | 144 | 9 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6895.3 | 17147.4 | 441.1 | 10405.7 | 161876.6 | 163713.1 | 80430.2 | 600 | 92 | 28 | 145.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2564.1 | 2722.6 | 2774.9 | 3130.0 |
| solved_with_slack | 155 | 2566.8 | 3212.8 | 3769.3 | 4125.0 |
| normal_contact_contingency | 144 | 3658.8 | 56638.8 | 88990.3 | 121053.3 |
| contact_release_contingency | 9 | 104104.8 | 162554.5 | 163644.6 | 163917.2 |
| precontact_transition | 163 | 3519.4 | 11273.5 | 49069.0 | 53586.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.71 | 11.0 | 13.0 | 16 | 5.38 | 12.0 | 14 | 0.1566 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 90.33/8.0/3608.6/5876 | 19609.03/1776.0/779451.1/1269216 | 1.75/17.0/18 | 1.36/16.0/17 | 11.39/141.0/159 | 0.4810 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 155 | 7.44/12.0/13 | 4.70/11.5/12 |
| normal_contact_contingency | 144 | 9.00/14.0/15 | 8.15/13.0/13 |
| contact_release_contingency | 9 | 8.33/9.9/10 | 6.89/8.9/9 |
| precontact_transition | 163 | 8.94/13.0/16 | 7.75/11.4/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.97/6.0/7 | 8.74/30.0/35 | 1.45/6.0/7 | 305 |
| viability | 1.74/6.0/8 | 10.65/38.0/65 | 1.33/6.0/8 | 371 |
| intent | 1.60/5.0/10 | 8.34/24.0/39 | 1.34/5.0/9 | 468 |
| preference | 1.31/6.0/8 | 10.70/43.0/70 | 0.85/6.0/8 | 340 |
| style | 1.09/3.0/6 | 9.10/20.0/32 | 0.41/3.0/6 | 201 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 82 | 316 | 0 | 202 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 447 | 153 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `316` ticks, planned normal touchdown `0` ticks, normal fallback `156` ticks.
Precontact sole-center tangential speed: p50 `2.4864 m/s`, p95 `7.1615 m/s`, max `9.8597 m/s` over 316 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.137 | 4.128 | 4.128 | 0.998 | 0.998 | 47.074 | 47.309 | 0.234 | 49.102 | 0.001 | 0 | 44 | 0 | 0 | 786 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2530.9 | 2708.1 | 5.00 | 33.62 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.15e-11 | 0 |
| 60–119 | 2610.2 | 3537.9 | 5.33 | 36.25 | 0.72 | 1.00 | 234.00 | 0.155 | 0.000 | 1.35e-09 | 4.77e-11 | 0 |
| 120–179 | 2599.8 | 3324.3 | 5.92 | 38.57 | 1.75 | 1.00 | 234.00 | 1.363 | 0.000 | 9.67e-10 | 4.84e-11 | 0 |
| 180–239 | 2462.5 | 3227.4 | 7.33 | 44.02 | 4.57 | 1.00 | 226.40 | 6.395 | 1.311 | 1.21e-09 | 4.53e-11 | 0 |
| 240–299 | 2395.9 | 3984.4 | 8.45 | 50.47 | 6.50 | 2.28 | 506.90 | 7.617 | 17.667 | 9.56e-10 | 1.08e-11 | 16 |
| 300–359 | 3258.0 | 4544.3 | 8.55 | 51.98 | 7.08 | 4.38 | 973.10 | 3.855 | 20.083 | 8.21e-10 | 1.58e-11 | 60 |
| 360–419 | 3471.4 | 52260.6 | 9.17 | 52.95 | 8.40 | 142.05 | 31535.10 | 16.241 | 22.512 | 6.29e-10 | 2.00e-11 | 60 |
| 420–479 | 10486.5 | 104573.5 | 10.02 | 59.40 | 9.28 | 610.27 | 131839.20 | 59.986 | 55.947 | 1.90e-09 | 6.61e-11 | 60 |
| 480–539 | 4594.9 | 161907.3 | 8.78 | 53.85 | 7.83 | 7.42 | 1597.60 | 132.934 | 80.554 | 4.66e-09 | 1.83e-10 | 60 |
| 540–599 | 3046.2 | 22990.4 | 8.57 | 54.22 | 7.65 | 132.92 | 28710.00 | 159.700 | 119.193 | 5.87e-10 | 1.68e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.56x | 600 | 68.668 | 50.050 | 82.625 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
