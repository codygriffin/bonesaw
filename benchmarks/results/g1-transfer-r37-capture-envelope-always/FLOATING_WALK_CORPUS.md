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
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
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
| 461 | 2.305 s | 13.459 cm | 8.474 cm | 24.712 cm | 41.371 cm | 52.982° | 8.000 rad/s | 14511.1 µs |

Nominal hard residual maxima: dynamics `9.004e-09`, contact acceleration `4.098e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 58.131 cm |
| authored reference vs measured CoM RMS / p95 | 51.384 / 136.075 cm |
| stance foot RMS | 28.011 cm |
| swing foot RMS | 46.834 cm |
| hand RMS | 82.122 cm |
| maximum root rotation | 179.518° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.004e-09 |
| contact acceleration residual | 4.098e-10 |
| raw max dynamics residual, including rejected ticks | 9.004e-09 |
| raw max contact residual, including rejected ticks | 4.098e-10 |
| active normal force range | 0.000–638.468 N |
| centroidal momentum-rate residual RMS / max | 53.633 / 259.172 N·m |
| point-task acceleration RMS max | 118.758 m/s² |
| frame-angular acceleration RMS max | 341.982 rad/s² |
| longest pre-contact / touchdown transition | 233 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `23.169` / `42.140 cm`.
- Virtual ZMP clipped on `57.33%` of ticks; clip-distance RMS / max `45.055` / `99.420 cm`.
- Measured-height natural frequency min / p50 / max: `3.672` / `3.730` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.691 m`; height-floor ticks: `108`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-69.776` / `-62.881 cm`; inside on `48.33%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8575 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 35`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.2845 m / 0.6958 / 0.0107 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 7`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `521` ticks; maximum active coordinates `10`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2661.0 µs | 13722.0 µs | 120243.2 µs | 280690.6 µs | 129 | 99 | 233 | 0 | 127 | 12 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6982.3 | 24815.2 | 336.8 | 5530.7 | 256043.1 | 278225.9 | 102880.9 | 600 | 72 | 16 | 143.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2524.4 | 2669.7 | 2708.1 | 2715.0 |
| solved_with_slack | 99 | 2618.1 | 2953.9 | 3090.8 | 3209.8 |
| normal_contact_contingency | 127 | 4704.9 | 14507.7 | 21778.8 | 193187.7 |
| contact_release_contingency | 12 | 113004.8 | 258059.4 | 276164.4 | 280690.6 |
| precontact_transition | 233 | 2677.6 | 4230.4 | 15930.0 | 33775.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.70 | 12.0 | 13.0 | 15 | 5.46 | 12.0 | 15 | 0.0926 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 3.22/8.0/8.0/8 | 706.98/1776.0/1776.0/1776 | 1.51/14.0/26 | 1.20/13.0/25 | 9.95/118.0/222 | 0.1205 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| normal_contact_contingency | 127 | 9.06/13.7/15 | 8.22/12.7/15 |
| contact_release_contingency | 12 | 8.25/10.0/10 | 6.75/8.9/9 |
| precontact_transition | 233 | 8.82/13.7/15 | 7.63/12.7/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.92/6.0/7 | 8.28/25.0/30 | 1.39/6.0/7 | 286 |
| viability | 1.79/6.0/9 | 11.71/42.0/60 | 1.41/6.0/9 | 373 |
| intent | 1.57/4.0/5 | 8.32/24.0/30 | 1.34/4.0/5 | 470 |
| preference | 1.36/6.0/7 | 11.02/50.0/71 | 0.94/6.0/7 | 369 |
| style | 1.05/3.0/5 | 9.10/19.0/28 | 0.39/2.0/5 | 208 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 461 | 139 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `142` ticks.
Precontact sole-center tangential speed: p50 `2.0635 m/s`, p95 `7.5834 m/s`, max `8.9294 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.190 | 4.131 | 4.131 | 0.986 | 0.986 | 47.031 | 47.398 | 0.367 | 49.211 | 0.001 | 0 | 93 | 0 | 0 | 352 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2498.6 | 2643.3 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2625.7 | 2825.0 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2594.8 | 3111.8 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2559.7 | 3040.5 | 7.15 | 45.20 | 4.62 | 1.00 | 225.80 | 6.402 | 0.788 | 1.12e-09 | 4.18e-11 | 12 |
| 240–299 | 2563.5 | 3321.5 | 8.38 | 52.05 | 6.88 | 1.00 | 222.00 | 6.230 | 21.736 | 1.04e-09 | 2.33e-11 | 60 |
| 300–359 | 2501.1 | 3460.1 | 8.77 | 55.18 | 7.28 | 1.00 | 222.00 | 6.674 | 25.141 | 8.86e-10 | 1.11e-11 | 60 |
| 360–419 | 3134.0 | 4286.7 | 9.07 | 53.93 | 8.10 | 4.15 | 921.30 | 19.953 | 12.154 | 6.33e-10 | 9.77e-12 | 60 |
| 420–479 | 3900.4 | 99134.3 | 9.97 | 61.62 | 9.27 | 6.83 | 1501.80 | 41.851 | 30.258 | 9.00e-09 | 4.10e-10 | 60 |
| 480–539 | 5688.3 | 256413.5 | 8.83 | 54.07 | 7.95 | 7.18 | 1546.90 | 100.711 | 68.244 | 2.42e-09 | 5.81e-11 | 60 |
| 540–599 | 3516.2 | 5410.5 | 8.35 | 51.68 | 7.45 | 8.00 | 1728.00 | 146.198 | 75.374 | 2.49e-10 | 2.82e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 58.131 | 35.365 | 82.122 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
