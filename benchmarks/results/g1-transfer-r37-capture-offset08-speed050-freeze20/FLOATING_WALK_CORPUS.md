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
- Landing commitment: freeze the retargeted anchor for the final `20` scheduled ticks and throughout delayed admission.
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
| 482 | 2.410 s | 13.768 cm | 14.676 cm | 25.062 cm | 41.774 cm | 53.095° | 8.000 rad/s | 12158.2 µs |

Nominal hard residual maxima: dynamics `5.411e-09`, contact acceleration `2.195e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 36.377 cm |
| authored reference vs measured CoM RMS / p95 | 34.427 / 81.139 cm |
| stance foot RMS | 29.707 cm |
| swing foot RMS | 36.663 cm |
| hand RMS | 69.740 cm |
| maximum root rotation | 177.545° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 5.750e-09 |
| contact acceleration residual | 3.210e-10 |
| raw max dynamics residual, including rejected ticks | 5.750e-09 |
| raw max contact residual, including rejected ticks | 3.210e-10 |
| active normal force range | 0.000–716.262 N |
| centroidal momentum-rate residual RMS / max | 54.916 / 319.980 N·m |
| point-task acceleration RMS max | 135.186 m/s² |
| frame-angular acceleration RMS max | 250.125 rad/s² |
| longest pre-contact / touchdown transition | 254 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `27.394` / `56.454 cm`.
- Virtual ZMP clipped on `58.17%` of ticks; clip-distance RMS / max `42.885` / `113.749 cm`.
- Measured-height natural frequency min / p50 / max: `3.692` / `3.762` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.056 m`; height-floor ticks: `88`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-73.328` / `-67.353 cm`; inside on `48.83%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `180`, frozen `192`.
- Maximum applied offset / root reach: `0.0800 / 0.8466 m`.
- Authored-offset / reach / slew limited ticks: `180 / 0 / 26`.
- Authored geometry outside configured reach: `0` target-ticks.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `451` ticks; maximum active coordinates `8`; mean target/applied scale `0.666` / `0.745`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2666.9 µs | 89899.5 µs | 205036.4 µs | 221529.1 µs | 129 | 99 | 254 | 0 | 93 | 25 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12903.3 | 37936.9 | 348.9 | 13427.8 | 216959.5 | 221072.2 | 94172.5 | 600 | 86 | 53 | 77.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2590.7 | 2795.3 | 2935.5 | 3081.5 |
| solved_with_slack | 99 | 2655.1 | 3022.6 | 3236.8 | 3292.8 |
| normal_contact_contingency | 93 | 4944.5 | 91121.2 | 111612.0 | 213900.3 |
| contact_release_contingency | 25 | 195220.0 | 210911.5 | 219037.5 | 221529.1 |
| precontact_transition | 254 | 2686.5 | 5921.0 | 13492.9 | 17819.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.59 | 12.0 | 13.0 | 14 | 5.33 | 12.0 | 14 | 0.1057 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 125.03/8.0/5084.4/6794 | 27020.02/1776.0/1098221.8/1467504 | 1.19/15.0/47 | 0.96/14.0/46 | 8.25/124.1/452 | 0.2675 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| normal_contact_contingency | 93 | 9.22/14.0/14 | 8.26/14.0/14 |
| contact_release_contingency | 25 | 7.80/10.5/11 | 6.52/8.8/9 |
| precontact_transition | 254 | 8.61/13.0/14 | 7.45/12.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.93/6.0/8 | 8.21/29.0/37 | 1.38/6.0/8 | 283 |
| viability | 1.74/7.0/8 | 11.14/49.1/64 | 1.35/7.0/8 | 368 |
| intent | 1.59/5.0/8 | 8.25/28.0/32 | 1.35/5.0/8 | 467 |
| preference | 1.27/5.0/8 | 10.81/43.0/79 | 0.84/5.0/7 | 357 |
| style | 1.06/3.0/5 | 9.02/21.0/42 | 0.41/3.0/5 | 217 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 482 | 118 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `121` ticks.
Precontact sole-center tangential speed: p50 `2.2344 m/s`, p95 `6.3948 m/s`, max `7.6400 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.742 | 7.717 | 7.717 | 0.997 | 0.997 | 48.770 | 48.910 | 0.141 | 49.762 | 0.001 | 0 | 35 | 0 | 0 | 1,370 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2545.1 | 2772.1 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2652.3 | 3074.4 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2651.0 | 3216.1 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2422.2 | 3328.4 | 7.13 | 46.18 | 4.57 | 1.00 | 225.80 | 6.405 | 0.736 | 7.54e-10 | 4.18e-11 | 12 |
| 240–299 | 2624.9 | 3446.1 | 8.30 | 50.87 | 7.03 | 1.00 | 222.00 | 6.440 | 20.801 | 9.62e-10 | 1.00e-11 | 60 |
| 300–359 | 2522.4 | 4636.5 | 8.42 | 49.38 | 6.83 | 2.05 | 455.10 | 7.161 | 24.581 | 8.83e-10 | 1.13e-11 | 60 |
| 360–419 | 2509.5 | 4337.2 | 8.70 | 52.22 | 7.50 | 2.75 | 610.50 | 16.798 | 18.593 | 1.89e-09 | 1.72e-11 | 60 |
| 420–479 | 3702.2 | 13482.4 | 9.02 | 59.03 | 8.42 | 5.90 | 1309.80 | 32.259 | 35.105 | 1.72e-09 | 5.07e-11 | 60 |
| 480–539 | 19797.2 | 217028.1 | 8.62 | 50.70 | 7.73 | 294.88 | 63694.90 | 72.362 | 54.886 | 5.54e-09 | 2.20e-10 | 60 |
| 540–599 | 7239.4 | 202794.3 | 9.28 | 55.35 | 8.15 | 939.75 | 202980.10 | 80.860 | 68.736 | 5.75e-09 | 3.21e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 36.377 | 32.176 | 69.740 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
