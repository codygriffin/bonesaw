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
- Joint-velocity envelope: `viability` priority with weight `0.150`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
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
| 491 | 2.455 s | 14.516 cm | 10.633 cm | 25.401 cm | 40.679 cm | 41.421° | 8.000 rad/s | 61730.3 µs |

Nominal hard residual maxima: dynamics `4.398e-09`, contact acceleration `1.909e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 41.686 cm |
| authored reference vs measured CoM RMS / p95 | 37.227 / 96.782 cm |
| stance foot RMS | 24.677 cm |
| swing foot RMS | 42.325 cm |
| hand RMS | 68.672 cm |
| maximum root rotation | 179.854° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.813e-09 |
| contact acceleration residual | 4.290e-10 |
| raw max dynamics residual, including rejected ticks | 9.813e-09 |
| raw max contact residual, including rejected ticks | 4.290e-10 |
| active normal force range | 0.000–890.429 N |
| centroidal momentum-rate residual RMS / max | 50.490 / 289.745 N·m |
| point-task acceleration RMS max | 181.808 m/s² |
| frame-angular acceleration RMS max | 278.253 rad/s² |
| longest pre-contact / touchdown transition | 263 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `27.248` / `57.591 cm`.
- Virtual ZMP clipped on `53.17%` of ticks; clip-distance RMS / max `45.572` / `123.572 cm`.
- Measured-height natural frequency min / p50 / max: `3.693` / `3.744` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.343 m`; height-floor ticks: `64`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-81.279` / `-70.311 cm`; inside on `48.50%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8456 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 28`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.2314 m / 1.1698 / 0.0069 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 15`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `448` ticks; maximum active coordinates `8`; mean target/applied scale `0.671` / `0.739`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2654.2 µs | 110405.1 µs | 198904.6 µs | 219188.3 µs | 129 | 99 | 263 | 0 | 74 | 35 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 15353.9 | 40739.0 | 491.8 | 16465.4 | 216172.4 | 218886.7 | 123512.2 | 600 | 107 | 56 | 65.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2541.5 | 3986.6 | 4074.4 | 4506.0 |
| solved_with_slack | 99 | 2592.9 | 2863.0 | 3007.1 | 3083.1 |
| normal_contact_contingency | 74 | 10406.4 | 103482.3 | 133937.1 | 198689.2 |
| contact_release_contingency | 35 | 177006.7 | 213314.7 | 217476.4 | 219188.3 |
| precontact_transition | 263 | 2763.3 | 14885.9 | 77709.1 | 88244.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.58 | 11.0 | 13.0 | 15 | 5.23 | 13.0 | 13 | 0.1067 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 135.05/8.0/5160.7/6836 | 29555.22/1776.0/1145671.0/1476576 | 1.51/16.0/27 | 1.25/15.0/26 | 10.36/129.0/223 | 0.2621 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.75/11.0/13 | 3.78/8.0/10 |
| normal_contact_contingency | 74 | 9.49/14.0/14 | 8.73/13.0/13 |
| contact_release_contingency | 35 | 7.66/11.0/11 | 6.00/9.0/9 |
| precontact_transition | 263 | 8.61/14.0/15 | 7.26/13.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.86/6.0/7 | 7.86/25.0/35 | 1.30/6.0/7 | 290 |
| viability | 1.74/6.0/8 | 10.84/42.0/56 | 1.34/6.0/7 | 369 |
| intent | 1.60/5.0/10 | 8.46/25.0/52 | 1.34/5.0/10 | 456 |
| preference | 1.31/6.0/8 | 10.72/52.0/76 | 0.85/6.0/8 | 336 |
| style | 1.07/3.0/5 | 8.76/21.0/36 | 0.39/3.0/4 | 203 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 491 | 109 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `112` ticks.
Precontact sole-center tangential speed: p50 `1.8781 m/s`, p95 `7.8752 m/s`, max `12.2902 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9.213 | 9.195 | 9.195 | 0.998 | 0.998 | 48.035 | 48.395 | 0.359 | 49.133 | 0.001 | 0 | 91 | 0 | 0 | 78 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2587.0 | 4259.6 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2579.1 | 2773.7 | 5.43 | 36.93 | 0.92 | 1.00 | 234.00 | 0.118 | 0.000 | 9.56e-10 | 5.55e-11 | 0 |
| 120–179 | 2576.8 | 2929.9 | 5.90 | 39.13 | 1.93 | 1.00 | 234.00 | 1.293 | 0.000 | 1.37e-09 | 4.75e-11 | 0 |
| 180–239 | 2263.0 | 3194.6 | 7.27 | 45.85 | 4.80 | 1.00 | 225.80 | 6.452 | 1.009 | 9.83e-10 | 2.39e-11 | 12 |
| 240–299 | 2426.4 | 3351.7 | 8.20 | 50.63 | 6.75 | 1.00 | 222.00 | 6.224 | 21.244 | 1.44e-09 | 1.93e-11 | 60 |
| 300–359 | 2599.1 | 5357.3 | 8.57 | 49.00 | 6.80 | 2.05 | 455.10 | 6.759 | 24.540 | 1.30e-09 | 1.30e-11 | 60 |
| 360–419 | 3366.3 | 4479.8 | 8.60 | 48.52 | 7.32 | 4.27 | 947.20 | 16.559 | 19.269 | 1.13e-09 | 1.11e-11 | 60 |
| 420–479 | 3359.4 | 86160.7 | 8.90 | 52.82 | 7.78 | 625.03 | 138757.40 | 30.901 | 24.633 | 3.12e-09 | 1.66e-10 | 60 |
| 480–539 | 99166.0 | 216217.7 | 9.12 | 54.38 | 8.08 | 631.62 | 136427.10 | 66.415 | 45.086 | 5.70e-09 | 2.46e-10 | 60 |
| 540–599 | 6903.5 | 199882.2 | 8.82 | 55.60 | 7.93 | 82.52 | 17815.60 | 107.749 | 77.047 | 9.81e-09 | 4.29e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 41.686 | 31.625 | 68.672 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
