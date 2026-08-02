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
- Capture landing retarget: `enabled`; authored offset ≤ `0.040 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.250 m/s`.
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
| 467 | 2.335 s | 15.055 cm | 10.969 cm | 23.883 cm | 42.481 cm | 47.400° | 8.000 rad/s | 8547.8 µs |

Nominal hard residual maxima: dynamics `1.546e-09`, contact acceleration `5.965e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 55.623 cm |
| authored reference vs measured CoM RMS / p95 | 48.845 / 128.845 cm |
| stance foot RMS | 32.938 cm |
| swing foot RMS | 59.539 cm |
| hand RMS | 76.924 cm |
| maximum root rotation | 179.858° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.712e-09 |
| contact acceleration residual | 3.012e-10 |
| raw max dynamics residual, including rejected ticks | 7.712e-09 |
| raw max contact residual, including rejected ticks | 3.012e-10 |
| active normal force range | 0.000–907.834 N |
| centroidal momentum-rate residual RMS / max | 61.757 / 242.773 N·m |
| point-task acceleration RMS max | 155.730 m/s² |
| frame-angular acceleration RMS max | 233.902 rad/s² |
| longest pre-contact / touchdown transition | 239 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `34.503` / `60.499 cm`.
- Virtual ZMP clipped on `57.83%` of ticks; clip-distance RMS / max `59.831` / `161.592 cm`.
- Measured-height natural frequency min / p50 / max: `3.659` / `3.730` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.547 m`; height-floor ticks: `90`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-105.424` / `-81.654 cm`; inside on `48.33%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`.
- Maximum applied offset / root reach: `0.0400 / 0.8546 m`.
- Authored-offset / reach / slew limited ticks: `372 / 0 / 39`.
- Authored geometry outside configured reach: `0` target-ticks.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `455` ticks; maximum active coordinates `8`; mean target/applied scale `0.665` / `0.750`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2748.2 µs | 102994.2 µs | 179524.9 µs | 305170.1 µs | 129 | 99 | 239 | 0 | 107 | 26 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11853.7 | 34945.0 | 521.8 | 10949.5 | 258229.1 | 300476.0 | 103883.0 | 600 | 109 | 35 | 84.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2585.9 | 4140.3 | 4289.7 | 4552.1 |
| solved_with_slack | 99 | 2672.5 | 4143.8 | 4221.4 | 4234.2 |
| normal_contact_contingency | 107 | 7194.3 | 91853.9 | 118380.2 | 196508.7 |
| contact_release_contingency | 26 | 161413.4 | 223563.8 | 285578.7 | 305170.1 |
| precontact_transition | 239 | 2601.0 | 4760.3 | 11740.8 | 20204.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.59 | 11.0 | 14.0 | 15 | 5.36 | 13.0 | 14 | 0.0701 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 74.18/8.0/5613.3/6688 | 16033.82/1776.0/1212483.6/1444608 | 1.72/15.0/18 | 1.45/14.0/17 | 12.06/119.0/149 | 0.2794 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| normal_contact_contingency | 107 | 9.45/15.0/15 | 8.78/13.0/14 |
| contact_release_contingency | 26 | 7.73/10.0/10 | 5.85/8.0/8 |
| precontact_transition | 239 | 8.47/13.0/14 | 7.32/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.83/5.0/6 | 7.88/25.0/30 | 1.29/5.0/6 | 281 |
| viability | 1.73/6.0/8 | 11.30/45.0/71 | 1.33/6.0/8 | 363 |
| intent | 1.61/5.0/10 | 8.19/25.0/39 | 1.35/5.0/9 | 457 |
| preference | 1.33/6.0/10 | 11.21/48.0/87 | 0.94/5.0/10 | 379 |
| style | 1.08/3.0/5 | 9.35/21.0/33 | 0.45/2.0/5 | 227 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 467 | 133 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `136` ticks.
Precontact sole-center tangential speed: p50 `2.2223 m/s`, p95 `9.6876 m/s`, max `12.4971 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.113 | 7.107 | 7.107 | 0.999 | 0.999 | 47.016 | 47.254 | 0.238 | 48.887 | 0.001 | 0 | 38 | 0 | 0 | 347 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2561.9 | 4135.6 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2660.8 | 4413.1 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2651.1 | 4173.5 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2367.9 | 4226.5 | 7.13 | 46.18 | 4.57 | 1.00 | 225.80 | 6.405 | 0.736 | 7.54e-10 | 4.18e-11 | 12 |
| 240–299 | 2575.2 | 4072.3 | 8.33 | 51.33 | 7.00 | 1.00 | 222.00 | 6.435 | 20.883 | 8.05e-10 | 2.26e-11 | 60 |
| 300–359 | 2448.2 | 4193.0 | 8.15 | 48.50 | 6.47 | 1.00 | 222.00 | 7.343 | 24.329 | 1.48e-09 | 2.34e-11 | 60 |
| 360–419 | 2489.0 | 3843.4 | 8.92 | 55.27 | 8.15 | 1.37 | 303.40 | 21.614 | 13.725 | 1.13e-09 | 1.70e-11 | 60 |
| 420–479 | 3865.8 | 150685.3 | 8.40 | 52.57 | 7.77 | 500.27 | 108093.10 | 41.746 | 32.388 | 1.55e-09 | 5.96e-11 | 60 |
| 480–539 | 13266.6 | 258934.4 | 9.85 | 61.62 | 8.93 | 226.20 | 48852.30 | 87.461 | 68.870 | 2.38e-09 | 1.56e-10 | 60 |
| 540–599 | 6824.8 | 185300.3 | 8.62 | 53.22 | 7.63 | 8.00 | 1717.60 | 144.713 | 109.446 | 7.71e-09 | 3.01e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 55.623 | 43.574 | 76.924 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
