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
| 472 | 2.360 s | 14.329 cm | 18.559 cm | 24.314 cm | 45.469 cm | 46.156° | 8.000 rad/s | 5681.6 µs |

Nominal hard residual maxima: dynamics `6.764e-09`, contact acceleration `1.616e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 56.716 cm |
| authored reference vs measured CoM RMS / p95 | 50.927 / 135.649 cm |
| stance foot RMS | 38.085 cm |
| swing foot RMS | 53.613 cm |
| hand RMS | 81.787 cm |
| maximum root rotation | 179.602° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.824e-09 |
| contact acceleration residual | 2.535e-10 |
| raw max dynamics residual, including rejected ticks | 6.824e-09 |
| raw max contact residual, including rejected ticks | 2.535e-10 |
| active normal force range | 0.000–742.175 N |
| centroidal momentum-rate residual RMS / max | 50.666 / 238.174 N·m |
| point-task acceleration RMS max | 99.970 m/s² |
| frame-angular acceleration RMS max | 230.753 rad/s² |
| longest pre-contact / touchdown transition | 244 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `33.520` / `63.188 cm`.
- Virtual ZMP clipped on `59.00%` of ticks; clip-distance RMS / max `60.204` / `130.354 cm`.
- Measured-height natural frequency min / p50 / max: `3.683` / `3.743` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.660 m`; height-floor ticks: `89`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-85.855` / `-82.451 cm`; inside on `48.33%` of ticks.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `448` ticks; maximum active coordinates `8`; mean target/applied scale `0.665` / `0.739`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2547.1 µs | 13010.4 µs | 195071.2 µs | 219564.1 µs | 129 | 99 | 244 | 0 | 114 | 14 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7592.4 | 28002.4 | 354.7 | 4537.1 | 214238.1 | 219031.5 | 87870.0 | 600 | 52 | 15 | 131.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2448.4 | 2565.6 | 2577.1 | 2622.7 |
| solved_with_slack | 99 | 2528.9 | 2821.7 | 3010.5 | 3111.2 |
| normal_contact_contingency | 114 | 3397.9 | 14599.8 | 15035.3 | 187437.1 |
| contact_release_contingency | 14 | 191528.2 | 213784.6 | 218408.2 | 219564.1 |
| precontact_transition | 244 | 2676.1 | 4197.4 | 11453.8 | 16246.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.71 | 12.0 | 13.0 | 15 | 5.51 | 13.0 | 15 | 0.0784 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 3.31/8.0/8.0/8 | 728.98/1776.0/1776.0/1776 | 1.31/15.0/15 | 0.98/14.0/14 | 8.22/123.0/125 | 0.2385 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| normal_contact_contingency | 114 | 9.23/14.0/15 | 8.39/12.9/14 |
| contact_release_contingency | 14 | 8.43/11.7/12 | 7.07/9.9/10 |
| precontact_transition | 244 | 8.76/13.0/15 | 7.69/13.0/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.94/7.0/8 | 8.17/29.0/40 | 1.40/7.0/8 | 308 |
| viability | 1.75/7.0/8 | 10.91/42.0/56 | 1.37/7.0/8 | 373 |
| intent | 1.63/5.0/6 | 8.65/30.0/36 | 1.40/5.0/6 | 469 |
| preference | 1.32/6.0/8 | 11.45/54.0/72 | 0.94/5.0/7 | 388 |
| style | 1.07/3.0/4 | 9.24/19.0/32 | 0.41/3.0/4 | 208 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 472 | 128 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `131` ticks.
Precontact sole-center tangential speed: p50 `1.9615 m/s`, p95 `4.8086 m/s`, max `6.3815 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.556 | 4.555 | 4.555 | 1.000 | 1.000 | 48.473 | 48.777 | 0.305 | 48.777 | 0.001 | 0 | 61 | 0 | 0 | 62 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2432.5 | 2554.3 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2513.5 | 2708.4 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2514.4 | 3050.6 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2278.5 | 3030.2 | 7.13 | 46.18 | 4.57 | 1.00 | 225.80 | 6.405 | 0.736 | 7.54e-10 | 4.18e-11 | 12 |
| 240–299 | 2467.8 | 3143.1 | 8.43 | 51.77 | 7.13 | 1.00 | 222.00 | 6.401 | 20.905 | 9.71e-10 | 1.76e-11 | 60 |
| 300–359 | 3342.1 | 4818.4 | 8.63 | 52.97 | 7.60 | 5.67 | 1258.00 | 7.462 | 26.046 | 7.22e-10 | 2.29e-11 | 60 |
| 360–419 | 2453.8 | 4305.8 | 8.82 | 54.03 | 7.78 | 2.87 | 636.40 | 19.217 | 11.832 | 1.21e-09 | 1.44e-11 | 60 |
| 420–479 | 3295.7 | 86434.7 | 9.25 | 54.92 | 8.40 | 5.20 | 1152.90 | 37.855 | 49.814 | 6.76e-09 | 1.62e-10 | 60 |
| 480–539 | 10956.0 | 214318.1 | 9.43 | 56.73 | 8.62 | 7.53 | 1616.70 | 89.806 | 61.756 | 6.82e-09 | 2.54e-10 | 60 |
| 540–599 | 2938.4 | 4567.4 | 8.90 | 57.05 | 7.97 | 6.83 | 1476.00 | 148.864 | 108.000 | 2.08e-09 | 5.60e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 56.716 | 43.835 | 81.787 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
