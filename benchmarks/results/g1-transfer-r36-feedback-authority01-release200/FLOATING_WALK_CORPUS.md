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
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `200` ticks.
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
| 467 | 2.335 s | 12.582 cm | 5.290 cm | 21.828 cm | 40.463 cm | 47.587° | 8.000 rad/s | 12455.1 µs |

Nominal hard residual maxima: dynamics `7.450e-09`, contact acceleration `3.898e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 60.684 cm |
| authored reference vs measured CoM RMS / p95 | 53.698 / 150.128 cm |
| stance foot RMS | 28.436 cm |
| swing foot RMS | 53.166 cm |
| hand RMS | 85.913 cm |
| maximum root rotation | 179.865° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.479e-09 |
| contact acceleration residual | 3.898e-10 |
| raw max dynamics residual, including rejected ticks | 8.479e-09 |
| raw max contact residual, including rejected ticks | 3.898e-10 |
| active normal force range | 0.000–752.401 N |
| centroidal momentum-rate residual RMS / max | 60.134 / 253.265 N·m |
| point-task acceleration RMS max | 146.480 m/s² |
| frame-angular acceleration RMS max | 288.157 rad/s² |
| longest pre-contact / touchdown transition | 239 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `27.465` / `64.132 cm`.
- Virtual ZMP clipped on `57.83%` of ticks; clip-distance RMS / max `59.594` / `154.178 cm`.
- Measured-height natural frequency min / p50 / max: `3.670` / `3.730` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.781 m`; height-floor ticks: `102`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-110.224` / `-91.217 cm`; inside on `47.50%` of ticks.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `503` ticks; maximum active coordinates `12`; mean target/applied scale `0.661` / `0.791`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2640.5 µs | 82000.4 µs | 106898.1 µs | 178882.6 µs | 129 | 99 | 239 | 0 | 111 | 22 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9704.4 | 25700.0 | 395.7 | 7658.6 | 178494.9 | 178843.8 | 94619.3 | 600 | 88 | 38 | 103.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2447.2 | 2579.4 | 2607.7 | 2645.9 |
| solved_with_slack | 99 | 2539.1 | 2826.8 | 2950.8 | 3120.6 |
| normal_contact_contingency | 111 | 4646.5 | 88734.4 | 103077.3 | 122583.4 |
| contact_release_contingency | 22 | 104133.4 | 177645.2 | 178746.7 | 178882.6 |
| precontact_transition | 239 | 3048.9 | 8766.2 | 13151.3 | 77131.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.64 | 11.0 | 13.0 | 14 | 5.42 | 12.0 | 13 | 0.1630 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 138.44/8.0/5473.9/6779 | 30006.30/1776.0/1182364.6/1464264 | 1.39/13.0/14 | 1.02/12.0/13 | 8.30/103.1/117 | 0.4683 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| normal_contact_contingency | 111 | 8.86/14.0/14 | 8.05/12.0/13 |
| contact_release_contingency | 22 | 7.86/10.8/11 | 6.45/9.6/10 |
| precontact_transition | 239 | 8.83/13.0/14 | 7.71/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.93/6.0/8 | 8.22/25.0/40 | 1.39/6.0/7 | 295 |
| viability | 1.80/6.0/7 | 11.52/42.0/51 | 1.41/6.0/7 | 369 |
| intent | 1.57/4.0/7 | 8.32/24.0/35 | 1.33/4.0/7 | 467 |
| preference | 1.27/5.0/8 | 10.38/46.0/67 | 0.89/5.0/7 | 379 |
| style | 1.06/3.0/4 | 9.24/20.0/31 | 0.40/2.0/4 | 215 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 467 | 133 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `136` ticks.
Precontact sole-center tangential speed: p50 `1.7331 m/s`, p95 `7.6345 m/s`, max `8.6723 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.823 | 5.821 | 5.821 | 1.000 | 1.000 | 47.637 | 47.953 | 0.316 | 48.910 | 0.001 | 0 | 61 | 0 | 0 | 92 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2422.5 | 2535.1 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2522.1 | 2732.8 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2518.7 | 3018.3 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2216.5 | 3091.4 | 7.08 | 45.43 | 4.52 | 1.00 | 225.80 | 6.401 | 0.807 | 7.54e-10 | 4.18e-11 | 12 |
| 240–299 | 2406.8 | 3255.8 | 8.28 | 51.95 | 6.85 | 1.00 | 222.00 | 6.280 | 21.667 | 1.35e-09 | 2.30e-11 | 60 |
| 300–359 | 3238.7 | 4554.2 | 8.95 | 54.80 | 7.65 | 4.73 | 1050.80 | 6.039 | 22.055 | 8.87e-10 | 1.12e-11 | 60 |
| 360–419 | 3250.5 | 5016.1 | 8.72 | 53.72 | 7.73 | 4.97 | 1102.60 | 17.047 | 9.349 | 8.91e-10 | 1.44e-11 | 60 |
| 420–479 | 3945.2 | 96630.4 | 9.43 | 56.30 | 8.75 | 555.35 | 120887.90 | 36.384 | 17.358 | 7.45e-09 | 3.90e-10 | 60 |
| 480–539 | 13437.9 | 178500.7 | 9.03 | 52.00 | 8.12 | 806.30 | 174146.30 | 96.331 | 63.536 | 2.09e-09 | 1.09e-10 | 60 |
| 540–599 | 3981.5 | 106717.0 | 8.42 | 52.08 | 7.53 | 8.00 | 1725.60 | 160.663 | 96.851 | 8.48e-09 | 2.40e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 60.684 | 38.422 | 85.913 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
