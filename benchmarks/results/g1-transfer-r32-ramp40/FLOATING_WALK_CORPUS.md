# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
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
| `no_infeasible_or_failed_ticks` | FAIL |
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

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 452 | 2.260 s | 11.049 cm | 19.283 cm | 34.009 cm | 33.798 cm | 47.884° | 8.000 rad/s | 89512.3 µs |

Nominal hard residual maxima: dynamics `1.622e-09`, contact acceleration `5.634e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 82.344 cm |
| CoM RMS / p95 | 80.942 / 221.136 cm |
| stance foot RMS | 61.664 cm |
| swing foot RMS | 82.678 cm |
| hand RMS | 95.430 cm |
| maximum root rotation | 169.035° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.933e-09 |
| contact acceleration residual | 5.826e-11 |
| raw max dynamics residual, including rejected ticks | 6.237e+02 |
| raw max contact residual, including rejected ticks | 5.826e-11 |
| active normal force range | 0.000–427.279 N |
| centroidal momentum-rate residual RMS / max | 41.318 / 319.006 N·m |
| point-task acceleration RMS max | 177.654 m/s² |
| frame-angular acceleration RMS max | 340.813 rad/s² |
| longest pre-contact / touchdown transition | 279 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 182 / 182 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2690.2 µs | 190797.5 µs | 315490.1 µs | 318732.0 µs | 115 | 58 | 279 | 0 | 83 | 45 | 20 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 29656.5 | 69654.0 | 692.3 | 103619.0 | 318033.9 | 318662.2 | 96632.6 | 600 | 137 | 117 | 33.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 115 | 2451.3 | 2545.1 | 2592.7 | 2622.1 |
| solved_with_slack | 58 | 2788.6 | 4847.0 | 5377.9 | 5956.8 |
| primal_infeasible | 20 | 314931.0 | 317624.8 | 318510.5 | 318732.0 |
| normal_contact_contingency | 83 | 3251.1 | 30498.8 | 60613.2 | 110552.5 |
| contact_release_contingency | 45 | 168826.0 | 209424.2 | 213754.1 | 213867.0 |
| precontact_transition | 279 | 2980.8 | 71801.0 | 92725.6 | 103529.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.57 | 11.0 | 13.0 | 16 | 5.50 | 12.0 | 15 | -0.3934 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 502.13/4948.4/6720.0/6720 | 108400.81/1098555.9/1411200.0/1473192 | 2.32/40.0/40 | 1.96/39.0/39 | 16.59/336.0/336 | 0.6758 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 115 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 58 | 8.64/14.3/16 | 6.03/11.9/13 |
| primal_infeasible | 20 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 83 | 8.98/12.7/16 | 7.58/12.5/15 |
| contact_release_contingency | 45 | 7.73/10.0/10 | 6.42/9.0/9 |
| precontact_transition | 279 | 8.50/13.0/14 | 7.29/12.2/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.00/6.0/7 | 8.49/25.0/34 | 1.55/6.0/7 | 349 |
| viability | 1.96/7.0/12 | 11.66/37.0/56 | 1.75/7.0/12 | 458 |
| intent | 1.38/5.0/8 | 6.91/26.0/44 | 1.14/5.0/8 | 436 |
| preference | 1.17/5.0/8 | 11.78/50.1/81 | 0.74/5.0/8 | 332 |
| style | 1.05/3.0/6 | 9.01/27.0/39 | 0.32/3.0/5 | 156 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 70 | 357 | 0 | 173 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 452 | 148 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `357` ticks, planned normal touchdown `0` ticks, normal fallback `151` ticks.
Precontact sole-center tangential speed: p50 `1.5645 m/s`, p95 `6.0053 m/s`, max `7.6720 m/s` over 357 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 17.794 | 17.789 | 17.789 | 1.000 | 1.000 | 47.738 | 47.910 | 0.172 | 48.039 | 0.001 | 0 | 43 | 0 | 0 | 302 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2466.3 | 2860.4 | 6.07 | 36.82 | 1.67 | 1.00 | 234.00 | 0.172 | 0.000 | 1.62e-09 | 5.63e-11 | 0 |
| 60–119 | 2439.1 | 3511.2 | 5.10 | 34.98 | 0.17 | 1.00 | 234.00 | 0.032 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–179 | 2610.8 | 5357.6 | 7.67 | 51.07 | 4.70 | 1.88 | 439.30 | 0.391 | 0.073 | 1.10e-09 | 5.19e-11 | 7 |
| 180–239 | 1898.9 | 3329.3 | 8.62 | 55.97 | 6.83 | 2.52 | 558.70 | 4.147 | 16.139 | 1.11e-09 | 1.33e-11 | 60 |
| 240–299 | 2066.7 | 13725.5 | 8.32 | 54.93 | 7.15 | 30.32 | 6730.30 | 8.435 | 32.301 | 8.45e-10 | 1.20e-11 | 60 |
| 300–359 | 2798.0 | 3972.2 | 8.37 | 53.20 | 7.37 | 4.50 | 999.00 | 11.223 | 35.713 | 8.58e-10 | 2.14e-11 | 60 |
| 360–419 | 3143.5 | 55311.7 | 8.50 | 54.72 | 7.62 | 229.45 | 50937.90 | 15.669 | 24.986 | 5.23e-10 | 5.41e-12 | 60 |
| 420–479 | 28358.1 | 106409.2 | 9.38 | 59.93 | 8.12 | 2208.18 | 488170.30 | 44.578 | 64.921 | 6.19e-10 | 8.14e-12 | 60 |
| 480–539 | 103300.9 | 211182.8 | 8.05 | 44.63 | 6.65 | 297.15 | 64160.60 | 132.303 | 88.538 | 7.93e-09 | 5.83e-11 | 60 |
| 540–599 | 91632.4 | 318044.4 | 5.60 | 32.40 | 4.77 | 2245.33 | 471544.00 | 218.755 | 182.641 | 6.24e+02 | 2.99e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 82.344 | 69.725 | 95.430 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
