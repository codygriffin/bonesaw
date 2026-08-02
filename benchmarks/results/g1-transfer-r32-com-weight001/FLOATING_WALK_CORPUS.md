# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.010` and `2.000 Hz` response. Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 459 | 2.295 s | 11.671 cm | 7.549 cm | 28.001 cm | 26.667 cm | 17.278° | 8.000 rad/s | 16016.4 µs |

Nominal hard residual maxima: dynamics `1.657e-09`, contact acceleration `5.311e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 47.512 cm |
| CoM RMS / p95 | 42.073 / 107.273 cm |
| stance foot RMS | 39.094 cm |
| swing foot RMS | 61.627 cm |
| hand RMS | 58.956 cm |
| maximum root rotation | 17.278° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.657e-09 |
| contact acceleration residual | 5.311e-11 |
| raw max dynamics residual, including rejected ticks | 1.657e-09 |
| raw max contact residual, including rejected ticks | 5.311e-11 |
| active normal force range | 0.000–401.868 N |
| centroidal momentum-rate residual RMS / max | 20.722 / 137.838 N·m |
| point-task acceleration RMS max | 82.284 m/s² |
| frame-angular acceleration RMS max | 114.529 rad/s² |
| longest pre-contact / touchdown transition | 231 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2707.0 µs | 5327.6 µs | 16866.6 µs | 117081.0 µs | 65 | 163 | 231 | 0 | 141 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3296.8 | 5361.1 | 617.8 | 4347.9 | 68939.8 | 112266.9 | 17064.7 | 600 | 37 | 4 | 303.3 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 65 | 2468.6 | 2569.6 | 2823.1 | 2982.3 |
| solved_with_slack | 163 | 3083.6 | 5574.5 | 6440.0 | 7167.4 |
| normal_contact_contingency | 141 | 1989.8 | 3347.3 | 35552.0 | 117081.0 |
| precontact_transition | 231 | 2937.1 | 4058.6 | 17120.8 | 22183.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.84 | 11.0 | 13.0 | 24 | 5.70 | 12.0 | 20 | 0.1339 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 15.22/8.0/8.0/2325 | 3340.85/1872.0/1872.0/502200 | 1.09/3.1/16 | 0.67/2.1/15 | 5.58/19.9/136 | 0.3977 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 65 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 163 | 8.13/15.0/24 | 6.13/14.0/20 |
| normal_contact_contingency | 141 | 8.35/12.6/13 | 6.32/11.0/12 |
| precontact_transition | 231 | 8.12/12.0/14 | 6.61/11.7/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.61/4.0/6 | 6.30/16.0/24 | 0.91/4.0/6 | 297 |
| viability | 2.46/7.0/13 | 14.85/42.0/54 | 2.27/7.0/13 | 485 |
| intent | 1.50/4.0/6 | 7.83/24.0/30 | 1.33/4.0/6 | 503 |
| preference | 1.21/5.0/10 | 11.65/48.0/92 | 0.79/5.0/10 | 359 |
| style | 1.05/2.0/3 | 9.73/26.0/44 | 0.39/2.0/3 | 218 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 459 | 141 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `144` ticks.
Precontact sole-center tangential speed: p50 `1.7140 m/s`, p95 `3.3278 m/s`, max `3.5564 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.978 | 1.978 | 1.978 | 1.000 | 1.000 | 47.777 | 48.031 | 0.254 | 48.086 | 0.001 | 0 | 44 | 0 | 0 | 27 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2718.8 | 6901.9 | 7.63 | 44.57 | 5.08 | 2.98 | 698.10 | 0.335 | 0.000 | 1.66e-09 | 4.72e-11 | 0 |
| 60–119 | 2565.4 | 5988.8 | 6.38 | 42.15 | 3.00 | 2.28 | 534.30 | 0.335 | 0.000 | 1.32e-09 | 5.31e-11 | 0 |
| 120–179 | 2659.6 | 5894.4 | 6.93 | 46.72 | 3.32 | 1.82 | 425.10 | 0.159 | 0.000 | 1.39e-09 | 3.52e-11 | 0 |
| 180–239 | 3109.2 | 4819.6 | 8.10 | 50.35 | 6.37 | 8.00 | 1806.40 | 2.934 | 3.801 | 1.32e-10 | 6.45e-12 | 12 |
| 240–299 | 1916.2 | 20294.9 | 7.93 | 49.63 | 6.48 | 40.95 | 9090.90 | 7.815 | 18.032 | 1.04e-09 | 1.22e-11 | 60 |
| 300–359 | 2983.0 | 3902.4 | 8.32 | 53.07 | 6.72 | 5.08 | 1128.50 | 11.191 | 26.181 | 7.97e-10 | 2.60e-11 | 60 |
| 360–419 | 2058.4 | 3840.5 | 7.88 | 50.00 | 6.35 | 4.15 | 921.30 | 18.038 | 24.167 | 4.60e-10 | 7.79e-12 | 60 |
| 420–479 | 3559.2 | 69663.1 | 8.62 | 53.55 | 7.07 | 81.43 | 17615.90 | 32.838 | 27.535 | 5.41e-10 | 2.80e-11 | 60 |
| 480–539 | 2075.9 | 3554.3 | 8.40 | 59.87 | 6.60 | 2.98 | 644.40 | 69.795 | 66.988 | 6.67e-10 | 1.47e-11 | 60 |
| 540–599 | 1845.4 | 3083.5 | 8.18 | 53.87 | 5.98 | 2.52 | 543.60 | 126.899 | 126.243 | 1.06e-09 | 1.46e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 47.512 | 47.741 | 58.956 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
