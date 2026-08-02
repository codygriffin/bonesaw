# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `viability` priority with weight `0.001`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; contact forces remain off until the authored contact edge.
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
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 434 | 2.170 s | 9.578 cm | 3.304 cm | 37.173 cm | 20.519 cm | 2.726° | 8.000 rad/s | 5026.5 µs |

Nominal hard residual maxima: dynamics `1.758e-09`, contact acceleration `5.441e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 13.658 cm |
| stance foot RMS | 14.103 cm |
| swing foot RMS | 36.494 cm |
| hand RMS | 27.917 cm |
| maximum root rotation | 3.531° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.758e-09 |
| contact acceleration residual | 5.441e-11 |
| raw max dynamics residual, including rejected ticks | 1.758e-09 |
| raw max contact residual, including rejected ticks | 5.441e-11 |
| active normal force range | 0.000–570.613 N |
| centroidal momentum-rate residual RMS / max | 23.048 / 111.273 N·m |
| point-task acceleration RMS max | 100.107 m/s² |
| frame-angular acceleration RMS max | 141.153 rad/s² |
| longest pre-contact / touchdown transition | 200 / 6 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2770.1 µs | 4549.1 µs | 103622.2 µs | 313947.8 µs | 150 | 78 | 200 | 6 | 161 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5643.0 | 22660.7 | 407.9 | 4038.5 | 256411.3 | 308194.1 | 16118.2 | 600 | 17 | 11 | 177.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 150 | 2376.7 | 2451.3 | 2482.3 | 2491.4 |
| solved_with_slack | 78 | 2871.1 | 4412.1 | 5048.1 | 5093.0 |
| normal_contact_contingency | 161 | 3083.5 | 4847.2 | 78322.8 | 103584.6 |
| contact_release_contingency | 5 | 208982.0 | 294736.9 | 310105.6 | 313947.8 |
| touchdown_transition | 6 | 4519.9 | 104697.1 | 106816.0 | 107345.7 |
| precontact_transition | 200 | 2873.7 | 4509.7 | 4936.9 | 5261.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.04 | 9.0 | 11.0 | 13 | 3.25 | 9.0 | 12 | 0.0374 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 57.55/1.3/42.5/6860 | 12439.81/308.7/9190.8/1481760 | 0.09/2.0/2 | 0.05/1.0/1 | 0.41/8.0/11 | 0.3689 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 150 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 78 | 7.31/12.2/13 | 4.41/9.2/10 |
| normal_contact_contingency | 161 | 6.36/10.0/10 | 3.90/7.4/9 |
| contact_release_contingency | 5 | 6.40/8.9/9 | 4.00/6.9/7 |
| touchdown_transition | 6 | 7.17/8.9/9 | 4.67/6.9/7 |
| precontact_transition | 200 | 6.77/11.0/12 | 4.63/10.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.28/4.0/6 | 4.84/16.0/24 | 0.42/4.0/6 | 102 |
| viability | 2.73/8.0/10 | 22.25/63.0/70 | 2.46/8.0/10 | 445 |
| intent | 1.00/1.0/1 | 4.00/5.0/5 | 0.00/0.0/1 | 1 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.03/2.0/3 | 8.78/23.0/35 | 0.36/1.0/3 | 211 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 6 | 199 | 166 |
| right_ankle_roll_link | 168 | 0 | 0 | 432 | 0 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `6` ticks, normal fallback `169` ticks.
Precontact sole-center tangential speed: p50 `0.8785 m/s`, p95 `2.4373 m/s`, max `3.6125 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `4.1039 m/s`, p95 `4.6293 m/s`, max `4.6973 m/s` over 5 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.386 | 3.384 | 3.384 | 1.000 | 1.000 | 47.727 | 47.977 | 0.250 | 48.012 | 0.001 | 0 | 41 | 0 | 0 | 127 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2398.5 | 3928.3 | 4.77 | 28.87 | 0.88 | 1.00 | 234.00 | 0.764 | 0.000 | 1.47e-09 | 4.71e-11 | 0 |
| 60–119 | 2349.3 | 2441.6 | 4.00 | 24.05 | 0.00 | 1.00 | 234.00 | 1.313 | 0.000 | 1.76e-09 | 5.44e-11 | 0 |
| 120–179 | 2408.5 | 5058.6 | 5.20 | 32.17 | 1.65 | 1.00 | 234.00 | 1.387 | 0.000 | 1.36e-09 | 4.60e-11 | 0 |
| 180–239 | 2842.6 | 4148.9 | 6.87 | 45.43 | 4.03 | 1.00 | 225.80 | 5.032 | 3.276 | 1.19e-09 | 4.58e-11 | 12 |
| 240–299 | 2747.2 | 4808.7 | 6.50 | 41.12 | 4.27 | 1.00 | 222.00 | 7.466 | 21.387 | 1.50e-09 | 2.46e-11 | 60 |
| 300–359 | 3086.1 | 5113.1 | 7.22 | 48.95 | 5.32 | 1.00 | 222.00 | 10.346 | 31.685 | 1.26e-09 | 1.66e-11 | 60 |
| 360–419 | 2882.5 | 4462.7 | 6.72 | 43.87 | 4.45 | 1.00 | 222.00 | 18.544 | 33.151 | 1.52e-09 | 1.63e-11 | 60 |
| 420–479 | 3570.9 | 257275.8 | 6.50 | 40.97 | 4.55 | 566.53 | 122372.30 | 23.544 | 32.756 | 7.84e-10 | 2.06e-11 | 54 |
| 480–539 | 3237.3 | 4491.4 | 6.45 | 48.00 | 3.98 | 1.00 | 216.00 | 16.760 | 32.046 | 7.61e-10 | 4.78e-12 | 60 |
| 540–599 | 2974.4 | 4709.6 | 6.17 | 45.27 | 3.32 | 1.00 | 216.00 | 22.224 | 32.650 | 1.26e-09 | 1.34e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 13.658 | 23.952 | 27.917 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
