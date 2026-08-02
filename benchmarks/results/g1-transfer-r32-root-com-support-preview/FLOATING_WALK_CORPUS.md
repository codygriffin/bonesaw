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
- Root horizontal reference: `support-preview`.
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
| 368 | 1.840 s | 9.082 cm | 7.930 cm | 46.751 cm | 22.394 cm | 59.026° | 8.000 rad/s | 87975.5 µs |

Nominal hard residual maxima: dynamics `4.086e-09`, contact acceleration `1.432e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 123.996 cm |
| stance foot RMS | 107.102 cm |
| swing foot RMS | 99.061 cm |
| hand RMS | 134.244 cm |
| maximum root rotation | 178.526° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.859e-09 |
| contact acceleration residual | 2.051e-10 |
| raw max dynamics residual, including rejected ticks | 9.859e-09 |
| raw max contact residual, including rejected ticks | 2.051e-10 |
| active normal force range | 0.000–670.050 N |
| centroidal momentum-rate residual RMS / max | 68.977 / 406.671 N·m |
| point-task acceleration RMS max | 111.234 m/s² |
| frame-angular acceleration RMS max | 271.815 rad/s² |
| longest pre-contact / touchdown transition | 140 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2579.7 µs | 73293.8 µs | 100737.3 µs | 211330.0 µs | 148 | 80 | 140 | 0 | 221 | 11 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8454.5 | 22672.9 | 665.1 | 6705.3 | 194576.1 | 209654.6 | 74020.7 | 600 | 94 | 38 | 118.3 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 148 | 2422.4 | 2531.6 | 2586.5 | 2636.9 |
| solved_with_slack | 80 | 2611.1 | 3905.7 | 5661.1 | 9708.7 |
| normal_contact_contingency | 221 | 2984.7 | 40917.1 | 96099.8 | 183360.2 |
| contact_release_contingency | 11 | 99860.7 | 156579.2 | 200379.8 | 211330.0 |
| precontact_transition | 140 | 3137.8 | 76369.8 | 101687.5 | 109679.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.71 | 11.0 | 13.0 | 15 | 5.50 | 12.0 | 14 | 0.1439 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 232.44/325.2/6179.5/6972 | 50875.67/70243.2/1356308.8/1547784 | 1.28/8.0/16 | 0.93/7.0/15 | 7.67/63.0/131 | 0.7229 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 148 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 80 | 8.71/14.2/15 | 6.42/13.0/13 |
| normal_contact_contingency | 221 | 8.58/12.8/14 | 7.46/12.0/14 |
| contact_release_contingency | 11 | 7.91/10.8/11 | 6.64/9.8/10 |
| precontact_transition | 140 | 8.59/12.0/12 | 7.61/11.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.20/6.0/10 | 9.24/30.0/48 | 1.78/6.0/10 | 387 |
| viability | 1.86/7.0/9 | 11.18/41.0/56 | 1.61/7.0/9 | 452 |
| intent | 1.42/4.0/6 | 7.19/23.0/29 | 1.17/4.0/6 | 450 |
| preference | 1.15/4.0/7 | 11.20/33.1/56 | 0.62/4.0/6 | 299 |
| style | 1.08/3.0/5 | 9.65/24.0/66 | 0.31/2.0/4 | 167 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 368 | 232 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `235` ticks.
Precontact sole-center tangential speed: p50 `2.7122 m/s`, p95 `7.7493 m/s`, max `10.6751 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.073 | 5.073 | 5.072 | 1.000 | 1.000 | 48.332 | 48.332 | 0.000 | 48.332 | 0.001 | 0 | 0 | 0 | 0 | 11 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2462.4 | 2799.0 | 6.07 | 36.48 | 1.48 | 1.00 | 234.00 | 0.651 | 0.000 | 1.22e-09 | 5.23e-11 | 0 |
| 60–119 | 2408.9 | 2534.8 | 5.00 | 33.43 | 0.00 | 1.00 | 234.00 | 0.051 | 0.000 | 9.40e-10 | 3.47e-11 | 0 |
| 120–179 | 2497.9 | 5626.5 | 6.28 | 40.13 | 2.25 | 1.12 | 261.30 | 1.818 | 0.000 | 1.88e-09 | 5.93e-11 | 0 |
| 180–239 | 2549.4 | 4224.0 | 8.23 | 49.97 | 6.10 | 3.45 | 771.10 | 5.446 | 4.643 | 4.09e-09 | 1.43e-10 | 12 |
| 240–299 | 2086.6 | 4039.3 | 8.60 | 56.35 | 7.43 | 3.80 | 843.60 | 5.245 | 28.606 | 5.92e-10 | 2.18e-11 | 60 |
| 300–359 | 3621.3 | 64967.8 | 8.62 | 53.23 | 7.95 | 331.02 | 73485.70 | 17.196 | 43.383 | 6.16e-10 | 2.00e-11 | 60 |
| 360–419 | 7602.6 | 139888.8 | 8.83 | 54.70 | 7.35 | 1962.03 | 428398.60 | 73.450 | 101.868 | 2.32e-10 | 1.27e-12 | 60 |
| 420–479 | 5934.9 | 146724.0 | 8.73 | 53.70 | 8.03 | 12.90 | 2786.00 | 179.047 | 130.900 | 9.86e-09 | 2.05e-10 | 60 |
| 480–539 | 2789.8 | 4479.8 | 8.47 | 54.23 | 7.43 | 5.20 | 1123.20 | 223.241 | 165.729 | 8.02e-10 | 6.70e-12 | 60 |
| 540–599 | 1781.7 | 3038.4 | 8.22 | 52.35 | 6.97 | 2.87 | 619.20 | 257.108 | 226.989 | 3.86e-10 | 3.23e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 123.996 | 104.510 | 134.244 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
