# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `rooted` reference at `viability` priority with weight `0.000` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `0` ticks (`0.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; contact forces remain off until the authored contact edge.
- Reference touchdown blend: `0` ticks (`0.000 s`) with a quintic zero-velocity endpoint.
- Touchdown stabilization point: `ankle origin`.

## Acceptance

Functional: **PASS**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 260 | 1.300 s | 0.706 cm | 0.000 cm | 0.326 cm | 10.898 cm | 4.390° | 8.000 rad/s | 11175.2 µs |

Nominal hard residual maxima: dynamics `1.218e-09`, contact acceleration `5.152e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 0.706 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 0.326 cm |
| hand RMS | 10.898 cm |
| maximum root rotation | 4.390° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.218e-09 |
| contact acceleration residual | 5.152e-11 |
| raw max dynamics residual, including rejected ticks | 1.218e-09 |
| raw max contact residual, including rejected ticks | 5.152e-11 |
| active normal force range | 0.000–224.661 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 21.972 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 260 | 1.3 s | 2449.2 µs | 9173.7 µs | 11175.2 µs | 12121.1 µs | 199 | 61 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2920.9 | 1910.1 | 15.4 | 2508.1 | 11896.0 | 12098.6 | 3377.1 | 260 | 20 | 0 | 342.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 2450.8 | 2486.3 | 2532.3 | 2569.7 |
| solved_with_slack | 61 | 2317.0 | 11134.2 | 11599.6 | 12121.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.94 | 9.0 | 11.0 | 12 | 1.17 | 9.4 | 10 | 0.5490 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 27.48/331.1/462.9/493 | 6110.77/73504.2/102768.2/109446 | 0.08/1.0/1 | 0.08/1.0/1 | 0.64/9.0/9 | 0.9943 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 61 | 8.00/11.4/12 | 4.97/10.0/10 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.10/3.0/3 | 3.65/12.0/12 | 0.17/3.0/3 | 20 |
| viability | 0.25/1.4/2 | 0.88/5.6/8 | 0.05/1.0/2 | 10 |
| intent | 1.23/5.0/7 | 2.45/10.0/14 | 0.35/5.0/7 | 32 |
| preference | 1.36/5.0/7 | 9.85/37.0/53 | 0.59/5.0/6 | 61 |
| style | 1.00/1.0/1 | 10.67/12.0/13 | 0.02/1.0/1 | 4 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 61 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 260 | 0 |
| left_wrist_roll_rubber_hand | 260 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 260 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.760 | 0.759 | 0.759 | 1.000 | 1.000 | 47.574 | 47.574 | 0.000 | 47.574 | 0.001 | 0 | 0 | 0 | 0 | 4 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–25 | 2435.5 | 2474.8 | 4.00 | 23.73 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 26–51 | 2447.8 | 2463.3 | 4.00 | 23.88 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 52–77 | 2455.1 | 2466.9 | 4.00 | 23.81 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 78–103 | 2455.6 | 2471.2 | 4.00 | 23.88 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 104–129 | 2453.0 | 2464.6 | 4.00 | 23.81 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 130–155 | 2456.6 | 2488.5 | 4.00 | 23.85 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 156–181 | 2455.3 | 2560.2 | 4.00 | 23.81 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 182–207 | 2386.0 | 2533.3 | 5.04 | 28.92 | 1.04 | 1.00 | 229.85 | 0.007 | 0.001 | 8.52e-10 | 5.15e-11 | 0 |
| 208–233 | 2209.2 | 2698.4 | 7.62 | 41.46 | 3.81 | 1.00 | 222.00 | 0.032 | 0.010 | 1.14e-09 | 1.32e-11 | 0 |
| 234–259 | 9229.2 | 11903.8 | 8.73 | 37.81 | 6.81 | 265.85 | 59017.85 | 2.232 | 0.352 | 2.17e-10 | 4.72e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 260 | 0.706 | 0.112 | 10.898 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
