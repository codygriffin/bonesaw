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
- Whole-body posture: `viability` priority with weight `0.050`.
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
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 411 | 2.055 s | 15.622 cm | 1.118 cm | 25.687 cm | 24.793 cm | 49.305° | 8.000 rad/s | 12793.2 µs |

Nominal hard residual maxima: dynamics `5.840e-09`, contact acceleration `1.770e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 93.051 cm |
| stance foot RMS | 52.571 cm |
| swing foot RMS | 91.944 cm |
| hand RMS | 101.866 cm |
| maximum root rotation | 179.184° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.054e-09 |
| contact acceleration residual | 3.292e-10 |
| raw max dynamics residual, including rejected ticks | 9.054e-09 |
| raw max contact residual, including rejected ticks | 3.292e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 64.104 / 248.539 N·m |
| point-task acceleration RMS max | 173.480 m/s² |
| frame-angular acceleration RMS max | 143.268 rad/s² |
| longest pre-contact / touchdown transition | 183 / 0 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2857.4 µs | 178266.9 µs | 211821.3 µs | 428840.2 µs | 185 | 43 | 183 | 0 | 110 | 79 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 26584.6 | 60250.6 | 550.5 | 155153.0 | 346273.6 | 420583.5 | 174964.7 | 600 | 146 | 82 | 37.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 185 | 2351.8 | 2422.8 | 2459.4 | 2473.9 |
| solved_with_slack | 43 | 3093.7 | 4276.6 | 4349.3 | 4374.3 |
| normal_contact_contingency | 110 | 4759.3 | 14940.6 | 24706.1 | 227652.8 |
| contact_release_contingency | 79 | 168632.0 | 215902.0 | 321324.4 | 428840.2 |
| precontact_transition | 183 | 2806.7 | 8832.4 | 14303.8 | 79347.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.92 | 9.0 | 10.0 | 11 | 3.38 | 8.0 | 9 | 0.0939 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 12.28/8.0/8.0/4838 | 2709.83/1728.0/1776.0/1074036 | 1.68/15.0/23 | 1.37/14.0/22 | 11.12/119.0/197 | 0.0410 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 185 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 43 | 7.26/10.6/11 | 4.51/8.2/9 |
| normal_contact_contingency | 110 | 6.96/10.0/11 | 5.32/9.0/9 |
| contact_release_contingency | 79 | 6.25/9.2/10 | 4.13/7.2/8 |
| precontact_transition | 183 | 6.78/10.0/10 | 5.03/8.0/9 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.97/6.0/8 | 8.40/26.0/39 | 1.45/6.0/8 | 290 |
| viability | 1.88/7.0/8 | 15.65/57.0/64 | 1.54/6.0/8 | 407 |
| intent | 1.00/1.0/1 | 4.15/6.0/6 | 0.00/0.0/0 | 0 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.07/3.0/4 | 7.87/28.0/40 | 0.39/3.0/4 | 196 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 4 | 199 | 168 |
| right_ankle_roll_link | 168 | 0 | 0 | 411 | 21 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `4` ticks, normal fallback `171` ticks.
Precontact sole-center tangential speed: p50 `1.7834 m/s`, p95 `4.0196 m/s`, max `6.1359 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `7.2849 m/s`, p95 `7.5347 m/s`, max `7.5624 m/s` over 3 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 15.951 | 15.938 | 15.938 | 0.999 | 0.999 | 47.551 | 47.703 | 0.152 | 47.770 | 0.001 | 0 | 38 | 0 | 0 | 959 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2392.7 | 2469.7 | 4.00 | 23.90 | 0.00 | 1.00 | 234.00 | 0.153 | 0.000 | 1.21e-09 | 4.02e-11 | 0 |
| 60–119 | 2356.7 | 2424.7 | 4.00 | 23.40 | 0.00 | 1.00 | 234.00 | 0.349 | 0.000 | 1.25e-09 | 4.72e-11 | 0 |
| 120–179 | 2330.6 | 3543.0 | 4.72 | 27.75 | 0.92 | 1.00 | 234.00 | 0.495 | 0.000 | 1.37e-09 | 4.89e-11 | 0 |
| 180–239 | 2995.6 | 4339.2 | 6.17 | 43.37 | 3.23 | 1.00 | 225.80 | 3.038 | 0.535 | 1.01e-09 | 4.08e-11 | 12 |
| 240–299 | 3165.8 | 8322.5 | 6.72 | 44.95 | 4.30 | 11.43 | 2538.20 | 3.533 | 13.242 | 7.96e-10 | 1.40e-11 | 60 |
| 300–359 | 2473.5 | 35423.1 | 6.63 | 41.22 | 5.17 | 81.62 | 18118.90 | 15.638 | 16.345 | 1.74e-09 | 1.54e-11 | 60 |
| 360–419 | 2637.1 | 103411.6 | 7.02 | 42.77 | 5.75 | 3.57 | 785.30 | 43.136 | 30.074 | 5.84e-09 | 1.77e-10 | 60 |
| 420–479 | 159173.9 | 347514.1 | 6.58 | 33.10 | 4.70 | 6.37 | 1346.60 | 94.557 | 58.806 | 4.89e-09 | 1.56e-10 | 60 |
| 480–539 | 6667.7 | 209959.9 | 6.97 | 42.43 | 5.25 | 8.00 | 1717.60 | 174.062 | 130.920 | 4.98e-09 | 1.14e-10 | 60 |
| 540–599 | 5182.1 | 216094.2 | 6.42 | 37.85 | 4.45 | 7.77 | 1663.90 | 212.647 | 156.575 | 9.05e-09 | 3.29e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 93.051 | 68.162 | 101.866 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
