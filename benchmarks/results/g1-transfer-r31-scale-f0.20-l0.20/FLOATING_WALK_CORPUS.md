# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.206 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.157 m/s` (`0.20×` forward, `0.20×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
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
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 425 | 2.125 s | 14.505 cm | 14.115 cm | 46.933 cm | 32.292 cm | 77.419° | 8.000 rad/s | 17756.1 µs |

Nominal hard residual maxima: dynamics `1.622e-09`, contact acceleration `5.634e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 44.174 cm |
| stance foot RMS | 45.116 cm |
| swing foot RMS | 60.870 cm |
| hand RMS | 57.376 cm |
| maximum root rotation | 142.294° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.622e-09 |
| contact acceleration residual | 5.634e-11 |
| raw max dynamics residual, including rejected ticks | 1.622e-09 |
| raw max contact residual, including rejected ticks | 5.634e-11 |
| active normal force range | 0.000–502.397 N |
| centroidal momentum-rate residual RMS / max | 31.600 / 183.452 N·m |
| point-task acceleration RMS max | 104.780 m/s² |
| frame-angular acceleration RMS max | 91.189 rad/s² |
| longest pre-contact / touchdown transition | 197 / 157 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2609.1 µs | 73108.7 µs | 100539.2 µs | 197553.4 µs | 141 | 87 | 197 | 157 | 8 | 10 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11211.4 | 24210.5 | 658.5 | 47178.7 | 151838.0 | 192981.8 | 52001.9 | 600 | 105 | 74 | 89.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2447.6 | 2569.7 | 2735.3 | 2903.4 |
| solved_with_slack | 87 | 2781.0 | 5029.3 | 6001.6 | 7377.5 |
| normal_contact_contingency | 8 | 19136.8 | 113988.6 | 119784.9 | 121234.0 |
| contact_release_contingency | 10 | 100216.5 | 154940.7 | 189030.8 | 197553.4 |
| touchdown_transition | 157 | 4082.5 | 86890.2 | 96534.9 | 100494.6 |
| precontact_transition | 197 | 2973.2 | 4345.2 | 30854.7 | 33695.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.84 | 12.0 | 14.0 | 17 | 5.55 | 13.0 | 17 | 0.2141 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 443.16/4169.1/6137.9/6912 | 95826.93/900525.6/1325784.2/1492992 | 1.06/12.0/16 | 0.75/11.0/15 | 6.10/95.0/128 | 0.7832 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 87 | 8.64/14.3/16 | 6.13/13.1/14 |
| normal_contact_contingency | 8 | 12.88/16.9/17 | 12.38/16.9/17 |
| contact_release_contingency | 10 | 7.80/9.0/9 | 6.30/7.9/8 |
| touchdown_transition | 157 | 8.78/13.4/15 | 7.39/13.0/15 |
| precontact_transition | 197 | 8.58/14.0/15 | 7.47/12.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.18/6.0/8 | 9.33/30.0/40 | 1.73/6.0/8 | 362 |
| viability | 1.91/7.0/12 | 11.23/37.0/63 | 1.66/7.0/12 | 452 |
| intent | 1.34/4.0/7 | 6.93/23.0/40 | 1.06/4.0/7 | 434 |
| preference | 1.28/6.0/8 | 12.74/56.1/88 | 0.73/5.0/8 | 289 |
| style | 1.14/3.0/6 | 10.10/29.0/46 | 0.36/3.0/6 | 169 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 200 | 161 | 199 | 11 |
| right_ankle_roll_link | 168 | 0 | 0 | 425 | 7 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `200` ticks, planned normal touchdown `161` ticks, normal fallback `14` ticks.
Precontact sole-center tangential speed: p50 `1.7201 m/s`, p95 `2.3671 m/s`, max `2.4417 m/s` over 200 samples.
Touchdown Normal sole-center tangential speed: p50 `3.0363 m/s`, p95 `5.0090 m/s`, max `5.8760 m/s` over 160 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.727 | 6.721 | 6.721 | 0.999 | 0.999 | 47.527 | 47.695 | 0.168 | 47.828 | 0.001 | 0 | 42 | 0 | 0 | 493 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2489.4 | 2873.9 | 6.07 | 36.82 | 1.67 | 1.00 | 234.00 | 0.172 | 0.000 | 1.62e-09 | 5.63e-11 | 0 |
| 60–119 | 2437.3 | 3523.6 | 5.10 | 34.98 | 0.17 | 1.00 | 234.00 | 0.032 | 0.000 | 1.23e-09 | 4.79e-11 | 0 |
| 120–179 | 2492.2 | 4840.2 | 6.03 | 38.68 | 1.53 | 1.12 | 261.30 | 0.002 | 0.000 | 1.46e-09 | 4.52e-11 | 0 |
| 180–239 | 2937.7 | 6433.6 | 8.80 | 57.30 | 6.75 | 4.03 | 909.00 | 3.523 | 3.267 | 1.28e-09 | 2.37e-11 | 12 |
| 240–299 | 3011.7 | 3429.9 | 7.92 | 50.52 | 6.60 | 5.90 | 1309.80 | 9.058 | 23.560 | 9.55e-10 | 1.04e-11 | 60 |
| 300–359 | 1842.4 | 3791.0 | 8.37 | 51.72 | 7.35 | 2.07 | 458.80 | 13.960 | 34.941 | 7.73e-10 | 1.20e-11 | 60 |
| 360–419 | 3209.2 | 31732.2 | 9.42 | 60.15 | 8.67 | 118.83 | 26381.00 | 31.248 | 52.611 | 7.95e-10 | 1.22e-11 | 60 |
| 420–479 | 3369.7 | 108997.8 | 9.22 | 61.18 | 8.22 | 152.78 | 33198.40 | 67.184 | 105.743 | 1.10e-09 | 2.16e-11 | 12 |
| 480–539 | 56661.8 | 96460.3 | 9.13 | 61.68 | 7.93 | 3291.63 | 710992.80 | 75.937 | 75.685 | 4.64e-10 | 8.18e-12 | 0 |
| 540–599 | 1770.2 | 141683.4 | 8.40 | 50.35 | 6.58 | 853.20 | 184290.20 | 89.258 | 66.442 | 3.42e-10 | 6.33e-12 | 11 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 44.174 | 50.871 | 57.376 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
