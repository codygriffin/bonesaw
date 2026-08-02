# Bonesaw floating CMU walking corpus

This is the first moving-root, contact-aware retarget through the Rust floating inverse-dynamics WBC. Python owns source reconstruction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target stride: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `rooted` reference at `viability` priority with weight `0.000` and `2.000 Hz` response. Hand task weight: `0.000` (hand error is observational when zero).
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.

## Acceptance

Overall: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | FAIL |
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

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 330 | 1.650 s | 12.029 cm | 1.494 cm | 3.596 cm | 24.186 cm | 19.673° | 8.000 rad/s | 129868.6 µs |

Nominal hard residual maxima: dynamics `1.218e-09`, contact acceleration `5.324e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 156.891 cm |
| stance foot RMS | 106.353 cm |
| swing foot RMS | 143.447 cm |
| hand RMS | 185.917 cm |
| maximum root rotation | 144.209° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.218e-09 |
| contact acceleration residual | 5.324e-11 |
| raw max dynamics residual, including rejected ticks | 8.483e+02 |
| raw max contact residual, including rejected ticks | 5.324e-11 |
| active normal force range | 0.000–548.554 N |
| point-task acceleration RMS max | 164.615 m/s² |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 11018.8 µs | 222610.1 µs | 225371.2 µs | 273885.0 µs | 196 | 131 | 4 | 27 | 111 | 131 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 81536.6 | 91469.7 | 9037.9 | 221336.9 | 266179.7 | 273114.5 | 131771.0 | 600 | 333 | 272 | 12.3 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 196 | 2380.6 | 2414.2 | 3170.4 | 3489.3 |
| solved_with_slack | 131 | 5741.8 | 116191.7 | 136006.9 | 137119.7 |
| primal_infeasible | 131 | 221237.8 | 224430.1 | 239613.6 | 254296.4 |
| normal_contact_contingency | 27 | 73446.9 | 128842.6 | 131210.2 | 131906.0 |
| contact_release_contingency | 111 | 133945.6 | 174281.1 | 253424.0 | 273885.0 |
| touchdown_transition | 4 | 2242.3 | 5329.9 | 5762.4 | 5870.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.08 | 10.0 | 12.0 | 14 | 3.07 | 11.0 | 13 | -0.4709 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 196 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 131 | 8.74/13.0/14 | 6.81/12.0/13 |
| primal_infeasible | 131 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 27 | 8.56/11.0/11 | 7.89/11.0/11 |
| contact_release_contingency | 111 | 7.81/10.0/10 | 6.60/9.0/9 |
| touchdown_transition | 4 | 5.50/6.9/7 | 1.50/5.8/6 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.46/5.0/7 | 5.96/25.0/35 | 1.04/5.0/7 | 231 |
| viability | 0.73/5.0/7 | 3.75/24.0/32 | 0.61/5.0/7 | 213 |
| intent | 1.02/4.0/7 | 2.06/8.0/14 | 0.63/4.0/7 | 240 |
| preference | 1.07/5.0/8 | 8.55/40.0/58 | 0.70/5.0/7 | 247 |
| style | 0.80/2.0/4 | 6.11/12.0/26 | 0.09/2.0/3 | 42 |

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 48.922 | 48.910 | 48.910 | 1.000 | 1.000 | 46.219 | 46.426 | 0.207 | 46.578 | 0.001 | 0 | 37 | 0 | 0 | 543 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2375.4 | 2422.8 | 4.05 | 24.17 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 60–119 | 2390.0 | 3388.6 | 4.00 | 23.85 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 120–179 | 2381.0 | 2448.7 | 4.00 | 23.78 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 180–239 | 2180.4 | 8749.8 | 6.45 | 34.60 | 2.75 | 0.096 | 0.012 | 1.03e-09 | 5.32e-11 | 0 |
| 240–299 | 10410.3 | 16886.4 | 8.90 | 41.53 | 7.65 | 10.522 | 2.113 | 6.08e-13 | 6.61e-14 | 0 |
| 300–359 | 133204.6 | 193193.5 | 9.10 | 44.02 | 7.97 | 55.370 | 7.825 | 3.56e-10 | 6.46e-12 | 30 |
| 360–419 | 133599.3 | 156813.1 | 7.98 | 39.73 | 6.93 | 146.514 | 73.232 | 1.10e-09 | 1.18e-11 | 60 |
| 420–479 | 138921.4 | 237664.0 | 6.33 | 32.63 | 5.43 | 247.715 | 193.628 | 8.48e+02 | 3.55e-14 | 59 |
| 480–539 | 221800.5 | 248549.0 | 0.00 | 0.00 | 0.00 | 283.474 | 225.088 | 8.48e+02 | 0.00e+00 | 60 |
| 540–599 | 221125.3 | 225122.3 | 0.00 | 0.00 | 0.00 | 282.462 | 224.008 | 8.48e+02 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 156.891 | 119.902 | 185.917 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
