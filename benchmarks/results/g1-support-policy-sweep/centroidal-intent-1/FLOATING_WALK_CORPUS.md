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
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.

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
| 350 | 1.750 s | 16.429 cm | 1.053 cm | 1.590 cm | 29.243 cm | 52.715° | 8.000 rad/s | 61012.7 µs |

Nominal hard residual maxima: dynamics `1.502e-09`, contact acceleration `4.972e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 117.682 cm |
| stance foot RMS | 71.208 cm |
| swing foot RMS | 126.229 cm |
| hand RMS | 142.920 cm |
| maximum root rotation | 179.685° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.502e-09 |
| contact acceleration residual | 4.972e-11 |
| raw max dynamics residual, including rejected ticks | 6.898e+02 |
| raw max contact residual, including rejected ticks | 4.972e-11 |
| active normal force range | 0.000–251.327 N |
| point-task acceleration RMS max | 163.890 m/s² |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2505.8 µs | 225277.2 µs | 227744.6 µs | 295942.0 µs | 196 | 151 | 3 | 33 | 51 | 166 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 77817.2 | 99125.8 | 741.0 | 224291.0 | 279687.7 | 294316.6 | 134124.8 | 600 | 249 | 244 | 12.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 196 | 2385.0 | 2525.0 | 2553.3 | 2565.5 |
| solved_with_slack | 151 | 2101.4 | 42615.9 | 82542.8 | 92259.2 |
| primal_infeasible | 166 | 224076.0 | 226391.4 | 228439.2 | 295942.0 |
| normal_contact_contingency | 33 | 1900.0 | 2395.1 | 100812.3 | 147104.0 |
| contact_release_contingency | 51 | 136171.6 | 178083.6 | 226432.1 | 268806.2 |
| touchdown_transition | 3 | 2218.2 | 2224.0 | 2224.5 | 2224.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.89 | 11.0 | 13.0 | 16 | 2.93 | 12.0 | 15 | -0.6553 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 196 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 151 | 9.34/14.5/15 | 7.47/13.0/15 |
| primal_infeasible | 166 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 33 | 9.82/15.4/16 | 9.03/14.4/15 |
| contact_release_contingency | 51 | 7.86/10.0/10 | 6.55/9.0/9 |
| touchdown_transition | 3 | 5.00/5.0/5 | 0.00/0.0/0 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.30/5.0/8 | 5.26/25.0/32 | 0.88/5.0/8 | 195 |
| viability | 0.72/7.0/10 | 3.44/35.0/50 | 0.56/7.0/10 | 158 |
| intent | 1.05/5.0/7 | 5.35/30.1/42 | 0.68/5.0/7 | 212 |
| preference | 1.08/5.0/7 | 11.14/55.1/72 | 0.70/5.0/7 | 218 |
| style | 0.73/1.0/4 | 5.30/11.0/18 | 0.12/1.0/4 | 67 |

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 46.690 | 46.681 | 46.681 | 1.000 | 1.000 | 46.238 | 46.461 | 0.223 | 46.578 | 0.001 | 0 | 36 | 0 | 0 | 504 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2385.9 | 2524.4 | 4.05 | 29.15 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 60–119 | 2381.9 | 2554.1 | 4.00 | 29.00 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 120–179 | 2382.3 | 2548.8 | 4.00 | 29.03 | 0.00 | 0.000 | 0.000 | 1.50e-09 | 4.76e-11 | 0 |
| 180–239 | 2379.8 | 2989.5 | 7.15 | 49.98 | 3.78 | 0.191 | 0.093 | 1.22e-09 | 4.97e-11 | 0 |
| 240–299 | 1927.7 | 84114.1 | 8.73 | 51.98 | 7.08 | 13.536 | 1.567 | 1.13e-09 | 1.33e-11 | 0 |
| 300–359 | 2147.6 | 138900.5 | 10.32 | 57.32 | 9.20 | 46.088 | 2.703 | 7.19e-10 | 7.90e-12 | 10 |
| 360–419 | 132692.7 | 136883.6 | 8.75 | 49.45 | 7.65 | 125.471 | 61.793 | 7.41e-10 | 8.19e-12 | 60 |
| 420–479 | 224061.9 | 243776.2 | 1.90 | 8.92 | 1.62 | 200.103 | 162.862 | 6.90e+02 | 0.00e+00 | 60 |
| 480–539 | 224063.1 | 227987.4 | 0.00 | 0.00 | 0.00 | 200.845 | 168.085 | 6.90e+02 | 0.00e+00 | 60 |
| 540–599 | 223907.7 | 256166.5 | 0.00 | 0.00 | 0.00 | 200.152 | 167.452 | 6.90e+02 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 117.682 | 93.083 | 142.920 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
