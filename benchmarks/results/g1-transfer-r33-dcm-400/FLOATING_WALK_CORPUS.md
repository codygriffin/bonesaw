# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-centroid-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| `touchdown_admission_completes_within_8_ticks` | PASS |
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
| 378 | 1.890 s | 18.366 cm | 16.903 cm | 37.499 cm | 34.720 cm | 101.830° | 8.000 rad/s | 6630.8 µs |

Nominal hard residual maxima: dynamics `1.878e-09`, contact acceleration `8.151e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 24.722 cm |
| CoM RMS / p95 | 22.882 / 62.639 cm |
| stance foot RMS | 25.516 cm |
| swing foot RMS | 50.054 cm |
| hand RMS | 42.493 cm |
| maximum root rotation | 150.196° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.198e-09 |
| contact acceleration residual | 8.151e-11 |
| raw max dynamics residual, including rejected ticks | 2.198e-09 |
| raw max contact residual, including rejected ticks | 8.151e-11 |
| active normal force range | 0.000–450.278 N |
| centroidal momentum-rate residual RMS / max | 27.743 / 107.538 N·m |
| point-task acceleration RMS max | 149.875 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `30.087` / `75.150 cm`.
- Virtual ZMP clipped on `45.00%` of ticks; clip-distance RMS / max `50.247` / `125.692 cm`.
- Measured-height natural frequency min / p50 / max: `3.713` / `3.784` / `7.004 rad/s`.
- CoM command acceleration p95 / max: `67.940` / `77.586 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 400 | 2.0 s | 2493.8 µs | 12630.1 µs | 185946.0 µs | 195721.6 µs | 158 | 220 | 0 | 0 | 9 | 13 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8031.0 | 27717.5 | 286.2 | 3649.6 | 194876.6 | 195637.1 | 84181.1 | 400 | 30 | 15 | 124.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 158 | 2422.6 | 2515.7 | 2788.9 | 2906.9 |
| solved_with_slack | 220 | 2973.3 | 3743.9 | 6886.2 | 10869.1 |
| normal_contact_contingency | 9 | 13558.1 | 123116.3 | 177271.7 | 190810.5 |
| contact_release_contingency | 13 | 156533.0 | 194450.9 | 195467.5 | 195721.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.21 | 12.0 | 14.0 | 17 | 4.26 | 13.0 | 15 | 0.1145 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 3.62/8.0/8.0/8 | 806.52/1776.0/1776.0/1776 | 1.22/14.0/22 | 0.85/13.0/21 | 7.04/112.0/184 | 0.2442 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 158 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 220 | 8.59/16.0/17 | 6.89/13.0/15 |
| normal_contact_contingency | 9 | 11.11/13.9/14 | 10.67/12.9/13 |
| contact_release_contingency | 13 | 8.15/10.0/10 | 7.08/9.0/9 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.73/5.0/9 | 6.68/24.0/37 | 1.09/5.0/9 | 179 |
| viability | 1.91/9.0/13 | 6.95/30.0/40 | 1.39/9.0/12 | 215 |
| intent | 1.41/4.0/5 | 6.89/24.0/30 | 1.01/4.0/5 | 242 |
| preference | 1.12/4.0/5 | 11.86/45.0/55 | 0.52/4.0/5 | 165 |
| style | 1.05/2.0/5 | 9.66/21.0/24 | 0.25/2.0/4 | 86 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 201 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 378 | 22 |
| left_wrist_roll_rubber_hand | 400 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 400 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `25` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.213 | 3.211 | 3.211 | 1.000 | 1.000 | 46.953 | 47.207 | 0.254 | 48.113 | 0.001 | 0 | 46 | 0 | 0 | 51 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–39 | 2424.4 | 2710.9 | 5.00 | 32.45 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.88e-09 | 5.29e-11 | 0 |
| 40–79 | 2435.2 | 2859.5 | 5.00 | 32.35 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 9.55e-10 | 5.86e-11 | 0 |
| 80–119 | 2432.1 | 2524.5 | 5.00 | 31.70 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.43e-09 | 5.70e-11 | 0 |
| 120–159 | 2415.6 | 2942.1 | 5.25 | 32.15 | 0.35 | 1.00 | 234.00 | 0.001 | 0.000 | 1.25e-09 | 4.22e-11 | 0 |
| 160–199 | 2497.1 | 3092.9 | 10.30 | 45.58 | 7.88 | 1.00 | 233.70 | 1.337 | 0.000 | 1.12e-09 | 4.39e-11 | 0 |
| 200–239 | 2962.5 | 3718.1 | 7.62 | 45.55 | 5.10 | 6.60 | 1465.20 | 3.535 | 0.298 | 6.77e-10 | 6.76e-12 | 0 |
| 240–279 | 3099.6 | 3654.5 | 8.00 | 46.85 | 6.45 | 6.78 | 1504.05 | 8.226 | 2.244 | 3.52e-10 | 1.17e-11 | 0 |
| 280–319 | 3072.7 | 3722.7 | 8.55 | 51.67 | 7.60 | 6.95 | 1542.90 | 16.758 | 12.406 | 4.79e-10 | 2.65e-11 | 0 |
| 320–359 | 1974.4 | 3582.5 | 8.53 | 51.25 | 7.28 | 2.92 | 649.35 | 38.186 | 47.381 | 1.20e-09 | 1.94e-11 | 0 |
| 360–399 | 12893.5 | 194895.7 | 8.90 | 50.77 | 7.95 | 8.00 | 1734.00 | 65.506 | 93.626 | 2.20e-09 | 8.15e-11 | 22 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 400 | 24.722 | 33.421 | 42.493 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
