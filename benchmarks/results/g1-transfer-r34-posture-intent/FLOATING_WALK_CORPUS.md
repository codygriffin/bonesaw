# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `dcm-backward-preview` `DCM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `intent` priority with weight `0.250`.
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
| 483 | 2.415 s | 9.653 cm | 8.193 cm | 25.126 cm | 21.937 cm | 42.587° | 8.000 rad/s | 93842.2 µs |

Nominal hard residual maxima: dynamics `1.998e-09`, contact acceleration `5.232e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 58.987 cm |
| authored reference vs measured CoM RMS / p95 | 54.821 / 154.438 cm |
| stance foot RMS | 40.797 cm |
| swing foot RMS | 26.545 cm |
| hand RMS | 72.604 cm |
| maximum root rotation | 128.100° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.465e-09 |
| contact acceleration residual | 1.815e-10 |
| raw max dynamics residual, including rejected ticks | 8.465e-09 |
| raw max contact residual, including rejected ticks | 1.815e-10 |
| active normal force range | 0.000–715.730 N |
| centroidal momentum-rate residual RMS / max | 62.854 / 277.118 N·m |
| point-task acceleration RMS max | 105.631 m/s² |
| frame-angular acceleration RMS max | 249.032 rad/s² |
| longest pre-contact / touchdown transition | 255 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `20.945` / `54.376 cm`.
- Virtual ZMP clipped on `58.50%` of ticks; clip-distance RMS / max `30.902` / `106.794 cm`.
- Measured-height natural frequency min / p50 / max: `3.654` / `3.768` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.839 m`; height-floor ticks: `103`.
- CoM command acceleration p95 / max: `21.946` / `23.756 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2457.6 µs | 66040.6 µs | 128144.5 µs | 207941.1 µs | 224 | 4 | 255 | 0 | 109 | 8 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9573.3 | 24719.2 | 573.6 | 27241.9 | 201113.9 | 207258.4 | 42546.2 | 600 | 111 | 63 | 104.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 224 | 2456.4 | 2675.7 | 3461.1 | 3744.1 |
| solved_with_slack | 4 | 2279.1 | 2537.4 | 2559.4 | 2564.9 |
| normal_contact_contingency | 109 | 6131.6 | 34472.7 | 35158.1 | 127940.7 |
| contact_release_contingency | 8 | 157486.1 | 203951.9 | 207143.2 | 207941.1 |
| precontact_transition | 255 | 1989.4 | 73443.8 | 102743.0 | 110651.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.40 | 10.0 | 12.0 | 14 | 4.25 | 11.0 | 13 | 0.1539 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 298.38/2119.1/5780.9/7087 | 65749.35/459474.9/1283355.4/1573314 | 0.86/8.0/12 | 0.70/7.0/11 | 5.95/63.0/97 | 0.6466 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 224 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 4 | 7.50/9.0/9 | 5.25/7.9/8 |
| normal_contact_contingency | 109 | 8.15/11.0/12 | 7.38/11.0/11 |
| contact_release_contingency | 8 | 6.00/6.9/7 | 4.75/5.9/6 |
| precontact_transition | 255 | 7.76/12.5/14 | 6.61/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.00/6.0/9 | 8.76/30.0/38 | 1.47/6.0/9 | 285 |
| viability | 1.76/7.0/9 | 9.70/42.0/56 | 1.33/6.0/8 | 369 |
| intent | 1.53/6.0/7 | 13.21/53.0/72 | 1.09/6.0/7 | 351 |
| preference | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| style | 1.11/4.0/4 | 9.11/26.0/28 | 0.36/3.0/4 | 174 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 483 | 117 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `120` ticks.
Precontact sole-center tangential speed: p50 `1.6416 m/s`, p95 `5.3605 m/s`, max `7.6531 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.744 | 5.739 | 5.739 | 0.999 | 0.999 | 47.219 | 47.516 | 0.297 | 48.445 | 0.001 | 0 | 60 | 0 | 0 | 537 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2513.9 | 3639.0 | 4.00 | 25.02 | 0.00 | 1.00 | 234.00 | 0.269 | 0.000 | 1.29e-09 | 4.72e-11 | 0 |
| 60–119 | 2447.8 | 2538.4 | 4.00 | 25.50 | 0.00 | 1.00 | 234.00 | 1.114 | 0.000 | 1.43e-09 | 4.89e-11 | 0 |
| 120–179 | 2453.3 | 3229.8 | 4.00 | 25.02 | 0.00 | 1.00 | 234.00 | 3.171 | 0.000 | 2.00e-09 | 5.23e-11 | 0 |
| 180–239 | 1849.2 | 3278.9 | 5.08 | 31.83 | 1.53 | 1.00 | 225.80 | 7.808 | 0.465 | 1.45e-09 | 3.19e-11 | 12 |
| 240–299 | 1845.0 | 3503.0 | 7.28 | 46.30 | 5.45 | 1.00 | 222.00 | 7.065 | 13.632 | 1.22e-09 | 1.86e-11 | 60 |
| 300–359 | 2022.7 | 3959.7 | 8.13 | 52.77 | 6.97 | 1.58 | 351.50 | 7.648 | 27.249 | 6.93e-10 | 1.67e-11 | 60 |
| 360–419 | 1993.0 | 3970.3 | 7.82 | 48.35 | 7.07 | 2.28 | 506.90 | 9.862 | 22.657 | 5.20e-10 | 2.04e-11 | 60 |
| 420–479 | 2808.2 | 98670.6 | 7.73 | 49.55 | 7.10 | 1809.25 | 401653.50 | 19.605 | 18.997 | 7.16e-10 | 1.18e-11 | 60 |
| 480–539 | 29379.7 | 201216.5 | 8.43 | 54.27 | 7.78 | 1157.77 | 252129.00 | 95.114 | 48.776 | 5.41e-09 | 1.82e-10 | 60 |
| 540–599 | 5294.1 | 9523.0 | 7.55 | 49.22 | 6.58 | 7.88 | 1702.80 | 158.385 | 96.367 | 8.46e-09 | 1.14e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 58.987 | 36.699 | 72.604 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
