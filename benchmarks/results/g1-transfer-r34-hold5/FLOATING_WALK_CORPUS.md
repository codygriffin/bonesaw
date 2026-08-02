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
| 325 | 1.625 s | 13.388 cm | 14.962 cm | 36.215 cm | 34.384 cm | 88.055° | 8.000 rad/s | 5483.2 µs |

Nominal hard residual maxima: dynamics `1.688e-09`, contact acceleration `5.341e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 105.351 cm |
| authored reference vs measured CoM RMS / p95 | 107.904 / 185.748 cm |
| stance foot RMS | 83.885 cm |
| swing foot RMS | 106.116 cm |
| hand RMS | 124.111 cm |
| maximum root rotation | 179.223° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.387e-09 |
| contact acceleration residual | 2.125e-10 |
| raw max dynamics residual, including rejected ticks | 3.387e-09 |
| raw max contact residual, including rejected ticks | 2.125e-10 |
| active normal force range | 0.000–678.514 N |
| centroidal momentum-rate residual RMS / max | 58.480 / 362.986 N·m |
| point-task acceleration RMS max | 116.187 m/s² |
| frame-angular acceleration RMS max | 260.722 rad/s² |
| longest pre-contact / touchdown transition | 92 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 167 / 167 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `91.614` / `168.328 cm`.
- Virtual ZMP clipped on `61.83%` of ticks; clip-distance RMS / max `126.027` / `238.459 cm`.
- Measured-height natural frequency min / p50 / max: `3.670` / `4.345` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.520 m`; height-floor ticks: `257`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2573.9 µs | 8895.5 µs | 186509.5 µs | 209078.1 µs | 100 | 133 | 92 | 0 | 256 | 19 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7546.2 | 26326.9 | 620.1 | 4495.9 | 208546.5 | 209024.9 | 111039.0 | 600 | 45 | 19 | 132.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 100 | 2435.9 | 2537.8 | 2595.2 | 3159.8 |
| solved_with_slack | 133 | 2689.8 | 4978.0 | 5741.5 | 6926.3 |
| normal_contact_contingency | 256 | 2675.8 | 8605.6 | 18535.5 | 19224.0 |
| contact_release_contingency | 19 | 114464.1 | 208279.3 | 208918.3 | 209078.1 |
| precontact_transition | 92 | 3114.8 | 4496.1 | 4668.2 | 5146.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.81 | 12.0 | 14.0 | 17 | 5.80 | 13.0 | 16 | 0.0121 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 6.15/8.0/8.5/393 | 1348.65/1872.0/1972.1/84888 | 1.30/17.0/18 | 0.88/16.0/17 | 7.39/139.1/156 | 0.0062 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 100 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 133 | 7.11/11.0/11 | 4.77/9.0/9 |
| normal_contact_contingency | 256 | 9.04/15.0/17 | 7.89/14.0/16 |
| contact_release_contingency | 19 | 7.68/10.0/10 | 6.21/8.8/9 |
| precontact_transition | 92 | 8.49/13.0/13 | 7.71/13.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.21/6.0/11 | 9.67/29.0/52 | 1.84/6.0/11 | 402 |
| viability | 1.50/5.0/8 | 8.37/35.0/48 | 1.16/5.0/8 | 408 |
| intent | 1.73/5.0/10 | 8.98/25.0/57 | 1.56/5.0/10 | 499 |
| preference | 1.32/5.0/10 | 12.67/47.0/94 | 0.93/5.0/10 | 388 |
| style | 1.04/2.0/4 | 9.11/19.0/33 | 0.31/2.0/4 | 165 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 367 | 0 | 204 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 325 | 275 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `367` ticks, planned normal touchdown `0` ticks, normal fallback `278` ticks.
Precontact sole-center tangential speed: p50 `2.7169 m/s`, p95 `6.3331 m/s`, max `7.6112 m/s` over 367 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.528 | 4.527 | 4.527 | 1.000 | 1.000 | 48.125 | 48.562 | 0.438 | 48.562 | 0.001 | 0 | 111 | 0 | 0 | 57 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2425.2 | 2569.6 | 5.00 | 33.50 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.69e-09 | 5.16e-11 | 0 |
| 60–119 | 2482.6 | 2961.1 | 5.50 | 37.10 | 1.17 | 1.00 | 234.00 | 0.065 | 0.000 | 1.28e-09 | 5.15e-11 | 0 |
| 120–179 | 4213.9 | 6291.7 | 7.28 | 44.88 | 5.40 | 5.08 | 1189.50 | 1.084 | 0.000 | 1.36e-09 | 3.07e-11 | 0 |
| 180–239 | 2523.3 | 3832.9 | 7.25 | 44.02 | 4.67 | 3.45 | 770.70 | 5.632 | 0.332 | 8.30e-10 | 4.56e-11 | 7 |
| 240–299 | 3644.3 | 4836.5 | 8.18 | 49.87 | 7.47 | 7.53 | 1672.40 | 15.479 | 21.269 | 8.91e-10 | 5.34e-11 | 60 |
| 300–359 | 4469.0 | 208554.5 | 9.62 | 58.50 | 8.75 | 9.43 | 2037.90 | 63.364 | 87.978 | 3.39e-09 | 2.13e-10 | 60 |
| 360–419 | 2942.7 | 193834.0 | 8.87 | 53.98 | 7.57 | 5.67 | 1220.80 | 139.862 | 93.721 | 2.68e-09 | 9.09e-11 | 60 |
| 420–479 | 1735.6 | 2722.7 | 8.75 | 55.75 | 7.48 | 1.12 | 241.20 | 170.361 | 131.044 | 1.50e-09 | 9.31e-12 | 60 |
| 480–539 | 2771.4 | 3679.6 | 8.75 | 55.25 | 7.62 | 5.08 | 1098.00 | 183.182 | 148.542 | 6.00e-10 | 1.32e-11 | 60 |
| 540–599 | 2756.1 | 7858.5 | 8.92 | 55.13 | 7.90 | 22.17 | 4788.00 | 156.721 | 167.185 | 1.32e-09 | 1.31e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 105.351 | 91.741 | 124.111 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
