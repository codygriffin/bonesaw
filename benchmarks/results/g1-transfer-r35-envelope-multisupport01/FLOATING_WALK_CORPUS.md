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
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root horizontal task weight `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `multi-support` measured-phase policy.
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
| 400 | 2.000 s | 17.257 cm | 19.237 cm | 59.712 cm | 37.856 cm | 89.621° | 8.000 rad/s | 82720.5 µs |

Nominal hard residual maxima: dynamics `2.025e-09`, contact acceleration `5.679e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 138.009 cm |
| authored reference vs measured CoM RMS / p95 | 142.582 / 332.235 cm |
| stance foot RMS | 133.530 cm |
| swing foot RMS | 151.544 cm |
| hand RMS | 141.600 cm |
| maximum root rotation | 154.227° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.835e-09 |
| contact acceleration residual | 1.394e-10 |
| raw max dynamics residual, including rejected ticks | 2.835e-09 |
| raw max contact residual, including rejected ticks | 1.394e-10 |
| active normal force range | 0.000–612.618 N |
| centroidal momentum-rate residual RMS / max | 38.568 / 144.295 N·m |
| point-task acceleration RMS max | 88.299 m/s² |
| frame-angular acceleration RMS max | 179.862 rad/s² |
| longest pre-contact / touchdown transition | 172 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `163.314` / `371.441 cm`.
- Virtual ZMP clipped on `63.33%` of ticks; clip-distance RMS / max `239.371` / `587.045 cm`.
- Measured-height natural frequency min / p50 / max: `3.692` / `3.921` / `7.004 rad/s`.
- Minimum measured CoM height: `0.022 m`; height-floor ticks: `125`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `121` ticks; maximum active coordinates `8`; mean phase scale `0.332`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3009.6 µs | 90058.9 µs | 188902.7 µs | 212168.0 µs | 129 | 99 | 172 | 0 | 173 | 27 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 13895.1 | 34187.6 | 536.5 | 36289.0 | 208845.8 | 211835.8 | 94587.9 | 600 | 142 | 67 | 72.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2500.1 | 2631.0 | 2715.0 | 3103.0 |
| solved_with_slack | 99 | 2629.7 | 4549.0 | 5334.4 | 5522.3 |
| normal_contact_contingency | 173 | 3856.0 | 36873.1 | 81011.2 | 183958.3 |
| contact_release_contingency | 27 | 156655.3 | 204615.0 | 210726.0 | 212168.0 |
| precontact_transition | 172 | 3345.1 | 69669.3 | 90888.6 | 97226.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.75 | 12.0 | 14.0 | 14 | 5.52 | 13.0 | 14 | 0.1209 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 239.80/2350.7/5107.6/6141 | 52734.39/507751.2/1133582.0/1363302 | 2.06/15.0/22 | 1.58/14.0/21 | 13.01/124.0/176 | 0.3679 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 7.13/12.0/13 | 4.26/10.1/13 |
| normal_contact_contingency | 173 | 9.20/14.0/14 | 8.04/13.0/14 |
| contact_release_contingency | 27 | 7.74/9.7/10 | 6.41/8.7/9 |
| precontact_transition | 172 | 8.71/14.0/14 | 7.70/13.3/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.10/6.0/8 | 9.31/29.0/40 | 1.67/6.0/8 | 386 |
| viability | 1.63/6.0/7 | 9.89/41.0/49 | 1.28/6.0/7 | 390 |
| intent | 1.53/4.0/6 | 8.21/24.0/36 | 1.30/4.0/6 | 471 |
| preference | 1.39/5.0/8 | 13.45/55.0/80 | 0.93/5.0/7 | 359 |
| style | 1.10/3.0/6 | 8.97/24.0/36 | 0.34/3.0/6 | 166 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 400 | 200 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `203` ticks.
Precontact sole-center tangential speed: p50 `3.0343 m/s`, p95 `7.7736 m/s`, max `8.7000 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.337 | 8.335 | 8.335 | 1.000 | 1.000 | 48.578 | 48.812 | 0.234 | 48.812 | 0.001 | 0 | 59 | 0 | 0 | 128 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2479.5 | 2604.4 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2575.1 | 2973.7 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2555.3 | 3054.5 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 3547.8 | 5409.2 | 7.82 | 47.47 | 5.52 | 4.70 | 1047.20 | 6.521 | 0.940 | 7.07e-10 | 4.18e-11 | 12 |
| 240–299 | 3192.9 | 4657.0 | 8.47 | 52.68 | 7.47 | 5.98 | 1328.30 | 13.529 | 28.529 | 9.06e-10 | 2.85e-11 | 60 |
| 300–359 | 3123.7 | 39705.8 | 8.00 | 47.88 | 7.12 | 86.20 | 19136.40 | 22.122 | 46.050 | 1.29e-09 | 1.30e-11 | 60 |
| 360–419 | 15340.7 | 204949.2 | 10.00 | 67.88 | 8.93 | 1462.02 | 324546.90 | 56.550 | 98.866 | 2.02e-09 | 5.68e-11 | 60 |
| 420–479 | 3023.4 | 74347.0 | 8.32 | 52.82 | 7.20 | 306.97 | 66304.80 | 143.955 | 160.805 | 7.96e-10 | 1.64e-11 | 60 |
| 480–539 | 5980.1 | 102277.9 | 8.83 | 57.00 | 7.62 | 386.08 | 83390.10 | 244.258 | 227.815 | 5.51e-10 | 2.24e-11 | 60 |
| 540–599 | 4805.5 | 197477.3 | 9.58 | 61.93 | 8.28 | 143.03 | 30888.20 | 325.830 | 323.766 | 2.84e-09 | 1.39e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 138.009 | 139.747 | 141.600 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
