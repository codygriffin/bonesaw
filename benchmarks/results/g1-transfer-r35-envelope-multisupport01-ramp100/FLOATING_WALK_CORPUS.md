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
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `multi-support` measured-phase policy slewed over `100` ticks.
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
| `no_infeasible_or_failed_ticks` | FAIL |
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

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 433 | 2.165 s | 13.656 cm | 6.930 cm | 33.146 cm | 38.407 cm | 52.745° | 8.000 rad/s | 16779.1 µs |

Nominal hard residual maxima: dynamics `9.570e-09`, contact acceleration `3.762e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 63.161 cm |
| authored reference vs measured CoM RMS / p95 | 59.623 / 132.032 cm |
| stance foot RMS | 40.145 cm |
| swing foot RMS | 83.550 cm |
| hand RMS | 82.817 cm |
| maximum root rotation | 179.106° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.570e-09 |
| contact acceleration residual | 3.762e-10 |
| raw max dynamics residual, including rejected ticks | 1.605e+03 |
| raw max contact residual, including rejected ticks | 3.762e-10 |
| active normal force range | 0.000–746.560 N |
| centroidal momentum-rate residual RMS / max | 60.560 / 234.754 N·m |
| point-task acceleration RMS max | 240.190 m/s² |
| frame-angular acceleration RMS max | 215.101 rad/s² |
| longest pre-contact / touchdown transition | 205 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `28.618` / `48.001 cm`.
- Virtual ZMP clipped on `56.50%` of ticks; clip-distance RMS / max `47.973` / `101.516 cm`.
- Measured-height natural frequency min / p50 / max: `3.661` / `3.752` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.534 m`; height-floor ticks: `137`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `220` ticks; maximum active coordinates `8`; mean phase scale `0.332`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2568.9 µs | 266791.6 µs | 269681.2 µs | 341540.7 µs | 129 | 99 | 205 | 0 | 30 | 48 | 89 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 56270.9 | 99891.6 | 569.1 | 266251.6 | 304266.3 | 337813.3 | 153844.1 | 600 | 161 | 144 | 17.8 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2448.9 | 2707.4 | 2979.9 | 3034.9 |
| solved_with_slack | 99 | 2523.9 | 2756.5 | 4337.9 | 4469.3 |
| primal_infeasible | 89 | 266473.0 | 270297.5 | 286780.3 | 341540.7 |
| normal_contact_contingency | 30 | 4531.2 | 67041.4 | 159965.0 | 183505.4 |
| contact_release_contingency | 48 | 172366.2 | 212282.5 | 213229.8 | 213243.2 |
| precontact_transition | 205 | 2408.9 | 8635.3 | 32596.8 | 74533.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.43 | 12.0 | 15.0 | 25 | 4.29 | 14.0 | 22 | -0.6514 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1028.42/6720.0/6720.0/6720 | 216276.95/1411200.0/1411200.0/1441800 | 5.50/31.0/31 | 5.16/30.0/30 | 45.52/267.0/267 | 0.8794 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 7.09/24.0/25 | 4.14/21.0/22 |
| primal_infeasible | 89 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 30 | 10.17/15.0/15 | 9.27/15.0/15 |
| contact_release_contingency | 48 | 7.77/10.0/10 | 6.52/9.0/9 |
| precontact_transition | 205 | 8.94/14.0/15 | 7.66/14.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.50/5.0/7 | 6.19/21.0/30 | 0.97/5.0/7 | 209 |
| viability | 1.56/8.0/10 | 9.41/49.0/62 | 1.18/8.0/10 | 292 |
| intent | 1.36/5.0/6 | 6.97/24.0/36 | 1.12/5.0/6 | 376 |
| preference | 1.11/5.0/20 | 10.17/53.1/140 | 0.72/5.0/20 | 287 |
| style | 0.91/2.0/3 | 7.52/20.0/26 | 0.29/2.0/3 | 156 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 433 | 167 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `170` ticks.
Precontact sole-center tangential speed: p50 `3.2680 m/s`, p95 `11.8323 m/s`, max `11.8323 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 33.763 | 33.754 | 33.754 | 1.000 | 1.000 | 48.516 | 48.750 | 0.234 | 48.895 | 0.001 | 0 | 59 | 0 | 0 | 464 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2434.9 | 2898.1 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2524.2 | 3623.0 | 5.67 | 38.40 | 1.25 | 1.00 | 234.00 | 0.116 | 0.000 | 1.18e-09 | 3.96e-11 | 0 |
| 120–179 | 2502.5 | 3420.7 | 6.20 | 41.12 | 2.28 | 1.00 | 234.00 | 1.297 | 0.000 | 1.38e-09 | 5.14e-11 | 0 |
| 180–239 | 2400.4 | 3539.7 | 7.40 | 47.22 | 4.85 | 1.00 | 225.80 | 6.565 | 1.134 | 8.11e-10 | 4.28e-11 | 12 |
| 240–299 | 2413.1 | 3759.0 | 8.40 | 53.58 | 6.70 | 1.00 | 222.00 | 5.746 | 20.515 | 8.52e-10 | 1.46e-11 | 60 |
| 300–359 | 2243.9 | 3520.6 | 9.12 | 56.32 | 7.63 | 4.27 | 947.20 | 8.550 | 26.759 | 1.19e-09 | 1.37e-11 | 60 |
| 360–419 | 2000.9 | 70937.0 | 9.10 | 55.40 | 8.30 | 153.98 | 34184.30 | 28.059 | 25.604 | 1.01e-09 | 1.89e-11 | 60 |
| 420–479 | 100280.9 | 213018.3 | 8.90 | 51.85 | 7.77 | 148.80 | 32329.00 | 70.246 | 60.168 | 9.57e-09 | 3.76e-10 | 60 |
| 480–539 | 212048.5 | 270637.9 | 4.52 | 25.20 | 4.07 | 3252.13 | 682959.20 | 126.728 | 109.311 | 1.61e+03 | 5.95e-11 | 60 |
| 540–599 | 266531.7 | 304826.3 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 134.015 | 128.512 | 1.61e+03 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 63.161 | 58.205 | 82.817 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
