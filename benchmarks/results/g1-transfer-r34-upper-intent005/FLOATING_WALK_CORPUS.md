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
- Protected upper-body posture: `intent` priority with weight `0.050` over 11 waist/arm coordinates.
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
| 446 | 2.230 s | 9.291 cm | 5.571 cm | 29.142 cm | 18.760 cm | 38.211° | 8.000 rad/s | 86030.8 µs |

Nominal hard residual maxima: dynamics `1.483e-09`, contact acceleration `5.520e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 92.794 cm |
| authored reference vs measured CoM RMS / p95 | 87.625 / 246.846 cm |
| stance foot RMS | 47.881 cm |
| swing foot RMS | 89.260 cm |
| hand RMS | 105.725 cm |
| maximum root rotation | 174.714° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.365e-09 |
| contact acceleration residual | 1.085e-10 |
| raw max dynamics residual, including rejected ticks | 9.736e+02 |
| raw max contact residual, including rejected ticks | 1.085e-10 |
| active normal force range | 0.000–780.591 N |
| centroidal momentum-rate residual RMS / max | 42.324 / 220.072 N·m |
| point-task acceleration RMS max | 118.229 m/s² |
| frame-angular acceleration RMS max | 202.882 rad/s² |
| longest pre-contact / touchdown transition | 218 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `36.511` / `82.926 cm`.
- Virtual ZMP clipped on `60.33%` of ticks; clip-distance RMS / max `64.485` / `180.232 cm`.
- Measured-height natural frequency min / p50 / max: `3.716` / `3.769` / `7.004 rad/s`.
- Minimum measured CoM height: `-1.630 m`; height-floor ticks: `134`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2492.0 µs | 251574.4 µs | 253383.1 µs | 268680.2 µs | 199 | 29 | 218 | 0 | 56 | 59 | 39 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 36530.7 | 72937.8 | 513.0 | 161911.4 | 262200.3 | 268032.2 | 101205.2 | 600 | 145 | 140 | 27.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 2511.5 | 2600.3 | 2621.5 | 2637.4 |
| solved_with_slack | 29 | 1959.4 | 3435.0 | 3788.3 | 3888.5 |
| primal_infeasible | 39 | 252228.7 | 256102.3 | 264569.4 | 268680.2 |
| normal_contact_contingency | 56 | 3026.3 | 49255.9 | 100760.3 | 130358.5 |
| contact_release_contingency | 59 | 153435.4 | 207318.7 | 209551.6 | 210860.6 |
| precontact_transition | 218 | 1986.1 | 67973.5 | 88658.1 | 92491.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.68 | 11.0 | 12.0 | 16 | 3.73 | 10.0 | 14 | -0.4314 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 665.16/6720.0/6720.0/6720 | 142083.81/1411200.0/1411200.0/1411200 | 2.56/32.0/32 | 2.38/31.0/31 | 20.23/266.0/266 | 0.7441 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 29 | 6.21/7.0/7 | 2.38/4.0/4 |
| primal_infeasible | 39 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 56 | 8.89/11.5/12 | 7.20/10.0/10 |
| contact_release_contingency | 59 | 7.73/10.4/11 | 5.83/9.4/10 |
| precontact_transition | 218 | 8.63/12.8/16 | 6.52/11.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.82/6.0/7 | 8.07/25.0/35 | 1.32/6.0/7 | 265 |
| viability | 1.60/6.0/9 | 9.17/42.0/63 | 1.18/6.0/9 | 322 |
| intent | 1.33/5.0/8 | 10.02/38.0/60 | 0.95/5.0/7 | 350 |
| preference | 0.94/1.0/2 | 4.61/6.0/12 | 0.03/1.0/2 | 15 |
| style | 1.00/4.0/5 | 7.72/28.0/35 | 0.25/3.0/4 | 126 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 446 | 154 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `157` ticks.
Precontact sole-center tangential speed: p50 `1.8861 m/s`, p95 `5.2771 m/s`, max `5.5501 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 21.919 | 21.914 | 21.913 | 1.000 | 1.000 | 48.250 | 48.484 | 0.234 | 48.484 | 0.001 | 0 | 59 | 0 | 0 | 349 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2533.8 | 2618.5 | 5.00 | 28.75 | 0.00 | 1.00 | 234.00 | 0.267 | 0.000 | 1.32e-09 | 5.52e-11 | 0 |
| 60–119 | 2507.7 | 2584.1 | 5.00 | 29.42 | 0.00 | 1.00 | 234.00 | 1.105 | 0.000 | 1.48e-09 | 5.09e-11 | 0 |
| 120–179 | 2506.7 | 2631.3 | 5.00 | 29.03 | 0.00 | 1.00 | 234.00 | 3.146 | 0.000 | 1.17e-09 | 4.99e-11 | 0 |
| 180–239 | 2022.9 | 3677.3 | 6.10 | 35.35 | 1.93 | 1.55 | 347.90 | 7.833 | 0.230 | 1.45e-09 | 4.65e-11 | 12 |
| 240–299 | 1991.2 | 3009.3 | 8.85 | 54.37 | 6.38 | 1.00 | 222.00 | 6.750 | 17.806 | 1.22e-09 | 2.04e-11 | 60 |
| 300–359 | 1917.7 | 3253.3 | 8.15 | 48.97 | 5.98 | 1.00 | 222.00 | 5.132 | 25.968 | 1.02e-09 | 1.22e-11 | 60 |
| 360–419 | 1951.9 | 48477.9 | 8.77 | 52.70 | 7.22 | 139.77 | 31028.20 | 10.019 | 25.660 | 7.50e-10 | 1.73e-11 | 60 |
| 420–479 | 26172.0 | 108017.1 | 9.15 | 56.22 | 7.37 | 1883.67 | 416291.40 | 54.046 | 31.710 | 1.01e-09 | 1.19e-11 | 60 |
| 480–539 | 101642.0 | 191782.3 | 8.12 | 46.78 | 6.35 | 250.83 | 54156.60 | 152.753 | 82.415 | 6.37e-09 | 1.09e-10 | 60 |
| 540–599 | 251581.0 | 262297.6 | 2.72 | 14.30 | 2.05 | 4370.80 | 917868.00 | 244.147 | 179.590 | 9.74e+02 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 92.794 | 64.576 | 105.725 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
