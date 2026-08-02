# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response. Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 280 | 1.400 s | 21.451 cm | 15.015 cm | 71.277 cm | 37.771 cm | 83.867° | 8.000 rad/s | 75057.4 µs |

Nominal hard residual maxima: dynamics `1.212e-09`, contact acceleration `4.690e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 265.807 cm |
| stance foot RMS | 238.833 cm |
| swing foot RMS | 323.794 cm |
| hand RMS | 262.578 cm |
| maximum root rotation | 174.100° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.638e-09 |
| contact acceleration residual | 2.364e-10 |
| raw max dynamics residual, including rejected ticks | 6.638e-09 |
| raw max contact residual, including rejected ticks | 2.364e-10 |
| active normal force range | 0.000–659.405 N |
| centroidal momentum-rate residual RMS / max | 58.295 / 247.981 N·m |
| point-task acceleration RMS max | 145.288 m/s² |
| frame-angular acceleration RMS max | 222.121 rad/s² |
| longest pre-contact / touchdown transition | 52 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 3093.7 µs | 37811.1 µs | 102850.3 µs | 207495.0 µs | 0 | 228 | 52 | 0 | 306 | 14 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8708.0 | 23332.8 | 1074.4 | 6634.7 | 205458.7 | 207291.4 | 94549.2 | 600 | 86 | 43 | 114.8 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved_with_slack | 228 | 3284.9 | 5305.0 | 6539.9 | 8124.7 |
| normal_contact_contingency | 306 | 2850.5 | 21965.7 | 50006.9 | 204095.5 |
| contact_release_contingency | 14 | 101844.3 | 194487.6 | 204893.6 | 207495.0 |
| precontact_transition | 52 | 4297.3 | 76697.4 | 94885.2 | 100865.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.82 | 13.0 | 16.0 | 18 | 7.68 | 14.0 | 16 | 0.0111 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 164.83/1062.5/4232.7/6641 | 36079.88/229500.0/939670.5/1434456 | 1.65/9.0/22 | 1.08/8.0/21 | 8.84/63.0/195 | 0.5222 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved_with_slack | 228 | 9.27/16.0/18 | 7.88/15.0/16 |
| normal_contact_contingency | 306 | 8.54/13.0/14 | 7.52/12.0/13 |
| contact_release_contingency | 14 | 7.86/9.0/9 | 6.71/8.0/8 |
| precontact_transition | 52 | 8.73/13.5/14 | 8.04/13.5/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.64/7.0/8 | 11.48/28.0/32 | 2.45/7.0/8 | 528 |
| viability | 2.24/10.0/13 | 13.18/41.0/52 | 2.20/10.0/12 | 589 |
| intent | 1.57/4.0/6 | 8.35/24.0/36 | 1.56/4.0/6 | 591 |
| preference | 1.24/6.0/9 | 11.74/66.0/81 | 0.96/6.0/9 | 439 |
| style | 1.12/3.0/6 | 10.37/32.0/40 | 0.51/3.0/5 | 262 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 280 | 320 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `323` ticks.
Precontact sole-center tangential speed: p50 `4.1937 m/s`, p95 `8.0946 m/s`, max `9.4899 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.225 | 5.224 | 5.224 | 1.000 | 1.000 | 47.684 | 47.906 | 0.223 | 48.004 | 0.001 | 0 | 39 | 0 | 0 | 58 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 4431.9 | 5706.4 | 9.00 | 52.77 | 7.13 | 5.73 | 1341.60 | 0.984 | 0.000 | 1.21e-09 | 4.22e-11 | 0 |
| 60–119 | 4401.1 | 7231.2 | 9.58 | 60.18 | 7.78 | 6.37 | 1489.80 | 5.255 | 0.001 | 6.06e-10 | 2.54e-11 | 0 |
| 120–179 | 2714.0 | 4736.5 | 9.63 | 60.08 | 8.52 | 1.47 | 343.20 | 11.840 | 0.833 | 1.05e-09 | 4.69e-11 | 0 |
| 180–239 | 2986.8 | 4274.5 | 8.80 | 53.90 | 8.17 | 4.38 | 976.90 | 22.874 | 19.444 | 1.17e-09 | 3.95e-11 | 12 |
| 240–299 | 8219.3 | 205489.3 | 8.82 | 53.10 | 8.00 | 1087.25 | 239348.10 | 57.174 | 94.888 | 4.02e-09 | 1.28e-10 | 60 |
| 300–359 | 5908.0 | 102362.5 | 9.15 | 59.58 | 8.23 | 521.57 | 112655.20 | 144.970 | 156.049 | 6.64e-09 | 2.36e-10 | 60 |
| 360–419 | 2829.0 | 3498.3 | 7.97 | 51.10 | 6.87 | 5.78 | 1249.20 | 244.588 | 220.546 | 1.13e-09 | 9.19e-12 | 60 |
| 420–479 | 1779.9 | 3519.7 | 8.27 | 50.83 | 7.12 | 3.33 | 720.00 | 316.094 | 335.689 | 1.03e-09 | 8.25e-12 | 60 |
| 480–539 | 2578.1 | 3905.5 | 8.50 | 54.92 | 7.80 | 4.38 | 946.80 | 433.407 | 442.512 | 1.01e-09 | 6.31e-12 | 60 |
| 540–599 | 2923.8 | 4557.6 | 8.45 | 54.77 | 7.18 | 8.00 | 1728.00 | 577.905 | 581.098 | 1.17e-09 | 2.58e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 265.807 | 269.917 | 262.578 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
