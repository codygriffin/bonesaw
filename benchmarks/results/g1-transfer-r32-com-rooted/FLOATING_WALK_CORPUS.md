# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `rooted` reference at `viability` priority with weight `0.100` and `2.000 Hz` response. Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 344 | 1.720 s | 13.937 cm | 0.703 cm | 25.281 cm | 27.293 cm | 55.879° | 8.000 rad/s | 14112.6 µs |

Nominal hard residual maxima: dynamics `7.947e-09`, contact acceleration `2.655e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 106.543 cm |
| CoM RMS / p95 | 101.255 / 199.768 cm |
| stance foot RMS | 70.082 cm |
| swing foot RMS | 79.023 cm |
| hand RMS | 125.118 cm |
| maximum root rotation | 179.829° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.600e-09 |
| contact acceleration residual | 2.655e-10 |
| raw max dynamics residual, including rejected ticks | 8.600e-09 |
| raw max contact residual, including rejected ticks | 2.655e-10 |
| active normal force range | 0.000–757.219 N |
| centroidal momentum-rate residual RMS / max | 67.250 / 321.209 N·m |
| point-task acceleration RMS max | 152.596 m/s² |
| frame-angular acceleration RMS max | 248.171 rad/s² |
| longest pre-contact / touchdown transition | 116 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2804.9 µs | 99888.0 µs | 190450.4 µs | 211308.1 µs | 125 | 103 | 116 | 0 | 228 | 28 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10920.1 | 30939.8 | 543.1 | 13369.1 | 208610.6 | 211038.4 | 99936.2 | 600 | 108 | 38 | 91.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 125 | 2409.4 | 2535.4 | 2578.0 | 3038.8 |
| solved_with_slack | 103 | 3099.3 | 4615.0 | 5888.4 | 6193.6 |
| normal_contact_contingency | 228 | 2969.9 | 16881.6 | 99567.4 | 190422.9 |
| contact_release_contingency | 28 | 103080.2 | 205749.1 | 210092.2 | 211308.1 |
| precontact_transition | 116 | 2292.8 | 12802.9 | 25438.9 | 28779.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.73 | 11.0 | 13.0 | 23 | 5.43 | 13.0 | 18 | 0.1133 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 74.87/8.0/1601.9/6716 | 16198.62/1776.0/355285.4/1450656 | 1.84/17.0/24 | 1.46/16.0/23 | 12.13/136.0/203 | 0.2729 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 125 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 103 | 7.44/11.0/23 | 3.88/10.0/18 |
| normal_contact_contingency | 228 | 8.85/14.0/15 | 7.81/13.0/14 |
| contact_release_contingency | 28 | 7.71/10.0/10 | 6.39/8.7/9 |
| precontact_transition | 116 | 8.76/12.8/13 | 7.74/12.8/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.18/5.0/7 | 9.31/25.0/30 | 1.76/5.0/7 | 365 |
| viability | 1.66/6.0/8 | 10.66/39.0/56 | 1.33/6.0/8 | 401 |
| intent | 1.27/4.0/6 | 6.30/20.0/36 | 0.93/4.0/6 | 397 |
| preference | 1.45/7.0/19 | 14.71/77.0/228 | 1.01/7.0/18 | 346 |
| style | 1.18/4.0/7 | 10.29/36.0/50 | 0.40/4.0/6 | 166 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 344 | 256 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `259` ticks.
Precontact sole-center tangential speed: p50 `4.0360 m/s`, p95 `7.3156 m/s`, max `12.3519 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.552 | 6.550 | 6.550 | 1.000 | 1.000 | 47.680 | 47.910 | 0.230 | 47.910 | 0.001 | 0 | 43 | 0 | 0 | 119 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2955.1 | 4787.4 | 6.47 | 50.70 | 2.27 | 1.00 | 234.00 | 0.002 | 0.000 | 1.22e-09 | 3.75e-11 | 0 |
| 60–119 | 2511.6 | 6020.3 | 6.25 | 48.10 | 1.72 | 1.00 | 234.00 | 0.004 | 0.000 | 1.67e-09 | 3.76e-11 | 0 |
| 120–179 | 2405.6 | 2550.8 | 5.00 | 34.10 | 0.00 | 1.00 | 234.00 | 0.002 | 0.000 | 6.91e-10 | 4.03e-11 | 0 |
| 180–239 | 2944.2 | 3423.6 | 7.08 | 45.67 | 3.88 | 4.85 | 1080.50 | 0.948 | 2.797 | 7.29e-10 | 1.93e-11 | 12 |
| 240–299 | 1828.8 | 13289.7 | 8.55 | 53.80 | 7.37 | 27.10 | 6016.20 | 12.975 | 7.069 | 1.05e-09 | 1.37e-11 | 60 |
| 300–359 | 3350.1 | 138699.6 | 9.52 | 62.00 | 8.82 | 627.13 | 135492.80 | 41.935 | 32.428 | 7.95e-09 | 2.66e-10 | 60 |
| 360–419 | 14081.0 | 208651.1 | 9.13 | 53.50 | 8.03 | 6.02 | 1295.50 | 106.288 | 58.634 | 3.29e-09 | 1.50e-10 | 60 |
| 420–479 | 4549.7 | 118193.9 | 8.57 | 53.87 | 7.87 | 7.42 | 1598.80 | 169.483 | 96.758 | 8.60e-09 | 2.02e-10 | 60 |
| 480–539 | 2778.2 | 3917.9 | 8.42 | 57.62 | 7.50 | 4.73 | 1022.40 | 164.100 | 99.243 | 6.84e-10 | 2.54e-11 | 60 |
| 540–599 | 2591.8 | 15485.6 | 8.35 | 53.30 | 6.85 | 68.42 | 14778.00 | 211.273 | 172.531 | 3.76e-10 | 2.95e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 106.543 | 73.161 | 125.118 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
