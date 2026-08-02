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
| 348 | 1.740 s | 10.421 cm | 11.628 cm | 25.153 cm | 32.480 cm | 69.580° | 8.000 rad/s | 68340.8 µs |

Nominal hard residual maxima: dynamics `1.401e-09`, contact acceleration `6.165e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 145.431 cm |
| authored reference vs measured CoM RMS / p95 | 148.184 / 312.277 cm |
| stance foot RMS | 125.222 cm |
| swing foot RMS | 159.249 cm |
| hand RMS | 146.057 cm |
| maximum root rotation | 179.854° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.362e-09 |
| contact acceleration residual | 3.672e-10 |
| raw max dynamics residual, including rejected ticks | 4.362e-09 |
| raw max contact residual, including rejected ticks | 3.672e-10 |
| active normal force range | 0.000–552.998 N |
| centroidal momentum-rate residual RMS / max | 64.901 / 357.946 N·m |
| point-task acceleration RMS max | 126.803 m/s² |
| frame-angular acceleration RMS max | 265.051 rad/s² |
| longest pre-contact / touchdown transition | 110 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 162 / 162 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `149.503` / `345.448 cm`.
- Virtual ZMP clipped on `57.67%` of ticks; clip-distance RMS / max `206.226` / `568.329 cm`.
- Measured-height natural frequency min / p50 / max: `3.643` / `3.806` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.815 m`; height-floor ticks: `231`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2864.8 µs | 59934.0 µs | 170213.9 µs | 211216.6 µs | 103 | 135 | 110 | 0 | 236 | 16 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9131.2 | 26109.3 | 469.7 | 7731.5 | 209141.4 | 211009.0 | 92169.1 | 600 | 76 | 37 | 109.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 103 | 2420.1 | 2528.8 | 3405.7 | 3531.7 |
| solved_with_slack | 135 | 2582.7 | 4761.4 | 5595.5 | 6172.3 |
| normal_contact_contingency | 236 | 2918.7 | 9849.1 | 67424.9 | 98816.8 |
| contact_release_contingency | 16 | 104993.6 | 208618.2 | 210696.9 | 211216.6 |
| precontact_transition | 110 | 3648.5 | 64200.9 | 100027.9 | 108707.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.76 | 12.0 | 13.0 | 15 | 5.71 | 12.0 | 14 | 0.1481 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 154.03/8.0/4308.1/7043 | 33965.26/1872.0/956092.1/1563546 | 1.73/11.0/25 | 1.20/10.0/24 | 9.88/83.0/209 | 0.4493 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 103 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 135 | 7.00/10.7/12 | 4.61/9.0/10 |
| normal_contact_contingency | 236 | 8.93/13.0/15 | 7.92/13.0/14 |
| contact_release_contingency | 16 | 7.94/9.0/9 | 6.62/8.0/8 |
| precontact_transition | 110 | 8.73/13.0/14 | 7.52/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.17/7.0/8 | 9.72/28.0/40 | 1.75/7.0/8 | 377 |
| viability | 1.66/6.0/7 | 9.14/36.0/45 | 1.28/6.0/7 | 401 |
| intent | 1.57/4.0/6 | 8.31/24.0/36 | 1.40/4.0/6 | 496 |
| preference | 1.31/5.0/7 | 13.06/51.0/73 | 0.93/5.0/7 | 391 |
| style | 1.04/2.0/4 | 9.14/20.0/30 | 0.34/2.0/3 | 193 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 362 | 0 | 209 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 348 | 252 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `362` ticks, planned normal touchdown `0` ticks, normal fallback `255` ticks.
Precontact sole-center tangential speed: p50 `2.9931 m/s`, p95 `9.5794 m/s`, max `11.5298 m/s` over 362 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.479 | 5.478 | 5.478 | 1.000 | 1.000 | 48.211 | 48.664 | 0.453 | 48.664 | 0.001 | 0 | 115 | 0 | 0 | 78 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2419.0 | 3459.0 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 4.72e-11 | 0 |
| 60–119 | 2446.1 | 2644.8 | 5.52 | 37.37 | 1.22 | 1.00 | 234.00 | 0.052 | 0.000 | 1.40e-09 | 6.17e-11 | 0 |
| 120–179 | 2659.0 | 5892.5 | 7.18 | 44.13 | 5.27 | 3.68 | 861.90 | 1.018 | 0.000 | 1.06e-09 | 3.95e-11 | 0 |
| 180–239 | 2435.7 | 4709.9 | 6.83 | 41.93 | 3.95 | 3.10 | 698.20 | 4.117 | 0.029 | 1.07e-09 | 4.02e-11 | 2 |
| 240–299 | 3605.1 | 4165.0 | 8.12 | 48.70 | 6.72 | 7.18 | 1594.70 | 8.872 | 7.250 | 4.48e-10 | 1.15e-11 | 60 |
| 300–359 | 4639.2 | 149315.8 | 10.00 | 63.40 | 9.07 | 1130.35 | 250929.30 | 31.300 | 50.335 | 4.47e-10 | 1.83e-11 | 60 |
| 360–419 | 7766.1 | 204163.7 | 9.63 | 61.50 | 8.82 | 373.75 | 80722.90 | 101.349 | 115.676 | 3.58e-09 | 3.67e-10 | 60 |
| 420–479 | 2752.1 | 3638.9 | 8.52 | 54.67 | 7.40 | 6.25 | 1350.00 | 196.859 | 162.495 | 6.78e-10 | 8.84e-12 | 60 |
| 480–539 | 2891.4 | 4146.4 | 8.40 | 54.05 | 7.45 | 6.83 | 1476.00 | 256.641 | 219.160 | 4.36e-09 | 1.62e-10 | 60 |
| 540–599 | 2903.1 | 3938.1 | 8.37 | 54.45 | 7.17 | 7.18 | 1551.60 | 309.088 | 312.441 | 1.35e-09 | 3.24e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 145.431 | 137.121 | 146.057 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
