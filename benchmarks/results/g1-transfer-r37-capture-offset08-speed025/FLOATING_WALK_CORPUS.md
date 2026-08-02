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
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `enabled`; authored offset ≤ `0.080 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.250 m/s`.
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
| 470 | 2.350 s | 13.003 cm | 10.347 cm | 24.039 cm | 41.812 cm | 65.350° | 8.000 rad/s | 10807.2 µs |

Nominal hard residual maxima: dynamics `1.942e-09`, contact acceleration `5.511e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 61.614 cm |
| authored reference vs measured CoM RMS / p95 | 56.193 / 148.573 cm |
| stance foot RMS | 39.866 cm |
| swing foot RMS | 37.248 cm |
| hand RMS | 85.370 cm |
| maximum root rotation | 179.752° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.211e-09 |
| contact acceleration residual | 2.525e-10 |
| raw max dynamics residual, including rejected ticks | 9.211e-09 |
| raw max contact residual, including rejected ticks | 2.525e-10 |
| active normal force range | 0.000–751.035 N |
| centroidal momentum-rate residual RMS / max | 79.367 / 466.491 N·m |
| point-task acceleration RMS max | 123.772 m/s² |
| frame-angular acceleration RMS max | 269.764 rad/s² |
| longest pre-contact / touchdown transition | 242 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `20.240` / `39.276 cm`.
- Virtual ZMP clipped on `58.17%` of ticks; clip-distance RMS / max `41.254` / `82.632 cm`.
- Measured-height natural frequency min / p50 / max: `3.691` / `3.734` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.782 m`; height-floor ticks: `109`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-64.884` / `-61.208 cm`; inside on `48.33%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`.
- Maximum applied offset / root reach: `0.0800 / 0.8709 m`.
- Authored-offset / reach / slew limited ticks: `372 / 0 / 189`.
- Authored geometry outside configured reach: `0` target-ticks.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `454` ticks; maximum active coordinates `8`; mean target/applied scale `0.655` / `0.750`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2657.0 µs | 10750.9 µs | 107885.9 µs | 207700.9 µs | 129 | 99 | 242 | 0 | 125 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5633.1 | 17925.7 | 409.0 | 4507.4 | 197789.6 | 206709.7 | 104564.9 | 600 | 47 | 13 | 177.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2572.2 | 3306.6 | 3816.5 | 3998.9 |
| solved_with_slack | 99 | 2633.4 | 3026.5 | 4151.9 | 4347.0 |
| normal_contact_contingency | 125 | 3929.3 | 22123.6 | 92982.7 | 122087.3 |
| contact_release_contingency | 5 | 185461.4 | 204391.6 | 207039.0 | 207700.9 |
| precontact_transition | 242 | 2653.5 | 4495.6 | 11332.3 | 116639.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.57 | 12.0 | 13.0 | 15 | 5.30 | 12.0 | 15 | 0.1344 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 58.29/8.0/51.8/6922 | 12672.60/1776.0/11234.2/1536684 | 1.18/14.0/20 | 0.89/13.0/19 | 7.35/111.0/170 | 0.4825 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| normal_contact_contingency | 125 | 8.92/13.0/15 | 7.97/13.0/15 |
| contact_release_contingency | 5 | 7.80/9.0/9 | 6.60/8.0/8 |
| precontact_transition | 242 | 8.57/13.0/13 | 7.36/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.89/5.0/8 | 8.29/25.0/33 | 1.34/5.0/8 | 288 |
| viability | 1.68/6.0/8 | 10.91/42.0/66 | 1.28/6.0/8 | 367 |
| intent | 1.61/5.0/6 | 8.47/24.0/30 | 1.38/5.0/6 | 470 |
| preference | 1.33/5.0/6 | 10.76/43.0/59 | 0.87/5.0/6 | 345 |
| style | 1.06/3.0/7 | 9.24/21.0/45 | 0.42/2.0/7 | 224 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 470 | 130 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `133` ticks.
Precontact sole-center tangential speed: p50 `2.2163 m/s`, p95 `6.6722 m/s`, max `8.3339 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.380 | 3.380 | 3.380 | 1.000 | 1.000 | 47.859 | 48.070 | 0.211 | 48.938 | 0.001 | 0 | 38 | 0 | 0 | 14 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2550.9 | 2976.0 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2657.6 | 3944.0 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2622.6 | 4229.5 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2373.9 | 3156.6 | 7.13 | 46.18 | 4.57 | 1.00 | 225.80 | 6.405 | 0.736 | 7.54e-10 | 4.18e-11 | 12 |
| 240–299 | 2574.7 | 3403.4 | 8.30 | 50.87 | 7.03 | 1.00 | 222.00 | 6.440 | 20.801 | 9.62e-10 | 1.00e-11 | 60 |
| 300–359 | 2262.4 | 2996.1 | 8.13 | 46.58 | 6.15 | 1.00 | 222.00 | 6.990 | 25.088 | 8.75e-10 | 1.20e-11 | 60 |
| 360–419 | 2905.1 | 4726.3 | 8.78 | 53.10 | 7.98 | 3.68 | 817.70 | 19.785 | 14.390 | 7.14e-10 | 9.35e-12 | 60 |
| 420–479 | 3941.2 | 118873.1 | 9.62 | 60.25 | 9.02 | 486.78 | 105874.50 | 34.285 | 29.849 | 1.94e-09 | 4.89e-11 | 60 |
| 480–539 | 3237.5 | 194579.6 | 8.82 | 56.18 | 7.57 | 6.13 | 1321.60 | 103.283 | 54.847 | 1.52e-09 | 1.75e-11 | 60 |
| 540–599 | 4125.2 | 120189.1 | 8.47 | 52.93 | 7.68 | 80.28 | 17340.40 | 159.985 | 100.280 | 9.21e-09 | 2.53e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 61.614 | 39.019 | 85.370 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
