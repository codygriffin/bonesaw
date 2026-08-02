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
- Capture landing retarget: `enabled`; authored offset ≤ `0.120 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
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
| 461 | 2.305 s | 12.679 cm | 8.470 cm | 24.511 cm | 41.732 cm | 57.127° | 8.000 rad/s | 10412.0 µs |

Nominal hard residual maxima: dynamics `7.224e-09`, contact acceleration `3.324e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 61.291 cm |
| authored reference vs measured CoM RMS / p95 | 54.892 / 150.743 cm |
| stance foot RMS | 33.437 cm |
| swing foot RMS | 52.176 cm |
| hand RMS | 84.564 cm |
| maximum root rotation | 179.641° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.751e-09 |
| contact acceleration residual | 3.324e-10 |
| raw max dynamics residual, including rejected ticks | 7.751e-09 |
| raw max contact residual, including rejected ticks | 3.324e-10 |
| active normal force range | 0.000–601.809 N |
| centroidal momentum-rate residual RMS / max | 62.677 / 335.213 N·m |
| point-task acceleration RMS max | 157.194 m/s² |
| frame-angular acceleration RMS max | 170.663 rad/s² |
| longest pre-contact / touchdown transition | 233 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `22.582` / `38.868 cm`.
- Virtual ZMP clipped on `58.50%` of ticks; clip-distance RMS / max `49.730` / `178.924 cm`.
- Measured-height natural frequency min / p50 / max: `3.682` / `3.748` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.824 m`; height-floor ticks: `110`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-129.084` / `-61.834 cm`; inside on `48.33%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`.
- Maximum applied offset / root reach: `0.1200 / 0.8948 m`.
- Authored-offset / reach / slew limited ticks: `372 / 0 / 154`.
- Authored geometry outside configured reach: `0` target-ticks.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `442` ticks; maximum active coordinates `8`; mean target/applied scale `0.650` / `0.730`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2546.1 µs | 77789.6 µs | 109023.7 µs | 196421.4 µs | 129 | 99 | 233 | 0 | 119 | 20 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8802.7 | 24091.9 | 292.3 | 7027.4 | 180070.5 | 194786.3 | 97367.1 | 600 | 77 | 34 | 113.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2476.0 | 2596.2 | 2629.4 | 2661.4 |
| solved_with_slack | 99 | 2561.0 | 2837.8 | 2978.4 | 3110.2 |
| normal_contact_contingency | 119 | 3968.1 | 90524.9 | 101858.0 | 169124.4 |
| contact_release_contingency | 20 | 103529.9 | 159161.7 | 188969.5 | 196421.4 |
| precontact_transition | 233 | 2497.1 | 4008.5 | 11563.9 | 12105.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.57 | 12.0 | 14.0 | 14 | 5.33 | 13.0 | 14 | 0.0842 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 130.19/8.0/5540.6/6739 | 28133.37/1776.0/1196780.4/1455624 | 1.12/14.0/15 | 0.88/13.0/14 | 7.28/112.0/124 | 0.5211 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| normal_contact_contingency | 119 | 8.86/14.0/14 | 7.97/13.0/13 |
| contact_release_contingency | 20 | 7.50/10.6/11 | 5.95/9.4/10 |
| precontact_transition | 233 | 8.68/13.7/14 | 7.53/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.88/6.0/7 | 8.20/27.0/35 | 1.35/6.0/7 | 292 |
| viability | 1.71/6.0/9 | 11.35/44.0/63 | 1.31/6.0/9 | 365 |
| intent | 1.67/6.0/7 | 8.63/29.0/35 | 1.42/6.0/7 | 463 |
| preference | 1.25/5.0/7 | 10.18/40.0/72 | 0.85/5.0/6 | 372 |
| style | 1.05/2.0/5 | 8.89/19.0/37 | 0.39/2.0/5 | 208 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 461 | 139 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `142` ticks.
Precontact sole-center tangential speed: p50 `2.0476 m/s`, p95 `7.4573 m/s`, max `11.2234 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.282 | 5.281 | 5.281 | 1.000 | 1.000 | 46.840 | 47.250 | 0.410 | 48.926 | 0.001 | 0 | 87 | 0 | 0 | 77 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2447.2 | 2587.3 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2548.2 | 2747.6 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2544.1 | 3030.8 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2329.7 | 3089.3 | 7.13 | 46.18 | 4.57 | 1.00 | 225.80 | 6.405 | 0.736 | 7.54e-10 | 4.18e-11 | 12 |
| 240–299 | 2506.1 | 3126.1 | 8.33 | 51.53 | 7.05 | 1.00 | 222.00 | 6.340 | 20.838 | 8.56e-10 | 1.46e-11 | 60 |
| 300–359 | 2362.8 | 4232.4 | 8.12 | 47.25 | 6.57 | 2.40 | 532.80 | 6.260 | 24.227 | 6.02e-10 | 1.43e-11 | 60 |
| 360–419 | 2332.8 | 4877.4 | 8.98 | 51.27 | 8.15 | 2.77 | 614.20 | 18.911 | 14.421 | 7.70e-10 | 1.21e-11 | 60 |
| 420–479 | 3857.9 | 134275.2 | 9.30 | 58.87 | 8.57 | 1202.27 | 259713.40 | 39.811 | 29.394 | 7.22e-09 | 3.32e-10 | 60 |
| 480–539 | 5811.4 | 173281.2 | 9.07 | 53.23 | 7.88 | 82.77 | 17875.10 | 99.146 | 61.270 | 5.80e-09 | 1.61e-10 | 60 |
| 540–599 | 3273.2 | 114260.8 | 8.30 | 53.72 | 7.43 | 6.72 | 1448.40 | 160.223 | 103.153 | 7.75e-09 | 6.75e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 61.291 | 40.606 | 84.564 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
