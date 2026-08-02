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
- Capture landing retarget: `enabled`; authored offset ≤ `0.160 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
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
| 460 | 2.300 s | 14.185 cm | 10.157 cm | 25.945 cm | 44.962 cm | 43.962° | 8.000 rad/s | 8351.5 µs |

Nominal hard residual maxima: dynamics `2.215e-09`, contact acceleration `6.709e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 63.754 cm |
| authored reference vs measured CoM RMS / p95 | 55.433 / 151.915 cm |
| stance foot RMS | 36.667 cm |
| swing foot RMS | 52.956 cm |
| hand RMS | 82.530 cm |
| maximum root rotation | 179.704° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.671e-09 |
| contact acceleration residual | 2.505e-10 |
| raw max dynamics residual, including rejected ticks | 7.671e-09 |
| raw max contact residual, including rejected ticks | 2.505e-10 |
| active normal force range | 0.000–604.677 N |
| centroidal momentum-rate residual RMS / max | 63.661 / 389.470 N·m |
| point-task acceleration RMS max | 202.231 m/s² |
| frame-angular acceleration RMS max | 281.682 rad/s² |
| longest pre-contact / touchdown transition | 232 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `31.333` / `61.912 cm`.
- Virtual ZMP clipped on `58.67%` of ticks; clip-distance RMS / max `63.334` / `131.793 cm`.
- Measured-height natural frequency min / p50 / max: `3.650` / `3.734` / `7.004 rad/s`.
- Minimum measured CoM height: `-1.024 m`; height-floor ticks: `94`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-94.512` / `-91.086 cm`; inside on `48.33%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`.
- Maximum applied offset / root reach: `0.1600 / 0.9000 m`.
- Authored-offset / reach / slew limited ticks: `372 / 61 / 180`.
- Authored geometry outside configured reach: `0` target-ticks.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `454` ticks; maximum active coordinates `8`; mean target/applied scale `0.656` / `0.749`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2694.3 µs | 25020.0 µs | 211995.1 µs | 230375.7 µs | 129 | 99 | 232 | 0 | 112 | 28 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12285.4 | 37715.2 | 460.5 | 14247.4 | 224753.8 | 229813.5 | 98671.6 | 600 | 116 | 33 | 81.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2572.3 | 2717.2 | 3121.9 | 3209.0 |
| solved_with_slack | 99 | 2637.7 | 3446.5 | 4191.5 | 4428.0 |
| normal_contact_contingency | 112 | 6078.1 | 17654.3 | 25395.5 | 116048.8 |
| contact_release_contingency | 28 | 179037.7 | 220044.7 | 227841.6 | 230375.7 |
| precontact_transition | 232 | 2721.5 | 6502.2 | 8736.3 | 17130.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.80 | 12.0 | 13.0 | 15 | 5.64 | 13.0 | 15 | 0.0519 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 3.33/8.0/8.0/8 | 731.10/1776.0/1776.0/1776 | 2.08/16.0/22 | 1.75/15.0/21 | 14.74/133.0/185 | 0.3084 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.78/9.0/9 | 3.78/6.0/7 |
| normal_contact_contingency | 112 | 9.79/14.9/15 | 9.17/14.0/15 |
| contact_release_contingency | 28 | 7.82/10.7/11 | 6.61/9.7/10 |
| precontact_transition | 232 | 8.83/13.0/15 | 7.75/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.97/6.0/7 | 8.45/25.0/30 | 1.44/6.0/7 | 290 |
| viability | 1.76/7.0/7 | 11.39/46.0/56 | 1.36/6.0/7 | 368 |
| intent | 1.59/5.0/8 | 8.30/25.0/40 | 1.36/5.0/8 | 471 |
| preference | 1.37/6.0/7 | 11.35/50.0/67 | 1.02/5.0/7 | 401 |
| style | 1.12/4.0/6 | 9.36/27.0/43 | 0.46/4.0/6 | 219 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 460 | 140 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `143` ticks.
Precontact sole-center tangential speed: p50 `2.1576 m/s`, p95 `10.6259 m/s`, max `13.8324 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.371 | 7.367 | 7.367 | 0.999 | 0.999 | 46.930 | 47.148 | 0.219 | 48.922 | 0.001 | 0 | 38 | 0 | 0 | 50 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2551.4 | 3163.3 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2617.0 | 3984.1 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2638.5 | 4157.9 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2409.5 | 3192.8 | 7.13 | 46.18 | 4.57 | 1.00 | 225.80 | 6.405 | 0.736 | 7.54e-10 | 4.18e-11 | 12 |
| 240–299 | 2581.5 | 3310.4 | 8.18 | 50.17 | 6.93 | 1.00 | 222.00 | 6.421 | 20.860 | 8.05e-10 | 1.55e-11 | 60 |
| 300–359 | 2542.3 | 3816.9 | 8.97 | 54.12 | 7.57 | 1.35 | 299.70 | 7.276 | 26.934 | 5.70e-10 | 1.19e-11 | 60 |
| 360–419 | 2649.9 | 6567.2 | 8.90 | 54.42 | 8.10 | 3.57 | 791.80 | 20.590 | 14.917 | 9.41e-10 | 9.99e-12 | 60 |
| 420–479 | 4512.3 | 140189.7 | 9.82 | 60.80 | 9.12 | 7.77 | 1708.80 | 44.006 | 32.049 | 2.21e-09 | 8.90e-11 | 60 |
| 480–539 | 17684.2 | 224838.3 | 9.78 | 56.62 | 9.07 | 7.65 | 1634.50 | 97.912 | 58.931 | 6.71e-09 | 2.01e-10 | 60 |
| 540–599 | 3872.9 | 104679.5 | 8.73 | 55.70 | 8.00 | 8.00 | 1726.40 | 169.002 | 111.313 | 7.67e-09 | 2.51e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 63.754 | 42.749 | 82.530 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
