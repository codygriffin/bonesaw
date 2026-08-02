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
- Capture landing retarget: `enabled`; authored offset ≤ `0.080 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `0` scheduled ticks and throughout delayed admission.
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
| 344 | 1.720 s | 8.305 cm | 8.868 cm | 43.922 cm | 33.755 cm | 65.177° | 8.000 rad/s | 89743.0 µs |

Nominal hard residual maxima: dynamics `1.309e-09`, contact acceleration `5.511e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 130.742 cm |
| authored reference vs measured CoM RMS / p95 | 140.178 / 297.273 cm |
| stance foot RMS | 116.570 cm |
| swing foot RMS | 158.035 cm |
| hand RMS | 156.585 cm |
| maximum root rotation | 179.332° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.310e-09 |
| contact acceleration residual | 3.552e-10 |
| raw max dynamics residual, including rejected ticks | 1.277e-08 |
| raw max contact residual, including rejected ticks | 5.909e-10 |
| active normal force range | 0.000–706.675 N |
| centroidal momentum-rate residual RMS / max | 73.336 / 326.071 N·m |
| point-task acceleration RMS max | 126.832 m/s² |
| frame-angular acceleration RMS max | 263.282 rad/s² |
| longest pre-contact / touchdown transition | 106 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 162 / 162 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `127.131` / `250.902 cm`.
- Virtual ZMP clipped on `58.17%` of ticks; clip-distance RMS / max `176.353` / `422.638 cm`.
- Measured-height natural frequency min / p50 / max: `3.693` / `3.830` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.894 m`; height-floor ticks: `228`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-255.286` / `-213.584 cm`; inside on `43.67%` of ticks.

## Capture-aware landing

- Active target-ticks: `362`; policy updates `200`, frozen `162`.
- Maximum applied offset / root reach: `0.0800 / 0.8356 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 29`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `162 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `1.4468 m / 0.3329 / 0.0166 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 22`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `361`, multi-support `209`.
- Joint-velocity envelope active on `347` ticks; maximum active coordinates `8`; mean target/applied scale `0.492` / `0.582`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2824.7 µs | 105611.0 µs | 209674.9 µs | 234467.5 µs | 129 | 109 | 106 | 0 | 163 | 30 | 0 | 63 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 14843.3 | 40678.3 | 637.2 | 14547.3 | 229704.0 | 233991.2 | 180446.2 | 600 | 155 | 49 | 67.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2540.9 | 2694.4 | 3499.2 | 3851.3 |
| solved_with_slack | 109 | 2623.6 | 3682.1 | 4377.2 | 4464.6 |
| failed | 63 | 8068.6 | 8213.1 | 9670.2 | 9758.1 |
| normal_contact_contingency | 163 | 2938.1 | 15182.0 | 19730.6 | 46561.7 |
| contact_release_contingency | 30 | 186004.7 | 221990.9 | 232161.3 | 234467.5 |
| precontact_transition | 106 | 3400.7 | 81361.0 | 106107.6 | 114207.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.95 | 12.0 | 13.0 | 14 | 5.85 | 12.0 | 14 | 0.0500 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 131.17/8.0/4798.3/7100 | 29075.21/1776.0/1065224.8/1576200 | 2.46/15.0/21 | 2.05/14.0/20 | 16.61/121.1/177 | 0.2606 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 109 | 7.49/13.9/14 | 4.63/11.0/12 |
| failed | 63 | 10.19/11.0/11 | 10.19/11.0/11 |
| normal_contact_contingency | 163 | 9.26/13.4/14 | 8.29/13.0/14 |
| contact_release_contingency | 30 | 7.67/10.0/10 | 6.20/9.0/9 |
| precontact_transition | 106 | 8.74/12.0/12 | 7.78/12.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.11/6.0/7 | 9.21/28.0/35 | 1.67/6.0/7 | 352 |
| viability | 1.56/7.0/10 | 9.60/42.0/50 | 1.17/7.0/9 | 371 |
| intent | 1.83/5.0/7 | 9.52/26.0/30 | 1.59/5.0/7 | 465 |
| preference | 1.38/6.0/7 | 11.91/53.0/70 | 0.98/5.0/6 | 382 |
| style | 1.08/3.0/6 | 8.93/23.0/38 | 0.44/3.0/6 | 227 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 362 | 0 | 209 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 344 | 256 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `362` ticks, planned normal touchdown `0` ticks, normal fallback `259` ticks.
Precontact sole-center tangential speed: p50 `2.7767 m/s`, p95 `5.6593 m/s`, max `6.3822 m/s` over 362 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.906 | 8.905 | 8.905 | 1.000 | 1.000 | 48.203 | 48.648 | 0.445 | 49.477 | 0.001 | 0 | 93 | 0 | 0 | 46 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2535.0 | 3707.5 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2623.1 | 2822.5 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2598.0 | 4156.5 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2554.0 | 3958.1 | 8.12 | 49.88 | 5.55 | 1.00 | 227.80 | 6.457 | 0.019 | 9.36e-10 | 4.18e-11 | 2 |
| 240–299 | 3352.8 | 4389.6 | 8.45 | 50.18 | 7.33 | 6.37 | 1413.40 | 6.814 | 18.603 | 9.54e-10 | 1.13e-11 | 60 |
| 300–359 | 12258.3 | 157375.4 | 9.55 | 55.20 | 8.82 | 1225.32 | 272009.90 | 28.648 | 65.329 | 1.03e-09 | 2.14e-11 | 60 |
| 360–419 | 6527.0 | 220395.9 | 8.78 | 54.02 | 7.88 | 7.42 | 1596.40 | 98.027 | 110.129 | 7.09e-09 | 1.06e-10 | 60 |
| 420–479 | 1916.5 | 7039.5 | 8.78 | 53.75 | 7.70 | 3.10 | 669.60 | 162.974 | 150.025 | 8.31e-09 | 3.55e-10 | 60 |
| 480–539 | 3440.1 | 223844.0 | 9.12 | 55.32 | 7.95 | 57.47 | 12405.00 | 234.160 | 235.434 | 1.28e-08 | 5.91e-10 | 60 |
| 540–599 | 8071.3 | 9674.4 | 10.22 | 62.85 | 10.22 | 8.00 | 1728.00 | 281.110 | 279.157 | 1.28e-08 | 5.91e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 130.742 | 131.380 | 156.585 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
