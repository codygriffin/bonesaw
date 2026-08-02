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
- Coupled touchdown phase retiming: `enabled`; minimum rate `0.000`, guard `0.020 s`, engagement / release `0 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
- Coupled balance phase retiming: `enabled`; hold at signed DCM margin ≤ `-0.020 m`, recover nominal rate at `0.000 m`.
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
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `retiming_reaches_first_authored_touchdown` | FAIL |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `retiming_reaches_first_authored_touchdown`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 268 | 1.340 s | 4.152 cm | 0.353 cm | 12.810 cm | 31.503 cm | 0.470° | 7.000 rad/s | 3278.2 µs |

Nominal hard residual maxima: dynamics `1.353e-09`, contact acceleration `5.554e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 67.884 cm |
| authored reference vs measured CoM RMS / p95 | 64.160 / 172.737 cm |
| stance foot RMS | 54.066 cm |
| swing foot RMS | 81.026 cm |
| hand RMS | 84.402 cm |
| maximum root rotation | 137.574° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.370e-09 |
| contact acceleration residual | 2.429e-10 |
| raw max dynamics residual, including rejected ticks | 7.370e-09 |
| raw max contact residual, including rejected ticks | 2.429e-10 |
| active normal force range | 0.000–910.610 N |
| centroidal momentum-rate residual RMS / max | 53.710 / 369.648 N·m |
| point-task acceleration RMS max | 133.765 m/s² |
| frame-angular acceleration RMS max | 278.290 rad/s² |
| longest pre-contact / touchdown transition | 178 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `61.177` / `146.671 cm`.
- Virtual ZMP clipped on `60.67%` of ticks; clip-distance RMS / max `89.245` / `312.057 cm`.
- Measured-height natural frequency min / p50 / max: `3.686` / `3.769` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.732 m`; height-floor ticks: `91`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-204.256` / `-135.772 cm`; inside on `48.33%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True` (touchdown `True`, balance `True`); final source tick `296.926`, progress `296.926` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `False`.
- Target / applied minimum rate: `0.0000` / `0.0000`; mean / p50 applied `0.4965` / `0.5449`.
- Limited / zero-rate hold ticks: `310` / `268`; maximum required landing time `0.9795 s`.
- Position / tangential / normal limiting ticks: `372` / `0` / `0`; unsafe-edge ticks `0`.
- Balance-margin limited ticks: `310`.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `372`, frozen `0`.
- Maximum applied offset / root reach: `0.0800 / 0.8153 m`.
- Authored-offset / reach / slew limited ticks: `372 / 0 / 23`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `443` ticks; maximum active coordinates `8`; mean target/applied scale `0.650` / `0.734`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2535.0 µs | 11912.3 µs | 40329.1 µs | 203546.5 µs | 126 | 102 | 218 | 0 | 148 | 5 | 0 | 1 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5205.4 | 14482.9 | 272.8 | 4490.8 | 175252.1 | 200717.1 | 95885.8 | 600 | 47 | 26 | 192.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 126 | 2468.0 | 2593.3 | 2650.1 | 2671.9 |
| solved_with_slack | 102 | 2543.3 | 2817.1 | 4016.7 | 4488.2 |
| failed | 1 | 2483.3 | 2483.3 | 2483.3 | 2483.3 |
| normal_contact_contingency | 148 | 3334.2 | 36804.3 | 39059.0 | 203546.5 |
| contact_release_contingency | 5 | 111011.1 | 148350.9 | 154718.6 | 156310.5 |
| precontact_transition | 218 | 2577.4 | 3918.7 | 4662.3 | 4812.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.69 | 12.0 | 13.0 | 25 | 5.44 | 13.0 | 21 | 0.1055 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 68.86/8.0/2258.4/2368 | 14886.45/1776.0/487823.0/511488 | 0.93/13.0/22 | 0.70/12.0/21 | 5.73/105.0/182 | 0.3675 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 126 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 102 | 7.22/14.0/25 | 4.25/11.0/21 |
| failed | 1 | 10.00/10.0/10 | 8.00/8.0/8 |
| normal_contact_contingency | 148 | 9.11/13.5/14 | 8.08/13.0/13 |
| contact_release_contingency | 5 | 7.00/8.9/9 | 5.60/7.0/7 |
| precontact_transition | 218 | 8.51/13.0/14 | 7.33/12.8/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.90/6.0/8 | 8.33/30.0/40 | 1.36/6.0/8 | 294 |
| viability | 1.81/7.0/9 | 11.26/42.0/63 | 1.44/7.0/9 | 389 |
| intent | 1.56/6.0/8 | 8.49/30.0/38 | 1.34/6.0/8 | 473 |
| preference | 1.37/6.0/20 | 11.01/49.0/140 | 0.94/6.0/19 | 369 |
| style | 1.05/2.0/4 | 9.17/20.0/32 | 0.35/2.0/3 | 194 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 447 | 153 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `156` ticks.
Precontact sole-center tangential speed: p50 `1.5281 m/s`, p95 `8.4529 m/s`, max `11.3676 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.124 | 3.123 | 3.123 | 1.000 | 1.000 | 49.461 | 50.000 | 0.539 | 50.742 | 0.001 | 0 | 118 | 0 | 0 | 50 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2447.0 | 2611.9 | 5.00 | 33.53 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| 60–119 | 2535.2 | 3229.8 | 5.47 | 37.28 | 1.05 | 1.00 | 234.00 | 0.151 | 0.000 | 1.27e-09 | 3.93e-11 | 0 |
| 120–179 | 2521.0 | 2816.2 | 5.98 | 39.17 | 2.03 | 1.00 | 234.00 | 1.369 | 0.000 | 1.03e-09 | 5.03e-11 | 0 |
| 180–239 | 2493.5 | 3615.6 | 8.05 | 51.20 | 5.53 | 1.00 | 225.80 | 6.458 | 1.270 | 1.35e-09 | 5.23e-11 | 12 |
| 240–299 | 2556.7 | 3321.8 | 8.35 | 53.92 | 6.55 | 1.00 | 222.00 | 8.152 | 21.598 | 9.05e-10 | 2.63e-11 | 60 |
| 300–359 | 2326.9 | 4558.3 | 8.62 | 52.08 | 7.55 | 1.93 | 429.20 | 10.834 | 24.618 | 6.64e-10 | 2.07e-11 | 60 |
| 360–419 | 2468.5 | 3891.5 | 8.68 | 49.97 | 7.95 | 3.45 | 765.90 | 22.210 | 34.480 | 1.14e-09 | 1.37e-11 | 60 |
| 420–479 | 4039.4 | 106881.4 | 8.38 | 49.18 | 7.33 | 371.53 | 80272.80 | 50.486 | 63.753 | 4.14e-09 | 1.56e-10 | 60 |
| 480–539 | 2175.9 | 37197.4 | 9.67 | 60.92 | 8.48 | 299.30 | 64648.80 | 103.965 | 113.308 | 1.03e-09 | 3.87e-12 | 60 |
| 540–599 | 4602.8 | 132829.6 | 8.72 | 55.40 | 7.88 | 7.42 | 1598.00 | 178.896 | 149.116 | 7.37e-09 | 2.43e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 67.884 | 64.344 | 84.402 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
