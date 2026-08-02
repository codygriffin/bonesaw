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
- Coupled touchdown phase retiming: `enabled`; minimum rate `0.500`, guard `0.020 s`, engagement / release `50 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
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
| `retiming_reaches_first_authored_touchdown` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 490 | 2.450 s | 24.576 cm | 1.407 cm | 46.115 cm | 41.352 cm | 82.016° | 8.000 rad/s | 70563.1 µs |

Nominal hard residual maxima: dynamics `5.727e-09`, contact acceleration `2.985e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 57.095 cm |
| authored reference vs measured CoM RMS / p95 | 54.294 / 118.319 cm |
| stance foot RMS | 31.368 cm |
| swing foot RMS | 46.330 cm |
| hand RMS | 79.113 cm |
| maximum root rotation | 179.785° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.776e-09 |
| contact acceleration residual | 4.298e-10 |
| raw max dynamics residual, including rejected ticks | 9.776e-09 |
| raw max contact residual, including rejected ticks | 4.298e-10 |
| active normal force range | 0.000–795.097 N |
| centroidal momentum-rate residual RMS / max | 51.049 / 319.934 N·m |
| point-task acceleration RMS max | 122.703 m/s² |
| frame-angular acceleration RMS max | 378.476 rad/s² |
| longest pre-contact / touchdown transition | 262 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 39 / 39 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `55.245` / `140.604 cm`.
- Virtual ZMP clipped on `59.83%` of ticks; clip-distance RMS / max `81.607` / `337.368 cm`.
- Measured-height natural frequency min / p50 / max: `3.665` / `3.729` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.386 m`; height-floor ticks: `111`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-184.534` / `-109.724 cm`; inside on `46.17%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True` (touchdown `True`, balance `True`); final source tick `447.087`, progress `447.087` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `True`.
- Target / applied minimum rate: `0.5000` / `0.5000`; mean / p50 applied `0.7468` / `0.7323`.
- Limited / zero-rate hold ticks: `323` / `0`; maximum required landing time `0.9060 s`.
- Position / tangential / normal limiting ticks: `372` / `0` / `0`; unsafe-edge ticks `39`.
- Balance-margin limited ticks: `213`.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `333`, frozen `39`.
- Maximum applied offset / root reach: `0.0800 / 0.8153 m`.
- Authored-offset / reach / slew limited ticks: `333 / 0 / 82`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `39 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.6392 m / 6.0437 / 0.0132 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 6`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `458` ticks; maximum active coordinates `11`; mean target/applied scale `0.673` / `0.759`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2542.4 µs | 183875.4 µs | 208714.7 µs | 215641.2 µs | 126 | 102 | 262 | 0 | 71 | 39 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 17801.0 | 46058.9 | 316.7 | 55950.0 | 212394.3 | 215316.5 | 159226.8 | 600 | 108 | 76 | 56.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 126 | 2436.2 | 2569.5 | 2591.7 | 2621.6 |
| solved_with_slack | 102 | 2511.2 | 2854.5 | 4002.0 | 4528.4 |
| normal_contact_contingency | 71 | 5049.4 | 32568.9 | 93415.6 | 215641.2 |
| contact_release_contingency | 39 | 190304.6 | 209896.2 | 210159.8 | 210220.8 |
| precontact_transition | 262 | 2615.6 | 63569.5 | 73375.8 | 95734.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.87 | 12.0 | 15.0 | 25 | 5.66 | 15.0 | 21 | 0.1108 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 146.78/8.0/4380.1/5732 | 32576.95/1776.0/972391.1/1272504 | 1.73/25.0/31 | 1.47/24.0/30 | 12.54/217.1/272 | 0.2099 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 126 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 102 | 7.22/14.0/25 | 4.25/11.0/21 |
| normal_contact_contingency | 71 | 10.06/16.3/17 | 9.31/16.3/17 |
| contact_release_contingency | 39 | 7.92/11.6/12 | 6.59/10.0/10 |
| precontact_transition | 262 | 8.90/13.0/17 | 7.79/13.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.98/7.0/11 | 9.06/35.0/55 | 1.43/7.0/11 | 284 |
| viability | 1.85/6.0/11 | 11.50/42.0/66 | 1.49/6.0/11 | 389 |
| intent | 1.56/5.0/8 | 8.14/25.0/38 | 1.34/5.0/8 | 470 |
| preference | 1.38/5.0/20 | 11.40/47.0/140 | 0.99/5.0/19 | 389 |
| style | 1.09/3.0/5 | 9.13/22.0/47 | 0.41/3.0/5 | 206 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 490 | 110 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `113` ticks.
Precontact sole-center tangential speed: p50 `2.2897 m/s`, p95 `10.3196 m/s`, max `12.6122 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10.681 | 10.678 | 10.678 | 1.000 | 1.000 | 49.469 | 49.992 | 0.523 | 50.754 | 0.001 | 0 | 116 | 0 | 0 | 189 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2410.6 | 2515.0 | 5.00 | 33.53 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| 60–119 | 2513.0 | 3209.7 | 5.47 | 37.28 | 1.05 | 1.00 | 234.00 | 0.151 | 0.000 | 1.27e-09 | 3.93e-11 | 0 |
| 120–179 | 2483.9 | 3018.1 | 5.98 | 39.17 | 2.03 | 1.00 | 234.00 | 1.369 | 0.000 | 1.03e-09 | 5.03e-11 | 0 |
| 180–239 | 2468.1 | 3617.7 | 8.05 | 51.20 | 5.53 | 1.00 | 225.80 | 6.458 | 1.270 | 1.35e-09 | 5.23e-11 | 12 |
| 240–299 | 2564.2 | 3581.4 | 8.67 | 55.60 | 7.27 | 1.00 | 222.00 | 7.494 | 22.446 | 1.20e-09 | 2.07e-11 | 60 |
| 300–359 | 2502.5 | 4209.0 | 8.52 | 52.75 | 7.30 | 2.75 | 610.50 | 11.944 | 25.004 | 6.59e-10 | 1.35e-11 | 60 |
| 360–419 | 2530.9 | 4303.2 | 8.70 | 52.62 | 7.72 | 3.45 | 765.90 | 20.899 | 52.204 | 9.23e-10 | 1.20e-11 | 60 |
| 420–479 | 3405.1 | 73492.2 | 9.38 | 58.13 | 8.60 | 1257.98 | 279272.30 | 54.505 | 34.200 | 5.15e-10 | 1.23e-11 | 60 |
| 480–539 | 30671.4 | 208264.4 | 10.20 | 61.25 | 9.52 | 191.38 | 42433.10 | 112.229 | 29.597 | 9.78e-09 | 4.30e-10 | 60 |
| 540–599 | 4090.1 | 210126.1 | 8.70 | 50.80 | 7.53 | 7.18 | 1537.90 | 127.879 | 88.009 | 3.29e-09 | 1.17e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 57.095 | 36.942 | 79.113 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
