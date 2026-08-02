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
| 424 | 2.120 s | 12.795 cm | 1.060 cm | 23.222 cm | 35.047 cm | 56.407° | 8.000 rad/s | 100643.1 µs |

Nominal hard residual maxima: dynamics `1.314e-09`, contact acceleration `5.554e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 74.310 cm |
| authored reference vs measured CoM RMS / p95 | 68.315 / 160.387 cm |
| stance foot RMS | 46.204 cm |
| swing foot RMS | 62.300 cm |
| hand RMS | 100.140 cm |
| maximum root rotation | 179.853° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.015e-09 |
| contact acceleration residual | 2.396e-10 |
| raw max dynamics residual, including rejected ticks | 9.015e-09 |
| raw max contact residual, including rejected ticks | 2.396e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 93.366 / 455.193 N·m |
| point-task acceleration RMS max | 136.425 m/s² |
| frame-angular acceleration RMS max | 255.530 rad/s² |
| longest pre-contact / touchdown transition | 196 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `34.605` / `85.483 cm`.
- Virtual ZMP clipped on `58.17%` of ticks; clip-distance RMS / max `69.821` / `224.905 cm`.
- Measured-height natural frequency min / p50 / max: `3.687` / `3.766` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.876 m`; height-floor ticks: `145`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-145.672` / `-102.840 cm`; inside on `38.50%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8471 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 36`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.2887 m / 1.3309 / 0.0046 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 8`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `443` ticks; maximum active coordinates `8`; mean target/applied scale `0.645` / `0.738`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2597.7 µs | 57446.5 µs | 150766.2 µs | 217352.7 µs | 126 | 102 | 196 | 0 | 158 | 18 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10198.4 | 28071.9 | 380.0 | 7840.5 | 214718.7 | 217089.3 | 101010.0 | 600 | 95 | 45 | 98.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 126 | 2502.6 | 2631.2 | 2655.4 | 2672.6 |
| solved_with_slack | 102 | 2572.7 | 2808.4 | 3019.2 | 4091.6 |
| normal_contact_contingency | 158 | 4710.4 | 53762.0 | 70841.7 | 140507.1 |
| contact_release_contingency | 18 | 145191.7 | 213615.0 | 216605.2 | 217352.7 |
| precontact_transition | 196 | 2590.5 | 4423.9 | 101070.9 | 102258.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.82 | 12.0 | 13.0 | 16 | 5.57 | 13.0 | 15 | 0.1169 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 188.19/103.5/6103.1/6167 | 41147.61/22356.0/1354883.8/1369074 | 1.11/10.0/11 | 0.85/9.0/10 | 7.00/73.0/90 | 0.4782 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 126 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 102 | 6.87/10.0/10 | 3.81/7.0/8 |
| normal_contact_contingency | 158 | 9.42/14.4/16 | 8.49/14.0/15 |
| contact_release_contingency | 18 | 7.50/9.0/9 | 6.00/7.8/8 |
| precontact_transition | 196 | 8.88/13.0/13 | 7.68/12.1/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.99/6.0/7 | 8.58/27.0/35 | 1.47/6.0/7 | 293 |
| viability | 1.79/6.0/8 | 11.30/42.0/51 | 1.41/6.0/8 | 385 |
| intent | 1.58/5.0/8 | 8.45/30.0/42 | 1.35/5.0/8 | 469 |
| preference | 1.37/6.0/7 | 11.90/55.0/67 | 0.94/5.0/7 | 376 |
| style | 1.09/3.0/5 | 9.17/26.0/38 | 0.40/3.0/5 | 198 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 424 | 176 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `179` ticks.
Precontact sole-center tangential speed: p50 `2.1345 m/s`, p95 `8.0604 m/s`, max `8.9420 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.119 | 6.118 | 6.118 | 1.000 | 1.000 | 49.145 | 49.570 | 0.426 | 49.570 | 0.001 | 0 | 93 | 0 | 0 | 87 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2478.5 | 2594.2 | 5.00 | 33.53 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| 60–119 | 2568.1 | 3276.8 | 5.47 | 37.28 | 1.05 | 1.00 | 234.00 | 0.151 | 0.000 | 1.27e-09 | 3.93e-11 | 0 |
| 120–179 | 2557.9 | 2963.6 | 6.12 | 40.07 | 2.18 | 1.00 | 234.00 | 1.340 | 0.000 | 1.31e-09 | 5.38e-11 | 0 |
| 180–239 | 2292.9 | 2952.5 | 7.17 | 44.67 | 4.47 | 1.00 | 225.80 | 6.419 | 1.318 | 1.10e-09 | 3.59e-11 | 12 |
| 240–299 | 2615.4 | 4023.9 | 8.88 | 58.27 | 7.27 | 1.00 | 222.00 | 5.721 | 19.302 | 8.49e-10 | 2.10e-11 | 60 |
| 300–359 | 2553.2 | 4502.1 | 8.75 | 52.05 | 7.58 | 2.75 | 610.50 | 11.769 | 20.714 | 1.05e-09 | 1.12e-11 | 60 |
| 360–419 | 2766.4 | 101670.5 | 9.10 | 54.95 | 8.38 | 411.70 | 91397.40 | 29.017 | 13.665 | 9.28e-10 | 1.54e-11 | 60 |
| 420–479 | 5330.8 | 117048.3 | 10.35 | 66.83 | 9.03 | 1321.88 | 287976.20 | 74.979 | 40.233 | 9.39e-10 | 9.90e-12 | 60 |
| 480–539 | 5682.1 | 214758.3 | 8.83 | 52.90 | 8.08 | 7.88 | 1700.40 | 152.992 | 95.889 | 9.01e-09 | 1.92e-10 | 60 |
| 540–599 | 3993.8 | 170850.3 | 8.57 | 53.45 | 7.67 | 132.65 | 28641.80 | 158.542 | 123.786 | 8.81e-09 | 2.40e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 74.310 | 52.083 | 100.140 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
