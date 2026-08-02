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
- Joint-velocity envelope: `viability` priority with weight `0.200`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
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
| 533 | 2.665 s | 15.110 cm | 19.765 cm | 37.140 cm | 36.475 cm | 57.483° | 8.000 rad/s | 26512.8 µs |

Nominal hard residual maxima: dynamics `1.309e-09`, contact acceleration `5.133e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 38.848 cm |
| authored reference vs measured CoM RMS / p95 | 41.458 / 102.451 cm |
| stance foot RMS | 33.968 cm |
| swing foot RMS | 45.036 cm |
| hand RMS | 52.012 cm |
| maximum root rotation | 170.604° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.816e-09 |
| contact acceleration residual | 1.759e-10 |
| raw max dynamics residual, including rejected ticks | 6.816e-09 |
| raw max contact residual, including rejected ticks | 1.759e-10 |
| active normal force range | 0.000–708.941 N |
| centroidal momentum-rate residual RMS / max | 46.183 / 272.219 N·m |
| point-task acceleration RMS max | 192.594 m/s² |
| frame-angular acceleration RMS max | 290.956 rad/s² |
| longest pre-contact / touchdown transition | 305 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `37.859` / `86.966 cm`.
- Virtual ZMP clipped on `59.67%` of ticks; clip-distance RMS / max `47.298` / `156.744 cm`.
- Measured-height natural frequency min / p50 / max: `3.694` / `3.769` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.507 m`; height-floor ticks: `47`.
- CoM command acceleration p95 / max: `22.254` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-90.927` / `-53.799 cm`; inside on `50.67%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8426 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 50`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.1018 m / 1.6739 / 0.0506 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 13`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `433` ticks; maximum active coordinates `9`; mean target/applied scale `0.666` / `0.724`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2872.7 µs | 101631.8 µs | 184464.1 µs | 272373.3 µs | 129 | 99 | 305 | 0 | 41 | 26 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12723.7 | 33465.6 | 552.6 | 12039.0 | 242435.2 | 269379.5 | 95382.7 | 600 | 83 | 56 | 78.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2541.3 | 2889.9 | 3599.3 | 3692.5 |
| solved_with_slack | 99 | 2619.4 | 2928.0 | 3407.6 | 3441.6 |
| normal_contact_contingency | 41 | 56805.8 | 104025.0 | 133521.0 | 151902.7 |
| contact_release_contingency | 26 | 105380.6 | 220461.0 | 259878.3 | 272373.3 |
| precontact_transition | 305 | 3391.6 | 5029.6 | 83369.3 | 113495.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.58 | 12.0 | 13.0 | 17 | 5.32 | 12.0 | 16 | 0.1618 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 228.56/8.0/6162.3/7087 | 49676.87/1776.0/1331177.8/1573314 | 1.26/14.0/31 | 0.88/13.0/30 | 7.28/107.0/283 | 0.4567 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.71/10.1/13 | 3.57/7.1/10 |
| normal_contact_contingency | 41 | 9.85/15.4/17 | 9.17/14.8/16 |
| contact_release_contingency | 26 | 7.92/10.0/10 | 6.38/9.0/9 |
| precontact_transition | 305 | 8.62/13.0/14 | 7.54/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.99/6.0/7 | 8.57/28.0/35 | 1.47/6.0/7 | 340 |
| viability | 1.65/6.0/8 | 10.05/42.0/49 | 1.25/6.0/8 | 365 |
| intent | 1.57/4.0/9 | 8.30/24.0/52 | 1.32/4.0/9 | 461 |
| preference | 1.30/6.0/7 | 10.96/49.0/73 | 0.88/5.0/6 | 367 |
| style | 1.07/3.0/4 | 9.09/23.0/39 | 0.40/2.0/4 | 217 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 533 | 67 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `70` ticks.
Precontact sole-center tangential speed: p50 `2.6037 m/s`, p95 `8.0246 m/s`, max `12.2303 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.635 | 7.631 | 7.631 | 1.000 | 1.000 | 48.062 | 48.426 | 0.363 | 49.328 | 0.001 | 0 | 92 | 0 | 0 | 275 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2516.4 | 3641.3 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2621.9 | 2934.8 | 5.40 | 36.67 | 0.90 | 1.00 | 234.00 | 0.116 | 0.000 | 1.22e-09 | 4.26e-11 | 0 |
| 120–179 | 2608.1 | 3421.1 | 5.95 | 39.57 | 1.98 | 1.00 | 234.00 | 1.270 | 0.000 | 1.09e-09 | 5.13e-11 | 0 |
| 180–239 | 2549.8 | 3193.4 | 7.18 | 44.25 | 4.32 | 1.00 | 225.80 | 6.141 | 0.399 | 1.24e-09 | 3.59e-11 | 12 |
| 240–299 | 2644.0 | 4216.9 | 8.43 | 50.40 | 7.17 | 2.98 | 662.30 | 7.757 | 25.030 | 1.07e-09 | 1.30e-11 | 60 |
| 300–359 | 3661.2 | 4477.3 | 8.48 | 47.75 | 7.20 | 6.72 | 1491.10 | 9.634 | 24.373 | 6.51e-10 | 9.13e-12 | 60 |
| 360–419 | 3553.9 | 4615.7 | 8.35 | 49.73 | 7.42 | 8.00 | 1776.00 | 15.493 | 40.764 | 1.30e-09 | 4.29e-11 | 60 |
| 420–479 | 3540.9 | 28086.8 | 8.88 | 54.83 | 8.05 | 79.05 | 17549.10 | 22.045 | 47.603 | 1.00e-09 | 1.15e-11 | 60 |
| 480–539 | 3456.8 | 129242.6 | 8.97 | 59.30 | 8.13 | 842.10 | 184338.70 | 37.773 | 41.160 | 1.04e-09 | 1.26e-11 | 60 |
| 540–599 | 97079.6 | 242885.0 | 9.15 | 53.72 | 8.07 | 1342.75 | 290023.70 | 112.901 | 87.112 | 6.82e-09 | 1.76e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 38.848 | 37.989 | 52.012 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
