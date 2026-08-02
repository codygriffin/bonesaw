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
- Joint-velocity envelope: `viability` priority with weight `0.250`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
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
| 426 | 2.130 s | 14.922 cm | 1.078 cm | 38.477 cm | 40.210 cm | 51.668° | 8.000 rad/s | 12398.9 µs |

Nominal hard residual maxima: dynamics `2.702e-09`, contact acceleration `1.234e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 70.538 cm |
| authored reference vs measured CoM RMS / p95 | 63.719 / 140.745 cm |
| stance foot RMS | 44.732 cm |
| swing foot RMS | 62.712 cm |
| hand RMS | 94.754 cm |
| maximum root rotation | 179.650° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.279e-09 |
| contact acceleration residual | 4.854e-10 |
| raw max dynamics residual, including rejected ticks | 8.279e-09 |
| raw max contact residual, including rejected ticks | 4.854e-10 |
| active normal force range | 0.000–676.638 N |
| centroidal momentum-rate residual RMS / max | 71.052 / 459.957 N·m |
| point-task acceleration RMS max | 232.917 m/s² |
| frame-angular acceleration RMS max | 339.352 rad/s² |
| longest pre-contact / touchdown transition | 198 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `42.637` / `78.161 cm`.
- Virtual ZMP clipped on `64.00%` of ticks; clip-distance RMS / max `74.822` / `156.052 cm`.
- Measured-height natural frequency min / p50 / max: `3.689` / `3.758` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.599 m`; height-floor ticks: `133`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-104.512` / `-96.490 cm`; inside on `39.67%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8690 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 31`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.3571 m / 1.1069 / 0.0139 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 6`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `444` ticks; maximum active coordinates `8`; mean target/applied scale `0.637` / `0.748`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2659.0 µs | 15092.2 µs | 191815.6 µs | 228721.9 µs | 127 | 101 | 198 | 0 | 159 | 15 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9077.9 | 27928.5 | 384.6 | 5494.8 | 220167.3 | 227866.4 | 110604.5 | 600 | 70 | 29 | 110.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 127 | 2521.9 | 2633.4 | 2698.5 | 3267.3 |
| solved_with_slack | 101 | 2596.3 | 3095.2 | 3328.1 | 3420.5 |
| normal_contact_contingency | 159 | 4043.4 | 92060.4 | 106337.8 | 228721.9 |
| contact_release_contingency | 15 | 113025.7 | 207277.3 | 213007.9 | 214440.5 |
| precontact_transition | 198 | 2691.8 | 4622.3 | 13372.8 | 37775.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.84 | 12.0 | 14.0 | 16 | 5.66 | 13.0 | 15 | 0.0222 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 113.02/8.0/5816.7/6637 | 24450.51/1776.0/1256407.2/1433592 | 1.36/14.0/21 | 1.03/13.0/20 | 8.49/114.0/181 | 0.4156 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 127 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 101 | 7.26/12.0/14 | 4.43/11.0/13 |
| normal_contact_contingency | 159 | 9.35/15.0/16 | 8.40/13.0/15 |
| contact_release_contingency | 15 | 7.27/9.0/9 | 5.67/8.0/8 |
| precontact_transition | 198 | 8.79/13.0/14 | 7.71/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.97/6.0/7 | 8.56/25.0/35 | 1.45/6.0/7 | 310 |
| viability | 1.84/7.0/8 | 11.80/52.0/65 | 1.49/7.0/8 | 388 |
| intent | 1.56/4.0/7 | 8.23/24.0/36 | 1.33/4.0/7 | 468 |
| preference | 1.43/6.0/7 | 12.29/53.0/67 | 1.03/6.0/7 | 383 |
| style | 1.04/2.0/4 | 8.72/16.0/30 | 0.36/2.0/4 | 200 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 426 | 174 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `177` ticks.
Precontact sole-center tangential speed: p50 `2.7772 m/s`, p95 `11.0423 m/s`, max `12.8551 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.447 | 5.446 | 5.446 | 1.000 | 1.000 | 48.094 | 48.547 | 0.453 | 49.133 | 0.001 | 0 | 93 | 0 | 0 | 55 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2514.0 | 2933.9 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2573.1 | 3138.1 | 5.55 | 37.87 | 1.12 | 1.00 | 234.00 | 0.123 | 0.000 | 1.52e-09 | 6.08e-11 | 0 |
| 120–179 | 2569.7 | 3066.7 | 5.98 | 39.50 | 2.05 | 1.00 | 234.00 | 1.328 | 0.000 | 1.19e-09 | 6.02e-11 | 0 |
| 180–239 | 2462.7 | 3366.0 | 7.88 | 48.02 | 5.45 | 1.00 | 225.80 | 7.232 | 3.487 | 9.61e-10 | 3.99e-11 | 12 |
| 240–299 | 3027.4 | 4990.7 | 8.72 | 58.17 | 7.45 | 3.57 | 791.80 | 6.829 | 23.516 | 9.50e-10 | 9.92e-12 | 60 |
| 300–359 | 2655.9 | 4413.8 | 8.92 | 53.23 | 7.68 | 3.22 | 714.10 | 9.651 | 23.320 | 9.03e-10 | 1.38e-11 | 60 |
| 360–419 | 2401.8 | 3988.7 | 8.68 | 49.95 | 8.07 | 3.33 | 740.00 | 33.866 | 38.263 | 1.39e-09 | 1.55e-11 | 60 |
| 420–479 | 14141.3 | 214258.3 | 9.88 | 62.10 | 9.10 | 1100.98 | 238081.00 | 78.300 | 41.059 | 8.28e-09 | 4.85e-10 | 60 |
| 480–539 | 4051.6 | 191455.1 | 8.90 | 55.82 | 7.67 | 7.07 | 1522.40 | 141.632 | 102.183 | 6.91e-09 | 8.09e-11 | 60 |
| 540–599 | 3691.0 | 6367.3 | 8.88 | 57.77 | 7.97 | 8.00 | 1728.00 | 149.082 | 108.167 | 1.35e-09 | 5.35e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 70.538 | 51.381 | 94.754 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
