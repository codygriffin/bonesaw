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
| 334 | 1.670 s | 13.236 cm | 11.800 cm | 44.520 cm | 28.278 cm | 69.752° | 8.000 rad/s | 82040.8 µs |

Nominal hard residual maxima: dynamics `1.618e-09`, contact acceleration `8.797e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 125.338 cm |
| authored reference vs measured CoM RMS / p95 | 127.277 / 248.993 cm |
| stance foot RMS | 93.590 cm |
| swing foot RMS | 120.232 cm |
| hand RMS | 134.088 cm |
| maximum root rotation | 169.620° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.519e-09 |
| contact acceleration residual | 1.482e-10 |
| raw max dynamics residual, including rejected ticks | 7.519e-09 |
| raw max contact residual, including rejected ticks | 1.482e-10 |
| active normal force range | 0.000–788.911 N |
| centroidal momentum-rate residual RMS / max | 48.481 / 271.527 N·m |
| point-task acceleration RMS max | 131.698 m/s² |
| frame-angular acceleration RMS max | 228.900 rad/s² |
| longest pre-contact / touchdown transition | 106 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `106.467` / `201.557 cm`.
- Virtual ZMP clipped on `66.33%` of ticks; clip-distance RMS / max `140.451` / `291.254 cm`.
- Measured-height natural frequency min / p50 / max: `3.669` / `3.996` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.994 m`; height-floor ticks: `247`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-175.403` / `-168.399 cm`; inside on `33.67%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8510 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 29`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.7389 m / 0.9348 / 0.0063 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 12`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `320` ticks; maximum active coordinates `10`; mean target/applied scale `0.485` / `0.565`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2739.5 µs | 18137.2 µs | 152693.5 µs | 212331.5 µs | 158 | 70 | 106 | 0 | 247 | 19 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8606.0 | 25853.0 | 393.4 | 5808.2 | 208185.2 | 211916.8 | 43102.1 | 600 | 98 | 30 | 116.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 158 | 2508.8 | 2688.3 | 2874.2 | 2894.2 |
| solved_with_slack | 70 | 2538.2 | 2792.6 | 3305.7 | 4216.0 |
| normal_contact_contingency | 247 | 2923.1 | 6957.6 | 20734.5 | 49456.2 |
| contact_release_contingency | 19 | 101843.5 | 206101.6 | 211085.5 | 212331.5 |
| precontact_transition | 106 | 4633.4 | 74351.9 | 95696.1 | 103686.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.68 | 12.0 | 13.0 | 19 | 5.33 | 12.0 | 16 | 0.0364 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 86.43/8.0/4362.0/6512 | 19093.65/1776.0/968361.8/1445664 | 1.43/8.0/16 | 0.95/7.0/15 | 7.58/62.0/128 | 0.3510 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 158 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 70 | 7.36/14.2/19 | 4.24/11.9/16 |
| normal_contact_contingency | 247 | 9.17/13.0/16 | 8.08/12.5/15 |
| contact_release_contingency | 19 | 7.37/9.6/10 | 5.47/7.0/7 |
| precontact_transition | 106 | 8.47/12.0/13 | 7.54/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.18/6.0/7 | 9.57/29.0/35 | 1.77/6.0/7 | 373 |
| viability | 1.67/6.0/10 | 10.65/42.0/64 | 1.30/6.0/10 | 389 |
| intent | 1.53/6.0/8 | 7.83/30.0/48 | 1.23/6.0/8 | 433 |
| preference | 1.28/6.0/14 | 10.54/55.0/98 | 0.75/5.0/14 | 309 |
| style | 1.01/2.0/2 | 8.64/13.0/23 | 0.27/1.0/2 | 157 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 334 | 266 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `269` ticks.
Precontact sole-center tangential speed: p50 `2.8730 m/s`, p95 `4.7404 m/s`, max `5.0682 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.164 | 5.154 | 5.154 | 0.998 | 0.998 | 47.168 | 47.531 | 0.363 | 48.973 | 0.001 | 0 | 92 | 0 | 0 | 578 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2491.5 | 2825.2 | 5.00 | 32.48 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.23e-09 | 5.86e-11 | 0 |
| 60–119 | 2532.9 | 2882.5 | 5.28 | 33.78 | 0.58 | 1.00 | 234.00 | 0.025 | 0.000 | 1.62e-09 | 6.15e-11 | 0 |
| 120–179 | 2521.8 | 2694.8 | 5.30 | 33.27 | 0.52 | 1.00 | 234.00 | 2.412 | 0.000 | 1.08e-09 | 6.12e-11 | 0 |
| 180–239 | 2543.2 | 6293.0 | 7.67 | 47.08 | 5.13 | 2.28 | 510.70 | 7.947 | 1.170 | 8.69e-10 | 8.80e-11 | 12 |
| 240–299 | 4651.0 | 7375.1 | 8.25 | 49.42 | 7.42 | 8.00 | 1776.00 | 14.710 | 28.783 | 4.55e-10 | 1.62e-11 | 60 |
| 300–359 | 5430.9 | 208247.5 | 8.60 | 47.40 | 7.20 | 691.93 | 153585.80 | 50.231 | 79.932 | 7.52e-09 | 1.48e-10 | 60 |
| 360–419 | 4233.7 | 20045.4 | 8.88 | 51.72 | 7.78 | 48.23 | 10418.40 | 145.465 | 149.207 | 3.06e-09 | 1.19e-10 | 60 |
| 420–479 | 2954.2 | 41783.6 | 8.62 | 55.32 | 7.63 | 98.92 | 21366.00 | 240.233 | 189.996 | 1.11e-09 | 2.01e-11 | 60 |
| 480–539 | 2745.6 | 3425.1 | 9.20 | 57.48 | 8.02 | 6.38 | 1378.80 | 219.711 | 158.549 | 8.25e-10 | 5.24e-12 | 60 |
| 540–599 | 2858.5 | 4904.2 | 9.98 | 64.28 | 8.97 | 5.55 | 1198.80 | 164.754 | 125.378 | 6.73e-10 | 1.22e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 125.338 | 103.169 | 134.088 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
