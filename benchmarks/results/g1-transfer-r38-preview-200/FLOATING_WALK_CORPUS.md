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
| 480 | 2.400 s | 24.172 cm | 13.914 cm | 48.754 cm | 38.430 cm | 75.598° | 8.000 rad/s | 82085.5 µs |

Nominal hard residual maxima: dynamics `8.003e-09`, contact acceleration `5.028e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 59.631 cm |
| authored reference vs measured CoM RMS / p95 | 55.289 / 122.872 cm |
| stance foot RMS | 32.232 cm |
| swing foot RMS | 52.251 cm |
| hand RMS | 72.419 cm |
| maximum root rotation | 179.887° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.869e-09 |
| contact acceleration residual | 5.028e-10 |
| raw max dynamics residual, including rejected ticks | 9.869e-09 |
| raw max contact residual, including rejected ticks | 5.028e-10 |
| active normal force range | 0.000–576.092 N |
| centroidal momentum-rate residual RMS / max | 51.490 / 245.550 N·m |
| point-task acceleration RMS max | 116.102 m/s² |
| frame-angular acceleration RMS max | 237.614 rad/s² |
| longest pre-contact / touchdown transition | 252 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `39.360` / `69.683 cm`.
- Virtual ZMP clipped on `58.83%` of ticks; clip-distance RMS / max `60.773` / `218.798 cm`.
- Measured-height natural frequency min / p50 / max: `3.656` / `3.719` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.500 m`; height-floor ticks: `112`.
- CoM command acceleration p95 / max: `20.081` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-152.595` / `-94.518 cm`; inside on `44.33%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8153 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 23`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0973 m / 1.6260 / 0.0184 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 7`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `458` ticks; maximum active coordinates `8`; mean target/applied scale `0.685` / `0.760`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2607.8 µs | 81672.5 µs | 183856.6 µs | 201198.4 µs | 126 | 102 | 252 | 0 | 96 | 24 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12486.0 | 33434.6 | 260.6 | 15685.4 | 197108.6 | 200789.4 | 171153.4 | 600 | 101 | 57 | 80.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 126 | 2514.8 | 2633.2 | 2678.2 | 3111.8 |
| solved_with_slack | 102 | 2580.7 | 2918.8 | 4136.5 | 4670.3 |
| normal_contact_contingency | 96 | 5113.6 | 32032.4 | 48643.3 | 183919.4 |
| contact_release_contingency | 24 | 165366.9 | 193722.8 | 199628.0 | 201198.4 |
| precontact_transition | 252 | 2646.9 | 74764.5 | 88026.2 | 99046.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.75 | 12.0 | 13.0 | 25 | 5.51 | 12.0 | 21 | 0.1094 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 189.65/319.6/4922.2/5970 | 41958.15/70841.4/1092728.4/1325340 | 1.17/14.0/28 | 0.97/13.0/27 | 7.88/118.0/232 | 0.3785 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 126 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 102 | 7.22/14.0/25 | 4.25/11.0/21 |
| normal_contact_contingency | 96 | 8.83/13.0/13 | 8.20/12.0/12 |
| contact_release_contingency | 24 | 7.58/10.0/10 | 6.00/8.8/9 |
| precontact_transition | 252 | 8.94/13.0/15 | 7.71/12.5/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.85/5.0/8 | 8.19/25.0/40 | 1.28/5.0/8 | 270 |
| viability | 1.89/6.0/8 | 12.00/42.0/56 | 1.52/6.0/8 | 384 |
| intent | 1.54/5.0/8 | 8.12/29.0/38 | 1.31/5.0/8 | 468 |
| preference | 1.41/6.0/20 | 12.59/63.0/140 | 0.99/6.0/19 | 379 |
| style | 1.06/2.0/4 | 8.90/20.0/33 | 0.40/2.0/4 | 213 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 480 | 120 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `123` ticks.
Precontact sole-center tangential speed: p50 `3.3570 m/s`, p95 `7.3365 m/s`, max `9.9656 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.492 | 7.474 | 7.474 | 0.998 | 0.998 | 48.262 | 48.715 | 0.453 | 49.391 | 0.001 | 0 | 93 | 0 | 0 | 1,494 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2491.4 | 2828.6 | 5.00 | 33.53 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| 60–119 | 2593.1 | 3315.1 | 5.47 | 37.28 | 1.05 | 1.00 | 234.00 | 0.151 | 0.000 | 1.27e-09 | 3.93e-11 | 0 |
| 120–179 | 2557.2 | 2863.2 | 5.98 | 39.17 | 2.03 | 1.00 | 234.00 | 1.369 | 0.000 | 1.03e-09 | 5.03e-11 | 0 |
| 180–239 | 2531.2 | 3725.3 | 8.05 | 51.20 | 5.53 | 1.00 | 225.80 | 6.458 | 1.270 | 1.35e-09 | 5.23e-11 | 12 |
| 240–299 | 2597.9 | 4073.1 | 8.35 | 53.00 | 6.87 | 1.58 | 351.50 | 7.203 | 23.318 | 8.57e-10 | 2.38e-11 | 60 |
| 300–359 | 2524.9 | 3947.7 | 8.92 | 56.68 | 7.58 | 1.12 | 247.90 | 13.835 | 25.111 | 1.12e-09 | 2.29e-11 | 60 |
| 360–419 | 2560.7 | 32747.7 | 8.92 | 55.33 | 7.82 | 69.62 | 15454.90 | 29.366 | 61.577 | 8.20e-10 | 9.52e-12 | 60 |
| 420–479 | 3721.1 | 96418.4 | 9.65 | 64.95 | 8.70 | 1575.22 | 349698.10 | 59.372 | 40.263 | 8.00e-09 | 5.03e-10 | 60 |
| 480–539 | 21543.1 | 197170.1 | 8.83 | 54.12 | 8.07 | 237.78 | 51350.80 | 122.140 | 38.812 | 9.87e-09 | 1.97e-10 | 60 |
| 540–599 | 3495.5 | 180620.4 | 8.33 | 52.70 | 7.45 | 7.18 | 1550.50 | 126.356 | 88.820 | 5.05e-09 | 1.57e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 59.631 | 39.980 | 72.419 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
