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
| 457 | 2.285 s | 10.122 cm | 1.833 cm | 38.408 cm | 32.081 cm | 45.991° | 8.000 rad/s | 100373.7 µs |

Nominal hard residual maxima: dynamics `1.309e-09`, contact acceleration `7.602e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 72.731 cm |
| authored reference vs measured CoM RMS / p95 | 69.699 / 181.927 cm |
| stance foot RMS | 45.620 cm |
| swing foot RMS | 57.179 cm |
| hand RMS | 90.849 cm |
| maximum root rotation | 155.384° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 5.424e-09 |
| contact acceleration residual | 1.297e-10 |
| raw max dynamics residual, including rejected ticks | 5.424e-09 |
| raw max contact residual, including rejected ticks | 1.297e-10 |
| active normal force range | 0.000–872.561 N |
| centroidal momentum-rate residual RMS / max | 59.545 / 250.364 N·m |
| point-task acceleration RMS max | 137.571 m/s² |
| frame-angular acceleration RMS max | 217.777 rad/s² |
| longest pre-contact / touchdown transition | 149 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 92 / 92 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `42.514` / `122.835 cm`.
- Virtual ZMP clipped on `54.67%` of ticks; clip-distance RMS / max `54.537` / `224.968 cm`.
- Measured-height natural frequency min / p50 / max: `3.693` / `3.753` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.934 m`; height-floor ticks: `131`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-138.998` / `-89.513 cm`; inside on `47.33%` of ticks.

## Capture-aware landing

- Active target-ticks: `292`; policy updates `200`, frozen `92`.
- Maximum applied offset / root reach: `0.0800 / 0.8146 m`.
- Authored-offset / reach / slew limited ticks: `189 / 0 / 99`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `92 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.9467 m / 0.8440 / 0.0322 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 1`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `291`, multi-support `279`.
- Joint-velocity envelope active on `451` ticks; maximum active coordinates `9`; mean target/applied scale `0.695` / `0.764`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2740.1 µs | 70856.8 µs | 113333.6 µs | 211198.6 µs | 129 | 179 | 149 | 0 | 134 | 9 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9227.5 | 23690.4 | 378.1 | 8036.6 | 191453.7 | 209224.1 | 98918.7 | 600 | 85 | 55 | 108.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2549.4 | 2683.5 | 2753.3 | 2811.7 |
| solved_with_slack | 179 | 2962.1 | 4684.2 | 5321.9 | 5523.4 |
| normal_contact_contingency | 134 | 3615.7 | 27082.1 | 51946.6 | 143626.1 |
| contact_release_contingency | 9 | 108464.7 | 198013.3 | 208561.6 | 211198.6 |
| precontact_transition | 149 | 2417.1 | 90122.1 | 111110.9 | 117068.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.11 | 12.0 | 15.0 | 23 | 5.82 | 13.0 | 19 | 0.1352 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 253.68/1518.2/5690.5/6976 | 55980.50/327942.0/1263293.2/1548672 | 0.69/8.0/12 | 0.51/7.0/11 | 4.14/57.0/91 | 0.6844 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 179 | 9.01/18.1/23 | 6.65/16.4/19 |
| normal_contact_contingency | 134 | 8.81/12.0/12 | 7.71/11.0/11 |
| contact_release_contingency | 9 | 8.00/9.9/10 | 6.67/8.9/9 |
| precontact_transition | 149 | 9.10/13.0/15 | 8.12/13.0/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.06/6.0/7 | 9.09/29.0/30 | 1.57/6.0/7 | 319 |
| viability | 2.09/9.0/12 | 11.18/45.0/54 | 1.69/9.0/11 | 398 |
| intent | 1.48/4.0/6 | 8.08/24.0/30 | 1.25/4.0/6 | 469 |
| preference | 1.40/6.0/14 | 12.37/55.0/126 | 0.97/5.0/13 | 391 |
| style | 1.08/3.0/3 | 9.97/26.0/36 | 0.34/2.0/3 | 182 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 292 | 0 | 279 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 457 | 143 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `292` ticks, planned normal touchdown `0` ticks, normal fallback `146` ticks.
Precontact sole-center tangential speed: p50 `2.0894 m/s`, p95 `6.0600 m/s`, max `7.4756 m/s` over 292 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.537 | 5.536 | 5.536 | 1.000 | 1.000 | 48.156 | 48.590 | 0.434 | 49.289 | 0.001 | 0 | 93 | 0 | 0 | 35 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2523.3 | 2650.1 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2639.7 | 2917.3 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2604.3 | 3093.6 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 3298.5 | 5438.9 | 10.12 | 61.68 | 7.47 | 1.00 | 234.00 | 6.433 | 0.001 | 9.40e-10 | 7.60e-11 | 0 |
| 240–299 | 3096.6 | 4591.7 | 9.90 | 57.12 | 8.33 | 1.47 | 333.40 | 7.382 | 4.232 | 1.01e-09 | 7.16e-11 | 0 |
| 300–359 | 2427.6 | 5449.2 | 8.55 | 50.45 | 7.52 | 2.17 | 481.00 | 5.767 | 19.461 | 1.11e-09 | 1.84e-11 | 52 |
| 360–419 | 2327.1 | 49003.4 | 9.02 | 56.38 | 8.18 | 158.15 | 35109.30 | 9.948 | 37.223 | 7.59e-10 | 1.30e-11 | 60 |
| 420–479 | 26206.4 | 127957.3 | 10.25 | 66.15 | 8.92 | 2285.58 | 504490.40 | 47.188 | 36.157 | 2.75e-10 | 5.29e-12 | 60 |
| 480–539 | 4840.9 | 191750.3 | 8.77 | 53.63 | 7.72 | 78.38 | 16928.50 | 135.516 | 84.247 | 5.42e-09 | 1.30e-10 | 60 |
| 540–599 | 3062.9 | 7124.5 | 8.03 | 51.02 | 7.03 | 7.07 | 1526.40 | 179.102 | 117.318 | 3.22e-09 | 8.58e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 72.731 | 48.939 | 90.849 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
