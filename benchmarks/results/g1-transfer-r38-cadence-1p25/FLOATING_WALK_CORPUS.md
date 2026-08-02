# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.25×` multiplier.
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
| 431 | 2.155 s | 8.452 cm | 13.678 cm | 27.085 cm | 35.409 cm | 35.999° | 8.000 rad/s | 96361.9 µs |

Nominal hard residual maxima: dynamics `1.495e-09`, contact acceleration `4.947e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 69.539 cm |
| authored reference vs measured CoM RMS / p95 | 64.762 / 153.607 cm |
| stance foot RMS | 38.800 cm |
| swing foot RMS | 57.622 cm |
| hand RMS | 84.313 cm |
| maximum root rotation | 161.599° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.018e-09 |
| contact acceleration residual | 1.248e-10 |
| raw max dynamics residual, including rejected ticks | 9.018e-09 |
| raw max contact residual, including rejected ticks | 1.248e-10 |
| active normal force range | 0.000–820.239 N |
| centroidal momentum-rate residual RMS / max | 47.801 / 231.039 N·m |
| point-task acceleration RMS max | 141.196 m/s² |
| frame-angular acceleration RMS max | 248.974 rad/s² |
| longest pre-contact / touchdown transition | 234 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 146 / 146 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `17.536` / `45.353 cm`.
- Virtual ZMP clipped on `57.17%` of ticks; clip-distance RMS / max `39.635` / `115.545 cm`.
- Measured-height natural frequency min / p50 / max: `3.655` / `3.770` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.849 m`; height-floor ticks: `143`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-63.627` / `-62.082 cm`; inside on `46.00%` of ticks.

## Capture-aware landing

- Active target-ticks: `344`; policy updates `198`, frozen `146`.
- Maximum applied offset / root reach: `0.0800 / 0.8579 m`.
- Authored-offset / reach / slew limited ticks: `198 / 0 / 74`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `146 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.4416 m / 1.4294 / 0.0272 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 1`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `58`, precontact `344`, multi-support `197`.
- Joint-velocity envelope active on `430` ticks; maximum active coordinates `9`; mean target/applied scale `0.719` / `0.797`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2751.2 µs | 38307.4 µs | 137875.3 µs | 221118.9 µs | 128 | 69 | 234 | 0 | 162 | 7 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10110.7 | 26094.4 | 504.2 | 28024.1 | 219411.6 | 220948.2 | 31346.0 | 600 | 85 | 70 | 98.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 128 | 2557.7 | 2933.8 | 3605.9 | 3708.3 |
| solved_with_slack | 69 | 2693.2 | 3341.4 | 3774.7 | 4162.2 |
| normal_contact_contingency | 162 | 3634.0 | 37388.2 | 38358.1 | 137456.7 |
| contact_release_contingency | 7 | 199387.5 | 220263.8 | 220947.9 | 221118.9 |
| precontact_transition | 234 | 2659.9 | 75877.1 | 103620.2 | 112356.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.75 | 12.0 | 13.0 | 14 | 5.47 | 12.0 | 13 | 0.2079 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 306.50/2133.3/5592.8/6973 | 67288.08/462466.8/1241594.9/1548006 | 0.85/6.0/16 | 0.59/5.0/15 | 4.95/43.1/132 | 0.6039 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 128 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 69 | 6.94/11.0/11 | 3.81/8.0/8 |
| normal_contact_contingency | 162 | 9.23/13.0/14 | 8.25/12.4/13 |
| contact_release_contingency | 7 | 7.29/8.9/9 | 6.00/7.0/7 |
| precontact_transition | 234 | 8.49/13.0/14 | 7.02/12.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.98/6.0/10 | 8.90/30.0/49 | 1.48/6.0/10 | 304 |
| viability | 1.58/6.0/9 | 9.44/37.0/58 | 1.15/6.0/9 | 344 |
| intent | 1.61/5.0/6 | 8.77/25.0/31 | 1.39/5.0/6 | 470 |
| preference | 1.51/7.0/7 | 13.53/63.0/87 | 1.09/6.0/7 | 385 |
| style | 1.06/2.0/4 | 8.99/17.0/31 | 0.36/2.0/4 | 190 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 59 | 344 | 0 | 197 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 431 | 169 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `344` ticks, planned normal touchdown `0` ticks, normal fallback `172` ticks.
Precontact sole-center tangential speed: p50 `2.1171 m/s`, p95 `6.2949 m/s`, max `7.5078 m/s` over 344 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.067 | 6.065 | 6.065 | 1.000 | 1.000 | 47.043 | 47.215 | 0.172 | 49.027 | 0.001 | 0 | 43 | 0 | 0 | 62 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2547.8 | 2917.0 | 5.00 | 33.50 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.42e-09 | 4.94e-11 | 0 |
| 60–119 | 2657.1 | 3667.8 | 5.68 | 38.60 | 1.37 | 1.00 | 234.00 | 0.060 | 0.000 | 1.03e-09 | 4.95e-11 | 0 |
| 120–179 | 2635.0 | 3710.2 | 5.90 | 38.87 | 1.70 | 1.00 | 234.00 | 0.888 | 0.000 | 1.50e-09 | 3.82e-11 | 0 |
| 180–239 | 2246.9 | 3229.3 | 7.42 | 47.28 | 4.70 | 1.00 | 225.40 | 4.962 | 1.041 | 1.12e-09 | 4.73e-11 | 43 |
| 240–299 | 2575.7 | 4466.2 | 8.42 | 51.02 | 6.88 | 2.52 | 558.70 | 5.906 | 15.907 | 6.90e-10 | 2.51e-11 | 60 |
| 300–359 | 2542.9 | 26882.3 | 8.40 | 49.87 | 7.28 | 72.55 | 16106.10 | 7.366 | 29.154 | 9.15e-10 | 1.21e-11 | 60 |
| 360–419 | 3627.3 | 75885.2 | 9.08 | 57.05 | 8.17 | 664.28 | 147470.90 | 13.952 | 30.165 | 8.61e-10 | 1.60e-11 | 60 |
| 420–479 | 34275.9 | 122647.4 | 10.47 | 70.07 | 9.38 | 2307.93 | 504867.80 | 69.994 | 44.999 | 4.98e-10 | 2.47e-12 | 60 |
| 480–539 | 3898.0 | 219437.3 | 8.20 | 51.53 | 7.10 | 7.42 | 1599.90 | 146.377 | 76.445 | 9.02e-09 | 1.25e-10 | 60 |
| 540–599 | 3287.8 | 4783.4 | 8.98 | 58.50 | 8.15 | 6.25 | 1350.00 | 147.383 | 105.644 | 9.40e-10 | 8.48e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.94x | 600 | 69.539 | 45.873 | 84.313 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
