# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.50×` multiplier.
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
| 385 | 1.925 s | 9.059 cm | 11.385 cm | 40.025 cm | 33.545 cm | 68.342° | 8.000 rad/s | 85372.0 µs |

Nominal hard residual maxima: dynamics `1.212e-09`, contact acceleration `6.011e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 100.371 cm |
| authored reference vs measured CoM RMS / p95 | 94.708 / 193.575 cm |
| stance foot RMS | 69.678 cm |
| swing foot RMS | 113.197 cm |
| hand RMS | 109.874 cm |
| maximum root rotation | 158.189° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.648e-09 |
| contact acceleration residual | 1.316e-10 |
| raw max dynamics residual, including rejected ticks | 6.648e-09 |
| raw max contact residual, including rejected ticks | 1.316e-10 |
| active normal force range | 0.000–761.736 N |
| centroidal momentum-rate residual RMS / max | 47.140 / 247.430 N·m |
| point-task acceleration RMS max | 186.536 m/s² |
| frame-angular acceleration RMS max | 159.026 rad/s² |
| longest pre-contact / touchdown transition | 190 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 121 / 121 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `80.065` / `176.541 cm`.
- Virtual ZMP clipped on `60.00%` of ticks; clip-distance RMS / max `131.133` / `361.545 cm`.
- Measured-height natural frequency min / p50 / max: `3.665` / `3.773` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.875 m`; height-floor ticks: `172`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-162.192` / `-150.789 cm`; inside on `39.00%` of ticks.

## Capture-aware landing

- Active target-ticks: `299`; policy updates `178`, frozen `121`.
- Maximum applied offset / root reach: `0.0800 / 0.8573 m`.
- Authored-offset / reach / slew limited ticks: `178 / 0 / 68`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `121 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.6280 m / 0.5826 / 0.3615 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `105`, precontact `299`, multi-support `195`.
- Joint-velocity envelope active on `488` ticks; maximum active coordinates `9`; mean target/applied scale `0.736` / `0.815`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2722.8 µs | 104759.0 µs | 195796.9 µs | 215345.5 µs | 131 | 64 | 190 | 0 | 181 | 34 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 13474.1 | 36728.2 | 545.5 | 12355.9 | 214980.1 | 215309.0 | 100530.8 | 600 | 91 | 52 | 74.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 131 | 2547.0 | 2907.6 | 3624.0 | 3721.5 |
| solved_with_slack | 64 | 2650.7 | 3052.1 | 3291.7 | 3381.4 |
| normal_contact_contingency | 181 | 3306.7 | 12355.2 | 13113.7 | 139893.6 |
| contact_release_contingency | 34 | 154126.6 | 214235.8 | 215144.2 | 215345.5 |
| precontact_transition | 190 | 3256.4 | 58250.7 | 97452.2 | 113657.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.63 | 11.0 | 13.0 | 16 | 5.36 | 12.0 | 15 | 0.0724 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 120.32/8.0/4245.9/7017 | 26675.16/1776.0/942600.9/1557774 | 1.23/13.0/14 | 0.90/12.0/13 | 7.34/102.0/115 | 0.2815 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 131 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 64 | 6.92/10.4/11 | 3.66/7.4/8 |
| normal_contact_contingency | 181 | 8.94/14.2/16 | 7.83/13.2/15 |
| contact_release_contingency | 34 | 7.82/11.7/12 | 6.62/10.7/11 |
| precontact_transition | 190 | 8.41/12.1/13 | 7.05/12.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.05/6.0/12 | 9.34/28.0/60 | 1.60/6.0/12 | 344 |
| viability | 1.63/6.0/8 | 9.88/40.0/58 | 1.25/6.0/8 | 380 |
| intent | 1.59/5.0/7 | 8.30/26.0/42 | 1.35/5.0/7 | 467 |
| preference | 1.32/5.0/6 | 11.71/48.0/63 | 0.82/5.0/6 | 329 |
| style | 1.03/2.0/4 | 8.66/19.0/25 | 0.34/2.0/3 | 190 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 106 | 299 | 0 | 195 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 385 | 215 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `299` ticks, planned normal touchdown `0` ticks, normal fallback `218` ticks.
Precontact sole-center tangential speed: p50 `1.3867 m/s`, p95 `4.6393 m/s`, max `5.1312 m/s` over 299 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.085 | 8.084 | 8.084 | 1.000 | 1.000 | 48.066 | 48.301 | 0.234 | 49.047 | 0.001 | 0 | 44 | 0 | 0 | 37 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2516.2 | 3075.2 | 5.00 | 33.72 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 4.83e-11 | 0 |
| 60–119 | 2641.7 | 3669.3 | 5.60 | 37.90 | 1.27 | 1.00 | 234.00 | 0.051 | 0.000 | 1.19e-09 | 4.92e-11 | 0 |
| 120–179 | 2611.6 | 2999.3 | 5.92 | 38.25 | 1.67 | 1.00 | 234.00 | 0.934 | 0.000 | 1.20e-09 | 6.01e-11 | 0 |
| 180–239 | 2418.7 | 3222.8 | 7.45 | 42.67 | 4.52 | 1.00 | 225.00 | 5.397 | 1.407 | 9.07e-10 | 3.28e-11 | 45 |
| 240–299 | 3429.4 | 4450.3 | 8.72 | 54.88 | 7.58 | 5.20 | 1154.40 | 6.885 | 25.560 | 7.91e-10 | 1.17e-11 | 60 |
| 300–359 | 3603.4 | 28907.6 | 8.62 | 52.45 | 7.95 | 58.50 | 12987.00 | 9.614 | 35.248 | 6.43e-10 | 9.07e-12 | 60 |
| 360–419 | 2447.9 | 124414.5 | 9.00 | 58.87 | 7.65 | 1100.82 | 244200.60 | 52.366 | 74.566 | 1.11e-09 | 1.85e-11 | 60 |
| 420–479 | 3953.0 | 211655.4 | 8.38 | 52.70 | 7.18 | 22.77 | 4916.80 | 145.083 | 110.496 | 3.55e-09 | 1.32e-10 | 60 |
| 480–539 | 3024.2 | 4528.0 | 8.38 | 53.30 | 7.65 | 5.43 | 1173.60 | 202.667 | 172.661 | 2.21e-09 | 6.84e-11 | 60 |
| 540–599 | 12805.3 | 214985.5 | 9.28 | 54.27 | 8.13 | 6.50 | 1392.20 | 188.964 | 160.001 | 6.65e-09 | 1.27e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.12x | 600 | 100.371 | 86.648 | 109.874 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
