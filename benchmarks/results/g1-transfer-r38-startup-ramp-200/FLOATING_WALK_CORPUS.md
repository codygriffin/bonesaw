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
| 448 | 2.240 s | 12.752 cm | 10.364 cm | 34.146 cm | 37.566 cm | 54.142° | 8.000 rad/s | 4366.6 µs |

Nominal hard residual maxima: dynamics `1.665e-09`, contact acceleration `5.674e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 126.942 cm |
| authored reference vs measured CoM RMS / p95 | 119.416 / 238.880 cm |
| stance foot RMS | 89.629 cm |
| swing foot RMS | 116.804 cm |
| hand RMS | 137.260 cm |
| maximum root rotation | 179.152° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.608e-09 |
| contact acceleration residual | 2.413e-10 |
| raw max dynamics residual, including rejected ticks | 8.608e-09 |
| raw max contact residual, including rejected ticks | 2.413e-10 |
| active normal force range | 0.000–820.932 N |
| centroidal momentum-rate residual RMS / max | 60.074 / 328.553 N·m |
| point-task acceleration RMS max | 125.865 m/s² |
| frame-angular acceleration RMS max | 330.124 rad/s² |
| longest pre-contact / touchdown transition | 220 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 204 / 182 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `92.064` / `197.367 cm`.
- Virtual ZMP clipped on `68.62%` of ticks; clip-distance RMS / max `144.513` / `380.417 cm`.
- Measured-height natural frequency min / p50 / max: `3.627` / `3.847` / `7.004 rad/s`.
- Minimum measured CoM height: `-1.665 m`; height-floor ticks: `319`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-248.895` / `-190.284 cm`; inside on `34.12%` of ticks.

## Capture-aware landing

- Active target-ticks: `572`; policy updates `368`, frozen `204`.
- Maximum applied offset / root reach: `0.0800 / 0.8624 m`.
- Authored-offset / reach / slew limited ticks: `368 / 0 / 81`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `204 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.3820 m / 0.8308 / 0.0538 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 11`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `571`, multi-support `199`.
- Joint-velocity envelope active on `450` ticks; maximum active coordinates `8`; mean target/applied scale `0.479` / `0.557`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 800 | 4.0 s | 2622.7 µs | 72224.7 µs | 107625.4 µs | 179823.5 µs | 130 | 98 | 220 | 0 | 342 | 10 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8819.7 | 23490.7 | 456.9 | 5523.0 | 172926.2 | 179133.8 | 100108.9 | 800 | 89 | 53 | 113.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 130 | 2524.7 | 4028.8 | 4192.0 | 4316.4 |
| solved_with_slack | 98 | 2541.8 | 2914.2 | 3461.7 | 4411.1 |
| normal_contact_contingency | 342 | 3134.2 | 89078.0 | 108937.6 | 171191.1 |
| contact_release_contingency | 10 | 107318.9 | 173788.4 | 178616.5 | 179823.5 |
| precontact_transition | 220 | 2443.3 | 3722.8 | 30398.5 | 51114.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.19 | 12.0 | 14.0 | 24 | 6.15 | 13.0 | 20 | 0.2064 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 279.70/2954.2/6369.2/6834 | 60483.76/647287.2/1375749.4/1476144 | 1.02/9.0/13 | 0.73/8.0/12 | 5.85/64.0/97 | 0.7820 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 130 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 98 | 7.03/13.3/24 | 3.83/10.3/20 |
| normal_contact_contingency | 342 | 9.46/14.0/15 | 8.40/13.0/14 |
| contact_release_contingency | 10 | 7.20/8.9/9 | 5.70/7.0/7 |
| precontact_transition | 220 | 8.66/13.0/13 | 7.34/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.26/6.0/9 | 9.89/30.0/44 | 1.85/6.0/9 | 486 |
| viability | 1.71/6.0/8 | 11.19/42.0/56 | 1.41/6.0/8 | 573 |
| intent | 1.63/5.0/8 | 8.46/28.0/38 | 1.45/5.0/8 | 668 |
| preference | 1.55/6.0/19 | 14.25/63.0/133 | 1.10/6.0/18 | 513 |
| style | 1.05/2.0/3 | 8.41/18.0/33 | 0.34/2.0/3 | 250 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 572 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 448 | 352 |
| left_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `572` ticks, planned normal touchdown `0` ticks, normal fallback `355` ticks.
Precontact sole-center tangential speed: p50 `3.0410 m/s`, p95 `6.3647 m/s`, max `10.8131 m/s` over 572 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.056 | 7.055 | 7.055 | 1.000 | 1.000 | 48.059 | 48.770 | 0.711 | 50.352 | 0.001 | 0 | 181 | 0 | 0 | 52 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–79 | 2515.2 | 4245.0 | 5.00 | 33.77 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.30e-09 | 4.97e-11 | 0 |
| 80–159 | 2551.8 | 3009.7 | 5.62 | 37.71 | 1.18 | 1.00 | 234.00 | 0.533 | 0.000 | 1.11e-09 | 5.67e-11 | 0 |
| 160–239 | 2519.2 | 3637.8 | 7.50 | 47.60 | 4.74 | 1.00 | 227.85 | 5.516 | 0.825 | 1.66e-09 | 3.39e-11 | 12 |
| 240–319 | 2564.7 | 3566.9 | 8.66 | 55.52 | 6.97 | 1.00 | 222.00 | 5.005 | 24.161 | 1.40e-09 | 2.02e-11 | 80 |
| 320–399 | 2424.6 | 4310.6 | 8.62 | 52.91 | 7.58 | 2.58 | 571.65 | 14.147 | 22.990 | 9.12e-10 | 1.94e-11 | 80 |
| 400–479 | 3003.3 | 129809.3 | 9.51 | 60.20 | 8.49 | 2229.94 | 482283.90 | 43.432 | 38.776 | 1.03e-09 | 1.95e-11 | 80 |
| 480–559 | 4574.1 | 169228.6 | 9.24 | 58.44 | 8.04 | 542.54 | 117186.68 | 137.756 | 90.269 | 6.95e-09 | 2.41e-10 | 80 |
| 560–639 | 2921.7 | 107149.7 | 9.15 | 58.29 | 8.20 | 6.95 | 1500.00 | 180.340 | 135.135 | 4.41e-09 | 1.74e-10 | 80 |
| 640–719 | 3250.1 | 5151.8 | 9.38 | 58.94 | 8.28 | 5.81 | 1255.50 | 184.390 | 152.327 | 6.45e-10 | 6.71e-12 | 80 |
| 720–799 | 2779.5 | 106532.1 | 9.24 | 58.61 | 8.04 | 5.20 | 1122.00 | 271.113 | 220.806 | 8.61e-09 | 1.83e-10 | 80 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 800 | 126.942 | 100.491 | 137.260 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
