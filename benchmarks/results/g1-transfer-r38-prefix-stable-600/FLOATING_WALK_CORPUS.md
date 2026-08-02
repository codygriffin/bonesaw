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
| `no_infeasible_or_failed_ticks` | FAIL |
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

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 408 | 2.040 s | 13.893 cm | 0.815 cm | 26.966 cm | 31.256 cm | 70.384° | 8.000 rad/s | 14462.6 µs |

Nominal hard residual maxima: dynamics `8.820e-09`, contact acceleration `3.738e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 104.580 cm |
| authored reference vs measured CoM RMS / p95 | 98.021 / 216.920 cm |
| stance foot RMS | 67.525 cm |
| swing foot RMS | 84.928 cm |
| hand RMS | 122.246 cm |
| maximum root rotation | 138.710° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.691e-09 |
| contact acceleration residual | 3.738e-10 |
| raw max dynamics residual, including rejected ticks | 1.061e-08 |
| raw max contact residual, including rejected ticks | 3.738e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 72.558 / 361.916 N·m |
| point-task acceleration RMS max | 135.393 m/s² |
| frame-angular acceleration RMS max | 343.269 rad/s² |
| longest pre-contact / touchdown transition | 180 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `26.081` / `54.645 cm`.
- Virtual ZMP clipped on `66.83%` of ticks; clip-distance RMS / max `56.346` / `110.303 cm`.
- Measured-height natural frequency min / p50 / max: `3.671` / `3.786` / `7.004 rad/s`.
- Minimum measured CoM height: `-1.447 m`; height-floor ticks: `175`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-66.623` / `-61.736 cm`; inside on `33.17%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8900 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 78`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.5245 m / 1.6875 / 0.0045 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 4`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `290` ticks; maximum active coordinates `10`; mean target/applied scale `0.605` / `0.700`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2512.6 µs | 108562.1 µs | 179211.8 µs | 217662.4 µs | 160 | 68 | 180 | 0 | 82 | 33 | 0 | 77 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 19390.2 | 38900.6 | 451.3 | 52308.1 | 212517.2 | 217147.9 | 99413.6 | 600 | 178 | 146 | 51.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 160 | 2452.8 | 2544.8 | 2571.9 | 3661.1 |
| solved_with_slack | 68 | 2424.9 | 2838.4 | 3901.4 | 3913.7 |
| failed | 77 | 36275.5 | 37900.3 | 38669.2 | 40278.2 |
| normal_contact_contingency | 82 | 12990.4 | 69371.4 | 80202.4 | 113734.3 |
| contact_release_contingency | 33 | 163571.2 | 205896.1 | 214913.7 | 217662.4 |
| precontact_transition | 180 | 2367.4 | 5165.8 | 105377.6 | 106850.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.39 | 13.1 | 15.0 | 15 | 6.00 | 15.0 | 15 | 0.2223 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 219.75/2312.4/4382.1/6609 | 47677.44/499489.2/946525.0/1467198 | 4.47/29.0/29 | 4.21/28.0/28 | 37.49/252.0/252 | 0.2763 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 160 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 68 | 7.43/13.0/13 | 4.41/10.0/10 |
| failed | 77 | 13.78/15.0/15 | 13.78/15.0/15 |
| normal_contact_contingency | 82 | 9.89/13.0/13 | 8.94/13.0/13 |
| contact_release_contingency | 33 | 7.82/10.7/11 | 6.27/9.7/10 |
| precontact_transition | 180 | 8.89/13.0/13 | 7.23/12.2/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.66/9.0/9 | 11.71/41.0/41 | 2.16/9.0/9 | 300 |
| viability | 1.63/6.0/9 | 9.26/36.0/56 | 1.21/6.0/9 | 369 |
| intent | 1.53/5.0/9 | 7.24/29.0/45 | 1.23/5.0/8 | 428 |
| preference | 1.36/6.0/7 | 12.46/53.0/69 | 0.88/5.0/7 | 334 |
| style | 1.21/4.0/5 | 9.98/34.0/34 | 0.54/4.0/4 | 204 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 408 | 192 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `195` ticks.
Precontact sole-center tangential speed: p50 `3.3149 m/s`, p95 `7.2630 m/s`, max `8.3657 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11.634 | 11.632 | 11.632 | 1.000 | 1.000 | 48.176 | 48.617 | 0.441 | 49.305 | 0.001 | 0 | 93 | 0 | 0 | 181 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2464.3 | 2571.9 | 5.00 | 32.48 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.23e-09 | 5.86e-11 | 0 |
| 60–119 | 2452.8 | 3002.7 | 5.00 | 31.78 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.44e-09 | 5.70e-11 | 0 |
| 120–179 | 2448.6 | 3902.8 | 6.50 | 36.20 | 2.32 | 1.00 | 234.00 | 0.213 | 0.000 | 1.25e-09 | 5.52e-11 | 0 |
| 180–239 | 1857.4 | 2712.1 | 7.03 | 40.18 | 3.72 | 1.00 | 225.80 | 2.475 | 0.039 | 1.10e-09 | 4.65e-11 | 12 |
| 240–299 | 2258.0 | 3002.2 | 8.25 | 50.35 | 5.72 | 1.00 | 222.00 | 2.795 | 8.209 | 1.08e-09 | 1.74e-11 | 60 |
| 300–359 | 2439.2 | 4854.1 | 8.85 | 55.57 | 7.77 | 2.75 | 610.50 | 14.095 | 24.589 | 1.01e-09 | 1.64e-11 | 60 |
| 360–419 | 2915.4 | 109672.6 | 9.95 | 65.10 | 9.23 | 558.90 | 122768.50 | 42.061 | 28.955 | 8.82e-09 | 3.74e-10 | 60 |
| 420–479 | 54403.3 | 205949.0 | 9.73 | 60.90 | 8.65 | 1615.78 | 349005.10 | 120.235 | 53.359 | 9.69e-09 | 1.26e-10 | 60 |
| 480–539 | 37620.4 | 205854.4 | 9.92 | 56.20 | 8.95 | 7.07 | 1512.50 | 202.461 | 152.065 | 1.06e-08 | 4.37e-11 | 60 |
| 540–599 | 36141.0 | 37997.1 | 13.70 | 77.70 | 13.70 | 8.00 | 1728.00 | 227.906 | 163.990 | 1.06e-08 | 4.27e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 104.580 | 73.739 | 122.246 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
