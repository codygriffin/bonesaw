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
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `10.000` / `1.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `multi-support` measured-phase policy with immediate engagement and bounded release over `200` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
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
| 444 | 2.220 s | 8.955 cm | 8.281 cm | 30.245 cm | 36.688 cm | 78.365° | 8.000 rad/s | 90441.0 µs |

Nominal hard residual maxima: dynamics `1.309e-09`, contact acceleration `5.103e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 73.088 cm |
| authored reference vs measured CoM RMS / p95 | 69.376 / 162.614 cm |
| stance foot RMS | 39.158 cm |
| swing foot RMS | 46.486 cm |
| hand RMS | 90.568 cm |
| maximum root rotation | 162.077° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.661e-09 |
| contact acceleration residual | 2.912e-10 |
| raw max dynamics residual, including rejected ticks | 9.661e-09 |
| raw max contact residual, including rejected ticks | 2.912e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 58.463 / 297.707 N·m |
| point-task acceleration RMS max | 97.682 m/s² |
| frame-angular acceleration RMS max | 221.143 rad/s² |
| longest pre-contact / touchdown transition | 216 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `37.507` / `99.558 cm`.
- Virtual ZMP clipped on `54.17%` of ticks; clip-distance RMS / max `54.214` / `209.203 cm`.
- Measured-height natural frequency min / p50 / max: `3.680` / `3.769` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.807 m`; height-floor ticks: `145`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `318` ticks; maximum active coordinates `8`; mean phase scale `0.497`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2614.0 µs | 63544.7 µs | 114800.4 µs | 187645.4 µs | 130 | 98 | 216 | 0 | 145 | 11 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10162.2 | 24921.4 | 408.2 | 7949.5 | 183711.5 | 187252.0 | 75710.9 | 600 | 93 | 55 | 98.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 130 | 2522.2 | 2692.6 | 2741.8 | 2811.6 |
| solved_with_slack | 98 | 2551.8 | 2739.7 | 3014.9 | 3074.8 |
| normal_contact_contingency | 145 | 4036.1 | 55410.5 | 104751.5 | 125516.1 |
| contact_release_contingency | 11 | 105678.4 | 184361.6 | 186988.7 | 187645.4 |
| precontact_transition | 216 | 2925.0 | 68167.4 | 104069.2 | 114771.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.66 | 11.0 | 12.0 | 13 | 5.40 | 12.0 | 13 | 0.2403 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 299.54/3028.4/5960.7/7084 | 66039.49/672304.8/1287876.7/1572648 | 1.24/9.0/21 | 0.94/8.0/20 | 7.50/64.0/162 | 0.6763 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 130 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 98 | 6.66/11.0/12 | 3.34/8.0/9 |
| normal_contact_contingency | 145 | 9.24/12.0/13 | 8.40/12.0/12 |
| contact_release_contingency | 11 | 8.18/9.0/9 | 7.09/8.0/8 |
| precontact_transition | 216 | 8.62/12.8/13 | 7.48/11.8/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.97/6.0/7 | 8.16/24.0/35 | 1.48/6.0/7 | 321 |
| viability | 1.74/6.0/8 | 10.86/42.0/56 | 1.36/6.0/8 | 383 |
| intent | 1.55/4.0/7 | 8.26/24.0/36 | 1.30/4.0/7 | 470 |
| preference | 1.36/5.0/7 | 12.30/49.0/64 | 0.89/5.0/6 | 353 |
| style | 1.04/2.0/3 | 8.99/18.0/28 | 0.36/2.0/3 | 203 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 444 | 156 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `159` ticks.
Precontact sole-center tangential speed: p50 `2.3627 m/s`, p95 `4.0443 m/s`, max `4.9406 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.098 | 6.095 | 6.095 | 1.000 | 1.000 | 46.629 | 46.668 | 0.039 | 48.547 | 0.001 | 0 | 9 | 0 | 0 | 142 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2504.1 | 2772.6 | 5.00 | 32.88 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2571.3 | 2852.4 | 5.32 | 35.43 | 0.70 | 1.00 | 234.00 | 0.157 | 0.000 | 9.05e-10 | 4.00e-11 | 0 |
| 120–179 | 2554.6 | 3007.8 | 5.87 | 37.95 | 1.72 | 1.00 | 234.00 | 1.407 | 0.000 | 1.19e-09 | 5.10e-11 | 0 |
| 180–239 | 2330.3 | 2819.2 | 7.13 | 44.32 | 4.37 | 1.00 | 225.80 | 6.250 | 1.810 | 9.60e-10 | 2.83e-11 | 12 |
| 240–299 | 2631.3 | 4714.8 | 8.12 | 51.98 | 6.70 | 2.98 | 662.30 | 6.358 | 22.085 | 1.26e-09 | 1.59e-11 | 60 |
| 300–359 | 2400.4 | 5150.9 | 8.45 | 52.47 | 7.43 | 1.47 | 325.60 | 4.756 | 29.004 | 9.45e-10 | 1.34e-11 | 60 |
| 360–419 | 3691.2 | 51113.3 | 8.95 | 53.17 | 8.02 | 439.70 | 97613.40 | 11.282 | 18.178 | 3.62e-10 | 4.84e-12 | 60 |
| 420–479 | 53183.6 | 119176.9 | 9.85 | 63.20 | 8.60 | 2516.53 | 554229.50 | 61.166 | 55.254 | 8.62e-10 | 1.02e-11 | 60 |
| 480–539 | 5755.6 | 183770.6 | 8.63 | 54.83 | 7.87 | 22.75 | 4908.30 | 151.348 | 65.169 | 9.66e-09 | 2.91e-10 | 60 |
| 540–599 | 3506.7 | 7259.5 | 9.27 | 59.55 | 8.58 | 8.00 | 1728.00 | 162.911 | 91.908 | 5.93e-09 | 1.87e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 73.088 | 41.725 | 90.568 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
