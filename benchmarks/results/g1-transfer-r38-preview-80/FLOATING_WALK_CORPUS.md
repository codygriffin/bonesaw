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
| 465 | 2.325 s | 15.959 cm | 18.291 cm | 17.776 cm | 38.315 cm | 38.644° | 8.000 rad/s | 7870.9 µs |

Nominal hard residual maxima: dynamics `7.691e-09`, contact acceleration `3.586e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 89.408 cm |
| authored reference vs measured CoM RMS / p95 | 85.697 / 238.989 cm |
| stance foot RMS | 64.291 cm |
| swing foot RMS | 99.645 cm |
| hand RMS | 110.904 cm |
| maximum root rotation | 112.576° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.691e-09 |
| contact acceleration residual | 3.586e-10 |
| raw max dynamics residual, including rejected ticks | 1.016e-08 |
| raw max contact residual, including rejected ticks | 3.586e-10 |
| active normal force range | 0.000–863.275 N |
| centroidal momentum-rate residual RMS / max | 82.933 / 385.228 N·m |
| point-task acceleration RMS max | 159.966 m/s² |
| frame-angular acceleration RMS max | 354.171 rad/s² |
| longest pre-contact / touchdown transition | 237 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `99.578` / `231.543 cm`.
- Virtual ZMP clipped on `65.00%` of ticks; clip-distance RMS / max `150.331` / `389.522 cm`.
- Measured-height natural frequency min / p50 / max: `3.694` / `3.779` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.441 m`; height-floor ticks: `64`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-184.105` / `-162.533 cm`; inside on `39.00%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8512 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 44`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.3782 m / 1.1043 / 0.0051 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 9`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `406` ticks; maximum active coordinates `11`; mean target/applied scale `0.675` / `0.745`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2509.4 µs | 17698.1 µs | 156431.6 µs | 206353.4 µs | 163 | 65 | 237 | 0 | 104 | 8 | 0 | 23 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5798.0 | 19630.7 | 347.4 | 4390.1 | 191965.3 | 204914.6 | 28560.5 | 600 | 46 | 9 | 172.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 163 | 2477.1 | 2648.2 | 3007.5 | 3065.3 |
| solved_with_slack | 65 | 2492.5 | 2853.9 | 3169.3 | 3512.5 |
| failed | 23 | 17951.8 | 18014.4 | 18038.9 | 18045.8 |
| normal_contact_contingency | 104 | 2738.4 | 9083.2 | 13822.7 | 182333.3 |
| contact_release_contingency | 8 | 160025.0 | 196853.5 | 204453.4 | 206353.4 |
| precontact_transition | 237 | 2635.0 | 4600.6 | 11785.6 | 17748.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.61 | 12.0 | 13.0 | 15 | 5.21 | 12.0 | 14 | 0.0592 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 6.92/8.0/8.0/689 | 1509.36/1776.0/1776.0/148824 | 1.39/20.0/20 | 1.13/19.0/19 | 9.62/166.0/166 | 0.0361 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 163 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 65 | 7.09/10.0/10 | 3.91/7.0/7 |
| failed | 23 | 9.00/9.0/9 | 9.00/9.0/9 |
| normal_contact_contingency | 104 | 9.38/14.0/15 | 8.43/13.0/14 |
| contact_release_contingency | 8 | 7.50/8.9/9 | 5.50/7.0/7 |
| precontact_transition | 237 | 8.63/13.0/14 | 7.36/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.89/5.0/8 | 7.92/25.0/32 | 1.35/5.0/8 | 312 |
| viability | 1.63/6.0/9 | 9.98/42.0/64 | 1.23/6.0/9 | 368 |
| intent | 1.73/6.0/7 | 8.33/28.0/41 | 1.43/6.0/7 | 433 |
| preference | 1.29/6.0/7 | 11.29/52.0/67 | 0.82/5.0/6 | 338 |
| style | 1.07/3.0/5 | 9.24/23.0/32 | 0.38/2.0/5 | 194 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 465 | 135 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `138` ticks.
Precontact sole-center tangential speed: p50 `2.3392 m/s`, p95 `7.4224 m/s`, max `8.3987 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.479 | 3.476 | 3.476 | 0.999 | 0.999 | 47.098 | 47.539 | 0.441 | 49.410 | 0.001 | 0 | 94 | 0 | 0 | 230 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2482.3 | 2937.0 | 5.00 | 32.48 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.23e-09 | 5.86e-11 | 0 |
| 60–119 | 2476.6 | 2584.6 | 5.00 | 31.78 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.44e-09 | 5.70e-11 | 0 |
| 120–179 | 2496.6 | 3036.4 | 5.62 | 33.38 | 1.33 | 1.00 | 234.00 | 1.308 | 0.000 | 1.06e-09 | 4.67e-11 | 0 |
| 180–239 | 2262.8 | 3688.1 | 7.38 | 44.92 | 4.25 | 1.00 | 225.80 | 5.930 | 0.318 | 1.18e-09 | 3.07e-11 | 12 |
| 240–299 | 2426.4 | 3993.1 | 8.38 | 49.75 | 6.88 | 1.82 | 403.30 | 7.151 | 10.611 | 7.35e-10 | 1.33e-11 | 60 |
| 300–359 | 3734.0 | 4882.2 | 8.83 | 50.92 | 7.12 | 7.53 | 1672.40 | 6.471 | 17.808 | 5.07e-10 | 4.13e-12 | 60 |
| 360–419 | 2210.6 | 3761.8 | 8.40 | 48.92 | 7.40 | 1.82 | 403.30 | 21.285 | 13.325 | 1.00e-09 | 1.35e-11 | 60 |
| 420–479 | 2736.4 | 85228.4 | 9.12 | 56.82 | 8.57 | 3.68 | 815.50 | 49.361 | 56.958 | 7.69e-09 | 3.59e-10 | 60 |
| 480–539 | 2765.3 | 4225.6 | 9.30 | 61.87 | 8.40 | 3.45 | 745.20 | 133.572 | 103.456 | 1.23e-09 | 7.62e-12 | 60 |
| 540–599 | 17658.6 | 190339.3 | 9.05 | 56.82 | 8.15 | 46.90 | 10126.10 | 243.058 | 214.366 | 1.02e-08 | 1.16e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 89.408 | 77.787 | 110.904 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
