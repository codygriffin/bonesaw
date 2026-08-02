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
- Joint-velocity envelope: `viability` priority with weight `0.120`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
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
| 397 | 1.985 s | 8.390 cm | 6.277 cm | 34.147 cm | 33.897 cm | 57.709° | 8.000 rad/s | 93318.1 µs |

Nominal hard residual maxima: dynamics `1.516e-09`, contact acceleration `5.041e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 88.434 cm |
| authored reference vs measured CoM RMS / p95 | 85.350 / 177.992 cm |
| stance foot RMS | 71.131 cm |
| swing foot RMS | 64.913 cm |
| hand RMS | 106.605 cm |
| maximum root rotation | 137.928° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.121e-09 |
| contact acceleration residual | 1.567e-10 |
| raw max dynamics residual, including rejected ticks | 7.121e-09 |
| raw max contact residual, including rejected ticks | 1.567e-10 |
| active normal force range | 0.000–589.697 N |
| centroidal momentum-rate residual RMS / max | 59.442 / 301.158 N·m |
| point-task acceleration RMS max | 108.588 m/s² |
| frame-angular acceleration RMS max | 240.228 rad/s² |
| longest pre-contact / touchdown transition | 169 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `64.932` / `160.532 cm`.
- Virtual ZMP clipped on `62.33%` of ticks; clip-distance RMS / max `100.689` / `285.751 cm`.
- Measured-height natural frequency min / p50 / max: `3.690` / `3.772` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.902 m`; height-floor ticks: `182`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-187.930` / `-163.960 cm`; inside on `39.33%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8604 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 43`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.8590 m / 0.6978 / 0.0354 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 6`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `388` ticks; maximum active coordinates `9`; mean target/applied scale `0.584` / `0.661`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2883.2 µs | 44267.7 µs | 110795.2 µs | 204550.0 µs | 129 | 99 | 169 | 0 | 195 | 8 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8839.7 | 22876.7 | 507.2 | 6991.8 | 203965.6 | 204491.5 | 56538.4 | 600 | 95 | 47 | 113.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2539.3 | 3805.5 | 4099.9 | 4598.7 |
| solved_with_slack | 99 | 2616.7 | 2969.0 | 3621.5 | 3833.0 |
| normal_contact_contingency | 195 | 3569.7 | 21594.1 | 42857.5 | 113796.4 |
| contact_release_contingency | 8 | 138845.7 | 204208.5 | 204481.7 | 204550.0 |
| precontact_transition | 169 | 3428.9 | 79278.3 | 103599.6 | 109761.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.61 | 11.0 | 13.0 | 15 | 5.36 | 12.0 | 14 | 0.1264 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 232.42/1853.1/5554.4/6768 | 51269.48/404259.6/1229077.3/1502496 | 1.23/8.0/14 | 0.87/7.0/13 | 7.14/61.0/111 | 0.6302 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.99/12.0/14 | 3.95/11.0/11 |
| normal_contact_contingency | 195 | 9.01/13.0/13 | 8.04/12.0/12 |
| contact_release_contingency | 8 | 7.50/10.8/11 | 6.38/9.8/10 |
| precontact_transition | 169 | 8.37/12.3/15 | 7.12/12.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.08/6.0/7 | 9.32/27.0/35 | 1.61/6.0/7 | 335 |
| viability | 1.59/6.0/9 | 9.99/36.0/63 | 1.22/6.0/9 | 379 |
| intent | 1.52/5.0/8 | 8.39/25.0/42 | 1.29/5.0/8 | 471 |
| preference | 1.38/6.0/9 | 12.42/56.0/68 | 0.90/6.0/9 | 336 |
| style | 1.04/2.0/3 | 9.02/21.0/27 | 0.34/2.0/3 | 189 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 397 | 203 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `206` ticks.
Precontact sole-center tangential speed: p50 `2.0798 m/s`, p95 `7.6075 m/s`, max `8.6953 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.304 | 5.304 | 5.304 | 1.000 | 1.000 | 46.957 | 47.383 | 0.426 | 49.059 | 0.001 | 0 | 93 | 0 | 0 | 18 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2524.5 | 4311.4 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2622.2 | 3767.1 | 5.47 | 37.33 | 0.92 | 1.00 | 234.00 | 0.112 | 0.000 | 9.54e-10 | 4.10e-11 | 0 |
| 120–179 | 2598.6 | 3267.8 | 6.05 | 39.92 | 2.08 | 1.00 | 234.00 | 1.274 | 0.000 | 1.12e-09 | 4.38e-11 | 0 |
| 180–239 | 2407.9 | 4537.7 | 7.30 | 44.77 | 4.65 | 1.70 | 381.20 | 6.748 | 3.606 | 1.52e-09 | 4.03e-11 | 12 |
| 240–299 | 2830.0 | 17714.6 | 8.17 | 51.32 | 6.65 | 32.88 | 7300.10 | 6.651 | 23.154 | 1.00e-09 | 2.09e-11 | 60 |
| 300–359 | 3560.9 | 19014.9 | 8.55 | 55.97 | 7.63 | 44.03 | 9775.40 | 5.069 | 30.396 | 7.02e-10 | 1.92e-11 | 60 |
| 360–419 | 3681.3 | 111415.7 | 8.42 | 54.80 | 7.30 | 1693.25 | 375884.50 | 36.585 | 46.731 | 6.77e-10 | 1.52e-11 | 60 |
| 420–479 | 3847.1 | 203974.4 | 8.80 | 53.78 | 7.75 | 101.42 | 21905.20 | 124.078 | 92.633 | 6.02e-09 | 8.19e-11 | 60 |
| 480–539 | 4047.9 | 7414.0 | 8.38 | 52.27 | 7.52 | 6.72 | 1450.80 | 180.597 | 124.615 | 7.12e-09 | 1.57e-10 | 60 |
| 540–599 | 4213.4 | 39050.9 | 10.00 | 67.68 | 9.07 | 441.18 | 95295.60 | 169.525 | 141.528 | 1.01e-09 | 5.83e-12 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 88.434 | 69.135 | 106.605 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
