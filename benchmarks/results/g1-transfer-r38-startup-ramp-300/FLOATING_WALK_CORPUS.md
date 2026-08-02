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
| 400 | 2.000 s | 15.405 cm | 17.550 cm | 43.917 cm | 34.521 cm | 102.502° | 8.000 rad/s | 7624.5 µs |

Nominal hard residual maxima: dynamics `2.122e-09`, contact acceleration `7.569e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 173.573 cm |
| authored reference vs measured CoM RMS / p95 | 168.847 / 295.421 cm |
| stance foot RMS | 152.245 cm |
| swing foot RMS | 192.646 cm |
| hand RMS | 185.187 cm |
| maximum root rotation | 179.895° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.607e-09 |
| contact acceleration residual | 3.617e-10 |
| raw max dynamics residual, including rejected ticks | 9.607e-09 |
| raw max contact residual, including rejected ticks | 3.617e-10 |
| active normal force range | 0.000–835.939 N |
| centroidal momentum-rate residual RMS / max | 86.050 / 530.159 N·m |
| point-task acceleration RMS max | 207.089 m/s² |
| frame-angular acceleration RMS max | 292.532 rad/s² |
| longest pre-contact / touchdown transition | 121 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 182 / 182 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `161.997` / `315.247 cm`.
- Virtual ZMP clipped on `68.88%` of ticks; clip-distance RMS / max `252.372` / `639.945 cm`.
- Measured-height natural frequency min / p50 / max: `3.675` / `4.607` / `7.004 rad/s`.
- Minimum measured CoM height: `-1.241 m`; height-floor ticks: `315`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-355.167` / `-318.298 cm`; inside on `31.50%` of ticks.

## Capture-aware landing

- Active target-ticks: `382`; policy updates `200`, frozen `182`.
- Maximum applied offset / root reach: `0.0800 / 0.8576 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 25`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `182 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `1.6272 m / 0.3927 / 0.0345 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 16`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `205`, precontact `382`, multi-support `212`.
- Joint-velocity envelope active on `523` ticks; maximum active coordinates `9`; mean target/applied scale `0.603` / `0.666`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 800 | 4.0 s | 3508.8 µs | 103486.5 µs | 191623.7 µs | 435475.7 µs | 137 | 142 | 121 | 0 | 359 | 41 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12963.5 | 37340.2 | 1017.5 | 12038.4 | 258803.8 | 417808.5 | 107447.9 | 800 | 242 | 51 | 77.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 2552.9 | 4023.3 | 4148.2 | 4287.3 |
| solved_with_slack | 142 | 2556.1 | 3992.0 | 4209.3 | 4412.7 |
| normal_contact_contingency | 359 | 4301.2 | 14780.8 | 41645.0 | 45348.4 |
| contact_release_contingency | 41 | 170017.0 | 213131.9 | 347029.2 | 435475.7 |
| precontact_transition | 121 | 4272.4 | 7406.1 | 13129.8 | 13275.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.08 | 12.0 | 14.0 | 16 | 6.20 | 13.0 | 14 | 0.0689 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 34.98/8.0/2221.3/2801 | 7564.26/1776.0/479796.5/605016 | 2.87/15.0/23 | 2.33/14.0/22 | 19.07/122.0/194 | 0.0803 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 142 | 7.42/13.2/16 | 4.73/11.2/14 |
| normal_contact_contingency | 359 | 9.28/14.0/15 | 8.43/13.0/13 |
| contact_release_contingency | 41 | 8.22/13.0/13 | 7.00/12.0/12 |
| precontact_transition | 121 | 8.76/13.8/15 | 8.03/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.26/6.0/8 | 9.78/27.0/40 | 1.90/6.0/8 | 530 |
| viability | 1.69/6.0/11 | 10.70/42.0/66 | 1.39/6.0/11 | 574 |
| intent | 1.57/5.0/8 | 8.11/24.0/40 | 1.39/5.0/8 | 662 |
| preference | 1.46/6.0/9 | 12.87/50.0/69 | 1.08/6.0/8 | 547 |
| style | 1.10/3.0/5 | 8.89/23.0/35 | 0.44/3.0/4 | 299 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 206 | 382 | 0 | 212 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 400 | 400 |
| left_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `382` ticks, planned normal touchdown `0` ticks, normal fallback `403` ticks.
Precontact sole-center tangential speed: p50 `3.3672 m/s`, p95 `7.7412 m/s`, max `12.4500 m/s` over 382 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10.371 | 10.369 | 10.369 | 1.000 | 1.000 | 48.230 | 49.016 | 0.785 | 49.637 | 0.001 | 0 | 182 | 0 | 0 | 98 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–79 | 2525.3 | 4183.1 | 5.00 | 33.62 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 4.71e-11 | 0 |
| 80–159 | 2620.8 | 4199.8 | 5.49 | 36.52 | 1.00 | 1.00 | 234.00 | 0.186 | 0.000 | 1.33e-09 | 4.67e-11 | 0 |
| 160–239 | 2584.1 | 4178.3 | 7.25 | 45.16 | 4.21 | 1.00 | 229.80 | 4.033 | 0.007 | 1.11e-09 | 4.62e-11 | 0 |
| 240–319 | 3242.1 | 5364.1 | 8.06 | 48.41 | 6.81 | 4.06 | 901.88 | 11.110 | 19.817 | 9.47e-10 | 2.77e-11 | 41 |
| 320–399 | 5051.5 | 13248.0 | 9.18 | 53.14 | 8.53 | 6.86 | 1523.47 | 32.356 | 55.242 | 2.12e-09 | 7.57e-11 | 80 |
| 400–479 | 8385.7 | 259824.1 | 9.46 | 59.36 | 8.71 | 7.47 | 1607.92 | 132.369 | 116.195 | 9.48e-09 | 1.92e-10 | 80 |
| 480–559 | 5960.7 | 206063.8 | 8.99 | 52.48 | 8.12 | 6.78 | 1448.62 | 224.625 | 174.859 | 9.61e-09 | 3.62e-10 | 80 |
| 560–639 | 3542.6 | 12723.0 | 8.61 | 54.90 | 7.92 | 7.21 | 1557.90 | 265.743 | 226.302 | 8.47e-09 | 3.50e-10 | 80 |
| 640–719 | 7202.5 | 42603.4 | 9.93 | 63.40 | 8.81 | 237.11 | 51216.30 | 258.409 | 266.242 | 9.36e-10 | 3.32e-11 | 80 |
| 720–799 | 3579.4 | 44270.6 | 8.85 | 56.55 | 7.86 | 77.26 | 16688.70 | 307.760 | 335.544 | 1.30e-09 | 2.52e-11 | 80 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 800 | 173.573 | 167.990 | 185.187 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
