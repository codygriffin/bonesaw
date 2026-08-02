# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- CoM task: `support-centroid-preview` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Touchdown admission limits: `0.025 m` position, `0.200 m/s` tangential, and `0.200 m/s` normal; the outgoing support is retained until its replacement locks.
- Reference touchdown blend: `0` ticks (`0.000 s`) with a quintic zero-velocity endpoint.
- Touchdown stabilization point: `ankle origin`.

## Acceptance

Functional: **FAIL**  
5 ms p99 deadline: **PASS**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | PASS |

Failed checks: `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.500 s | 4.051 cm | 0.253 cm | 8.521 cm | 23.813 cm | 11.830° | 8.000 rad/s | 3722.5 µs |

Nominal hard residual maxima: dynamics `1.655e-09`, contact acceleration `5.735e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 4.051 cm |
| CoM RMS / p95 | 3.618 / 7.745 cm |
| stance foot RMS | 0.253 cm |
| swing foot RMS | 8.521 cm |
| hand RMS | 23.813 cm |
| maximum root rotation | 11.830° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.655e-09 |
| contact acceleration residual | 5.735e-11 |
| raw max dynamics residual, including rejected ticks | 1.655e-09 |
| raw max contact residual, including rejected ticks | 5.735e-11 |
| active normal force range | 0.000–373.671 N |
| centroidal momentum-rate residual RMS / max | 15.172 / 94.382 N·m |
| point-task acceleration RMS max | 117.043 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `2.489` / `7.287 cm`.
- Virtual ZMP clipped on `19.00%` of ticks; clip-distance RMS / max `2.145` / `14.425 cm`.
- Measured-height natural frequency min / p50 / max: `3.705` / `3.774` / `3.818 rad/s`.
- CoM command acceleration p95 / max: `7.407` / `11.414 m/s²`; support hull `4–4` vertices.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 | 1.5 s | 2467.2 µs | 3535.3 µs | 3722.5 µs | 4059.7 µs | 158 | 142 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2592.0 | 457.4 | 87.9 | 3206.6 | 3979.9 | 4051.7 | 1398.5 | 300 | 0 | 0 | 385.8 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 158 | 2439.5 | 2540.4 | 2592.8 | 2615.9 |
| solved_with_slack | 142 | 2956.1 | 3645.7 | 3791.4 | 4059.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.81 | 13.0 | 17.0 | 17 | 3.17 | 14.0 | 16 | 0.2987 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 2.77/8.0/8.0/8 | 623.64/1776.0/1776.0/1776 | 0.56/3.0/3 | 0.30/2.0/2 | 2.42/16.0/17 | 0.7774 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 158 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 142 | 8.82/17.0/17 | 6.69/15.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.26/3.0/7 | 4.51/12.0/28 | 0.37/3.0/7 | 60 |
| viability | 2.00/11.0/13 | 5.92/30.0/35 | 1.30/11.0/12 | 109 |
| intent | 1.49/5.0/7 | 7.71/25.0/40 | 0.96/5.0/7 | 141 |
| preference | 1.04/3.0/4 | 11.19/31.0/44 | 0.36/2.0/4 | 98 |
| style | 1.02/2.0/2 | 10.05/18.0/28 | 0.18/1.0/2 | 50 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 101 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 300 | 0 |
| left_wrist_roll_rubber_hand | 300 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 300 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.778 | 0.778 | 0.778 | 1.000 | 1.000 | 48.375 | 48.375 | 0.000 | 48.375 | 0.001 | 0 | 0 | 0 | 0 | 8 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–29 | 2454.4 | 2577.7 | 5.00 | 32.30 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.41e-09 | 4.34e-11 | 0 |
| 30–59 | 2417.1 | 2597.2 | 5.00 | 32.23 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.65e-09 | 5.16e-11 | 0 |
| 60–89 | 2447.3 | 2549.9 | 5.00 | 32.27 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.08e-09 | 5.33e-11 | 0 |
| 90–119 | 2433.3 | 2517.8 | 5.00 | 31.37 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.03e-09 | 3.21e-11 | 0 |
| 120–149 | 2457.8 | 2584.9 | 5.00 | 31.50 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.25e-09 | 5.60e-11 | 0 |
| 150–179 | 2458.2 | 3221.0 | 10.70 | 44.27 | 7.83 | 1.00 | 234.00 | 0.281 | 0.000 | 1.39e-09 | 4.94e-11 | 0 |
| 180–209 | 2446.7 | 3908.3 | 8.27 | 45.37 | 5.60 | 1.00 | 229.60 | 2.881 | 0.008 | 7.20e-10 | 5.74e-11 | 0 |
| 210–239 | 2945.8 | 3632.6 | 7.00 | 44.07 | 4.00 | 5.20 | 1154.40 | 5.226 | 0.074 | 7.53e-10 | 8.24e-12 | 0 |
| 240–269 | 3108.5 | 3791.9 | 8.30 | 49.10 | 6.87 | 8.00 | 1776.00 | 7.110 | 2.631 | 8.65e-11 | 2.72e-12 | 0 |
| 270–299 | 3131.5 | 3716.1 | 8.83 | 51.37 | 7.37 | 7.53 | 1672.40 | 8.826 | 10.762 | 3.54e-10 | 4.60e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 300 | 4.051 | 3.503 | 23.813 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
