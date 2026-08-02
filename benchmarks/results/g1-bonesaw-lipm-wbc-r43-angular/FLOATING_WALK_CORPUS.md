# Bonesaw floating G1 admitted-reference tracking

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: immutable open-loop-admitted artifact `benchmarks/results/g1-bonesaw-lipm-r43/reference-inputs.npz` from `Bonesaw two-stage support-constrained LIPM`.
- Admission contract: authored root, CoM, foot jets, and contact schedule are consumed unchanged; eval-side reconstruction, projection, and retiming are rejected.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.228 m` forward per `3.000 s` source cycle.
- Applied mean forward speed: `0.076 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `immutable authored tick sequence` in `3.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `rooted` `CoM` reference at `viability` priority with weight `1.000` and `0.500 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `2.000 Hz` critically damped; root angular/height/horizontal task weights `100.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `viability` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `0` ticks (`0.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `disabled`; authored offset ≤ `0.120 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `40` scheduled ticks and throughout delayed admission.
- Coupled touchdown phase retiming: `disabled`; minimum rate `0.000`, guard `0.020 s`, engagement / release `0 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
- Coupled balance phase retiming: `disabled`; hold at signed DCM margin ≤ `-0.020 m`, recover nominal rate at `0.000 m`.
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
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 286 | 1.430 s | 21.667 cm | 22.849 cm | 83.606 cm | 42.764 cm | 92.101° | 8.000 rad/s | 56676.3 µs |

Nominal hard residual maxima: dynamics `1.654e-09`, contact acceleration `5.593e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 236.319 cm |
| authored reference vs measured CoM RMS / p95 | 234.954 / 487.870 cm |
| stance foot RMS | 252.761 cm |
| swing foot RMS | 165.239 cm |
| hand RMS | 255.235 cm |
| maximum root rotation | 179.834° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.171e-09 |
| contact acceleration residual | 4.887e-10 |
| raw max dynamics residual, including rejected ticks | 8.171e-09 |
| raw max contact residual, including rejected ticks | 4.887e-10 |
| active normal force range | 0.000–931.566 N |
| centroidal momentum-rate residual RMS / max | 78.791 / 387.291 N·m |
| point-task acceleration RMS max | 179.533 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `False` (touchdown `False`, balance `False`); final source tick `599.000`, progress `599.000` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `True`.
- Target / applied minimum rate: `1.0000` / `1.0000`; mean / p50 applied `1.0000` / `1.0000`.
- Limited / zero-rate hold ticks: `0` / `0`; maximum required landing time `0.0000 s`.
- Position / tangential / normal limiting ticks: `0` / `0` / `0`; unsafe-edge ticks `0`.
- Balance-margin limited ticks: `0`.

## Capture-aware landing

- Active target-ticks: `0`; policy updates `0`, frozen `0`.
- Maximum applied offset / root reach: `0.0000 / 0.0000 m`.
- Authored-offset / reach / slew limited ticks: `0 / 0 / 0`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `229`, precontact `0`, multi-support `370`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 5352.5 µs | 281316.0 µs | 288586.4 µs | 331561.7 µs | 0 | 286 | 0 | 0 | 149 | 165 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 69213.8 | 103778.3 | 2921.8 | 270762.0 | 310022.2 | 329407.8 | 157664.0 | 600 | 318 | 211 | 14.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved_with_slack | 286 | 2944.9 | 30207.4 | 56676.3 | 64491.1 |
| normal_contact_contingency | 149 | 6794.7 | 40934.1 | 115092.8 | 219155.1 |
| contact_release_contingency | 165 | 252120.0 | 286948.5 | 294223.0 | 331561.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.89 | 13.0 | 16.0 | 23 | 7.67 | 15.0 | 22 | -0.2231 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 201.27/1844.0/3739.5/6873 | 44012.82/409368.0/830162.5/1525806 | 2.43/14.0/34 | 1.87/13.0/33 | 15.20/104.2/296 | -0.0366 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved_with_slack | 286 | 9.05/16.1/23 | 7.60/15.3/22 |
| normal_contact_contingency | 149 | 9.69/15.5/20 | 8.84/14.0/20 |
| contact_release_contingency | 165 | 7.90/11.0/12 | 6.72/10.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.93/8.0/12 | 11.93/32.0/48 | 2.76/8.0/12 | 528 |
| viability | 2.06/6.0/10 | 13.23/40.1/60 | 2.04/6.0/10 | 600 |
| intent | 1.47/5.0/8 | 2.92/10.0/16 | 1.40/5.0/8 | 566 |
| preference | 1.27/6.0/13 | 13.26/66.0/121 | 1.03/6.0/13 | 468 |
| style | 1.17/3.0/5 | 9.02/36.0/53 | 0.44/3.0/4 | 222 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 229 | 0 | 172 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 286 | 314 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `172` ticks, normal fallback `317` ticks.
Touchdown Normal sole-center tangential speed: p50 `6.0077 m/s`, p95 `8.5947 m/s`, max `8.7177 m/s` over 171 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 41.529 | 41.519 | 41.519 | 1.000 | 1.000 | 45.348 | 46.102 | 0.754 | 46.102 | 0.001 | 0 | 173 | 0 | 0 | 609 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2662.4 | 4780.4 | 7.80 | 49.47 | 5.27 | 2.98 | 698.10 | 0.416 | 0.001 | 1.60e-09 | 5.59e-11 | 0 |
| 60–119 | 2927.7 | 6365.1 | 9.53 | 56.67 | 7.93 | 3.10 | 725.40 | 1.985 | 1.672 | 9.93e-10 | 3.43e-11 | 0 |
| 120–179 | 4017.5 | 7219.1 | 10.90 | 65.05 | 10.23 | 3.57 | 834.60 | 3.827 | 2.511 | 8.19e-10 | 3.32e-11 | 0 |
| 180–239 | 1901.8 | 6565.7 | 8.60 | 51.38 | 7.55 | 2.75 | 633.90 | 14.165 | 23.046 | 1.65e-09 | 3.37e-11 | 0 |
| 240–299 | 24720.3 | 127903.3 | 9.33 | 52.75 | 8.00 | 1303.80 | 286200.60 | 61.388 | 104.334 | 8.49e-10 | 3.38e-11 | 14 |
| 300–359 | 7145.9 | 194179.1 | 8.85 | 49.65 | 7.93 | 259.33 | 56006.90 | 155.321 | 147.633 | 7.45e-09 | 2.99e-10 | 60 |
| 360–419 | 5954.8 | 25010.5 | 9.07 | 52.25 | 8.20 | 300.63 | 64936.80 | 216.886 | 210.138 | 1.30e-09 | 6.13e-11 | 60 |
| 420–479 | 196746.2 | 294183.3 | 8.37 | 44.90 | 7.15 | 121.13 | 26828.80 | 294.880 | 298.723 | 8.17e-09 | 7.67e-11 | 60 |
| 480–539 | 218720.2 | 283786.1 | 8.53 | 44.33 | 7.58 | 7.42 | 1583.10 | 395.509 | 366.057 | 7.38e-09 | 4.89e-10 | 60 |
| 540–599 | 273332.7 | 309074.0 | 7.97 | 37.12 | 6.83 | 8.00 | 1680.00 | 489.829 | 518.124 | 1.68e-09 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 600 | 236.319 | 238.551 | 255.235 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
