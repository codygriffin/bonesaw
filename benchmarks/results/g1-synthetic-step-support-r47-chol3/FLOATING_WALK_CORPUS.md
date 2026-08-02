# Bonesaw floating G1 synthetic-step acceptance

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: deterministic, predeclared G1-sized left step; no mocap or learned policy contributes contact labels or target motion.
- Contact schedule: liftoff tick `53`, touchdown tick `93`, step `0.010 m`, clearance `0.010 m`.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.010 m` forward per `0.800 s` source cycle.
- Applied mean forward speed: `0.012 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `constant synthetic phase rate` in `0.8 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Finite-support CoP constraint: `enabled`; required `5.000 mm`, measured minimum `5.000 mm` over `160` loaded ticks.
- Balance task: `rooted` `CoM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `40` ticks (`0.200 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `disabled`; authored offset ≤ `0.120 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `40` scheduled ticks and throughout delayed admission.
- Coupled touchdown phase retiming: `disabled`; minimum rate `0.000`, guard `0.020 s`, engagement / release `0 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
- Coupled balance phase retiming: `disabled`; hold at signed DCM margin ≤ `-0.020 m`, recover nominal rate at `0.000 m`.
- Touchdown admission limits: `0.025 m` position, `0.200 m/s` tangential, and `0.200 m/s` normal; the outgoing support is retained until its replacement locks.
- Reference touchdown blend: `0` ticks (`0.000 s`) with a quintic zero-velocity endpoint.
- Touchdown stabilization point: `ankle origin`.

## Acceptance

Functional: **PASS**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `finite_support_margin_respected` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.800 s | 4.332 cm | 0.065 cm | 0.639 cm | 27.482 cm | 1.528° | 8.000 rad/s | 5104.0 µs |

Nominal hard residual maxima: dynamics `1.212e-09`, contact acceleration `3.039e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 4.332 cm |
| authored reference vs measured CoM RMS / p95 | 4.914 / 10.291 cm |
| stance foot RMS | 0.065 cm |
| swing foot RMS | 0.639 cm |
| hand RMS | 27.482 cm |
| maximum root rotation | 1.528° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.212e-09 |
| contact acceleration residual | 3.039e-11 |
| raw max dynamics residual, including rejected ticks | 1.212e-09 |
| raw max contact residual, including rejected ticks | 3.039e-11 |
| active normal force range | 0.000–298.242 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 0.002 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 40 / 3 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `False` (touchdown `False`, balance `False`); final source tick `159.000`, progress `159.000` ticks.
- First post-liftoff authored touchdown source tick: `93`; reached before trace end: `True`.
- Target / applied minimum rate: `1.0000` / `1.0000`; mean / p50 applied `1.0000` / `1.0000`.
- Limited / zero-rate hold ticks: `0` / `0`; maximum required landing time `0.0000 s`.
- Position / tangential / normal limiting ticks: `0` / `0` / `0`; unsafe-edge ticks `0`.
- Balance-margin limited ticks: `0`.

## Capture-aware landing

- Active target-ticks: `40`; policy updates `0`, frozen `0`.
- Maximum applied offset / root reach: `0.0000 / 0.7933 m`.
- Authored-offset / reach / slew limited ticks: `0 / 0 / 0`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `0`, precontact `40`, multi-support `119`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 2477.5 µs | 4552.9 µs | 5104.0 µs | 5457.8 µs | 53 | 64 | 40 | 3 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2845.1 | 790.3 | 175.2 | 4020.7 | 5419.2 | 5453.9 | 1592.2 | 160 | 3 | 0 | 351.5 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 53 | 2403.6 | 2902.8 | 3272.2 | 3517.7 |
| solved_with_slack | 64 | 3503.3 | 4924.5 | 5304.9 | 5457.8 |
| touchdown_transition | 3 | 2649.3 | 3635.7 | 3723.3 | 3745.3 |
| precontact_transition | 40 | 2312.4 | 2712.7 | 2768.5 | 2789.0 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.33 | 12.0 | 14.4 | 16 | 4.33 | 13.0 | 15 | 0.4600 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 2.09/8.0/8.0/8 | 502.57/1936.0/1936.0/1936 | 0.31/2.0/2 | 0.16/1.0/1 | 0.00/0.0/0 | 0.7753 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 53 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 64 | 8.98/15.4/16 | 7.45/13.7/15 |
| touchdown_transition | 3 | 10.00/12.0/12 | 7.33/9.0/9 |
| precontact_transition | 40 | 8.88/12.6/13 | 4.85/9.8/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.39/6.0/10 | 0.00/0.0/0 | 0.49/6.0/9 | 36 |
| viability | 0.27/1.0/1 | 0.07/3.4/5 | 0.00/0.0/0 | 0 |
| intent | 2.03/7.0/7 | 0.04/1.2/3 | 1.49/7.0/7 | 74 |
| preference | 2.48/8.0/10 | 17.81/59.4/65 | 2.07/7.4/10 | 107 |
| style | 1.16/2.4/3 | 11.74/26.3/32 | 0.28/2.0/2 | 39 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 40 | 3 | 117 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 160 | 0 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `40` ticks, planned normal touchdown `3` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.0563 m/s`, p95 `0.0751 m/s`, max `0.0754 m/s` over 40 samples.
Touchdown Normal sole-center tangential speed: p50 `0.0000 m/s`, p95 `0.0000 m/s`, max `0.0000 m/s` over 3 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.456 | 0.455 | 0.455 | 1.000 | 1.000 | 45.785 | 45.922 | 0.137 | 45.922 | 0.001 | 0 | 35 | 0 | 0 | 7 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2421.1 | 3446.9 | 4.00 | 18.88 | 0.00 | 1.00 | 242.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 16–31 | 2414.4 | 2497.3 | 4.00 | 18.81 | 0.00 | 1.00 | 242.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 32–47 | 2378.9 | 2409.4 | 4.00 | 18.81 | 0.00 | 1.00 | 242.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 48–63 | 2378.7 | 2732.7 | 7.38 | 35.69 | 2.81 | 1.00 | 231.00 | 0.000 | 0.176 | 1.21e-09 | 3.03e-11 | 11 |
| 64–79 | 2459.8 | 2768.8 | 9.31 | 49.38 | 5.19 | 1.00 | 226.00 | 0.000 | 0.635 | 6.75e-10 | 9.27e-12 | 16 |
| 80–95 | 2069.9 | 3589.0 | 8.62 | 32.00 | 5.50 | 1.00 | 227.88 | 0.204 | 0.277 | 1.10e-09 | 1.03e-11 | 13 |
| 96–111 | 2668.6 | 3486.2 | 8.44 | 28.12 | 6.56 | 1.00 | 242.00 | 2.097 | 0.001 | 6.17e-10 | 3.01e-11 | 0 |
| 112–127 | 3736.1 | 5393.1 | 8.75 | 32.06 | 7.25 | 4.50 | 1089.00 | 5.119 | 0.001 | 4.01e-10 | 2.10e-11 | 0 |
| 128–143 | 4321.4 | 5170.5 | 9.25 | 32.69 | 7.75 | 8.00 | 1936.00 | 7.726 | 0.001 | 1.14e-13 | 3.45e-14 | 0 |
| 144–159 | 3303.8 | 4012.5 | 9.50 | 30.12 | 8.25 | 1.44 | 347.88 | 9.867 | 0.193 | 7.25e-10 | 3.04e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 4.332 | 0.234 | 27.482 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
