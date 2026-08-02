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
- Finite-support CoP constraint: disabled; no loaded finite patch was declared.
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

Functional: **FAIL**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | FAIL |
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 137 | 0.685 s | 4.515 cm | 2.723 cm | 0.639 cm | 24.220 cm | 11.771° | 8.000 rad/s | 45053.6 µs |

Nominal hard residual maxima: dynamics `1.212e-09`, contact acceleration `3.026e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 5.914 cm |
| authored reference vs measured CoM RMS / p95 | 5.912 / 12.092 cm |
| stance foot RMS | 5.347 cm |
| swing foot RMS | 0.639 cm |
| hand RMS | 28.897 cm |
| maximum root rotation | 11.771° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.092e-09 |
| contact acceleration residual | 3.026e-11 |
| raw max dynamics residual, including rejected ticks | 2.092e-09 |
| raw max contact residual, including rejected ticks | 3.026e-11 |
| active normal force range | 0.000–272.637 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 70.567 m/s² |
| frame-angular acceleration RMS max | 49.260 rad/s² |
| longest pre-contact / touchdown transition | 40 / 44 ticks |
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
| 160 | 0.8 s | 3615.9 µs | 35700.1 µs | 47117.3 µs | 165077.9 µs | 53 | 0 | 40 | 44 | 23 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11750.3 | 16882.9 | 1603.8 | 29183.0 | 146438.9 | 163214.0 | 81904.8 | 160 | 69 | 38 | 85.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 53 | 2342.1 | 3850.6 | 3936.6 | 3965.1 |
| normal_contact_contingency | 23 | 24075.9 | 39581.4 | 137501.5 | 165077.9 |
| touchdown_transition | 44 | 19548.9 | 42244.0 | 47316.4 | 47851.4 |
| precontact_transition | 40 | 2375.2 | 7627.5 | 8735.3 | 9285.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.98 | 17.0 | 20.4 | 22 | 6.72 | 19.8 | 22 | 0.5697 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 99.72/1110.6/2004.0/2077 | 22728.79/253228.2/456905.2/473556 | 5.43/27.0/27 | 5.04/26.0/26 | 44.42/236.0/246 | 0.3774 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 53 | 4.00/4.0/4 | 0.00/0.0/0 |
| normal_contact_contingency | 23 | 12.91/20.8/21 | 12.74/20.6/21 |
| touchdown_transition | 44 | 12.84/20.7/22 | 12.20/20.7/22 |
| precontact_transition | 40 | 9.07/13.6/14 | 6.12/13.2/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.83/10.8/12 | 10.99/43.3/48 | 2.28/10.8/12 | 75 |
| viability | 1.42/7.0/9 | 5.88/28.0/35 | 1.14/7.0/9 | 75 |
| intent | 1.36/4.0/4 | 2.81/8.0/8 | 0.82/3.0/4 | 92 |
| preference | 1.59/7.2/11 | 10.72/48.7/66 | 1.26/7.2/11 | 107 |
| style | 1.77/8.4/9 | 17.71/86.6/98 | 1.23/7.4/9 | 96 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 40 | 44 | 53 | 23 |
| right_ankle_roll_link | 0 | 0 | 0 | 137 | 23 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `40` ticks, planned normal touchdown `44` ticks, normal fallback `26` ticks.
Precontact sole-center tangential speed: p50 `0.0563 m/s`, p95 `0.0751 m/s`, max `0.0754 m/s` over 40 samples.
Touchdown Normal sole-center tangential speed: p50 `0.6560 m/s`, p95 `0.9341 m/s`, max `1.0110 m/s` over 44 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.880 | 1.880 | 1.880 | 1.000 | 1.000 | 45.660 | 45.887 | 0.227 | 45.887 | 0.001 | 0 | 34 | 0 | 0 | 15 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2944.4 | 3956.9 | 4.00 | 24.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 16–31 | 2318.2 | 3518.5 | 4.00 | 24.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 5.68e-14 | 4.89e-16 | 0 |
| 32–47 | 2324.7 | 2420.8 | 4.00 | 24.00 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 5.68e-14 | 5.01e-16 | 0 |
| 48–63 | 2315.2 | 3165.6 | 6.56 | 35.50 | 2.88 | 1.00 | 225.75 | 0.000 | 0.176 | 6.97e-11 | 1.13e-12 | 11 |
| 64–79 | 2284.5 | 3204.0 | 8.69 | 45.19 | 5.12 | 1.00 | 222.00 | 0.000 | 0.635 | 5.16e-12 | 8.50e-14 | 16 |
| 80–95 | 5419.2 | 27636.3 | 11.31 | 49.19 | 9.94 | 6.25 | 1396.50 | 0.213 | 0.283 | 3.17e-10 | 7.71e-12 | 13 |
| 96–111 | 19971.5 | 39813.9 | 15.94 | 77.00 | 15.75 | 93.06 | 21218.25 | 3.001 | 2.422 | 2.33e-10 | 3.49e-12 | 0 |
| 112–127 | 12011.1 | 31416.6 | 10.62 | 56.88 | 9.25 | 498.38 | 113629.50 | 8.853 | 5.323 | 2.52e-11 | 6.17e-13 | 0 |
| 128–143 | 19883.3 | 147493.9 | 12.44 | 77.19 | 12.12 | 387.00 | 88215.00 | 12.319 | 6.144 | 2.77e-10 | 8.00e-12 | 7 |
| 144–159 | 31232.9 | 39506.9 | 12.25 | 68.19 | 12.12 | 7.56 | 1678.88 | 10.514 | 13.349 | 2.09e-09 | 2.72e-11 | 16 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 5.914 | 5.006 | 28.897 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
