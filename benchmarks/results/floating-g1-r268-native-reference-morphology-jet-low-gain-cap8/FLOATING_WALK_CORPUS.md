# Bonesaw floating G1 admitted-reference tracking

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: immutable open-loop-admitted artifact `benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz` from `Python-authored alternating sequence of Rust Bonesaw LIPM boundary plans`.
- Admission contract: authored root, CoM, foot jets, and contact schedule are consumed unchanged; eval-side reconstruction, projection, and retiming are rejected.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.186 m` forward per `11.585 s` source cycle.
- Applied mean forward speed: `0.016 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `immutable authored tick sequence` in `11.6 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Finite-support CoP constraint: disabled; no loaded finite patch was declared.
- Balance task: `standalone-authored` `CoM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `standalone-authored`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.050`; morphology jet `enabled`.
- Protected coordinate posture: `intent` priority with weight `0.000` over 11 `upper-body` coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
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
5 ms p99 deadline: **PASS**
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | FAIL |
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | PASS |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 863 | 4.315 s | 1.845 cm | 0.356 cm | 0.110 cm | 34.873 cm | 0.015° | 8.000 rad/s | 5437.9 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `5.972e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1887.936 cm |
| authored reference vs measured CoM RMS / p95 | 1880.938 / 4076.523 cm |
| stance foot RMS | 1780.098 cm |
| swing foot RMS | 2129.117 cm |
| hand RMS | 1900.128 cm |
| maximum root rotation | 40.477° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.100e-09 |
| contact acceleration residual | 8.312e-11 |
| raw max dynamics residual, including rejected ticks | 7.100e-09 |
| raw max contact residual, including rejected ticks | 8.312e-11 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 137.614 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 42 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `False` (touchdown `False`, balance `False`); final source tick `2316.000`, progress `2316.000` ticks.
- First post-liftoff authored touchdown source tick: `529`; reached before trace end: `True`.
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

- Phase ticks: unsupported `1332`, single support `345`, precontact `0`, multi-support `640`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 6.6 µs | 3680.1 µs | 4911.4 µs | 192056.3 µs | 312 | 548 | 0 | 119 | 1,334 | 4 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1325.9 | 4255.3 | 1.8 | 3289.8 | 7035.7 | 150491.6 | 2221.6 | 988 | 23 | 1 | 754.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 312 | 1947.7 | 3131.2 | 3300.8 | 3522.6 |
| solved_with_slack | 548 | 3129.1 | 4687.6 | 5678.4 | 7052.8 |
| normal_contact_contingency | 1,334 | 4.9 | 8.1 | 11.9 | 4632.2 |
| contact_release_contingency | 4 | 1851.9 | 163588.7 | 186362.8 | 192056.3 |
| touchdown_transition | 119 | 2127.8 | 4435.7 | 6557.1 | 12588.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.95 | 10.0 | 15.0 | 23 | 1.91 | 13.0 | 21 | 0.3069 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.62/1.0/8.0/8 | 139.05/234.0/1728.0/1776 | 0.08/3.0/14 | 0.05/2.0/13 | 0.45/16.0/120 | 0.2041 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 312 | 4.55/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 548 | 7.91/18.5/23 | 6.12/17.1/21 |
| normal_contact_contingency | 1,334 | 0.04/0.0/12 | 0.04/0.0/11 |
| contact_release_contingency | 4 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 119 | 8.71/14.0/16 | 8.52/14.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.52/3.0/8 | 2.13/15.0/40 | 0.15/3.0/8 | 120 |
| viability | 0.20/2.8/7 | 0.96/14.5/41 | 0.10/2.8/7 | 122 |
| intent | 1.01/9.0/15 | 1.99/18.0/30 | 0.78/9.0/14 | 476 |
| preference | 0.78/8.0/19 | 6.41/65.0/157 | 0.62/8.0/18 | 671 |
| style | 0.44/2.0/3 | 6.31/31.0/49 | 0.26/1.0/3 | 575 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,635 | 0 | 45 | 631 | 6 |
| right_ankle_roll_link | 1,374 | 0 | 74 | 863 | 6 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `42` ticks, normal fallback `9` ticks.
Touchdown Normal sole-center tangential speed: p50 `4.5968 m/s`, p95 `6.4891 m/s`, max `7.1604 m/s` over 115 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.073 | 3.072 | 3.072 | 1.000 | 1.000 | 52.254 | 55.715 | 3.461 | 55.715 | 0.002 | 0 | 794 | 0 | 0 | 32 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3051.6 | 6529.5 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 2041.8 | 4730.9 | 6.72 | 37.97 | 3.28 | 1.57 | 352.78 | 0.665 | 0.334 | 1.29e-09 | 4.22e-11 | 0 |
| 464–695 | 3068.4 | 4951.9 | 6.98 | 41.75 | 4.09 | 1.00 | 230.56 | 0.221 | 0.286 | 1.11e-09 | 5.97e-11 | 0 |
| 696–927 | 2988.4 | 5726.7 | 6.01 | 33.00 | 5.16 | 0.84 | 194.28 | 14.962 | 12.059 | 1.27e-09 | 4.09e-11 | 65 |
| 928–1159 | 4.9 | 3545.3 | 1.36 | 8.56 | 1.34 | 0.44 | 94.97 | 353.141 | 346.912 | 2.96e-09 | 2.57e-11 | 195 |
| 1160–1391 | 4.8 | 10.7 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 867.597 | 828.708 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 4.9 | 12.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1573.766 | 1527.677 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 5.0 | 4271.3 | 1.58 | 9.89 | 1.56 | 0.64 | 137.45 | 2317.354 | 2260.391 | 8.15e-10 | 2.28e-11 | 189 |
| 1855–2085 | 4.9 | 10.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3191.448 | 3136.224 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.0 | 6315.4 | 1.46 | 8.97 | 1.45 | 0.68 | 145.87 | 4102.027 | 4048.302 | 7.10e-09 | 8.31e-11 | 194 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1887.936 | 1854.305 | 1900.128 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
