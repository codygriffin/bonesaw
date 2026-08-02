# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Finite-support CoP constraint: disabled; no loaded finite patch was declared.
- Balance task: `rooted` `CoM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
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

Functional: **PASS**  
5 ms p99 deadline: **PASS**  
Combined: **PASS**

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
| `p99_tick_le_5ms` | PASS |

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 260 | 1.300 s | 0.777 cm | 0.000 cm | 0.355 cm | 10.778 cm | 3.135° | 8.000 rad/s | 3738.0 µs |

Nominal hard residual maxima: dynamics `1.219e-09`, contact acceleration `5.865e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 0.777 cm |
| authored reference vs measured CoM RMS / p95 | 1.600 / 4.622 cm |
| stance foot RMS | 0.000 cm |
| swing foot RMS | 0.355 cm |
| hand RMS | 10.778 cm |
| maximum root rotation | 3.135° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.219e-09 |
| contact acceleration residual | 5.865e-11 |
| raw max dynamics residual, including rejected ticks | 1.219e-09 |
| raw max contact residual, including rejected ticks | 5.865e-11 |
| active normal force range | 0.000–231.189 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 18.220 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `False` (touchdown `False`, balance `False`); final source tick `259.000`, progress `259.000` ticks.
- First post-liftoff authored touchdown source tick: `None`; reached before trace end: `True`.
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

- Phase ticks: unsupported `1`, single support `60`, precontact `0`, multi-support `199`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 260 | 1.3 s | 2970.6 µs | 3339.1 µs | 3738.0 µs | 4613.3 µs | 199 | 61 | 0 | 0 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2879.4 | 385.1 | 40.6 | 3041.0 | 4446.8 | 4596.7 | 1076.6 | 260 | 0 | 0 | 347.3 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 2984.8 | 3043.3 | 3056.6 | 3072.1 |
| solved_with_slack | 61 | 2234.7 | 3698.8 | 4227.6 | 4613.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.82 | 9.0 | 11.8 | 13 | 1.21 | 10.4 | 11 | -0.1970 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.46/8.0/8.0/8 | 332.79/1776.0/1776.0/1776 | 0.17/3.0/3 | 0.11/2.0/2 | 0.89/16.8/18 | 0.4584 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 61 | 7.51/13.0/13 | 5.15/11.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.09/3.0/3 | 3.60/12.0/12 | 0.14/3.0/3 | 17 |
| viability | 0.24/1.0/3 | 0.86/4.0/11 | 0.05/1.0/3 | 11 |
| intent | 1.21/4.4/9 | 2.42/8.8/18 | 0.33/4.4/9 | 33 |
| preference | 1.28/5.8/7 | 10.48/56.2/72 | 0.51/5.8/7 | 61 |
| style | 1.00/1.0/1 | 15.10/17.0/17 | 0.17/1.0/1 | 44 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 61 | 0 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 260 | 0 |
| left_wrist_roll_rubber_hand | 260 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 260 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `0` ticks, normal fallback `0` ticks.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.749 | 0.749 | 0.749 | 1.000 | 1.000 | 52.645 | 52.961 | 0.316 | 52.961 | 0.001 | 0 | 50 | 0 | 0 | 4 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–25 | 2990.3 | 3051.1 | 4.00 | 28.31 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 26–51 | 2934.1 | 3039.4 | 4.00 | 28.12 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.02e-11 | 0 |
| 52–77 | 2996.6 | 3051.0 | 4.00 | 28.31 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.02e-11 | 0 |
| 78–103 | 2984.4 | 3049.8 | 4.00 | 28.27 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 104–129 | 2976.3 | 3030.2 | 4.00 | 28.31 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 130–155 | 2961.7 | 3047.1 | 4.00 | 28.23 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 3.02e-11 | 0 |
| 156–181 | 3006.3 | 3070.6 | 4.00 | 28.81 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| 182–207 | 2940.2 | 3039.8 | 4.81 | 34.00 | 0.85 | 1.00 | 229.85 | 0.007 | 0.001 | 8.04e-10 | 5.87e-11 | 0 |
| 208–233 | 2198.4 | 3241.9 | 7.27 | 50.19 | 4.19 | 1.00 | 222.00 | 0.042 | 0.010 | 1.14e-09 | 1.45e-11 | 0 |
| 234–259 | 3228.7 | 4452.6 | 8.15 | 42.08 | 7.04 | 5.58 | 1238.08 | 2.456 | 0.384 | 4.12e-10 | 8.40e-12 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 260 | 0.777 | 0.121 | 10.778 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
