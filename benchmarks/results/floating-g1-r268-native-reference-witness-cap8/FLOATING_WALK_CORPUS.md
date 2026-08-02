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
- Whole-body posture: `preference` priority with weight `0.250`; morphology jet `disabled`.
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
| 501 | 2.505 s | 7.890 cm | 0.409 cm | 0.693 cm | 18.518 cm | 7.743° | 8.000 rad/s | 11519.0 µs |

Nominal hard residual maxima: dynamics `6.196e-09`, contact acceleration `2.516e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2512.921 cm |
| authored reference vs measured CoM RMS / p95 | 2506.417 / 4870.724 cm |
| stance foot RMS | 2386.255 cm |
| swing foot RMS | 2744.446 cm |
| hand RMS | 2538.848 cm |
| maximum root rotation | 114.719° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.196e-09 |
| contact acceleration residual | 2.516e-10 |
| raw max dynamics residual, including rejected ticks | 6.196e-09 |
| raw max contact residual, including rejected ticks | 2.516e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 152.475 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 52 ticks |
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

- Phase ticks: unsupported `1705`, single support `312`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 5.7 µs | 3210.1 µs | 4992.5 µs | 120945.6 µs | 205 | 296 | 0 | 111 | 1,700 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 887.0 | 3108.5 | 0.8 | 3012.3 | 15508.4 | 104529.8 | 1676.6 | 617 | 24 | 2 | 1127.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 205 | 2918.8 | 3031.0 | 3065.1 | 3079.9 |
| solved_with_slack | 296 | 3055.7 | 5933.0 | 11898.5 | 14707.3 |
| normal_contact_contingency | 1,700 | 4.9 | 8.2 | 10.5 | 14.1 |
| contact_release_contingency | 5 | 2274.2 | 106769.6 | 118110.4 | 120945.6 |
| touchdown_transition | 111 | 2014.8 | 4898.6 | 5298.1 | 15878.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.68 | 8.0 | 12.0 | 18 | 1.02 | 11.0 | 17 | 0.4084 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.68/8.0/8.0/8 | 152.33/1728.0/1776.0/1776 | 0.21/3.0/19 | 0.15/2.0/18 | 1.24/18.0/158 | 0.3420 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 205 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 296 | 7.14/14.0/18 | 4.82/14.0/17 |
| normal_contact_contingency | 1,700 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 5 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 111 | 8.69/13.9/15 | 8.45/13.0/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.40/4.0/6 | 1.62/16.0/24 | 0.21/4.0/6 | 210 |
| viability | 0.19/3.0/7 | 0.91/15.0/35 | 0.12/3.0/7 | 159 |
| intent | 0.39/4.0/11 | 0.77/8.0/22 | 0.22/4.0/10 | 230 |
| preference | 0.44/4.0/9 | 4.11/41.0/118 | 0.33/4.0/9 | 406 |
| style | 0.27/1.0/3 | 3.65/17.0/47 | 0.14/1.0/3 | 319 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,985 | 0 | 32 | 300 | 0 |
| right_ankle_roll_link | 1,737 | 0 | 79 | 501 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `52` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `4.4420 m/s`, p95 `5.3006 m/s`, max `6.4021 m/s` over 107 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.056 | 2.055 | 2.055 | 1.000 | 1.000 | 52.797 | 56.352 | 3.555 | 56.352 | 0.002 | 0 | 887 | 0 | 0 | 23 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2994.1 | 3690.3 | 4.54 | 33.20 | 1.02 | 1.00 | 234.00 | 0.345 | 0.404 | 1.60e-09 | 7.15e-11 | 0 |
| 232–463 | 2862.2 | 4359.1 | 6.63 | 42.38 | 3.85 | 2.75 | 614.02 | 3.960 | 0.527 | 1.43e-09 | 4.48e-11 | 0 |
| 464–695 | 6.0 | 12107.6 | 1.93 | 11.19 | 1.69 | 1.39 | 308.38 | 159.602 | 146.456 | 6.20e-09 | 2.52e-10 | 182 |
| 696–927 | 4.9 | 10.3 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 654.615 | 652.117 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 5.7 | 2384.9 | 1.88 | 11.76 | 1.84 | 0.22 | 48.41 | 1247.120 | 1242.217 | 3.59e-09 | 4.83e-11 | 180 |
| 1160–1391 | 4.9 | 11.3 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1762.643 | 1695.479 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 4.9 | 10.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2532.559 | 2457.598 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 4.9 | 4941.9 | 0.74 | 5.37 | 0.71 | 0.66 | 142.13 | 3320.193 | 3235.404 | 5.52e-09 | 4.93e-11 | 212 |
| 1855–2085 | 4.9 | 10.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 4100.650 | 4021.286 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.0 | 5168.5 | 1.09 | 6.65 | 1.09 | 0.81 | 175.79 | 4890.814 | 4813.354 | 4.96e-09 | 2.12e-10 | 204 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2512.921 | 2461.196 | 2538.848 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
