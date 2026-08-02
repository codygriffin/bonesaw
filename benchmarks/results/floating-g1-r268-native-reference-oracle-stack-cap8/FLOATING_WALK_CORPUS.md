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
- Finite-support CoP constraint: `enabled`; required `5.000 mm`, measured minimum `5.000 mm` over `562` loaded ticks.
- Balance task: `standalone-authored` `CoM` reference at `viability` priority with weight `1.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `standalone-authored`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `1.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.010`; morphology jet `disabled`.
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
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `finite_support_margin_respected` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 188 | 0.940 s | 2.879 cm | 0.404 cm | nan cm | 32.097 cm | 1.269° | 8.000 rad/s | 5737.0 µs |

Nominal hard residual maxima: dynamics `1.112e-09`, contact acceleration `4.675e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 3241.310 cm |
| authored reference vs measured CoM RMS / p95 | 3236.396 / 5912.797 cm |
| stance foot RMS | 3136.018 cm |
| swing foot RMS | 3545.262 cm |
| hand RMS | 3254.732 cm |
| maximum root rotation | 58.918° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 2.855e-09 |
| contact acceleration residual | 1.092e-10 |
| raw max dynamics residual, including rejected ticks | 2.855e-09 |
| raw max contact residual, including rejected ticks | 1.092e-10 |
| active normal force range | 0.000–515.946 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 144.591 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 63 ticks |
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

- Phase ticks: unsupported `1756`, single support `261`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 5.6 µs | 3548.3 µs | 5031.3 µs | 171082.4 µs | 34 | 154 | 0 | 212 | 1,913 | 4 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 912.6 | 4507.8 | 0.8 | 3187.1 | 15453.6 | 158015.6 | 1928.5 | 565 | 24 | 2 | 1095.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 34 | 3086.9 | 3252.6 | 3747.3 | 3955.0 |
| solved_with_slack | 154 | 3342.2 | 5134.1 | 5753.7 | 6968.1 |
| normal_contact_contingency | 1,913 | 5.0 | 3093.8 | 4145.2 | 6179.4 |
| contact_release_contingency | 4 | 58038.7 | 162619.5 | 169389.8 | 171082.4 |
| touchdown_transition | 212 | 2791.0 | 4618.8 | 7792.3 | 17484.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.64 | 8.0 | 11.0 | 28 | 1.37 | 10.0 | 26 | 0.2851 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.81/8.0/8.0/8 | 183.35/1760.0/1936.0/1936 | 0.20/3.0/21 | 0.12/2.0/20 | 0.94/14.0/173 | 0.2211 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 34 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 154 | 7.16/14.0/16 | 5.91/13.5/16 |
| normal_contact_contingency | 1,913 | 0.59/9.0/28 | 0.48/8.0/26 |
| contact_release_contingency | 4 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 212 | 6.68/11.0/12 | 6.39/11.0/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.44/4.0/8 | 1.75/16.0/32 | 0.30/4.0/8 | 277 |
| viability | 0.62/6.0/25 | 4.11/40.0/204 | 0.61/6.0/25 | 526 |
| intent | 0.00/0.0/0 | 0.00/0.0/0 | 0.00/0.0/0 | 0 |
| preference | 0.33/4.0/9 | 4.33/49.0/123 | 0.28/4.0/9 | 473 |
| style | 0.25/1.0/3 | 3.60/18.0/48 | 0.19/1.0/3 | 427 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,914 | 0 | 103 | 188 | 112 |
| right_ankle_roll_link | 1,858 | 0 | 109 | 188 | 162 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `63` ticks, normal fallback `165` ticks.
Touchdown Normal sole-center tangential speed: p50 `4.7315 m/s`, p95 `6.8935 m/s`, max `7.1781 m/s` over 208 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.115 | 2.114 | 2.114 | 1.000 | 1.000 | 52.746 | 56.207 | 3.461 | 56.207 | 0.002 | 0 | 884 | 0 | 0 | 19 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3247.6 | 5764.4 | 6.75 | 56.60 | 5.06 | 1.84 | 444.17 | 8.117 | 7.620 | 1.11e-09 | 4.68e-11 | 44 |
| 232–463 | 1972.1 | 4870.4 | 3.48 | 32.14 | 2.81 | 1.90 | 422.59 | 75.842 | 92.837 | 8.17e-10 | 1.30e-11 | 232 |
| 464–695 | 6.1 | 3638.8 | 1.81 | 14.50 | 1.72 | 1.09 | 238.97 | 483.101 | 456.731 | 1.52e-09 | 4.24e-11 | 169 |
| 696–927 | 4.9 | 10.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1163.381 | 1132.511 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 5.2 | 4702.2 | 1.43 | 11.29 | 1.38 | 1.55 | 341.38 | 1907.413 | 1879.277 | 1.60e-09 | 2.57e-11 | 180 |
| 1160–1391 | 4.9 | 7273.6 | 0.22 | 1.86 | 0.22 | 0.21 | 45.52 | 2667.799 | 2622.435 | 2.05e-10 | 3.74e-12 | 226 |
| 1392–1623 | 4.9 | 10.3 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3525.750 | 3473.372 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 5.1 | 4485.5 | 1.21 | 9.92 | 1.13 | 0.26 | 58.10 | 4359.385 | 4327.930 | 1.24e-09 | 7.10e-11 | 191 |
| 1855–2085 | 5.0 | 10.6 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 5137.129 | 5126.027 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.1 | 3585.0 | 1.45 | 11.49 | 1.40 | 1.28 | 281.90 | 5924.419 | 5914.507 | 2.86e-09 | 1.09e-10 | 180 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 3241.310 | 3221.039 | 3254.732 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
