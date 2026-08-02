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
- Whole-body posture: `preference` priority with weight `0.250`; morphology jet `enabled`.
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
| 439 | 2.195 s | 4.849 cm | 0.410 cm | 0.571 cm | 33.276 cm | 7.424° | 8.000 rad/s | 6051.5 µs |

Nominal hard residual maxima: dynamics `1.629e-09`, contact acceleration `5.953e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2488.647 cm |
| authored reference vs measured CoM RMS / p95 | 2479.807 / 4555.870 cm |
| stance foot RMS | 2377.990 cm |
| swing foot RMS | 2697.850 cm |
| hand RMS | 2501.185 cm |
| maximum root rotation | 102.829° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.905e-09 |
| contact acceleration residual | 9.587e-11 |
| raw max dynamics residual, including rejected ticks | 4.905e-09 |
| raw max contact residual, including rejected ticks | 9.587e-11 |
| active normal force range | 0.000–736.985 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 175.032 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 70 ticks |
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

- Phase ticks: unsupported `1723`, single support `294`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 5.0 µs | 3364.9 µs | 4835.9 µs | 160547.1 µs | 137 | 302 | 0 | 154 | 1,719 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 923.9 | 4907.8 | 0.2 | 3091.4 | 18737.9 | 160403.5 | 2096.1 | 598 | 21 | 3 | 1082.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 2948.2 | 3209.5 | 3557.1 | 3738.6 |
| solved_with_slack | 302 | 3107.5 | 4837.5 | 6155.3 | 8614.3 |
| normal_contact_contingency | 1,719 | 4.9 | 8.2 | 10.7 | 7486.5 |
| contact_release_contingency | 5 | 1303.1 | 160423.1 | 160522.3 | 160547.1 |
| touchdown_transition | 154 | 2725.1 | 4363.0 | 8935.4 | 23243.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.86 | 9.0 | 12.8 | 22 | 1.38 | 12.0 | 20 | 0.2389 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.74/8.0/8.0/8 | 163.94/1728.0/1776.0/1776 | 0.19/4.0/23 | 0.12/3.0/22 | 0.98/24.0/199 | 0.1869 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 302 | 7.88/22.0/22 | 6.12/19.0/20 |
| normal_contact_contingency | 1,719 | 0.00/0.0/8 | 0.00/0.0/7 |
| contact_release_contingency | 5 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 154 | 8.88/14.0/14 | 8.65/13.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.44/4.0/8 | 1.83/20.0/39 | 0.27/4.0/8 | 236 |
| viability | 0.18/3.0/9 | 0.91/14.7/52 | 0.13/2.8/9 | 190 |
| intent | 0.55/7.0/17 | 1.11/14.0/34 | 0.45/7.0/16 | 381 |
| preference | 0.42/5.0/19 | 3.78/42.8/156 | 0.36/4.8/19 | 454 |
| style | 0.27/1.0/4 | 3.66/18.0/65 | 0.16/1.0/3 | 360 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,958 | 0 | 59 | 300 | 0 |
| right_ankle_roll_link | 1,782 | 0 | 95 | 439 | 1 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `70` ticks, normal fallback `4` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.2082 m/s`, p95 `3.6118 m/s`, max `4.1535 m/s` over 150 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.141 | 2.141 | 2.141 | 1.000 | 1.000 | 52.188 | 55.715 | 3.527 | 55.715 | 0.002 | 0 | 796 | 0 | 0 | 19 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3036.3 | 6152.5 | 5.50 | 37.76 | 2.25 | 1.00 | 234.00 | 0.345 | 0.404 | 1.63e-09 | 5.58e-11 | 0 |
| 232–463 | 3010.6 | 5606.7 | 7.16 | 37.60 | 5.75 | 3.37 | 751.60 | 9.679 | 2.255 | 1.19e-09 | 5.95e-11 | 25 |
| 464–695 | 4.9 | 3948.4 | 1.12 | 6.72 | 1.09 | 0.86 | 186.21 | 281.662 | 262.321 | 1.93e-09 | 9.59e-11 | 200 |
| 696–927 | 4.9 | 10.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 876.047 | 844.468 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 5.0 | 2778.5 | 1.90 | 11.59 | 1.87 | 0.28 | 61.45 | 1506.437 | 1477.254 | 4.91e-09 | 3.28e-11 | 180 |
| 1160–1391 | 4.9 | 2050.8 | 0.68 | 4.61 | 0.66 | 0.08 | 16.76 | 2062.935 | 2005.401 | 6.27e-10 | 5.24e-12 | 214 |
| 1392–1623 | 4.9 | 10.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2685.836 | 2629.673 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 4.9 | 3882.3 | 1.19 | 7.39 | 1.15 | 0.94 | 201.97 | 3316.556 | 3258.337 | 3.18e-09 | 6.19e-11 | 204 |
| 1855–2085 | 4.9 | 10.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3944.257 | 3881.150 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 4.9 | 8940.0 | 1.01 | 7.13 | 0.98 | 0.87 | 187.01 | 4573.379 | 4505.269 | 3.78e-09 | 8.62e-11 | 206 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2488.647 | 2444.538 | 2501.185 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
