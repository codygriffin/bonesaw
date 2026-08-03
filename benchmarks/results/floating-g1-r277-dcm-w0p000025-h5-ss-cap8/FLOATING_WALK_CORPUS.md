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
- Balance task: `standalone-authored` `DCM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `standalone-authored`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.050`; morphology jet `enabled`.
- Protected coordinate posture: `intent` priority with weight `0.000` over 11 `upper-body` coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.250`, activating at `50.0%` of each effective limit with `2.000 Hz` response and `multi-support` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Joint-velocity envelope scope: `lower-body allowlist`; hard acceleration-bound intersection `enabled`.
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
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 431 | 2.155 s | 2.422 cm | 0.404 cm | 1.670 cm | 32.852 cm | 2.405° | 8.000 rad/s | 9804.9 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `4.840e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2482.757 cm |
| authored reference vs measured CoM RMS / p95 | 2471.538 / 4522.098 cm |
| stance foot RMS | 2354.339 cm |
| swing foot RMS | 2657.154 cm |
| hand RMS | 2496.501 cm |
| maximum root rotation | 91.848° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.922e-09 |
| contact acceleration residual | 7.320e-11 |
| raw max dynamics residual, including rejected ticks | 7.922e-09 |
| raw max contact residual, including rejected ticks | 7.320e-11 |
| active normal force range | 0.000–616.876 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 137.017 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 47 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `5.458` / `9.253 cm`.
- Virtual ZMP clipped on `84.67%` of ticks; clip-distance RMS / max `6.619` / `15.907 cm`.
- Measured-height natural frequency min / p50 / max: `3.749` / `3.778` / `3.834 rad/s`.
- Minimum measured CoM height: `0.667 m`; height-floor ticks: `0`.
- CoM command acceleration p95 / max: `0.923` / `1.032 m/s²`; support hull `4–6` vertices.
- Signed measured DCM support margin min / p05: `-7.433` / `-6.010 cm`; inside on `43.80%` of ticks.

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

- Phase ticks: unsupported `1771`, single support `246`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.129` / `0.129`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `137` / `409` / `5` / `0` / `0` / `0` / `1766`.
- Bound-limited ticks / upper-bound ticks: `4` / `2`; maximum coordinate violation `70.58467930533953`.
- Named-linear-row-limited ticks / upper-row ticks: `5` / `3`; maximum row violation `2599.840901563872`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 7.0 µs | 4408.8 µs | 6879.1 µs | 209930.3 µs | 137 | 294 | 0 | 115 | 1,766 | 0 | 0 | 0 | 0 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1120.6 | 7024.9 | 1.1 | 3970.5 | 123050.7 | 204575.3 | 2891.8 | 551 | 60 | 3 | 892.4 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 4326.1 | 6613.1 | 9931.2 | 10270.3 |
| solved_with_slack | 294 | 3173.3 | 5622.9 | 7921.4 | 10166.4 |
| normal_contact_contingency | 1,766 | 6.8 | 10.1 | 13.9 | 116.2 |
| contact_release_contingency | 5 | 171661.7 | 205306.0 | 209005.4 | 209930.3 |
| touchdown_transition | 115 | 2802.4 | 5076.6 | 12119.4 | 17829.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.75 | 9.0 | 13.0 | 18 | 1.26 | 12.0 | 16 | 0.1752 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.54/1.0/8.0/8 | 120.24/234.0/1776.0/1776 | 0.12/2.0/19 | 0.08/1.0/18 | 0.64/9.0/155 | 0.1245 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 294 | 8.29/16.1/18 | 6.35/15.1/16 |
| normal_contact_contingency | 1,766 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 5 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 115 | 9.35/14.9/16 | 9.18/14.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.36/4.0/7 | 1.49/19.7/32 | 0.18/4.0/7 | 155 |
| viability | 0.19/4.0/8 | 1.03/18.0/48 | 0.15/4.0/8 | 163 |
| intent | 0.55/7.0/13 | 1.10/14.0/26 | 0.45/7.0/13 | 330 |
| preference | 0.39/5.0/14 | 3.53/42.8/112 | 0.32/4.8/14 | 407 |
| style | 0.26/2.0/3 | 3.66/28.0/49 | 0.15/2.0/3 | 319 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,971 | 0 | 46 | 300 | 0 |
| right_ankle_roll_link | 1,817 | 0 | 69 | 431 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `47` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.2566 m/s`, p95 `4.3433 m/s`, max `6.4520 m/s` over 111 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.597 | 2.558 | 2.558 | 0.985 | 0.985 | 52.867 | 56.230 | 3.363 | 56.230 | 0.002 | 0 | 800 | 0 | 0 | 54 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 4352.8 | 10127.4 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 2903.3 | 4905.9 | 7.46 | 40.85 | 5.88 | 2.12 | 475.27 | 7.280 | 3.616 | 1.54e-09 | 4.22e-11 | 33 |
| 464–695 | 6.9 | 3039.4 | 0.93 | 5.53 | 0.91 | 0.44 | 94.03 | 289.476 | 274.941 | 2.15e-09 | 1.50e-11 | 208 |
| 696–927 | 6.8 | 13.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 872.938 | 848.755 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 7.7 | 6002.0 | 1.96 | 12.90 | 1.93 | 0.62 | 135.00 | 1494.956 | 1465.143 | 3.80e-09 | 5.37e-11 | 185 |
| 1160–1391 | 6.8 | 12.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2095.861 | 2016.348 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.7 | 13.7 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2699.503 | 2623.582 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.8 | 4284.2 | 0.87 | 5.42 | 0.86 | 0.70 | 151.48 | 3309.967 | 3217.796 | 7.87e-09 | 6.58e-11 | 209 |
| 1855–2085 | 6.8 | 12.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3927.599 | 3824.611 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 6.0 | 10716.1 | 0.88 | 5.55 | 0.86 | 0.52 | 112.21 | 4541.045 | 4435.363 | 7.92e-09 | 7.32e-11 | 209 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2482.757 | 2417.206 | 2496.501 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
