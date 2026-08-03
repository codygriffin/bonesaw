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
- Balance task: `standalone-authored` `DCM` reference at `viability` priority with weight `0.025` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 466 | 2.330 s | 3.864 cm | 0.405 cm | 5.296 cm | 29.275 cm | 4.177° | 8.000 rad/s | 5627.9 µs |

Nominal hard residual maxima: dynamics `1.349e-09`, contact acceleration `5.761e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2488.213 cm |
| authored reference vs measured CoM RMS / p95 | 2482.557 / 4649.175 cm |
| stance foot RMS | 2388.809 cm |
| swing foot RMS | 2715.972 cm |
| hand RMS | 2496.106 cm |
| maximum root rotation | 94.431° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 4.603e-09 |
| contact acceleration residual | 5.761e-11 |
| raw max dynamics residual, including rejected ticks | 4.603e-09 |
| raw max contact residual, including rejected ticks | 5.761e-11 |
| active normal force range | 0.000–778.138 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 162.569 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 59 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `589.542` / `1739.264 cm`.
- Virtual ZMP clipped on `38.24%` of ticks; clip-distance RMS / max `438.943` / `1640.890 cm`.
- Measured-height natural frequency min / p50 / max: `3.650` / `3.815` / `6.370 rad/s`.
- Minimum measured CoM height: `0.242 m`; height-floor ticks: `0`.
- CoM command acceleration p95 / max: `21.946` / `25.000 m/s²`; support hull `4–6` vertices.
- Signed measured DCM support margin min / p05: `-166.054` / `-117.570 cm`; inside on `63.96%` of ticks.

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

- Phase ticks: unsupported `1707`, single support `310`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.129` / `0.129`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `181` / `429` / `5` / `0` / `0` / `0` / `1702`.
- Bound-limited ticks / upper-bound ticks: `1` / `0`; maximum coordinate violation `8.383319779015652`.
- Named-linear-row-limited ticks / upper-row ticks: `5` / `1`; maximum row violation `71.03917510159533`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 6.7 µs | 3134.8 µs | 3904.3 µs | 131585.5 µs | 181 | 285 | 0 | 144 | 1,702 | 0 | 0 | 0 | 0 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 809.9 | 3010.6 | 1.0 | 3033.7 | 6257.2 | 102871.1 | 1470.8 | 615 | 11 | 1 | 1234.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 181 | 3027.1 | 3165.4 | 3205.9 | 3210.1 |
| solved_with_slack | 285 | 2987.0 | 4112.8 | 6123.6 | 7602.6 |
| normal_contact_contingency | 1,702 | 6.5 | 9.4 | 12.9 | 18.9 |
| contact_release_contingency | 5 | 1361.6 | 105557.4 | 126379.9 | 131585.5 |
| touchdown_transition | 144 | 2365.4 | 3522.4 | 4739.6 | 5313.9 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.02 | 10.0 | 12.0 | 18 | 1.40 | 11.0 | 17 | 0.3781 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.68/8.0/8.0/8 | 151.62/1728.0/1776.0/1776 | 0.14/2.0/8 | 0.08/1.0/7 | 0.61/8.0/58 | 0.2734 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 181 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 285 | 8.95/15.2/18 | 7.22/14.2/17 |
| normal_contact_contingency | 1,702 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 5 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 144 | 8.44/13.0/13 | 8.20/13.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.43/4.0/7 | 1.76/16.0/28 | 0.26/4.0/7 | 248 |
| viability | 0.52/5.8/13 | 1.95/23.8/48 | 0.37/5.0/13 | 358 |
| intent | 0.46/5.0/9 | 0.93/10.0/18 | 0.36/5.0/9 | 387 |
| preference | 0.33/3.0/7 | 3.78/36.0/85 | 0.24/3.0/7 | 417 |
| style | 0.27/1.0/3 | 3.74/17.0/49 | 0.17/1.0/3 | 371 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,920 | 0 | 97 | 300 | 0 |
| right_ankle_roll_link | 1,804 | 0 | 47 | 466 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `59` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.4259 m/s`, p95 `6.1664 m/s`, max `7.7399 m/s` over 140 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.877 | 1.877 | 1.877 | 1.000 | 1.000 | 52.980 | 56.281 | 3.301 | 56.281 | 0.002 | 0 | 799 | 0 | 0 | 13 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 3039.9 | 4782.6 | 5.66 | 37.14 | 1.07 | 1.00 | 234.00 | 0.345 | 0.404 | 1.30e-09 | 4.92e-11 | 0 |
| 232–463 | 2945.4 | 5651.0 | 9.16 | 50.17 | 7.72 | 3.11 | 694.40 | 5.307 | 3.114 | 1.35e-09 | 5.76e-11 | 0 |
| 464–695 | 6.6 | 3154.2 | 1.45 | 8.96 | 1.41 | 0.90 | 193.89 | 209.564 | 185.478 | 1.26e-09 | 2.10e-11 | 192 |
| 696–927 | 6.5 | 11.6 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 799.986 | 759.573 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 5.8 | 3172.1 | 0.85 | 5.34 | 0.82 | 0.44 | 94.03 | 1450.235 | 1416.171 | 3.08e-09 | 4.10e-11 | 208 |
| 1160–1391 | 6.6 | 13.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2031.779 | 2032.191 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.1 | 12.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2684.781 | 2692.444 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.8 | 3674.0 | 2.17 | 14.02 | 2.10 | 0.59 | 127.17 | 3277.780 | 3245.719 | 4.60e-09 | 1.48e-11 | 172 |
| 1855–2085 | 6.5 | 12.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3924.888 | 3856.686 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.9 | 4266.1 | 0.87 | 5.97 | 0.84 | 0.80 | 172.05 | 4665.472 | 4591.624 | 3.24e-09 | 3.42e-11 | 208 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2488.213 | 2456.936 | 2496.106 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
