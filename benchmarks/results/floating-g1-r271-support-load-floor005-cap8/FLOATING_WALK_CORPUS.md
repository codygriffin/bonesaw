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
- Finite-support CoP constraint: `disabled`; required `0.000 mm`, measured minimum `0.000 mm` over `661` loaded ticks.
- Balance task: `standalone-authored` `CoM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
| 505 | 2.525 s | 5.143 cm | 0.404 cm | 0.656 cm | 35.307 cm | 7.651° | 8.000 rad/s | 5217.4 µs |

Nominal hard residual maxima: dynamics `1.688e-09`, contact acceleration `5.306e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2438.240 cm |
| authored reference vs measured CoM RMS / p95 | 2429.499 / 4580.301 cm |
| stance foot RMS | 2319.139 cm |
| swing foot RMS | 2631.341 cm |
| hand RMS | 2461.318 cm |
| maximum root rotation | 60.601° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.280e-09 |
| contact acceleration residual | 8.615e-11 |
| raw max dynamics residual, including rejected ticks | 3.280e-09 |
| raw max contact residual, including rejected ticks | 8.615e-11 |
| active normal force range | 0.000–497.566 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 122.950 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 117 ticks |
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

- Phase ticks: unsupported `1656`, single support `361`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.129` / `0.129`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 7.3 µs | 3169.6 µs | 4499.0 µs | 204608.0 µs | 137 | 368 | 0 | 156 | 1,651 | 0 | 0 | 5 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1046.8 | 6640.6 | 1.5 | 2964.4 | 113263.6 | 198065.5 | 1820.8 | 666 | 11 | 3 | 955.3 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 2902.8 | 3012.7 | 3048.8 | 3263.9 |
| solved_with_slack | 368 | 2968.2 | 4568.7 | 5275.5 | 6858.8 |
| normal_contact_contingency | 1,651 | 6.6 | 9.3 | 12.5 | 45.8 |
| contact_release_contingency | 5 | 161516.9 | 198958.1 | 203478.0 | 204608.0 |
| touchdown_transition | 156 | 2732.7 | 3782.9 | 4368.3 | 8816.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.19 | 10.0 | 13.8 | 22 | 1.67 | 13.0 | 22 | 0.1539 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.83/8.0/8.0/8 | 189.62/1768.0/1816.0/1816 | 0.16/2.0/11 | 0.08/1.0/10 | 0.66/8.0/83 | 0.1162 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 368 | 8.12/16.0/19 | 6.41/14.0/17 |
| normal_contact_contingency | 1,651 | 0.00/0.0/0 | 0.00/0.0/0 |
| contact_release_contingency | 5 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 156 | 9.82/17.9/22 | 9.68/17.4/22 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.50/5.0/8 | 2.08/20.0/35 | 0.32/5.0/8 | 300 |
| viability | 0.24/3.0/16 | 1.19/18.0/86 | 0.17/3.0/16 | 210 |
| intent | 0.67/7.8/12 | 1.35/15.7/24 | 0.57/7.8/12 | 438 |
| preference | 0.48/5.0/13 | 4.24/56.0/104 | 0.41/5.0/12 | 519 |
| style | 0.29/1.0/3 | 3.94/17.0/49 | 0.19/1.0/3 | 435 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,890 | 0 | 127 | 300 | 0 |
| right_ankle_roll_link | 1,783 | 0 | 29 | 505 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `117` ticks, normal fallback `0` ticks.
Touchdown Normal sole-center tangential speed: p50 `3.3004 m/s`, p95 `4.4041 m/s`, max `5.2639 m/s` over 152 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.426 | 2.425 | 2.425 | 1.000 | 1.000 | 52.406 | 55.738 | 3.332 | 55.738 | 0.002 | 0 | 792 | 0 | 0 | 21 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2948.2 | 6295.6 | 5.35 | 37.25 | 2.09 | 1.00 | 244.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 2923.8 | 4549.5 | 8.38 | 42.86 | 6.80 | 3.59 | 821.01 | 4.781 | 0.521 | 1.69e-09 | 5.31e-11 | 0 |
| 464–695 | 1838.7 | 3455.0 | 6.53 | 37.53 | 6.23 | 2.45 | 544.89 | 127.131 | 95.572 | 2.14e-09 | 2.23e-11 | 74 |
| 696–927 | 6.7 | 12.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 639.842 | 572.813 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 6.0 | 2937.6 | 0.49 | 3.04 | 0.49 | 0.32 | 70.49 | 1262.975 | 1189.353 | 1.11e-09 | 1.14e-11 | 221 |
| 1160–1391 | 6.6 | 12.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1918.936 | 1843.458 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.5 | 12.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2585.694 | 2516.897 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.6 | 3708.8 | 0.36 | 2.36 | 0.35 | 0.35 | 76.54 | 3256.848 | 3188.553 | 3.28e-09 | 8.61e-11 | 221 |
| 1855–2085 | 6.6 | 13.5 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3929.237 | 3855.530 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.8 | 3988.4 | 0.74 | 4.79 | 0.71 | 0.62 | 137.77 | 4598.029 | 4521.587 | 2.31e-09 | 2.27e-11 | 213 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2438.240 | 2384.096 | 2461.318 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
