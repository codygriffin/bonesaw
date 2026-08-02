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
| 875 | 4.375 s | 2.099 cm | 0.355 cm | 0.110 cm | 34.930 cm | 1.218° | 8.000 rad/s | 5230.4 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `5.972e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1672.537 cm |
| authored reference vs measured CoM RMS / p95 | 1664.740 / 3505.657 cm |
| stance foot RMS | 1567.147 cm |
| swing foot RMS | 1871.609 cm |
| hand RMS | 1689.731 cm |
| maximum root rotation | 50.855° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.618e-09 |
| contact acceleration residual | 2.295e-10 |
| raw max dynamics residual, including rejected ticks | 3.618e-09 |
| raw max contact residual, including rejected ticks | 2.295e-10 |
| active normal force range | 0.000–505.833 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 146.656 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 46 ticks |
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

- Phase ticks: unsupported `1267`, single support `400`, precontact `0`, multi-support `650`.
- Joint-velocity envelope active on `16` ticks; maximum active coordinates `1`; mean target/applied scale `0.281` / `0.281`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 8.6 µs | 3625.5 µs | 5184.8 µs | 11363.2 µs | 312 | 573 | 0 | 127 | 1,293 | 3 | 5 | 0 | 0 | 4 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1310.7 | 1551.4 | 3.0 | 3222.5 | 7754.9 | 10547.3 | 2281.3 | 1,053 | 30 | 0 | 763.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 312 | 1814.6 | 2954.3 | 3006.3 | 3035.4 |
| solved_with_slack | 573 | 3062.4 | 4671.1 | 6123.3 | 7778.8 |
| normal_contact_contingency | 1,293 | 6.4 | 10.4 | 1873.1 | 4037.9 |
| contact_release_contingency | 4 | 1291.9 | 1994.3 | 2093.5 | 2118.3 |
| touchdown_transition | 127 | 2813.7 | 4661.7 | 7804.6 | 11363.2 |
| normal_fallback_relock_probe_admitted | 3 | 6173.6 | 6576.0 | 6611.8 | 6620.7 |
| normal_fallback_relock_probe_rejected | 5 | 2540.4 | 2560.7 | 2564.5 | 2565.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.19 | 10.0 | 15.0 | 23 | 2.12 | 13.8 | 21 | 0.8760 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.80/1.0/8.0/8 | 178.46/234.0/1776.0/1776 | 0.16/4.0/12 | 0.11/3.0/11 | 0.87/24.0/93 | 0.5805 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 312 | 4.55/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 573 | 7.98/18.3/23 | 6.24/17.3/21 |
| normal_contact_contingency | 1,293 | 0.20/8.0/15 | 0.17/7.0/14 |
| contact_release_contingency | 4 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 127 | 8.48/14.7/15 | 8.25/14.0/14 |
| normal_fallback_relock_probe_admitted | 3 | 9.33/11.0/11 | 9.00/11.0/11 |
| normal_fallback_relock_probe_rejected | 5 | 7.80/10.9/11 | 6.40/9.9/10 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.58/4.0/6 | 2.38/18.8/26 | 0.20/4.0/6 | 155 |
| viability | 0.27/3.0/10 | 1.21/18.0/51 | 0.16/3.0/9 | 179 |
| intent | 1.06/9.0/15 | 2.14/18.0/30 | 0.83/9.0/14 | 540 |
| preference | 0.81/8.0/19 | 6.67/64.0/157 | 0.65/7.8/18 | 737 |
| style | 0.47/2.0/4 | 6.69/31.0/49 | 0.28/1.0/4 | 625 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,578 | 0 | 45 | 659 | 35 |
| right_ankle_roll_link | 1,356 | 0 | 82 | 875 | 4 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `46` ticks, normal fallback `18` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.3739 m/s`, p95 `5.4578 m/s`, max `6.9784 m/s` over 123 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.037 | 3.037 | 3.037 | 1.000 | 1.000 | 52.371 | 55.703 | 3.332 | 55.703 | 0.002 | 0 | 792 | 0 | 0 | 25 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2934.5 | 6299.3 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 2032.2 | 4554.6 | 6.72 | 37.97 | 3.28 | 1.57 | 352.78 | 0.665 | 0.334 | 1.29e-09 | 4.22e-11 | 0 |
| 464–695 | 2980.4 | 4734.1 | 6.98 | 41.75 | 4.09 | 1.00 | 230.56 | 0.221 | 0.286 | 1.11e-09 | 5.97e-11 | 0 |
| 696–927 | 2940.5 | 6291.9 | 8.21 | 45.78 | 7.12 | 1.29 | 295.53 | 11.017 | 8.084 | 1.27e-09 | 4.09e-11 | 40 |
| 928–1159 | 5.7 | 3443.8 | 1.33 | 7.69 | 1.29 | 0.91 | 196.45 | 251.494 | 240.158 | 2.64e-09 | 1.81e-11 | 196 |
| 1160–1391 | 6.5 | 14.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 816.507 | 769.373 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 5.8 | 11.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1482.766 | 1430.310 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.5 | 5105.4 | 1.54 | 9.76 | 1.52 | 1.00 | 216.00 | 2154.199 | 2087.552 | 3.62e-09 | 2.29e-10 | 189 |
| 1855–2085 | 6.5 | 11.3 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2836.434 | 2769.315 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.7 | 4186.1 | 1.71 | 10.04 | 1.69 | 1.20 | 259.01 | 3525.808 | 3460.581 | 1.38e-09 | 2.39e-11 | 185 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1672.537 | 1631.840 | 1689.731 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
